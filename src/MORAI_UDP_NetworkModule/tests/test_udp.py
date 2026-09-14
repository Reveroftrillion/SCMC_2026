"""Run with python -m unittest discover -s tests -v (no ROS required)."""
import ctypes
import math
from pathlib import Path
import socket
import struct
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.network.UDP import Receiver, Sender
from lib.network.parsers import (GPSParser, CameraParser, imu_parser,
                                 status_parser, command_values, watchdog_values)
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from lib.define.EgoCtrlCmd import EgoCtrlCmd


def nmea(body):
    check = 0
    for b in body.encode('ascii'):
        check ^= b
    return ('$%s*%02X\r\n' % (body, check)).encode('ascii')


def camera(index, data, tail=b'AI', sec=1, padded=False):
    payload = data.ljust(64979, b'\0') if padded else data
    return b'MOR' + struct.pack('<4I', sec, 0, index, len(data)) + payload + tail


class Parsers(unittest.TestCase):
    def test_gga_and_hemispheres(self):
        value = GPSParser()(nmea('GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,'))
        self.assertAlmostEqual(value[0], -48.1173)
        self.assertAlmostEqual(value[1], -11.5166666667)
        self.assertEqual(value[2:], (545.4, 1))

    def test_gps_invalid_fix(self):
        self.assertEqual(GPSParser()(nmea('GPGGA,123519,,,,,0,00,,,,,,,')), (0., 0., 0., 0))

    def test_gps_checksum_and_bounds(self):
        with self.assertRaises(ValueError):
            GPSParser()(b'$GPGGA,broken*00')
        with self.assertRaises(ValueError):
            GPSParser()(nmea('GPGGA,123519,9160.0,N,01131.0,E,1,08,0.9,2,M,0,M,,'))

    def test_gps_rmc_does_not_reset_altitude(self):
        raw = nmea('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,')
        raw += nmea('GPRMC,123519,A,4807.038,N,01131.000,E,0,0,010100,,,A')
        self.assertEqual(GPSParser()(raw)[2], 545.4)

    def test_camera_variable_and_padded(self):
        for padded in (False, True):
            parser = CameraParser()
            self.assertIsNone(parser(camera(0, b'\xff\xd8one', padded=padded)))
            self.assertEqual(parser(camera(1, b'two\xff\xd9', b'EI', padded=padded)),
                             b'\xff\xd8onetwo\xff\xd9')

    def test_camera_loss_reorder_and_recovery(self):
        parser = CameraParser()
        parser(camera(0, b'\xff\xd8'))
        with self.assertRaises(ValueError):
            parser(camera(2, b'lost\xff\xd9', b'EI'))
        self.assertEqual(parser(camera(0, b'\xff\xd8new\xff\xd9', b'EI', sec=2)),
                         b'\xff\xd8new\xff\xd9')

    def test_camera_expiry_and_limit(self):
        parser = CameraParser(timeout=0.5)
        with patch('lib.network.parsers.time.monotonic', return_value=1):
            parser(camera(0, b'\xff\xd8'))
        with patch('lib.network.parsers.time.monotonic', return_value=2):
            with self.assertRaises(ValueError):
                parser(camera(1, b'\xff\xd9', b'EI'))
        with self.assertRaises(ValueError):
            CameraParser(max_bytes=3)(camera(0, b'\xff\xd8\xff\xd9', b'EI'))

    def test_camera_rejects_gt_and_bad_size(self):
        with self.assertRaises(ValueError):
            CameraParser()(b'BOX' + bytes(30))
        with self.assertRaises(ValueError):
            CameraParser()(camera(0, b'\xff\xd8')[:-1])

    def test_imu_layouts(self):
        for layout, offset in [('timestamped_115', 33), ('legacy_107', 25)]:
            packet = b'#IMUData$' + bytes(offset-9)
            packet += struct.pack('<10d', 2, 0, 0, 0, 1, 2, 3, 4, 5, 6) + b'\r\n'
            self.assertEqual(imu_parser(layout)(packet), (1., 0., 0., 0., 1., 2., 3., 4., 5., 6.))
            other = 'legacy_107' if offset == 33 else 'timestamped_115'
            with self.assertRaises(ValueError):
                imu_parser(other)(packet)

    def test_status_exact_layout_and_snapshot(self):
        value = EgoVehicleStatus()
        value.header, value.tail = b'#MoraiInfo$', b'\r\n'
        value.vel_x, value.yaw = 12, 45
        packet = bytes(value)
        parse = status_parser(EgoVehicleStatus, b'#MoraiInfo$')
        self.assertEqual(parse(packet).vel_x, 12)
        for bad in (packet[:-1], packet + b'X', b'X' + packet[1:]):
            with self.assertRaises(ValueError):
                parse(bad)
        value.vel_x = math.nan
        with self.assertRaises(ValueError):
            parse(bytes(value))

    def test_command_ranges_and_nan(self):
        self.assertEqual(command_values(2, 0, -3), (1, 0, -1))
        self.assertEqual(command_values(1, 0.5, 0), (0, 0.5, 0))
        with self.assertRaises(ValueError):
            command_values(math.nan, 0, 0)

    def test_watchdog_requires_live_command_and_state(self):
        command = (0.3, 0., 0.1)
        self.assertEqual(watchdog_values(command, 1., 1., 1.1, .3, .5), command)
        for cmd_time, state_time, now in [(None, 1, 1), (1, None, 1), (1, 1.5, 1.5), (2, 1, 2)]:
            self.assertEqual(watchdog_values(command, cmd_time, state_time, now, .3, .5),
                             (0., 1., 0.))


class Transport(unittest.TestCase):
    def test_loopback_receive_reject_expire_close(self):
        warnings = []
        receiver = Receiver('127.0.0.1', 0, data_type=EgoCtrlCmd(), on_error=warnings.append)
        sender = Sender('127.0.0.1', receiver.socket.getsockname()[1])
        try:
            self.assertIsNone(receiver.get_data())
            sender.socket.sendto(b'bad', ('127.0.0.1', receiver.socket.getsockname()[1]))
            data = EgoCtrlCmd()
            data.accel = 0.25
            sender.send(data)
            deadline = time.monotonic() + 1
            while receiver.get_data() is None and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertAlmostEqual(receiver.get_data().accel, .25)
            self.assertTrue(warnings)
            self.assertIsNone(receiver.get_data(max_age=-1))
            self.assertEqual(ctypes.sizeof(data), 55)
        finally:
            receiver.close()
            sender.close()
        self.assertFalse(receiver._thread.is_alive())


if __name__ == '__main__':
    unittest.main()
