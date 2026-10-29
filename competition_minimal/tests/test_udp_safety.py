"""Synthetic tests only: these are not captures of a MORAI competition packet."""
import math
import os
import sys
import struct
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from udp_protocol import ControlGate, Status, StatusValues, StatusSequence, command_packet, parse_status, parse_status_packet, Command, status_diagnostic


def status(speed=0, gear=4, mode=2):
    return StatusValues(speed, 0., 0., gear, mode)


def competition_packet(sec=10, nsec=50, speed=36., yaw=90., steer=5., mode=2, gear=4):
    values = [0] * 30
    values[0:4] = [sec, nsec, mode, gear]
    values[19], values[20], values[29] = yaw, speed, steer
    body = struct.pack('<iibbfiff' + 'f'*22 + '38s', *values, b'link')
    return b'#MoraiInfo$' + struct.pack('<iiii', 152, 0, 0, 0) + body + b'\r\n'


class PacketSafetyTest(unittest.TestCase):
    def test_local_ego_layout_fields_and_invalid_values(self):
        data = Status()
        data.header, data.tail = b'#MoraiInfo$', b'\r\n'
        data.length = 200
        data.mode, data.gear = 2, 4
        data.vel_x, data.yaw, data.steer = 36., 45., .25
        self.assertEqual(parse_status_packet(bytes(data))[2], 'ego')
        self.assertEqual(parse_status(bytes(data)), StatusValues(10., math.radians(45.), .25, 4, 2))
        data.steer = math.nan
        with self.assertRaises(ValueError):
            parse_status(bytes(data))
        data.steer, data.mode = 0., 99
        with self.assertRaises(ValueError):
            parse_status(bytes(data))

    def test_length_alone_does_not_identify_competition(self):
        for length in (181, 229):
            raw = b'#MoraiInfo$' + bytes(length - 13) + b'\r\n'
            with self.assertRaises(ValueError):
                parse_status(raw)
            self.assertIn('length=%d' % length, status_diagnostic(raw))

    def test_competition_layout_and_units(self):
        packet = competition_packet()
        self.assertEqual(len(packet), 181)
        value, stamp, kind = parse_status_packet(packet)
        self.assertEqual(kind, 'competition')
        self.assertEqual(stamp, (10, 50))
        self.assertEqual(value, StatusValues(10., math.pi/2, 5., 4, 2))

    def test_competition_bad_framing_length_nonfinite_and_ranges(self):
        packet = competition_packet()
        for bad in (packet[:-1], b'!' + packet[1:], packet[:-2] + b'!!',
                    packet[:11] + struct.pack('<i', 200) + packet[15:],
                    competition_packet(speed=math.nan), competition_packet(yaw=math.inf),
                    competition_packet(mode=99), competition_packet(nsec=1000000000)):
            with self.subTest(length=len(bad)):
                with self.assertRaises(ValueError): parse_status_packet(bad)

    def test_repeated_or_old_timestamp_rejected(self):
        sequence = StatusSequence()
        sequence.accept((10, 50))
        for stamp in ((10, 50), (10, 49), (9, 999999999)):
            with self.assertRaises(ValueError): sequence.accept(stamp)
        sequence.accept((11, 0))

    def test_never_seize_manual_and_watchdog_brakes(self):
        gate = ControlGate()
        gate.observe(status(mode=1, gear=1), 0.)
        self.assertIsNone(gate.output((.5, 0., 0.), 0., .1))
        gate.observe(status(), .2)
        self.assertEqual(gate.output((.5, 0., .1), .2, .3), (.5, 0., .1, 2, 4))
        self.assertEqual(gate.output((.5, 0., .1), .2, .8), (0., 1., 0., 2, 4))
        gate.invalidate()
        self.assertEqual(gate.output((.5, 0., .1), .8, .8), (0., 1., 0., 2, 4))
        gate.observe(status(mode=1, gear=1), .9)
        self.assertIsNone(gate.output((.5, 0., .1), .9, .9))

    def test_finish_requires_stationary_hold_and_observed_park(self):
        gate = ControlGate()
        gate.observe(status(speed=10), 1.)
        self.assertEqual(gate.output((1., 0., 0.), 1., 1., True), (0., 1., 0., 2, 4))
        gate.observe(status(), 2.)
        self.assertEqual(gate.output(None, None, 2.), (0., 1., 0., 2, 4))
        gate.observe(status(), 3.1)
        self.assertEqual(gate.output(None, None, 3.1), (0., 1., 0., 2, 1))
        # Until the gear report changes, continue requesting P, not Manual.
        gate.observe(status(), 3.2)
        self.assertEqual(gate.output(None, None, 3.2), (0., 1., 0., 2, 1))
        gate.observe(status(gear=1), 3.3)
        handover = gate.output(None, None, 3.3)
        self.assertEqual(handover, (0., 1., 0., 1, 1))
        wire = Command.from_buffer_copy(command_packet(*handover))
        self.assertEqual((wire.mode, wire.gear, wire.brake), (1, 1, 1.))
        gate.observe(status(gear=1, mode=1), 3.4)
        self.assertIsNone(gate.output(None, None, 3.4))
        self.assertTrue(gate.complete)

    def test_finish_data_loss_cannot_start_handover(self):
        gate = ControlGate()
        gate.observe(status(), 1.)
        gate.output(None, None, 1., True)
        self.assertEqual(gate.output(None, None, 3.), (0., 1., 0., 2, 4))
        gate.observe(status(), 3.1)
        self.assertEqual(gate.output(None, None, 3.1), (0., 1., 0., 2, 4))

    def test_loss_during_handover_does_not_revert_gear_or_seize_manual(self):
        gate = ControlGate()
        gate.observe(status(), 1.)
        gate.output(None, None, 1., True)
        gate.observe(status(), 2.1)
        self.assertEqual(gate.output(None, None, 2.1), (0., 1., 0., 2, 1))
        self.assertEqual(gate.output(None, None, 3.), (0., 1., 0., 2, 1))
        gate.observe(status(gear=1), 3.1)
        gate.output(None, None, 3.1)
        gate.observe(status(gear=1), 4.2)
        self.assertEqual(gate.output(None, None, 4.2), (0., 1., 0., 1, 1))
        self.assertIsNone(gate.output(None, None, 5.))


if __name__ == '__main__':
    unittest.main()
