import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from control_core import Driver, uncertainty_limit
from udp_protocol import Command, Status, command_packet, parse_status, safe_command, validate_ports


class ControlTest(unittest.TestCase):
    def setUp(self):
        self.driver = Driver([(0, 0), (10, 0), (20, 0), (20, 10), (0, 0)])

    def test_stop_has_priority_and_invalid_data_stops(self):
        self.assertEqual(self.driver.update(0, 0, 0, 0, 0, 0, 5, True, .05), (0, 1, 0))
        self.assertEqual(self.driver.update(0, 0, math.nan, 0, 0, 0, 5, False, .05), (0, 1, 0))
        self.driver.integral = 3.0
        self.assertEqual(self.driver.update(0, 0, 0, 0, 0, 0, 0, False, .05), (0, 1, 0))

    def test_positive_speed_and_bounded_steering(self):
        a, b, steer = self.driver.update(0, 0, 0, 0, 0, 0, 5, False, .05)
        self.assertGreater(a, 0)
        self.assertEqual(b, 0)
        self.assertLessEqual(abs(steer), 1)

    def test_closed_start_is_not_finish_and_finish_latches(self):
        self.driver.update(0, 0, 0, 0, 0, 0, 5, False, .05)
        self.assertFalse(self.driver.finished)
        stop = self.driver.update(0, 0, 0, self.driver.s[-1]-.1, 0, 0, 5, False, .05)
        self.assertEqual(stop, (0, 1, 0))
        self.assertTrue(self.driver.finished)
        self.assertEqual(self.driver.update(0, 0, 0, 0, 0, 0, 5, False, .05), (0, 1, 0))

    def test_excessive_path_departure_brakes(self):
        self.assertEqual(self.driver.update(0, 10, 0, 0, 0, 0, 5, False, .05), (0, 1, 0))

    def test_slowing_begins_before_finish(self):
        s = self.driver.s[-1]-10.
        x,y,yaw = self.driver.point(s)
        accel, brake, _ = self.driver.update(x,y,yaw,s,10.,0.,20.,False,.05)
        self.assertEqual(accel, 0.)
        self.assertGreater(brake, 0.)

    def test_uncertain_mission_slows_and_stops_before_entry(self):
        sections = [{'start_s': 50.0, 'reason': 'unverified geometry'}]
        self.assertEqual(uncertainty_limit(sections, 45, 100), 0)
        self.assertEqual(uncertainty_limit(sections, 51, 100), 0)
        self.assertGreater(uncertainty_limit(sections, 20, 100), 0)
        self.assertEqual(uncertainty_limit([{}], 0, 100), 0)


class UdpTest(unittest.TestCase):
    def test_command_and_status_loss_hold_brake(self):
        value = (.2, 0., .1)
        self.assertEqual(safe_command(value, 1, 1, 1.1), value)
        for command_time, status_time in ((None, 1), (1, None), (.5, 1), (1, .4)):
            self.assertEqual(safe_command(value, command_time, status_time, 1.1), (0, 1, 0))

    def test_wire_command_braking_overrides_accel(self):
        m = Command.from_buffer_copy(command_packet(1, .5, 2))
        self.assertEqual(m.header, b'#MoraiCtrlCmd$')
        self.assertEqual(m.accel, 0)
        self.assertEqual(m.brake, .5)
        self.assertEqual(m.steer, 1)
        self.assertEqual(Command.from_buffer_copy(command_packet(math.nan, 0, 0)).brake, 1)

    def test_unknown_competition_packet_is_rejected(self):
        with self.assertRaises(ValueError): parse_status(b'unknown packet')
        candidate = Status()
        candidate.header, candidate.tail = b'#MoraiInfo$', b'\r\n'
        candidate.length = 200
        candidate.mode, candidate.gear = 2, 4
        candidate.vel_x, candidate.yaw = 36., 90.
        self.assertEqual(parse_status(bytes(candidate))[:2], (10., math.pi/2))

    def test_udp_port_collisions_are_rejected(self):
        validate_ports({'gps':1111, 'lidar':2368, 'camera':9297})
        with self.assertRaises(ValueError): validate_ports({'gps':1111, 'camera':1111})
        with self.assertRaises(ValueError): validate_ports({'gps':0})


if __name__ == '__main__': unittest.main()
