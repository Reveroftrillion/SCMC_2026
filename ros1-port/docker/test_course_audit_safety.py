import math
import struct
import unittest
from course_audit_safety import (DepartureGuard, collision_slots, decode_collision,
                                 input_loss_reason, manual_park_evidence, missing_required_nodes)


class AuditSafetyTest(unittest.TestCase):
    def test_input_loss_requires_fresh_collision_status_and_imu(self):
        topics = {t: 30. for t in ('/sensors/imu', '/sensors/lidar/points', '/perception/footprints',
                                  '/sensors/camera/traffic/compressed', '/localization/odometry')}
        wire = {'909': 30., '9092': 30.}
        self.assertIsNone(input_loss_reason(20., 0., {}, {}))
        self.assertIsNone(input_loss_reason(30., 0., wire, topics))
        self.assertEqual(input_loss_reason(30., 0., dict(wire, **{'9092': 28.9}), topics),
                         'collision_monitor_lost')
        self.assertEqual(input_loss_reason(30., 0., dict(wire, **{'909': 28.9}), topics),
                         'raw_vehicle_status_lost')
        self.assertIn('/sensors/imu', input_loss_reason(
            30., 0., wire, dict(topics, **{'/sensors/imu': 26.9})))
        # GPS can stop producing valid fixes in the tunnel; IMU/odom must remain fresh.
        self.assertIsNone(input_loss_reason(30., 0., wire, dict(topics, **{'/sensors/gps': 0.})))

    def test_required_perception_nodes_are_monitored_outside_signal_zones(self):
        names = {'/sensor_bridge', '/localization', '/path_follower', '/behavior_node',
                 '/lidar_bridge', '/obstacle_node', '/camera_bridge', '/yolo_signal_node'}
        self.assertEqual(missing_required_nodes(20., 0., []), [])
        self.assertEqual(missing_required_nodes(30., 0., names), [])
        self.assertEqual(missing_required_nodes(30., 0., names-{'/yolo_signal_node'}),
                         ['/yolo_signal_node'])

    def test_real_recorded_no_collision_and_object_alarm(self):
        self.assertEqual(collision_slots([dict(object_type=-1, object_id=0),
                                         dict(object_type=0, object_id=0)]), [])
        for kind in (0, 1, 2):
            self.assertEqual(len(collision_slots([dict(object_type=kind, object_id=17)])), 1)

    def samples(self):
        return [dict(time=1+i*.1, speed=0., mode=1, gear=1, stamp_hex=str(i)) for i in range(7)]

    def test_handoff_requires_fresh_distinct_stationary_manual_park(self):
        rows = self.samples()
        self.assertIsNotNone(manual_park_evidence(rows, 1., 1.65))
        for key, value in [('speed', .02), ('speed', math.nan), ('mode', 2), ('gear', 4)]:
            bad = [dict(row, **{key: value}) for row in rows]
            self.assertIsNone(manual_park_evidence(bad, 1., 1.65))
        self.assertIsNone(manual_park_evidence(rows, 1.5, 1.65))
        self.assertIsNone(manual_park_evidence(rows, 1., 3.))
        self.assertIsNone(manual_park_evidence([dict(row, stamp_hex='same') for row in rows], 1., 1.65))

    def test_legitimate_offset_and_transient_map_gap(self):
        guard = DepartureGuard()
        self.assertFalse(guard.update(0., 6., True))
        self.assertFalse(guard.update(1., 4., False))
        self.assertFalse(guard.update(1.5, 4., True))
        self.assertFalse(guard.update(2., 4., False))
        self.assertTrue(guard.update(3., 4., False))

    def test_large_departure_even_on_another_road(self):
        guard = DepartureGuard()
        self.assertFalse(guard.update(0., 8.1, True))
        self.assertTrue(guard.update(1., 8.1, True))

    def test_collision_framing_and_slots(self):
        raw = bytearray(181)
        raw[:15] = b'#CollisionData$'
        struct.pack_into('<i', raw, 15, 148)
        struct.pack_into('<hh6f', raw, 39, 1, 17, 1., 2., 3., 4., 5., 6.)
        raw[-2:] = b'\r\n'
        self.assertEqual(decode_collision(raw)[0]['object_id'], 17)
        with self.assertRaises(ValueError):
            decode_collision(raw[:-1])


if __name__ == '__main__':
    unittest.main()
