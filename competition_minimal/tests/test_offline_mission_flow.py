"""Synthetic multi-module scenarios; no ROS, MORAI, real sensor or vehicle model."""
import importlib.util
import math
from pathlib import Path
import struct
import sys
import threading
from types import ModuleType
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from control_core import Driver
from localization_core import CourseProgress, Estimator
from traffic_core import TrafficDecision
from udp_protocol import (Command, ControlGate, StatusSequence, command_packet,
                          parse_gps, parse_imu, parse_status_packet)


def fake_module(name, **values):
    result = ModuleType(name)
    for key, value in values.items(): setattr(result, key, value)
    return result


def packet(speed=0., sec=10):
    values = [0]*30
    values[:4] = [sec, 10, 2, 4]
    values[20] = speed
    return (b'#MoraiInfo$' + struct.pack('<iiii', 152, 0, 0, 0) +
            struct.pack('<iibbfiff' + 'f'*22 + '38s', *values, b'link') + b'\r\n')


def box(s, d, ego, ident=1):
    return dict(s=s, d=d, hs=.35, hd=.35, bx=s-ego, by=d, radius=.35,
                wx=s, wy=d, vx=0., vy=0., observed=1, id=ident,
                size=.7, height=1.5, clear=0)


class OfflineMissionFlow(unittest.TestCase):
    def setUp(self):
        dummy = type('Dummy', (), {})
        mods = {'rospy': fake_module('rospy', loginfo_throttle=lambda *a: None),
                'geometry_msgs': fake_module('geometry_msgs'),
                'geometry_msgs.msg': fake_module('geometry_msgs.msg', PoseStamped=dummy),
                'sensor_msgs': fake_module('sensor_msgs', point_cloud2=fake_module('point_cloud2')),
                'sensor_msgs.msg': fake_module('sensor_msgs.msg', PointCloud2=dummy),
                'std_msgs': fake_module('std_msgs'),
                'std_msgs.msg': fake_module('std_msgs.msg', Bool=dummy, Float64=dummy)}
        self.patcher = patch.dict(sys.modules, mods); self.patcher.start(); self.addCleanup(self.patcher.stop)
        spec = importlib.util.spec_from_file_location('offline_lidar_node', ROOT/'scripts/lidar_missions.py')
        self.lidar_mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(self.lidar_mod)
        n = self.lidar_mod.LidarSafety.__new__(self.lidar_mod.LidarSafety)
        n.path = np.array([[float(x), 0.] for x in range(321)])
        n.cum = np.arange(321, dtype=float); n.total = 320.
        n.zones = {'static':[40,80], 'pedestrian':[90,130], 'roundabout':[140,170],
                   'highway':[180,230], 'tunnel':[240,290]}
        n.allow_avoidance, n.road_half_width = True, 6.
        n.highway_return_s, n.offset, n.lookahead = 260., 3.2, 5.
        n.plan = n.plan_mode = n.ped_lock = None
        n.lateral = 0.; n.lock = threading.Lock()
        self.lidar = n
        self.driver = Driver(n.path)
        self.gate = ControlGate(); self.status, self.stamp, _ = parse_status_packet(packet())
        self.gate.observe(self.status, 10.)

    def ground(self, ego):
        return np.array([[x-ego,y] for x in np.arange(max(0.,ego-20), min(320.,ego+60), .5)
                         for y in np.arange(-6.,6.1,.5)])

    def command(self, s, stop=False, limit=8., offset=0.):
        cmd = self.driver.update(s, 0., 0., s, 0., offset, limit, stop, .05)
        output = self.gate.output(cmd, 10., 10.)
        return Command.from_buffer_copy(command_packet(*output))

    def test_01_sensor_packets_localization_start_and_control(self):
        body = 'GPGGA,123519,3723.2475,N,12658.3416,E,1,08,0.9,10.0,M,0.0,M,,'
        checksum = 0
        for ch in body.encode('ascii'): checksum ^= ch
        fix = parse_gps(('$%s*%02X\r\n' % (body, checksum)).encode())
        self.assertEqual(fix[3], 1)
        imu = b'#IMUData$' + bytes(24) + struct.pack('<10d', 1.,0.,0.,0.,0.,0.,0.,0.,0.,0.) + b'\r\n'
        self.assertEqual(parse_imu(imu)[:4], (1.,0.,0.,0.))
        sequence = StatusSequence(); sequence.accept(self.stamp)
        est = Estimator(); self.assertTrue(est.fix(0., 0., 10.))
        self.assertTrue(est.advance(10.05, self.status.speed, 0.))
        course = CourseProgress(self.lidar.path)
        self.assertTrue(course.update(est.x, est.y)); self.assertEqual(course.progress, 0.)
        self.assertGreater(self.command(course.progress).accel, 0.)
        with self.assertRaises(ValueError): sequence.accept(self.stamp)
        self.assertEqual(Command.from_buffer_copy(command_packet(*self.gate.output((1,0,0), 10., 11.))).brake, 1.)

    def test_02_red_stop_green_restarts(self):
        traffic = TrafficDecision([dict(start_s=10., stop_s=25., direction='straight', roi=[0,0,1,1])], True)
        traffic.observe(21., ['4red'], 10.)
        stop, limit, _ = traffic.decision(21., 10.)
        self.assertEqual(self.command(21., stop, limit).brake, 1.)
        traffic.observe(21., ['4green'], 10.1); traffic.observe(21., ['4green'], 10.2)
        stop, limit, _ = traffic.decision(21., 10.2)
        self.assertGreater(self.command(21., stop, limit).accel, 0.)

    def test_03_static_avoid_and_return(self):
        ego=50.; obstacle=box(73., 0., ego)
        self.lidar.decide([obstacle], ego, [obstacle], self.ground(ego), (ego,0.,0.))
        self.assertEqual(self.lidar.plan_mode, 'static')
        self.assertGreater(self.command(ego, self.lidar.stop, self.lidar.speed_limit, self.lidar.lateral).accel, 0.)
        end = self.lidar.plan[3] + self.lidar.plan[4] + .1
        self.lidar.decide([], end, [], self.ground(end), (end,0.,0.))
        self.assertIsNone(self.lidar.plan); self.assertEqual(self.lidar.lateral, 0.)

    def test_04_pedestrian_yield_and_observed_clear(self):
        ego=100.; person=box(107., 0., ego, ident=7)
        self.lidar.decide([person], ego, [person])
        self.assertEqual(self.command(ego, self.lidar.stop, self.lidar.speed_limit).brake, 1.)
        self.lidar.decide([], ego, []); self.assertTrue(self.lidar.stop)
        person.update(d=4., by=4.)
        for _ in range(3): self.lidar.decide([person], ego, [person])
        self.assertFalse(self.lidar.stop)
        self.assertGreater(self.command(ego, self.lidar.stop, self.lidar.speed_limit).accel, 0.)

    def test_05_roundabout_yield_and_clear(self):
        ego=150.; vehicle=box(160., 5., ego)
        self.lidar.decide([vehicle], ego, [vehicle])
        self.assertEqual(self.command(ego, self.lidar.stop, self.lidar.speed_limit).brake, 1.)
        self.lidar.decide([], ego, [])
        self.assertGreater(self.command(ego, self.lidar.stop, self.lidar.speed_limit).accel, 0.)

    def test_06_highway_change_return_before_cp12(self):
        ego=200.; lead=box(218., 0., ego)
        self.lidar.decide([lead], ego, [lead], self.ground(ego), (ego,0.,0.))
        self.assertEqual(self.lidar.plan_mode, 'highway')
        plan=self.lidar.plan; self.assertLessEqual(plan[3]+plan[4], self.lidar.highway_return_s-15.)
        self.assertGreater(self.command(ego, self.lidar.stop, self.lidar.speed_limit, self.lidar.lateral).accel, 0.)
        end = plan[3]+plan[4]+.1
        self.lidar.decide([], end, [], self.ground(end), (end,0.,0.))
        self.assertIsNone(self.lidar.plan); self.assertEqual(self.lidar.lateral, 0.)

    def test_07_tunnel_dr_and_observed_pass(self):
        est=Estimator(); est.fix(240., 0., 10.)
        self.assertTrue(est.advance(10.5, 2., 0.)); self.assertAlmostEqual(est.x, 241.)
        ego=est.x; obstacle=box(ego+18,0.,ego)
        self.lidar.decide([obstacle], ego, [obstacle], self.ground(ego), (ego,0.,0.))
        self.assertEqual(self.lidar.plan_mode, 'tunnel')
        self.assertGreater(self.command(ego, self.lidar.stop, self.lidar.speed_limit, self.lidar.lateral).accel, 0.)
        self.lidar.decide([obstacle], ego, [obstacle], None, (ego,0.,0.))
        self.assertTrue(self.lidar.stop); self.assertIsNotNone(self.lidar.plan)
        self.assertTrue(est.fix(242., 0., 10.5)); self.assertTrue(est.advance(10.6, 2., 0.))
        self.assertGreater(est.x, 241.2); self.assertLess(est.x, 242.2)

    def test_08_finish_manual_park_acknowledged(self):
        driver = Driver([(0,0),(10,0),(20,0),(20,10),(0,0)])
        command = driver.update(0, 0, 0, driver.s[-1]-.1, 0, 0, 5, False, .05)
        self.assertTrue(driver.finished)
        self.assertEqual(self.gate.output(command, 10., 10., True)[1], 1.)
        self.gate.observe(self.status, 11.1)
        park = self.gate.output(command, 11.1, 11.1)
        self.assertEqual(park[4], 1)
        self.gate.observe(self.status._replace(gear=1), 11.2)
        manual = self.gate.output(command, 11.2, 11.2)
        self.assertEqual(manual[3:5], (1,1))
        self.gate.observe(self.status._replace(gear=1,mode=1), 11.3)
        self.assertIsNone(self.gate.output(command,11.3,11.3)); self.assertTrue(self.gate.complete)


if __name__ == '__main__': unittest.main()
