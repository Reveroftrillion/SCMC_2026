import ast
import os
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from control_core import valid_zones
from reproject_missions import cumulative, point_at, project, MEASURED_XY, MAX_XY_RESIDUAL
from preflight import preflight


class ContractAndPathTest(unittest.TestCase):
    def test_launch_targets_exist(self):
        launch = ET.parse(ROOT / 'launch/competition.launch')
        for node in launch.findall('.//node'):
            if node.get('pkg') == 'competition_minimal':
                script = ROOT / 'scripts' / node.get('type')
                self.assertTrue(script.is_file())
                self.assertTrue(script.read_bytes().startswith(b'#!/usr/bin/env python3\n'))
        for script in (ROOT / 'scripts').glob('*.py'):
            self.assertNotIn(b'\r\n', script.read_bytes(), str(script))
            ast.parse(script.read_text(encoding='utf-8'), filename=str(script))

    def test_receive_only_forces_disabled_udp_control(self):
        launch = ET.parse(ROOT / 'launch/competition.launch')
        bridge = next(n for n in launch.findall('.//node') if n.get('type') == 'udp_bridge.py')
        switches = [p for p in bridge.findall('param') if p.get('name') == 'enable_control']
        self.assertTrue(any(p.get('unless') == '$(arg drive)' and p.get('value') == 'false' for p in switches))
        self.assertTrue(any(p.get('if') == '$(arg drive)' for p in switches))

    def test_preflight_reads_real_paths_and_rejects_duplicate_ports(self):
        self.assertTrue(preflight(ROOT, {'morai_ip':'127.0.0.1'}, probe_ports=False, load_model=False))
        with self.assertRaises(ValueError):
            preflight(ROOT, {'morai_ip':'127.0.0.1','gps_port':'9297'}, probe_ports=False, load_model=False)

    def test_message_contract(self):
        workspace = ROOT.parent
        for file, fields in (('src/simul_msgs/msg/ControlCmd.msg', ('accel','brake','steering')),
                             ('src/simul_msgs/msg/VehicleStatus.msg', ('vel_x','yaw')),
                             ('src/MORAI-ROS_morai_msgs/msg/GPSMessage.msg', ('latitude','longitude','altitude','status'))):
            names = {line.split()[-1] for line in (workspace / file).read_text().splitlines() if line.strip()}
            self.assertTrue(set(fields).issubset(names))
        def src(name): return (ROOT / 'scripts' / name).read_text(encoding='utf-8')
        for topic in ('/competition/pose', '/competition/progress', '/competition/localization_ok', '/competition/dr_active'):
            self.assertIn(topic, src('localization.py'))
            self.assertIn(topic, src('controller.py'))
        for topic in ('/competition/lidar_stop', '/competition/lidar_speed_limit', '/competition/lateral_offset'):
            self.assertIn(topic, src('lidar_missions.py'))
            self.assertIn(topic, src('controller.py'))
        for topic in ('/competition/traffic_stop', '/competition/traffic_speed_limit'):
            self.assertIn(topic, src('traffic_node.py'))
            self.assertIn(topic, src('controller.py'))
        self.assertIn('/competition/control_cmd', src('udp_bridge.py'))
        self.assertIn('/competition/control_cmd', src('controller.py'))

    def test_declared_ros_topic_types_match(self):
        publishers, subscribers = {}, {}
        for script in (ROOT / 'scripts').glob('*.py'):
            tree = ast.parse(script.read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in ('Publisher', 'Subscriber'):
                    if len(node.args) >= 2 and isinstance(node.args[0], ast.Constant) and isinstance(node.args[1], ast.Name):
                        (publishers if node.func.attr == 'Publisher' else subscribers).setdefault(node.args[0].value, set()).add(node.args[1].id)
                # The controller declares its subscriptions as (name, topic, type).
                if isinstance(node, ast.Assign) and any(isinstance(x, ast.Name) and x.id == 'definitions' for x in node.targets):
                    for entry in node.value.elts:
                        topic, typ = entry.elts[1], entry.elts[2]
                        subscribers.setdefault(topic.value, set()).add(typ.id)
        for topic in ('/gps', '/imu', '/vehicle_status', '/camera/traffic/image/compressed',
                      '/competition/pose', '/competition/progress', '/competition/localization_ok',
                      '/competition/dr_active', '/competition/traffic_stop', '/competition/traffic_speed_limit',
                      '/competition/lidar_stop', '/competition/lidar_speed_limit', '/competition/lateral_offset',
                      '/competition/control_cmd', '/competition/course_finished'):
            self.assertEqual(publishers.get(topic), subscribers.get(topic), topic)
            self.assertTrue(publishers.get(topic), topic)

    def test_course_and_known_checkpoints(self):
        path = np.loadtxt(ROOT / 'paths/global_path.txt', usecols=(0, 1))
        self.assertEqual(len(path), 4430)
        self.assertTrue(np.allclose(path[0], path[-1]))
        self.assertAlmostEqual(cumulative(path)[-1], 2184.6117, places=3)
        for i, xy in ((2275, (62.0739,256.4949)), (3231,(75.4142,-217.8551)),
                      (3529,(70.2141,-365.7485))):
            self.assertLess(np.linalg.norm(path[i]-xy), .001)

    def test_reprojection_uses_coordinates_and_detects_crossings(self):
        path = np.array([[0.,0.], [10.,0.], [10.,10.], [0.,10.], [0.,0.]])
        s = cumulative(path)
        result, error, ambiguous = project(path, s, point_at(path, s, 5.0))
        self.assertAlmostEqual(result, 5.0)
        self.assertAlmostEqual(error, 0.0)
        self.assertFalse(ambiguous)
        self.assertTrue(project(path, s, np.array([0., 0.]))[2])

    def test_empty_or_incomplete_zones_never_validate(self):
        self.assertFalse(valid_zones({}, 2000))
        self.assertFalse(valid_zones({'static': [0, 10]}, 2000))
        zones = {name: [1, 2] for name in ('static','pedestrian','roundabout','highway','tunnel')}
        self.assertTrue(valid_zones(zones, 2000))
        zones['tunnel'] = [2, 3000]
        self.assertFalse(valid_zones(zones, 2000))

    def test_measured_missions_are_projected_and_uncertainty_marked(self):
        with (ROOT / 'config/missions.yaml').open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        path = np.loadtxt(ROOT / 'paths/global_path.txt', usecols=(0, 1))
        distances = cumulative(path)
        self.assertTrue(config['mission_zones_verified'])
        self.assertTrue(config['positions_verified'])
        self.assertTrue(valid_zones(config['zones'], distances[-1]))
        self.assertEqual(len(config['intersections']), 4)
        last_s = -1
        projected = {}
        for name, xy in MEASURED_XY.items():
            s, err, ambiguous = project(path, distances, np.asarray(xy))
            self.assertGreater(s, last_s, name)
            self.assertFalse(ambiguous, name)
            projected[name] = (s, err)
            last_s = s
        for i, mission in enumerate(config['intersections'], 1):
            for field, suffix in (('start_s', 'start'), ('stop_s', 'stop')):
                self.assertAlmostEqual(mission[field], projected[f'traffic{i}_{suffix}'][0], places=4)
            expected = all(projected[f'traffic{i}_{suffix}'][1] <= MAX_XY_RESIDUAL
                           for suffix in ('start', 'stop'))
            self.assertEqual(mission['verified'], expected)
        for zone, begin, end in (('roundabout','roundabout_start','roundabout_end'),
                                 ('highway','cp10','cp13'), ('tunnel','tunnel_start','tunnel_end')):
            self.assertAlmostEqual(config['zones'][zone][0], projected[begin][0], places=4)
            self.assertAlmostEqual(config['zones'][zone][1], projected[end][0], places=4)
        self.assertAlmostEqual(config['zones']['static'][1], projected['static_end'][0], places=4)
        self.assertAlmostEqual(config['zones']['pedestrian'][1], projected['pedestrian_end'][0], places=4)
        self.assertAlmostEqual(config['highway_return_s'], projected['cp12'][0], places=4)
        self.assertAlmostEqual(config['zones']['static'][0], projected['traffic2_start'][0], places=4)
        self.assertEqual(len(config['unverified_sections']), 3)
        for section in config['unverified_sections']:
            self.assertLess(section['start_s'], section['end_s'])
            self.assertTrue(section['reason'])


if __name__ == '__main__': unittest.main()
