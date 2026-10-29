"""Run the real controller callback with ROS/message stubs on non-ROS PCs."""
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from control_core import Driver
from udp_protocol import ControlGate, StatusValues, Command, command_packet


class FakePublisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


def module(name, **items):
    result = ModuleType(name)
    for key, value in items.items(): setattr(result, key, value)
    return result


def message(**kwargs): return SimpleNamespace(**kwargs)


class ControllerIntegrationTest(unittest.TestCase):
    def setUp(self):
        dummy = type('Dummy', (), {})
        rospy = module('rospy', logwarn_throttle=lambda *a: None, logerr_throttle=lambda *a: None,
                       loginfo_throttle=lambda *a: None)
        localization = module('localization', read_path=lambda _: [], quaternion_yaw=lambda _: 0.0)
        modules = {'rospy': rospy, 'localization': localization,
                   'geometry_msgs': module('geometry_msgs'),
                   'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=dummy),
                   'simul_msgs': module('simul_msgs'),
                   'simul_msgs.msg': module('simul_msgs.msg', ControlCmd=lambda **k: message(**k), VehicleStatus=dummy),
                   'std_msgs': module('std_msgs'),
                   'std_msgs.msg': module('std_msgs.msg', Bool=lambda **k: message(**k), Float64=dummy)}
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        spec = importlib.util.spec_from_file_location('test_competition_controller', ROOT / 'scripts/controller.py')
        self.controller = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.controller)
        node = self.controller.ControlNode.__new__(self.controller.ControlNode)
        node.lock, node.driver = threading.RLock(), Driver([(0,0), (10,0), (20,0), (20,10), (0,0)])
        node.pub, node.finished_pub = FakePublisher(), FakePublisher()
        node.timeout, node.last_tick = .7, 9.95
        node.sensor_reference_verified = node.mission_zones_verified = True
        node.unverified_sections = []
        pose = message(pose=message(position=message(x=0., y=0.), orientation=message()), header=message(frame_id='map'))
        values = {'pose': pose, 'progress': message(data=0.), 'localization_ok': message(data=True),
                  'speed': message(vel_x=0.), 'traffic_stop': message(data=False),
                  'traffic_limit': message(data=math.inf), 'lidar_stop': message(data=False),
                  'lidar_limit': message(data=math.inf), 'offset': message(data=0.), 'dr': message(data=False)}
        node.values = {k: (v, 10.) for k, v in values.items()}
        self.node = node

    def tick(self, now=10.):
        with patch.object(self.controller.time, 'monotonic', return_value=now): self.node.tick(None)
        return self.node.pub.messages[-1]

    def test_clear_inputs_drive_and_stale_inputs_brake(self):
        self.assertGreater(self.tick().accel, 0.)
        self.assertEqual(self.tick(11.).brake, 1.)

    def test_each_stop_override_prevents_acceleration(self):
        for key in ('traffic_stop', 'lidar_stop'):
            with self.subTest(key=key):
                self.node.values[key][0].data = True
                output = self.tick()
                self.assertEqual((output.accel, output.brake), (0., 1.))
                self.node.values[key][0].data = False
        self.node.values['localization_ok'][0].data = False
        self.assertEqual(self.tick().brake, 1.)

    def test_unverified_config_always_brakes(self):
        self.node.mission_zones_verified = False
        output = self.tick()
        self.assertEqual((output.accel, output.brake), (0., 1.))

    def test_unverified_section_brakes_before_boundary(self):
        self.node.unverified_sections = [{'start_s': 10., 'reason': 'unverified mission'}]
        self.node.values['progress'][0].data = 5.
        self.node.values['pose'][0].pose.position.x = 5.
        self.assertEqual(self.tick().brake, 1.)

    def test_end_publishes_finished_and_latches_brake(self):
        self.node.values['progress'][0].data = self.node.driver.s[-1]-.1
        result = self.tick()
        self.assertEqual(result.brake, 1.)
        self.assertTrue(self.node.finished_pub.messages[-1].data)

    def test_controller_to_udp_gate_then_command_timeout(self):
        gate = ControlGate()
        gate.observe(StatusValues(speed=0., yaw=0., steer=0., gear=4, mode=2), 10.)
        output = self.tick()
        values = gate.output((output.accel, output.brake, output.steering), 10., 10.)
        wire = Command.from_buffer_copy(command_packet(*values))
        self.assertGreater(wire.accel, 0.)
        self.node.values['traffic_stop'][0].data = True
        output = self.tick()
        values = gate.output((output.accel, output.brake, output.steering), 10., 10.)
        wire = Command.from_buffer_copy(command_packet(*values))
        self.assertEqual((wire.accel, wire.brake), (0., 1.))
        values = gate.output((1., 0., 0.), 10., 10.4)
        self.assertEqual(Command.from_buffer_copy(command_packet(*values)).brake, 1.)


if __name__ == '__main__': unittest.main()
