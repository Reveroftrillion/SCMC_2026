"""Exercise real UDP bridge -> localization -> controller callbacks with ROS stubs."""
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from control_core import Driver
from localization_core import CourseProgress, Estimator
from udp_protocol import ControlGate, StatusSequence, Command, command_packet
from test_udp_safety import competition_packet


def mod(name, **attrs):
    m = ModuleType(name)
    for k,v in attrs.items(): setattr(m,k,v)
    return m


def obj(**kw): return SimpleNamespace(**kw)


class Out:
    def __init__(self, fn=None): self.messages=[]; self.fn=fn
    def publish(self, m):
        self.messages.append(m)
        if self.fn: self.fn(m)


class TestNodeChain(unittest.TestCase):
    def test_competition_packet_reaches_localization_and_control(self):
        dummy=type('Dummy',(),{})
        stamp=obj(is_zero=lambda:False)
        pose_factory=lambda: obj(header=obj(stamp=None, frame_id=''),
            pose=obj(position=obj(x=0.,y=0.,z=0.),orientation=obj(x=0.,y=0.,z=0.,w=1.)))
        rospy=mod('rospy', Time=obj(now=lambda:stamp), loginfo_throttle=lambda *a:None,
                  logwarn_throttle=lambda *a:None, logerr_throttle=lambda *a:None,
                  logwarn=lambda *a:None)
        modules={'rospy':rospy, 'geometry_msgs':mod('geometry_msgs'),
                 'geometry_msgs.msg':mod('geometry_msgs.msg', PoseStamped=pose_factory),
                 'morai_msgs':mod('morai_msgs'), 'morai_msgs.msg':mod('morai_msgs.msg',GPSMessage=dummy),
                 'sensor_msgs':mod('sensor_msgs'), 'sensor_msgs.msg':mod('sensor_msgs.msg',Imu=dummy,CompressedImage=dummy),
                 'simul_msgs':mod('simul_msgs'),
                 'simul_msgs.msg':mod('simul_msgs.msg',VehicleStatus=lambda **k:obj(**k), ControlCmd=lambda **k:obj(**k)),
                 'std_msgs':mod('std_msgs'),
                 'std_msgs.msg':mod('std_msgs.msg',Bool=lambda **k:obj(**k),Float64=lambda **k:obj(**k)),
                 'pyproj':mod('pyproj',Transformer=dummy)}
        with patch.dict(sys.modules, modules):
            def load(name):
                spec=importlib.util.spec_from_file_location('chain_'+name,ROOT/'scripts'/('%s.py'%name))
                module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
            locmod=load('localization'); sys.modules['localization']=locmod
            ctrlmod=load('controller'); bridgemod=load('udp_bridge')
            controller=ctrlmod.ControlNode.__new__(ctrlmod.ControlNode)
            controller.lock=threading.RLock(); controller.values={}; controller.driver=Driver([(x,0) for x in range(101)])
            controller.timeout=.7; controller.last_tick=9.95
            controller.sensor_reference_verified=controller.mission_zones_verified=True; controller.unverified_sections=[]
            controller.pub=Out(); controller.finished_pub=Out()
            loc=locmod.LocalizationNode.__new__(locmod.LocalizationNode)
            loc.lock=threading.Lock(); loc.sensor_timeout=.5; loc.yaw_offset=0.
            loc.transformer=obj(transform=lambda lon,lat:(302595.,4124145.)); loc.east=302595.; loc.north=4124145.
            loc.est=Estimator(); loc.course=CourseProgress([(x,0) for x in range(101)])
            loc.gps=loc.imu=loc.status=None; loc.gps_seq=loc.used_gps_seq=0
            loc.pose_pub=Out(lambda m:controller.receive(m,'pose'))
            loc.progress_pub=Out(lambda m:controller.receive(m,'progress'))
            loc.health_pub=Out(lambda m:controller.receive(m,'localization_ok'))
            loc.dr_pub=Out(lambda m:controller.receive(m,'dr'))
            bridge=bridgemod.Bridge.__new__(bridgemod.Bridge)
            bridge.lock=threading.Lock(); bridge.gate=ControlGate(); bridge.status_sequence=StatusSequence()
            bridge.pubs={'status':Out(lambda m:(loc.on_status(m),controller.receive(m,'speed')))}
            for name,value in [('traffic_stop',False),('traffic_limit',math.inf),('lidar_stop',False),
                               ('lidar_limit',math.inf),('offset',0.)]:
                controller.values[name]=(obj(data=value),10.)
            with patch.object(bridgemod.time,'monotonic',return_value=10.):
                loc.on_gps(obj(status=1,latitude=37.,longitude=127.))
                loc.on_imu(obj(orientation=obj(x=0.,y=0.,z=0.,w=1.)))
                bridge.receive('status',competition_packet(speed=0.))
                loc.tick(None)
                controller.tick(None)
                result=controller.pub.messages[-1]
                wire=Command.from_buffer_copy(command_packet(*bridge.gate.output(
                    (result.accel,result.brake,result.steering),10.,10.)))
                self.assertGreater(wire.accel,0.)
                self.assertEqual(loc.progress_pub.messages[-1].data,0.)
                with self.assertRaises(ValueError): bridge.receive('status',competition_packet(speed=0.))
                brake=bridge.gate.output((1.,0.,0.),10.,10.)
                self.assertEqual(brake[:2],(0.,1.))


if __name__=='__main__': unittest.main()
