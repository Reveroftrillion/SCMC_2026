from pathlib import Path
import json
from dataclasses import replace
from std_msgs.msg import String
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_runtime import HighwayRuntime
from test_highway_route import highway
from test_mission_node import node, tick, Capture

CONFIG=Path(__file__).resolve().parents[1]/'config/highway.yaml'

def test_actual_node_control_gate_and_mission_handoff(node,highway):
    cfg=replace(HighwayConfig.load(CONFIG),mode='control')
    node.highway=HighwayRuntime(cfg,highway)
    node.highway_pub=Capture();node.highway_marker_pub=None
    node.ego=lambda:(*highway.original.at(node.s_global),highway.original.heading_at(node.s_global),5.)
    out=tick(node,1150)
    # First tick of the mission: no scan yet, speed is held rather than stopped (2026-09-27).
    assert 0<out.v_max<=5. and out.state.startswith('HIGHWAY_')
    assert json.loads(node.highway_pub.values[-1].data)['reason']=='SCAN_UNKNOWN'
    out=tick(node,1151)
    assert out.v_max==0 and out.state.startswith('HIGHWAY_')   # still no scan: stop request
    out=tick(node,highway.end_s,dt=200)
    assert out.v_max==60/3.6
    assert json.loads(node.mission_pub.values[-1].data)['mission']=='CRUISE'
    assert not node.highway.active and not node.highway.observations.objects

def test_actual_node_shadow_cannot_change_cruise(node,highway):
    node.highway=HighwayRuntime(replace(HighwayConfig.load(CONFIG),mode='shadow'),highway)
    node.highway_pub=Capture();node.highway_marker_pub=None
    node.ego=lambda:(*highway.original.at(node.s_global),highway.original.heading_at(node.s_global),5.)
    out=tick(node,1150)
    assert out.state=='CRUISE' and out.v_max==float('inf')
    assert not out.reference_id

def test_cp12_keeps_curve_until_rejoin_end_and_highway_owns_until_cp13(node,highway,tmp_path):
    from highway_fixtures import calibrated_config
    node.highway=HighwayRuntime(calibrated_config(tmp_path),highway)
    node.highway_pub=Capture();node.highway_marker_pub=None
    node.ego=lambda:(*highway.original.at(node.s_global),highway.original.heading_at(node.s_global),5.)
    def scan_tick(s,dt):
        stamp=node.time+dt
        node.highway.accept(dict(stamp=stamp,frame='map',scan_valid=True,objects=[]),stamp)
        return tick(node,s,dt)
    scan_tick(highway.rejoin_s-1.,1.)
    out=scan_tick(highway.rejoin_s+.1,.3)
    assert not out.reference_release and out.reference_id
    out=scan_tick(node.highway.policy.rejoin.end_s+.1,5.)
    assert out.reference_release and not out.reference_id
    assert node.highway.active
    assert json.loads(node.mission_pub.values[-1].data)['mission']=='HIGHWAY'
    # The fixture route is the official path; the node's missions use the edited global_path.txt
    # (0.39 m shorter before CP13), so stay 0.5 m clear of the boundary.
    out=scan_tick(highway.end_s-.5,60.)
    assert out.reference_release and node.highway.active
    out=scan_tick(highway.end_s+.5,.1)
    assert not node.highway.active and out.v_max==60/3.6
    assert json.loads(node.mission_pub.values[-1].data)['mission']=='CRUISE'


def test_runtime_rejects_mismatched_mission_and_highway_geometry(highway):
    import pytest
    cfg=HighwayConfig.load(CONFIG)
    with pytest.raises(ValueError,match='boundary'):
        HighwayRuntime(cfg,highway,mission_bounds=(highway.start_s,1875.))


def test_launch_control_file_is_shared_by_highway_and_roundabout(tmp_path):
    import rclpy
    from foscar_planning.behavior_node import BehaviorNode
    control=tmp_path/'controller.yaml'
    control.write_text('path_follower:\n  ros__parameters:\n    a_lat_mps2: 1.2\n    decel_mps2: 0.9\n    accel_mps2: 0.6\n')
    rclpy.init(args=['--ros-args','-p','highway.config:='+str(CONFIG),
                    '-p','prediction.control_config:='+str(control)])
    n=None
    try:
        n=BehaviorNode()
        profile=n.highway.config.control_profile
        assert profile.source==str(control.resolve())
        assert (profile.a_lat_mps2,profile.decel_mps2,profile.accel_mps2)==(1.2,.9,.6)
        assert n.prediction.departure_profile_limits==(1.2,.9,.6)
        assert n.highway.config.bounds.latency_s>0
    finally:
        if n is not None:n.destroy_node()
        rclpy.shutdown()
