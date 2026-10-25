"""Actual node ticks with captured outputs; no simulator or control transmitter."""
import json
import math
from pathlib import Path
import pytest
import rclpy
from std_msgs.msg import String
from ctrl_interfaces.msg import Event
from ctrl_planning.ctrl_behavior_node import BehaviorNode
from ctrl_planning.ctrl_state import EventInfo, Target, event_key

CONFIG=Path(__file__).resolve().parents[1]/'config'


class Capture:
    def __init__(self): self.values=[]
    def publish(self,msg): self.values.append(msg)


@pytest.fixture
def node(request):
    rclpy.init(args=['--ros-args','--params-file',str(CONFIG/'behavior.yaml'),
        '-p','mission.config:='+str(CONFIG/'missions.yaml'),
        '-p','roundabout.enabled:=true','-p','roundabout.local_entry:=true',
        *getattr(request,'param',[])])
    n=BehaviorNode()
    n.time=100.
    n.now=lambda:n.time
    n.ego=lambda:(n.s_global,0.,0.,5.)
    n.target_pub=Capture();n.mission_pub=Capture()
    n.marker_pub=n.prediction_marker_pub=None
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def tick(n,s,dt=1.):
    n.s_global=float(s);n.time+=dt;n.tick()
    return n.target_pub.values[-1]


def test_unfinished_roundabout_gate_and_yield_cannot_block_highway(node):
    tick(node,890)
    node.entry_gate.committed=True
    node.entry_gate.entered_ring=True
    node.circular_gap.following_lead=object()
    y=next(s for s in node.behavior.states if s.name=='YIELD')
    y.target_s=890.;y.last_seen=node.time;y.target_type='PEDESTRIAN'
    node.behavior.engaged['YIELD']=True
    # Normal movement across exit (no respawn), then jump forward to highway.
    out=tick(node,920,dt=10)
    assert math.isnan(out.stop_s) and not node.entry_gate.committed
    assert node.circular_gap.following_lead is None
    assert 'YIELD' not in node.behavior.active_names()
    out=tick(node,1200,dt=100)
    assert math.isinf(out.v_max) and math.isnan(out.stop_s)
    status=json.loads(node.mission_pub.values[-1].data)
    assert status['mission']=='HIGHWAY' and status['speed_limit_kmh'] is None
    out=tick(node,1875,dt=200)
    assert out.v_max==pytest.approx(50/3.6)
    assert json.loads(node.mission_pub.values[-1].data)['mission']=='TUNNEL'


def test_respawn_removes_permission_and_rejects_old_packets(node):
    tick(node,890)
    node.entry_gate.committed=True
    node.entry_gate.finished=True
    node.prediction.tracks['stale']=object()
    node.circular_gap.footprints['stale']=object()
    tick(node,870)
    assert not node.entry_gate.finished and not node.entry_gate.committed
    assert not node.prediction.tracks and not node.circular_gap.footprints
    e=Event();e.value='lidar:old:1';e.type='VEHICLE';e.header.stamp.sec=100
    node.on_event(e)
    assert not node.events
    node.on_footprints(String(data=json.dumps(dict(frame='map',stamp=100,objects=[]))))
    assert node.circular_gap.scan_stamp==-math.inf


def test_false_pedestrian_and_static_fragment_cannot_own_roundabout(node):
    tick(node,880)
    for kind in ('PEDESTRIAN','OBSTACLE'):
        e=EventInfo(type=kind,stamp=node.time,s=884,d=0,size_x=.3,size_y=.3,value='lidar:noise:'+kind)
        node.events[event_key(e)]=e
    tick(node,880,dt=.05)
    assert not node.behavior.engaged['YIELD']
    assert not node.behavior.engaged['AVOID']
    assert not any(s.name=='YIELD' and s.target_type for s in node.behavior.states)


def test_no_vehicle_yield_where_no_npc_vehicles_appear(node):
    # NPC vehicles appear only in the roundabout and highway (2026-09-26); a VEHICLE label
    # elsewhere (e.g. a static barricade) must not stop the car through YIELD.
    for s in (1000., 450.):          # default cruise; pedestrian mission
        tick(node,s)
        e=EventInfo(type='VEHICLE',stamp=node.time,s=s+5,d=0,size_x=4.8,size_y=2,value='lidar:car:1')
        node.events[event_key(e)]=e
        out=tick(node,s,dt=.05)
        assert not node.behavior.engaged['YIELD'], s
        node.events.clear()


def test_nonfinite_pose_stops_without_stale_command(node):
    tick(node,1200)
    out=tick(node,math.nan)
    assert out.v_max==0 and out.state=='LOCALIZATION_INVALID'
    out=tick(node,1200)
    assert math.isinf(out.v_max)


def test_invalid_position_with_rviz_enabled_only_clears_markers(node):
    import numpy as np
    from ctrl_control.ctrl_core import Route
    node.route=Route(np.c_[np.arange(2001),np.zeros(2001)])
    node.s_offset=0.
    node.marker_pub=Capture()
    tick(node,1200)
    out=tick(node,math.nan)
    assert out.v_max==0
    assert len(node.marker_pub.values[-1].markers)==1


def test_pedestrian_to_cruise_drops_old_wait_even_when_yield_remains_allowed(node):
    tick(node,494.)
    y=next(s for s in node.behavior.states if s.name=='YIELD')
    y.target_s=500.;y.last_seen=node.time;y.target_type='PEDESTRIAN'
    node.behavior.engaged['YIELD']=True
    out=tick(node,495.,dt=.2)
    assert not node.behavior.engaged['YIELD'] and math.isnan(out.stop_s)
    assert json.loads(node.mission_pub.values[-1].data)['mission']=='CRUISE'


def test_departing_box_plan_cannot_request_offset_in_next_mission(node):
    tick(node,389.)
    a=next(s for s in node.behavior.states if s.name=='AVOID')
    a.compute_target=lambda ctx: Target(lat_offset=2.,stop_s=400.)
    a.check_exit=lambda ctx:False
    node.behavior.engaged['AVOID']=True
    out=tick(node,390.,dt=.2)
    assert out.lat_offset==0 and math.isnan(out.stop_s)
    assert not node.behavior.engaged['AVOID']


def test_clock_rewind_rejects_future_packets_from_previous_epoch(node):
    tick(node,890.)
    node.time=10.
    tick(node,870.,dt=0.)
    e=Event();e.value='lidar:epoch:1';e.type='VEHICLE';e.header.stamp.sec=100
    node.on_event(e)
    node.on_footprints(String(data=json.dumps(dict(frame='map',stamp=100.,objects=[]))))
    assert not node.events and node.circular_gap.scan_stamp==-math.inf
    e.header.stamp.sec=10
    node.on_event(e)
    assert next(iter(node.events.values())).stamp==10.


def test_empty_road_calibrates_route_offset_without_an_obstacle_event(node):
    import numpy as np
    from ctrl_control.ctrl_core import Route
    node.route=Route(np.c_[np.arange(-100.,2001.),np.zeros(2101)])
    tick(node,1200.)
    assert node.s_offset==pytest.approx(100.)
    # Teleport/reset must produce the newly calibrated offset in this same tick.
    tick(node,1190.)
    assert node.s_offset==pytest.approx(100.)


def test_mission_switch_clears_prediction_graphics_and_replaces_dashboard(node):
    import numpy as np
    from ctrl_control.ctrl_core import Route
    node.route=Route(np.c_[np.arange(2001),np.zeros(2001)])
    node.marker_pub=Capture();node.prediction_marker_pub=Capture()
    tick(node,890.)
    first=node.marker_pub.values[-1]
    assert any(m.ns=='mission_status' and 'ROUNDABOUT' in m.text for m in first.markers)
    assert all(m.lifetime.sec==1 for m in first.markers if m.action==m.ADD)
    tick(node,920.,dt=10.)
    assert node.prediction_marker_pub.values[-1].markers[0].action==3
    assert len(node.prediction_marker_pub.values[-1].markers)==1
    final=node.marker_pub.values[-1]
    assert final.markers[0].action==3
    assert any(m.ns=='mission_status' and 'CRUISE' in m.text for m in final.markers)
    assert not any(m.ns=='text' for m in final.markers)
    status=json.loads(node.mission_pub.values[-1].data)
    assert status['yield_status']['kind']=='NONE'


@pytest.mark.parametrize('node',[['-p','roundabout.box_planner:=false']],indirect=True)
def test_completed_entry_shows_exit_not_stale_front_following(node):
    # Exercise the legacy gate explicitly; the default box planner disables it.
    assert node.entry_gate.enabled and not node.box_planner_on
    import numpy as np
    from ctrl_control.ctrl_core import Route
    node.route=Route(np.c_[np.arange(2001),np.zeros(2001)])
    node.marker_pub=Capture()
    tick(node,905.64)               # old-path s915 (roundabout exit) on the 2026-10-01 path
    node.entry_gate.committed=node.entry_gate.entered_ring=node.entry_gate.finished=True
    node.prediction_result=dict(decision='EXITED',model='circular_gap',evaluated_at=node.time,evidence_stamp=node.time)
    node.prediction_at=node.time
    tick(node,905.64,dt=.05)
    labels='\n'.join(m.text for m in node.marker_pub.values[-1].markers)
    assert 'EXIT COMPLETE' in labels and 'FRONT NO MEASURED OBJECT' not in labels


def test_pedestrian_passage_unlocks_cap_same_tick_but_new_person_still_stops(node):
    import numpy as np
    from ctrl_control.ctrl_core import Route
    node.route=Route(np.c_[np.arange(2001),np.zeros(2001)])
    tick(node,450.)
    def observe(d,path=True):
        e=EventInfo(type='PEDESTRIAN',stamp=node.time,s=460.,d=d,x=460.,y=d,
                    size_x=.6,size_y=.6,box_yaw=0.,vy=2.,value='lidar:person:1',path_relevant=path)
        node.events[event_key(e)]=e
    observe(0.)
    assert tick(node,450.,dt=.05).v_max==0
    assert node.mission.speed_mps==pytest.approx(20/3.6)
    observe(2.5,False)
    tick(node,450.,dt=.1)
    node.time+=.4
    observe(3.,False)
    out=tick(node,450.,dt=.05)
    assert math.isnan(out.stop_s) and out.v_max==pytest.approx(60/3.6)
    assert node.mission.pedestrian_yield_completed
    observe(0.)
    assert tick(node,450.,dt=.05).v_max==0
    assert node.mission.pedestrian_yield_completed
    assert tick(node,430.).v_max==pytest.approx(20/3.6)
    assert not node.mission.pedestrian_yield_completed


def test_after_yield_cap_unlock_does_not_clear_another_states_stop(node):
    from ctrl_planning.ctrl_state import Behavior, BehaviorState, Target
    class Signal(BehaviorState):
        name='STOP_LINE';priority=90
        def check_entry(self,c):return True
        def check_exit(self,c):return False
        def compute_target(self,c):return Target(state=self.name,v_max=0.,stop_s=455.)
    tick(node,450.)
    node.behavior=Behavior(node.behavior.states+[Signal()])
    node.mission.pedestrian_yield_seen=True
    y=next(s for s in node.behavior.states if s.name=='YIELD')
    y.pedestrian_passage_confirmed=True
    out=tick(node,450.,dt=.05)
    assert node.mission.pedestrian_yield_completed
    assert node.mission.speed_mps==pytest.approx(60/3.6)
    assert out.v_max==0 and out.stop_s==455. and out.state=='STOP_LINE'


def test_highway_ignores_fresh_other_mission_events(node):
    tick(node,1200)
    for kind in ('PEDESTRIAN','VEHICLE','DYNAMIC_UNKNOWN','OBSTACLE'):
        e=EventInfo(type=kind,stamp=node.time,s=1210,d=0,x=1210,y=0,
                    size_x=3,size_y=2,value='lidar:highway:'+kind)
        node.events[event_key(e)]=e
    out=tick(node,1201,dt=.05)
    assert out.state=='CRUISE'
    assert math.isinf(out.v_max) and math.isnan(out.stop_s)
    assert out.lat_offset==0
    assert node.behavior.active_names()==['CRUISE']


def test_tunnel_mission_cap_is_the_same_in_gps_and_dead_reckoning(node):
    for mode in ('GPS','DR'):
        node.loc_mode=mode
        out=tick(node,1900,dt=.1)
        assert out.v_max==pytest.approx(50/3.6)
        assert math.isnan(out.stop_s)
        status=json.loads(node.mission_pub.values[-1].data)
        assert status['mission']=='TUNNEL' and status['speed_limit_kmh']==50.


def test_general_and_box_cruise_use_sixty_without_extra_highway_cap(node):
    for station in (100.,300.,500.,1000.):
        assert tick(node,station,dt=100.).v_max==pytest.approx(60/3.6)
    assert math.isinf(tick(node,1200.,dt=100.).v_max)


@pytest.mark.parametrize('kind,size', [('PEDESTRIAN',.6),('DYNAMIC_UNKNOWN',1.2)])
def test_tunnel_mission_yields_to_road_users_without_static_avoidance(node,kind,size):
    tick(node,1900,dt=.1)
    e=EventInfo(type=kind,stamp=node.time,s=1907,d=0,x=1907,y=0,
                size_x=size,size_y=.8,vy=1.,value='lidar:tunnel:mover')
    node.events[event_key(e)]=e
    out=tick(node,1900,dt=.05)
    assert node.mission.policy.pedestrian
    assert node.behavior.engaged['YIELD']
    assert not node.behavior.engaged['AVOID']
    assert math.isfinite(out.stop_s) and out.v_max==0.


@pytest.mark.parametrize('size',[.7,3.])
def test_tunnel_vehicle_label_is_static_even_for_partial_view(node,size):
    tick(node,1900,dt=.1)
    e=EventInfo(type='VEHICLE',stamp=node.time,s=1915,d=0,x=1915,y=0,
                size_x=size,size_y=.8,value='lidar:tunnel:parked')
    node.events[event_key(e)]=e
    tick(node,1900,dt=.05)
    assert node.behavior.engaged['AVOID']
    assert not node.behavior.engaged['YIELD']
