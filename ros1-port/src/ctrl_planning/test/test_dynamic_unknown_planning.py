import numpy as np
from ctrl_control.ctrl_core import Route
from ctrl_planning.ctrl_state import Context, EventInfo, Behavior
from ctrl_planning.states.ctrl_yield import Yield
from ctrl_planning.states.ctrl_avoid import Avoid
from ctrl_planning.ctrl_prediction import GapPredictor
from test_tunnel_avoidance import context


def test_unknown_moving_object_yields_instead_of_requesting_static_avoidance():
    e=EventInfo(type='DYNAMIC_UNKNOWN',stamp=1.,s=10.,d=0.,x=10.,y=0.,size_x=1.3,size_y=1.4,value='lidar:test:1')
    ctx=Context(s=0,x=0,y=0,yaw=0,v=3.,now=1.,events=[e])
    behavior=Behavior([Yield(),Avoid()]);target=behavior.tick(ctx)
    assert target.state=='YIELD' and target.v_max==0.
    assert target.lat_offset==0. and not behavior.engaged['AVOID']
    e.type='OBSTACLE';ctx.now=1.1
    assert behavior.tick(ctx).lat_offset==0.


def test_unknown_road_user_is_included_in_collision_prediction():
    e=EventInfo(type='DYNAMIC_UNKNOWN',stamp=1.,s=5.,d=0.,x=5.,y=0.,size_x=1.3,size_y=1.4,value='lidar:test:1')
    route=Route(np.c_[np.arange(101),np.zeros(101)])
    ctx=Context(s=0,x=0,y=0,yaw=0,v=0.,now=1.,route=route)
    p=GapPredictor();p.observe([e],1.)
    assert p.evaluate(ctx)['decision']=='WAIT'


def test_static_obstacle_zone_goes_around_a_box_tracked_as_pedestrian():
    # 2026-09-28 full lap 6: approaching the s329 box slowly for a red light, the lidar tracked it
    # as PEDESTRIAN (0.2-0.4 m/s centroid drift); AVOID ignored it and the car drove into it.
    def ctx(now, kind):
        e=EventInfo(type=kind,stamp=now,s=18.,d=0.,x=18.,y=0.,size_x=1.0,size_y=0.8,value='lidar:test:7')
        c=context();c.s=c.x=0.;c.v=8.;c.now=now;c.events=[e];c.mission_name='STATIC_OBSTACLE'
        return c
    avoid=Avoid()
    for i in range(5):
        c=ctx(1.+i*.1,'PEDESTRIAN')
        assert avoid.check_entry(c)
    t=avoid.compute_target(c)
    assert avoid.plan is not None and not avoid.plan.blocked and avoid.plan.d != 0.
    # Outside the static-obstacle mission a pedestrian is still never swerved around.
    other=Avoid()
    e=EventInfo(type='PEDESTRIAN',stamp=1.,s=18.,d=0.,x=18.,y=0.,size_x=1.0,size_y=0.8,value='lidar:test:7')
    assert not other.check_entry(Context(s=0,x=0,y=0,yaw=0,v=8.,now=1.,events=[e],mission_name='PEDESTRIAN'))
