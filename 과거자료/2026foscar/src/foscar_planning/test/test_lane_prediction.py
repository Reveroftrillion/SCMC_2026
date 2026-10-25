import math
import numpy as np
from foscar_planning.lane_prediction import LaneMap


def link(name, start, end, points):
    return dict(idx=name, from_node_idx=start, to_node_idx=end, points=points)


def test_association_uses_direction_and_rejects_offroad():
    lanes = LaneMap([link('east','a','b',[[0,0],[20,0]]),
                     link('west','c','d',[[20,.5],[0,.5]])])
    match = lanes.match(5,.4,0)
    assert match.link_id == 'east'
    assert abs(match.s-5)<1e-6
    assert lanes.match(5,10,0) is None
    assert lanes.match(5,0,math.pi/2) is None


def test_prediction_preserves_observed_offset_and_follows_curve():
    angles=np.linspace(0,math.pi/2,101)
    lanes=LaneMap([link('curve','a','b',np.c_[10*np.sin(angles),10*(1-np.cos(angles))])])
    match=lanes.match(0,0,0)
    paths=lanes.paths(match,10)
    x,y,h=paths[0].at(10)
    assert math.hypot(x-8.41471,y-4.59698)<.01
    assert abs(h-1)<.02
    straight=LaneMap([link('a','x','y',[[0,0],[20,0]])])
    path=straight.paths(straight.match(3,1,0),5)[0]
    assert np.allclose(path.at(5),(8,1,0))


def test_fork_keeps_both_connected_paths_but_not_near_unconnected_road():
    lanes=LaneMap([link('in','a','b',[[0,0],[10,0]]),
                   link('straight','b','c',[[10,0],[30,0]]),
                   link('turn','b','d',[[10,0],[10,20]]),
                   link('near','other','e',[[10,.1],[30,.1]])])
    paths=lanes.paths(lanes.match(5,0,0),15)
    ends=[tuple(np.round(p.at(15)[:2],5)) for p in paths]
    assert sorted(ends)==[(10.,10.),(20.,0.)]


def test_dead_end_does_not_extrapolate_unobserved_lane():
    lanes=LaneMap([link('end','a','b',[[0,0],[10,0]])])
    paths=lanes.paths(lanes.match(8,0,0),20)
    assert len(paths)==1
    assert paths[0].at(1)==(9.,0.,0.)
    assert paths[0].at(3) is None


def test_cycle_can_be_followed_for_more_than_one_link():
    lanes=LaneMap([link('a',0,1,[[0,0],[10,0]]),link('b',1,2,[[10,0],[10,10]]),
                   link('c',2,3,[[10,10],[0,10]]),link('d',3,0,[[0,10],[0,0]])])
    paths=lanes.paths(lanes.match(1,0,0),42)
    assert np.allclose(paths[0].at(42),(3,0,0))


def test_overlapping_lanes_keep_ambiguous_associations():
    lanes=LaneMap([link('exit','a','b',[[0,0],[10,0],[15,-5]]),
                   link('circle','a','c',[[0,.3],[10,.3],[15,5]])])
    matches=lanes.matches(5,.1,0)
    assert {m.link_id for m in matches}=={'exit','circle'}


def test_motion_rejects_duplicate_samples_and_smooths_single_speed_spike():
    from foscar_planning.lane_prediction import LaneMotion
    from foscar_planning.state import EventInfo
    lanes=LaneMap([link('a','x','y',[[0,0],[100,0]])]);motion=LaneMotion(lanes)
    for t,v in [(0.,4.),(.2,4.),(.4,4.),(.6,4.),(.8,12.)]:
        e=EventInfo(type='VEHICLE',stamp=t,x=10+t*4,y=0,vx=v,vy=0)
        motion.update(e,0.,0.)
    assert motion.speed==4.
    assert np.allclose(motion.poses(2.)[0],(21.2,0.,0.))
    e.vx=30.;motion.update(e,0.,0.)
    assert motion.speed==4.
    stopped=EventInfo(type='VEHICLE',stamp=1.,x=13.2,y=0,vx=0.,vy=0.)
    motion.update(stopped,0.,0.)
    assert motion.poses(2.)==[]
