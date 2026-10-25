import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from foscar_lidar.obstacle_node import ObstacleNode, Track


@pytest.fixture
def node():
    rclpy.init()
    n=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('use_road_mask',value=False)])
    yield n
    n.destroy_node();rclpy.shutdown()


def partial(xs):
    tr=Track(1,5);tr.motion_in_map=True
    for i,x in enumerate(xs):
        tr.update(i*.11,dict(mx=x,my=0.,x=20.,y=0.,size_x=1.3,size_y=1.4,
                           size_z=1.,z_max=1.4,n=20),5)
    return tr


def across(ys):
    """Like partial, moving in map y (across a route laid along x)."""
    tr=Track(1,5);tr.motion_in_map=True
    for i,y in enumerate(ys):
        tr.update(i*.11,dict(mx=0.,my=y,x=20.,y=0.,size_x=1.3,size_y=1.4,size_z=1.,z_max=1.4,n=20),5)
    return tr


def along_x(node):
    node.route_xy=np.c_[np.arange(-50.,51.),np.zeros(101)]


def test_partial_view_with_consistent_map_motion_is_dynamic_unknown(node):
    assert node.classify(partial([0.,.6,1.2]))=='DYNAMIC_UNKNOWN'


def test_single_box_centroid_jump_is_not_dynamic_evidence(node):
    assert node.classify(partial([0.,0.,1.2,1.2,1.2]))=='OBSTACLE'


def test_relative_motion_without_odometry_cannot_confirm_unknown_motion(node):
    tr=partial([0.,.6,1.2]);tr.motion_in_map=False
    assert node.classify(tr)=='OBSTACLE'


def test_structure_filter_still_precedes_motion(node):
    tr=partial([0.,.6,1.2]);tr.structure=True
    assert node.classify(tr)=='OBSTACLE'


def test_diagonal_long_vehicle_uses_oriented_width_and_preserves_structure_gate(node):
    import numpy as np
    node.ego_s=886.
    tr=partial([0.,.6,1.2]);tr.size_z=2.
    a=.785;rotation=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    points=np.array([[0.,0.],[12.,0.],[12.,2.5],[0.,2.5]])@rotation.T
    tr.footprint=points.tolist();tr.size_x,tr.size_y=np.ptp(points,axis=0)
    assert node.classify(tr)=='VEHICLE'
    tr.structure=True
    assert node.classify(tr)=='OBSTACLE'


def test_unknown_does_not_turn_static_when_it_stops_and_can_upgrade(node):
    tr=partial([0.,.6,1.2]);tr.kind=node.classify(tr)
    tr.vx=tr.vy=0.;tr.moving=False
    assert node.classify(tr)=='DYNAMIC_UNKNOWN'
    tr.vx=5.;tr.straightness=1.;tr.size_x=3.
    assert node.classify(tr)=='VEHICLE'


def test_recorded_partial_circulating_vehicle_is_not_static(node):
    import json
    from pathlib import Path
    e=json.loads((Path(__file__).parent/'data/partial_vehicle_motion.json').read_text())['sample']
    tr=Track(12,5);tr.motion_in_map=True
    for t,x,y in e['hist']:
        tr.update(t,dict(mx=x,my=y,x=e['x'],y=e['y'],size_x=e['sx'],size_y=e['sy'],
                         size_z=e['sz'],z_max=1.4,n=20),5)
    assert node.classify(tr)=='DYNAMIC_UNKNOWN'


def test_partial_view_centroid_shift_keeps_overlapping_moving_vehicle_identity(node):
    tr=partial([0.,.6,1.2]);tr.kind='VEHICLE'
    tr.footprint_map=[[-1.,-.9],[3.,-.9],[3.,.9],[-1.,.9]]
    node.tracks=[tr];node.ego_s=880.
    det=dict(mx=4.,my=0.,x=4.,y=0.,size_x=4.5,size_y=1.8,size_z=1.,z_max=1.4,n=40,
             footprint_map=[[0.,-.9],[4.5,-.9],[4.5,.9],[0.,.9]])
    assert node.match([det],.33)=={0}
    assert tr.history[-1][1]==4.


@pytest.mark.parametrize('kind',['PEDESTRIAN','OBSTACLE'])
def test_tunnel_publication_preserves_motion_classification(node,kind):
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    node.confirm_frames=1
    node.publish_markers=False
    node.classify=lambda tr:kind
    node.output_gates=lambda tr,idx,shift:(True,False)
    published=[]
    class Capture:
        def publish(self,msg):published.append(msg)
    node.event_pub=Capture()
    node.track_and_publish([dict(x=10.,y=0.,size_x=1.3,size_y=1.4,size_z=1.,z_max=1.4,n=20)],
                           node.get_clock().now().to_msg(),1.)
    assert published and published[-1].type==kind


@pytest.mark.parametrize('xs', [[0.,0.,1.2,1.2,1.2],[0.,.6,1.2]])
def test_tunnel_large_body_is_no_person_by_motion(node,xs):
    # a 3 m barricade gliding in the scan read as a person and left the BEV avoidance (2026-10-10)
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    along_x(node)   # gliding along the lane as we come
    tr=partial(xs);tr.size_x=3.;tr.kind='VEHICLE'
    assert node.classify(tr)=='OBSTACLE'


PERSON=dict(top=1.75,long=.6,short=.42,spread=.4,mid=.85,imed=None)   # shape only, no intensity


def test_tunnel_moving_person_shape_is_a_pedestrian_and_keeps_it_when_stopped(node):
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    tr=partial([0.,.3,.6,.9,1.2]);tr.size_x=.5;tr.size_y=.4;tr.ped=PERSON
    # confirmed by pedestrian_vote (walking over 0.4 s) on its second person-shaped scan, not by one scan's shape and
    # motion (that made a parked car's piece a pedestrian at 34 m, 2026-10-10)
    tr.kind=node.classify(tr)
    tr.kind=node.classify(tr)
    assert tr.kind=='PEDESTRIAN'
    tr.vx=tr.vy=0.;tr.moving=False
    assert node.classify(tr)=='PEDESTRIAN'


@pytest.mark.parametrize('length,width',[(1.3,.7),(3.,1.4)])
def test_tunnel_moving_body_without_person_shape_is_no_pedestrian(node,length,width):
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    along_x(node)   # the glide is along the lane: as we come, not into the road
    tr=partial([0.,.6,1.2]);tr.size_x=length;tr.size_y=width
    assert node.classify(tr)!='PEDESTRIAN'
    # a low barricade gliding in the scan stays an obstacle
    tr=partial([0.,.6,1.2]);tr.size_x=length;tr.size_y=width;tr.ped=dict(top=.81,long=2.6,short=1.1,spread=.2,mid=.9,imed=55.)
    assert node.classify(tr)!='PEDESTRIAN'


def test_tunnel_running_animal_is_a_pedestrian(node):
    # a deer: 1.9 m long, no person shape, running across the road (2026-10-10)
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    along_x(node)
    deer=dict(top=1.7,long=1.8,short=.5,spread=.4,mid=.7,imed=45.)
    tr=across([0.,.4,.8,1.2,1.6]);tr.size_x=.5;tr.size_y=1.8;tr.ped=deer   # 3.6 m/s across over 0.44 s
    assert node.classify(tr)=='PEDESTRIAN'
    # tall but only gliding along the lane (a parked car or a standing deer as we come): no person
    tr=partial([0.,.08,.16,.24,.32]);tr.size_x=.5;tr.size_y=1.8;tr.ped=deer
    assert node.classify(tr)!='PEDESTRIAN'


def test_tunnel_anything_moving_across_the_road_is_a_pedestrian(node):
    # whatever its shape (user 2026-10-10: in the final a person or animal may come from anywhere)
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    along_x(node)
    tr=across([0.,.3,.6,.9,1.2]);tr.size_x=1.3;tr.size_y=.7
    assert node.classify(tr)=='PEDESTRIAN'


def outlined(ys, widths):
    """Like across, with a map outline per scan: centre y, width across the route (map y)."""
    tr=Track(1,5);tr.motion_in_map=True
    for i,(y,w) in enumerate(zip(ys,widths)):
        box=[[-.3,y-w/2],[.3,y-w/2],[.3,y+w/2],[-.3,y+w/2]]
        tr.update(i*.11,dict(mx=0.,my=y,x=20.,y=0.,size_x=1.3,size_y=1.4,size_z=1.,z_max=1.4,n=20,footprint_map=box),5)
    return tr


def test_tunnel_points_joining_a_piece_are_no_motion(node):
    # a parked car's piece at 35 m: its points joined and left, 0.4 / 1.3 / 0.2 m wide, its centre ran across at 1.7 m/s
    # and it read as a person (2026-10-10). Waiting until it holds its shape (user 2026-10-10).
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    along_x(node)
    tr=outlined([0.,.3,.6,.9,1.2],[.4,1.3,.2,1.,.3]);tr.size_x=1.3;tr.size_y=.7
    assert node.classify(tr)!='PEDESTRIAN'
    # the same motion holding its shape is a pedestrian
    tr=outlined([0.,.3,.6,.9,1.2],[.5]*5);tr.size_x=1.3;tr.size_y=.7
    assert node.classify(tr)=='PEDESTRIAN'


@pytest.mark.parametrize('tops,imeds,marked', [
    ((1.63, 1.05), (18., 61.), True),     # a person: head ring at 28 m, waist ring at 20 m -- the most seen of it counts
    ((0.8, 1.6), (44., 45.), True),       # a deer met by the upper ring once
    ((1.4,), (42.,), False),              # one scan is not enough
    ((1.4, 1.5), (229., 229.), False),    # the parked car's pieces (1.2-1.9 m, returns 229)
    ((1.3, 1.3), (154., 30.), False),     # ... or 154 once
    ((0.94, 1.02), (60., 80.), False),    # cones and barricades stay under CAUTION_TOP
])
def test_tunnel_caution_is_the_most_seen_height_of_a_body_without_car_returns(node, tops, imeds, marked):
    # one ring meets a body at a time from 20 to 40 m: a person read 1.63 m at 28 m and 1.0-1.25 m at 17-23 m and was
    # marked only at 13-16 m in 3 of 10 runs (2026-10-11)
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    tr=partial([0.,0.,0.]);tr.size_x=.5;tr.size_y=.3;tr.kind='OBSTACLE';tr.structure=False
    for top,imed in zip(tops,imeds):
        tr.ped=dict(top=top,long=.3,short=.2,spread=.3,mid=.8,imed=imed)
        node.pedestrian_vote(tr)
        tr.kind='OBSTACLE'
    assert node.caution(tr) is marked


def test_tunnel_cone_with_recorded_height_overshoot_is_not_marked_as_an_animal(node):
    # CP13 run 20261012_021644/7: cone 33 briefly read 1.108 m; its caution capped us at 20 km/h.
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    tr=partial([0.,0.,0.]);tr.size_x=tr.size_y=.8;tr.kind='OBSTACLE'
    for top in (.61,1.108,.89):
        tr.ped=dict(top=top,long=.8,short=.6,spread=.2,mid=.9,imed=60.)
        node.pedestrian_vote(tr)
    assert not node.caution(tr)


@pytest.mark.parametrize('tunnel',[False,True])
def test_tunnel_caution_tuning_does_not_forget_a_confirmed_person(node,tunnel):
    from std_msgs.msg import String
    if tunnel:node.on_mission(String(data='{"mission":"TUNNEL"}'))
    # A low body slice (1.12 m) is not a new reason to forget a confirmed person.
    tr=partial([0.,0.,0.]);tr.x=10.;tr.size_x=.5;tr.size_y=.3
    tr.ped_confirmed=True;tr.kind='PEDESTRIAN'
    for _ in range(3):
        tr.ped=dict(top=1.12,long=.5,short=.3,spread=.2,mid=.9,imed=18.)
        tr.kind=node.classify(tr)
    assert tr.kind=='PEDESTRIAN'


def test_tunnel_row_of_cones_is_no_structure(node):
    # four cones 0.2 m apart across the lane read as a guardrail and left the footprints; the car hit one (2026-10-11)
    from std_msgs.msg import String
    row=lambda:[dict(x=10.,y=y,size_x=.8,size_y=.8) for y in (-2.,-1.,0.,1.)]
    dets=row();node.mark_chains(dets)
    assert all(d['structure'] for d in dets)          # elsewhere a tight row is a rail
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    dets=row();node.mark_chains(dets)
    assert not any(d['structure'] for d in dets)
