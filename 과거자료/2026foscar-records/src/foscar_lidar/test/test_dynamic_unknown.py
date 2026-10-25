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
    tr=partial(xs);tr.size_x=3.;tr.kind='VEHICLE'
    assert node.classify(tr)=='OBSTACLE'


PERSON=dict(top=1.75,long=.6,short=.42,spread=.4,mid=.85,imed=None)   # shape only, no intensity


def test_tunnel_moving_person_shape_is_a_pedestrian_and_keeps_it_when_stopped(node):
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    tr=partial([0.,.6,1.2]);tr.size_x=.5;tr.size_y=.4;tr.ped=PERSON
    tr.kind=node.classify(tr)
    assert tr.kind=='PEDESTRIAN'
    tr.vx=tr.vy=0.;tr.moving=False
    assert node.classify(tr)=='PEDESTRIAN'


@pytest.mark.parametrize('length,width',[(1.3,.7),(3.,1.4)])
def test_tunnel_moving_body_without_person_shape_is_no_pedestrian(node,length,width):
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    tr=partial([0.,.6,1.2]);tr.size_x=length;tr.size_y=width
    assert node.classify(tr)!='PEDESTRIAN'
    # a low barricade gliding in the scan stays an obstacle
    tr=partial([0.,.6,1.2]);tr.size_x=length;tr.size_y=width;tr.ped=dict(top=.81,long=2.6,short=1.1,spread=.2,mid=.9,imed=55.)
    assert node.classify(tr)!='PEDESTRIAN'


def test_tunnel_running_animal_is_a_pedestrian(node):
    # a deer: 1.9 m long, no person shape, but tall and running (2026-10-10)
    from std_msgs.msg import String
    node.on_mission(String(data='{"mission":"TUNNEL"}'))
    deer=dict(top=1.7,long=1.8,short=.5,spread=.4,mid=.7,imed=45.)
    tr=partial([0.,.4,.8,1.2,1.6]);tr.size_x=.5;tr.size_y=1.8;tr.ped=deer   # 3.6 m/s over 0.44 s
    assert node.classify(tr)=='PEDESTRIAN'
    # tall but only gliding briefly (a parked car or a standing deer as we come): no person
    tr=partial([0.,.6,1.2]);tr.size_x=.5;tr.size_y=1.8;tr.ped=deer
    assert node.classify(tr)!='PEDESTRIAN'
