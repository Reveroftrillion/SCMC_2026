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
