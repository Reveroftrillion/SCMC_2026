import numpy as np
from foscar_lidar.observed_footprint import observed_hull


def test_hull_keeps_visible_l_shape_bounds_without_invented_aabb_corners():
    pts=np.array([[x,0.] for x in np.linspace(0,4,20)]+[[0.,y] for y in np.linspace(0,2,12)])
    hull=observed_hull(pts)
    assert len(hull)==3
    assert not any(np.allclose(p,[4.,2.]) for p in hull)
    assert max(np.array(hull)[:,0])==4.


def test_line_and_invalid_points_have_finite_bounded_output():
    hull=observed_hull(np.array([[0.,0.],[1.,0.],[2.,0.],[float('nan'),0.]]))
    assert np.array_equal(hull,[[0.,0.],[2.,0.]])


def test_oriented_dimensions_do_not_turn_long_diagonal_vehicle_into_wide_wall():
    from foscar_lidar.observed_footprint import oriented_dimensions
    for angle in (0., .4, .785, 1.5):
        rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        hull=np.array([[0.,0.],[12.,0.],[12.,2.5],[0.,2.5]])@rotation.T
        assert np.allclose(oriented_dimensions(hull),(12.,2.5))


def test_large_vehicle_gate_keeps_truck_but_rejects_wall_and_other_mission():
    from types import SimpleNamespace
    from foscar_lidar.obstacle_node import ObstacleNode
    node=SimpleNamespace(ego_s=886.,fragment_merge_begin=860.,fragment_merge_end=920.,
                         max_object_size=5.,max_vehicle_length=14.,max_road_user_width=2.6,max_vehicle_width=3.2)
    truck=[[0.,0.],[12.,0.],[12.,2.5],[0.,2.5]]
    assert ObstacleNode.accept_cluster_size(node,truck,12.,2.5,2.)
    assert not ObstacleNode.accept_cluster_size(node,[[0.,0.],[12.,0.]],12.,0.,2.)
    assert not ObstacleNode.accept_cluster_size(node,[[0.,0.],[12.,0.],[12.,4.],[0.,4.]],12.,4.,2.)
    node.ego_s=700.
    assert not ObstacleNode.accept_cluster_size(node,truck,12.,2.5,2.)


def test_long_tall_side_is_retained_only_inside_road_and_box_follows_outline():
    from types import SimpleNamespace
    from foscar_lidar.obstacle_node import ObstacleNode
    from foscar_lidar.observed_footprint import observed_box
    n=SimpleNamespace(ego_s=886.,fragment_merge_begin=860.,fragment_merge_end=920.,
        max_object_size=5.,max_vehicle_length=14.,max_vehicle_width=3.2,
        road=SimpleNamespace(outside_one=lambda x,y:-1.),to_map=lambda pts,t:pts)
    side=[[0.,0.],[10.,0.]]
    assert ObstacleNode.accept_cluster_size(n,side,10.,0.,1.5,1.)
    n.road.outside_one=lambda x,y:0.
    assert not ObstacleNode.accept_cluster_size(n,side,10.,0.,1.5,1.)
    a=.7;axes=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    hull=np.array([[0.,0.],[10.,0.],[10.,3.],[0.,3.]])@axes.T
    x,y,h,l,w=observed_box(hull)
    assert np.allclose(sorted([l,w]),[3.,10.])
    assert abs(np.sin(2*(h-a)))<1e-6
