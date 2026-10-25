import numpy as np
from ctrl_lidar.ctrl_observed_footprint import observed_hull


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
    from ctrl_lidar.ctrl_observed_footprint import oriented_dimensions
    for angle in (0., .4, .785, 1.5):
        rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        hull=np.array([[0.,0.],[12.,0.],[12.,2.5],[0.,2.5]])@rotation.T
        assert np.allclose(oriented_dimensions(hull),(12.,2.5))


def test_large_vehicle_gate_keeps_truck_but_rejects_wall_and_other_mission():
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
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
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    from ctrl_lidar.ctrl_observed_footprint import observed_box
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


def test_recorded_asymmetric_cluster_event_keeps_all_observed_extents():
    """A dense near face must not shift the box away from its sparse far face."""
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode, Track
    # Recorded 2026-10-06 scan: 317 returns, centroid offset 1.488 m in x.
    det = dict(x=8.6420316696167, y=-3.5146915912628174,
               size_x=4.219895362854004, size_y=1.7219352722167969,
               size_z=1.1692055463790894, z_max=1.4111851453781128, n=317,
               mx=8.6420316696167, my=-3.5146915912628174,
               footprint=[[8.27499771118164, -4.584200382232666],
                          [12.239673614501953, -3.5959174633026123],
                          [8.577526092529297, -2.862265110015869],
                          [8.01977825164795, -4.019754886627197]])
    tr = Track(1, 8)
    tr.update(1., det, 8)
    node = SimpleNamespace(base_frame='base_link', track_namespace='test', confidence=lambda _: 1.)
    ev = ObstacleNode.make_event(node, tr, Time())
    points = np.asarray(det['footprint'])
    np.testing.assert_allclose([ev.x-ev.size_x/2., ev.y-ev.size_y/2.], points.min(axis=0), atol=1e-6)
    np.testing.assert_allclose([ev.x+ev.size_x/2., ev.y+ev.size_y/2.], points.max(axis=0), atol=1e-6)
    # Tracking continues to use the return centroid for association and motion.
    assert tr.history[-1][1:] == (det['mx'], det['my'])
    assert (tr.x, tr.y) == (det['x'], det['y'])


def test_frenet_gate_uses_same_observed_box_as_event():
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode, Track
    tr = Track(1, 8)
    tr.x, tr.y, tr.size_x, tr.size_y = 8.6, -3.5, 4.2, 1.7
    tr.footprint = [[8., -4.6], [12.2, -3.6], [8.6, -2.9]]
    node = SimpleNamespace(ego=(0., 0., 0.), project_to_route=lambda x, y, _: (x, y))
    assert np.allclose(ObstacleNode.frenet_box(node, tr, 0), [8., 12.2, -4.6, -2.9])


def test_held_outline_box_is_not_replaced_by_partial_current_scan():
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode, Track
    tr = Track(1, 8)
    tr.x, tr.y, tr.size_x, tr.size_y = 10., 1., 4., 2.
    tr.footprint = [[8., 0.], [8.2, .1]]
    tr.hold_fp = np.array([[8., 0.], [12., 2.]])
    node = SimpleNamespace(base_frame='base_link', track_namespace='test', confidence=lambda _: 1.)
    ev = ObstacleNode.make_event(node, tr, Time())
    assert (ev.x, ev.y, ev.size_x, ev.size_y) == (10., 1., 4., 2.)


def test_tunnel_road_gate_checks_observed_box_not_shifted_centroid_box():
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode, Track
    tr = Track(1, 8)
    tr.x, tr.y, tr.size_x, tr.size_y = 10., 1.8, 1., 2.
    tr.footprint = [[9.5, 0.], [10.5, 2.]]
    # Road upper edge y=2.2: all observed points fit, but centroid +/- 1 does not.
    node = SimpleNamespace(ego=(0., 0., 0.), road_point_margin=0.,
                           road=SimpleNamespace(outside=lambda xy: xy[:, 1]-2.2))
    assert ObstacleNode.box_on_road(node, tr)


def test_recorded_large_static_cluster_survives_size_gate_only_in_owned_mission():
    import json
    from pathlib import Path
    from time import perf_counter
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    scan = json.loads((Path(__file__).parent/'data/large_static_cluster_20261006.json').read_text())
    node = SimpleNamespace(ego_s=scan['s'], fragment_merge_begin=800.92, fragment_merge_end=910.41,
                           max_object_size=5., max_vehicle_length=14., max_vehicle_width=5.1,
                           mission_name='STATIC_OBSTACLE', mission_received_at=perf_counter())
    args = (scan['footprint'], scan['size_x'], scan['size_y'], scan['height'])
    assert ObstacleNode.accept_cluster_size(node, *args)
    node.mission_received_at -= 2.
    assert not ObstacleNode.accept_cluster_size(node, *args)
    node.mission_received_at = perf_counter()
    node.mission_name = 'CRUISE'
    assert not ObstacleNode.accept_cluster_size(node, *args)
    node.mission_name = 'STATIC_OBSTACLE'
    assert not ObstacleNode.accept_cluster_size(node, [[0.,0.],[8.,0.]],8.,0.,.3)
    assert not ObstacleNode.accept_cluster_size(node, [[0.,0.],[8.,0.],[8.,6.],[0.,6.]],8.,6.,2.)


def test_tunnel_observation_keeps_stationary_shoulder_inside_measured_walls():
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    node = SimpleNamespace(ego=(0., 0., 0.), road_margin=0., road_point_margin=2.5,
        road=SimpleNamespace(outside_one=lambda x,y: abs(y)-3.5),
        tunnel_owned=lambda: True,
        tunnel_walls=[[x,y,side] for x in (5.,15.) for y,side in ((-5.5,'R'),(5.5,'L'))])
    assert ObstacleNode.observation_on_road(node, 10., 4.8)
    assert not ObstacleNode.observation_on_road(node, 10., 5.8)
    node.tunnel_owned = lambda: False
    assert not ObstacleNode.observation_on_road(node, 10., 4.8)
