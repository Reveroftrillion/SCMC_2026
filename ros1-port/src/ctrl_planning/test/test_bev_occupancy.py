import numpy as np
import pytest

from ctrl_planning.ctrl_bev_occupancy import BevOccupancy, FREE, OCCUPIED, UNKNOWN


def test_no_scan_is_unknown_and_complete_empty_scan_is_visible():
    grid = BevOccupancy(resolution=.25, sight_m=10.)
    points = [[2., 0.], [12., 0.]]
    assert grid.query(points).tolist() == [UNKNOWN, UNKNOWN]
    assert not grid.update(1., None, (0., 0.))
    assert grid.query(points).tolist() == [UNKNOWN, UNKNOWN]
    assert grid.update(1., {}, (0., 0.))
    assert grid.query(points).tolist() == [FREE, UNKNOWN]


def test_diagonal_hull_does_not_fill_aabb_empty_corner():
    grid = BevOccupancy(resolution=.25)
    grid.update(1., {'shape': [(4., 0.), (5., -1.), (9., 3.), (8., 4.)]}, (0., 0.))
    assert grid.query([[4.5, 3.5], [6., 1.]]).tolist() == [FREE, OCCUPIED]
    assert not grid.hits_footprint([[4.2, 3.2], [4.7, 3.2], [4.7, 3.7], [4.2, 3.7]])
    assert grid.hits_footprint([[5.8, .8], [6.2, .8], [6.2, 1.2], [5.8, 1.2]])


def test_thin_polygon_crossing_cell_counts_without_inside_corners():
    grid = BevOccupancy(resolution=1.)
    hull = [(1.1, .45), (3.9, .45), (3.9, .55), (1.1, .55)]
    grid.update(1., {'thin': hull}, (0., 0.))
    assert grid.query([[2.5, .1]]).tolist() == [OCCUPIED]
    assert (2, 0) in {tuple(p) for p in grid.intersected_cells(hull)}


def test_shadow_and_range_are_unknown_and_occupied_dominates():
    grid = BevOccupancy(resolution=.25, sight_m=10.)
    grid.update(1., {'near': [(4., -.5), (5., -.5), (5., .5), (4., .5)],
                     'far': [(7., -.25), (8., -.25), (8., .25), (7., .25)]}, (0., 0.))
    assert grid.query([[2., 0.], [6., 0.], [7.5, 0.], [3., 3.], [11., 0.]]).tolist() == [FREE, UNKNOWN, OCCUPIED, FREE, UNKNOWN]
    body = [(5.8, -.1), (6.2, -.1), (6.2, .1), (5.8, .1)]
    assert grid.hits_footprint(body)
    assert not grid.hits_footprint(body, include_unknown=False)


def test_new_scan_replaces_occupancy_and_ids_do_not_change_geometry():
    grid = BevOccupancy()
    a = [(2., 2.), (3., 2.), (3., 3.), (2., 3.)]
    grid.update(1., {'vehicle': a}, (0., 0.))
    first = grid.occupied_indices.copy()
    assert not grid.update(.5, {}, (0., 0.))
    assert not grid.update(2., None, (0., 0.))
    assert grid.stamp == 1.
    grid.update(2., {'pedestrian': a}, (0., 0.))
    np.testing.assert_array_equal(grid.occupied_indices, first)
    grid.update(3., {}, (0., 0.))
    assert grid.query([[2.5, 2.5]]).tolist() == [FREE]
    assert grid.occupied_cells.shape == (0, 2)


def test_negative_map_cells_and_per_region_geometry_are_stable():
    grid = BevOccupancy(resolution=.5)
    grid.update(1., {'one': [(-1.9, -1.9), (-1.6, -1.9), (-1.6, -1.6), (-1.9, -1.6)]}, (0., 0.))
    assert grid.region_cells['one'].tolist() == [[-4, -4]]
    assert grid.occupied_cells.tolist() == [[-1.75, -1.75]]
    assert grid.cell_polygons(grid.occupied_indices).tolist() == [[[-2., -2.], [-1.5, -2.], [-1.5, -1.5], [-2., -1.5]]]


def test_shadow_wraps_at_negative_x_axis():
    grid = BevOccupancy(resolution=.25)
    grid.update(1., {'one': [(-5., -.5), (-4., -.5), (-4., .5), (-5., .5)]}, (0., 0.))
    assert grid.query([[-6., -.1], [-6., .1], [6., .1]]).tolist() == [UNKNOWN, UNKNOWN, FREE]


def test_invalid_scan_cannot_clear_last_observed_obstacle():
    grid = BevOccupancy()
    grid.update(1., {'one': [(2., 2.), (3., 2.), (3., 3.), (2., 3.)]}, (0., 0.))
    with pytest.raises(ValueError):
        grid.update(2., {'invalid': [[float('nan'), 0.], [1., 1.], [2., 2.]]}, (0., 0.))
    assert grid.stamp == 1.
    assert grid.query([[2.5, 2.5]]).tolist() == [OCCUPIED]


def test_sensor_inside_observed_hull_has_no_claimed_free_direction():
    grid = BevOccupancy()
    grid.update(1., {'one': [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]}, (0., 0.))
    assert grid.query([[5., 0.], [-5., 0.], [0., 5.], [0., -5.]]).tolist() == [UNKNOWN]*4


def test_batched_body_sat_checks_rotated_corner_and_clear_neighbor():
    grid = BevOccupancy(resolution=.05)
    grid.update(1., {'corner': [(1.551, .231), (1.599, .231), (1.599, .279), (1.551, .279)]}, (0., 0.))
    assert not grid.hits_bodies([[.5, 0.]], [0.], front=1., rear=.2, half=.5)
    assert grid.hits_bodies([[.5, 4./30.]], [np.arctan(.5)], front=1., rear=.2, half=.5)
    assert grid.hits_bodies([[-10., 0.], [.5, 4./30.]], [0., np.arctan(.5)], front=1., rear=.2, half=.5)


def test_batched_body_sat_ignores_unknown_and_preserves_empty_diagonal_corner():
    grid = BevOccupancy(resolution=.25, sight_m=10.)
    assert not grid.hits_bodies([[20., 0.]], [0.], front=1., rear=.2, half=.5)
    grid.update(1., {'diagonal': [(4., 0.), (5., -1.), (9., 3.), (8., 4.)]}, (0., 0.))
    assert not grid.hits_bodies([[4.3, 3.5]], [0.], front=.2, rear=.2, half=.2)
    assert grid.hits_bodies([[6., 1.]], [0.], front=.2, rear=.2, half=.2)


def test_batched_body_sat_matches_raster_footprints_for_crossing_shapes():
    grid = BevOccupancy(resolution=.25)
    grid.update(1., {'one': [(2.1, 2.1), (3.9, 2.1), (3.9, 3.9), (2.1, 3.9)]}, (0., 0.))
    positions = np.array([[-2., 0.], [0., 3.], [5., 5.], [4., 4.]])
    yaws = np.array([0., 0., np.pi/2., -.7])
    for position, yaw in zip(positions, yaws):
        corners = np.array([[-.3, -.4], [1., -.4], [1., .4], [-.3, .4]])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        expected = grid.hits_footprint(corners@rotation.T+position, include_unknown=False)
        assert grid.hits_bodies([position], [yaw], front=1., rear=.3, half=.4) == expected


def test_slanted_near_edge_does_not_shadow_visible_space_before_far_edge():
    grid = BevOccupancy(resolution=.25)
    grid.update(1., {'slanted': [(4., -2.), (5., -2.), (9., 2.), (8., 2.)]}, (0., 0.))
    assert grid.query([[5.5, 1.], [11., 1.]]).tolist() == [FREE, UNKNOWN]
