import numpy as np
from foscar_lidar.ground import estimate_ground_plane, relative_height


def road(slope=.03):
    x, y = np.meshgrid(np.linspace(3.1, 27.9, 80), np.linspace(-4.9, 4.9, 25))
    z = slope*x - .012*y + .03
    return np.c_[x.ravel(), y.ravel(), z.ravel()]


def test_braking_tilt_is_removed_but_low_obstacles_keep_their_height():
    ground = road()
    objects = ground[500:650].copy()
    objects[:, 2] += .7
    points = np.r_[ground, objects]
    plane = estimate_ground_plane(points)
    assert plane is not None
    corrected = relative_height(points, plane)
    assert np.max(np.abs(corrected[:len(ground)])) < .01
    assert np.min(corrected[len(ground):]) > .69


def test_no_plane_is_claimed_for_sparse_occluded_or_narrow_returns():
    assert estimate_ground_plane(road()[:20]) is None
    assert estimate_ground_plane(np.array([[10., y, .3] for y in np.linspace(-4, 4, 200)])) is None
    assert estimate_ground_plane(road()[np.abs(road()[:, 1]) < .3]) is None
    assert estimate_ground_plane(np.full((300, 3), np.nan)) is None


def test_roof_and_excessive_slope_are_not_accepted_as_ground():
    roof = road(0.)
    roof[:, 2] += .8
    assert estimate_ground_plane(roof) is None
    assert estimate_ground_plane(road(.4)) is None


def test_fallback_and_points_outside_supported_region_are_unchanged():
    points = np.array([[10., 0., .5], [35., 0., .5], [10., 6., .5]])
    np.testing.assert_array_equal(relative_height(points, None), points[:, 2])
    corrected = relative_height(points, np.array([.03, 0., 0.]))
    np.testing.assert_allclose(corrected, [.2, .5, .5])
