import numpy as np
from ctrl_lidar.ctrl_ground import estimate_ground_plane, estimate_material_ground_plane, material_height_region, relative_height


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


def test_material_reference_removes_elevated_road_but_preserves_obstacle_height():
    ground = road(.006)
    ground[:, 2] += .5
    objects = ground[500:650].copy()
    objects[:, 2] += .7
    points = np.r_[ground, objects]
    before = points.copy()
    assert estimate_ground_plane(points) is None
    plane = estimate_material_ground_plane(points, np.arange(len(points)) < len(ground))
    assert plane is not None
    heights = relative_height(points, plane)
    np.testing.assert_allclose(heights[:len(ground)], 0., atol=1e-12)
    np.testing.assert_allclose(heights[len(ground):], .7, atol=1e-12)
    np.testing.assert_array_equal(points, before)


def test_material_reference_requires_broad_supported_asphalt():
    points = road()
    for mask in [np.zeros(len(points), dtype=bool), np.arange(len(points)) < 20,
                 np.abs(points[:, 1]) < .3]:
        assert estimate_material_ground_plane(points, mask) is None
    for unsupported in [road(.4), road() + [0., 0., 2.], np.full((300, 3), np.nan)]:
        assert estimate_material_ground_plane(unsupported, np.ones(len(unsupported), dtype=bool)) is None


def test_material_continuation_requires_far_evidence_and_keeps_obstacle_height():
    near = road(.006)
    near[:, 2] += .5
    y = np.linspace(-4., 4., 61)
    far = np.c_[np.full(len(y), 33.), y, .006*33. - .012*y + .53]
    objects = np.array([[30., 0., .006*30. + .53 + .7],
                        [30., 1., .006*30. - .012 + .53],
                        [35., 0., .006*35. + .53]])
    points = np.vstack((near, far, objects))
    mask = np.arange(len(points)) < len(near) + len(far)
    plane = estimate_material_ground_plane(points, mask)
    supported = material_height_region(points, plane, mask, 40.)
    assert supported[-3:].tolist() == [True, True, False]
    height = relative_height(points, plane, supported)
    np.testing.assert_allclose(height[-3:-1], [.7, 0.], atol=1e-12)
    assert height[-1] == points[-1, 2]
    # Other missions/generic callers keep the exact existing 28 m boundary.
    np.testing.assert_array_equal(relative_height(points, plane)[-3:], points[-3:, 2])


def test_material_continuation_rejects_sparse_narrow_inconsistent_and_outside_roi_support():
    points = np.array([[29., -2., .53], [30., 0., .53], [31., 2., .53], [30., 1., .53]])
    plane = np.array([0., 0., .53])
    for mask in [np.array([True, False, False, False]), np.array([False, True, True, True])]:
        assert not material_height_region(points, plane, mask, 40.).any()
    bad = points.copy(); bad[:3, 2] += .5
    assert not material_height_region(bad, plane, np.ones(4, dtype=bool), 40.).any()
    assert not material_height_region(points, plane, np.ones(4, dtype=bool), 29.).any()
    assert not material_height_region(points, None, np.ones(4, dtype=bool), 40.).any()


def test_material_hull_follows_curved_road_but_does_not_fill_unobserved_corners():
    near = road(0.); near[:, 2] = .53
    y = np.linspace(-8., 2., 61)
    far = np.c_[np.full(len(y), 30.), y, np.full(len(y), .53)]
    objects = np.array([[29., -5.7, .53], [29., -5.7, 1.23],
                        [35., -5.7, .53], [29., 8., .53], [29., -5.7, .53]])
    points = np.vstack((near, far, objects))
    material = np.arange(len(points)) < len(near) + len(far)
    plane = estimate_material_ground_plane(points, material)
    supported = material_height_region(points, plane, material, 40., 20.)
    assert supported[-5:].tolist() == [True, True, False, False, True]
    np.testing.assert_allclose(relative_height(points, plane, supported)[-5:],
                               [0., .7, .53, .53, 0.], atol=1e-12)
    # The same observations cannot enlarge a caller's existing lateral ROI.
    limited = material_height_region(points, plane, material, 40., 5.)
    assert not limited[-5]


def test_observed_low_paint_supports_curve_edge_but_preserves_raised_objects():
    near = road(0.); near[:, 2] = .53
    y = np.linspace(-4., 4., 61)
    far = np.c_[np.full(len(y), 33.), y, np.full(len(y), .53)]
    # Lane paint lies outside the asphalt hull; a low edge is between those
    # same-scan observations. A raised paint-bright object must retain height.
    objects = np.array([[30., -8., .53], [29., -6.5, .53], [30., -8., 1.23],
                        [38., -14., 1.23], [45., -9., .53], [30., -25., .53]])
    points = np.vstack((near, far, objects))
    before = points.copy()
    asphalt = np.arange(len(points)) < len(near) + len(far)
    paint = np.zeros(len(points), dtype=bool)
    paint[-6] = paint[-3] = paint[-2] = paint[-1] = True
    plane = estimate_material_ground_plane(points, asphalt)
    original = material_height_region(points, plane, asphalt, 40., 20.)
    assert not original[-6:].any()
    supported = material_height_region(points, plane, asphalt, 40., 20., paint)
    assert supported[-6:].tolist() == [True, True, True, False, False, False]
    height = relative_height(points, plane, supported)
    np.testing.assert_allclose(height[-6:-3], [0., 0., .7], atol=1e-12)
    np.testing.assert_array_equal(height[-3:], points[-3:, 2])
    np.testing.assert_array_equal(points, before)
    np.testing.assert_array_equal(
        material_height_region(points, plane, asphalt, 40., 20., np.zeros(len(points), dtype=bool)),
        original)


def test_paint_does_not_replace_required_asphalt_support():
    x, y = np.meshgrid(np.linspace(29., 35., 30), np.linspace(-9., -2., 30))
    points = np.c_[x.ravel(), y.ravel(), np.full(x.size, .53)]
    paint = np.ones(len(points), dtype=bool)
    for asphalt in (np.zeros(len(points), dtype=bool), np.arange(len(points)) < 20):
        assert not material_height_region(points, np.array([0., 0., .53]), asphalt,
                                          40., 20., paint).any()
