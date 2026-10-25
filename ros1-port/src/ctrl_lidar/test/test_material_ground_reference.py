"""Sloped material-ground reference must not turn braking road into obstacles."""
import numpy as np
from ctrl_lidar.ctrl_ground import estimate_ground_plane, estimate_material_ground_plane, relative_height


def asphalt(slope=.025):
    x, y = np.meshgrid(np.linspace(3.1, 27.9, 60), np.linspace(-4.9, 4.9, 25))
    return np.c_[x.ravel(), y.ravel(), (slope*x + .002*y + .43).ravel()]


def test_sloped_elevated_asphalt_keeps_low_object_height_and_input():
    floor = asphalt()
    objects = floor[600:650].copy()
    objects[:, 2] += .7
    points = np.vstack((floor, objects))
    before = points.copy()
    mask = np.arange(len(points)) < len(floor)
    # This is the recorded failure boundary: centering by median road height
    # makes the origin intercept fail even though the plane has broad support.
    centered = floor.copy()
    centered[:, 2] -= np.median(floor[:, 2])
    assert estimate_ground_plane(centered) is None
    plane = estimate_material_ground_plane(points, mask)
    assert plane is not None
    np.testing.assert_allclose(plane, [.025, .002, .43], atol=1e-12)
    heights = relative_height(points, plane)
    np.testing.assert_allclose(heights[:len(floor)], 0., atol=1e-12)
    np.testing.assert_allclose(heights[len(floor):], .7, atol=1e-12)
    np.testing.assert_array_equal(points, before)


def test_successful_median_estimate_remains_exactly_identical():
    floor = asphalt(.006)
    reference = np.median(floor[:, 2])
    centered = floor.copy()
    centered[:, 2] -= reference
    expected = estimate_ground_plane(centered)
    assert expected is not None
    expected[2] += reference
    np.testing.assert_array_equal(
        estimate_material_ground_plane(floor, np.ones(len(floor), dtype=bool)), expected)


def test_retry_cannot_replace_missing_narrow_or_sparse_material_support():
    floor = asphalt()
    for mask in (np.zeros(len(floor), dtype=bool), np.arange(len(floor)) < 20,
                 abs(floor[:, 1]) < .3):
        assert estimate_material_ground_plane(floor, mask) is None


def test_retry_still_rejects_excessive_slope_and_nonplanar_returns():
    steep = asphalt(.4)
    assert estimate_material_ground_plane(steep, np.ones(len(steep), dtype=bool)) is None
    noise = asphalt()
    noise[:, 2] = np.random.default_rng(7).uniform(-.7, 1.1, len(noise))
    assert estimate_material_ground_plane(noise, np.ones(len(noise), dtype=bool)) is None
