import numpy as np
import pytest

from ctrl_planning.ctrl_bev_motion import OccupancyMotion
from ctrl_planning.ctrl_highway_box_tracks import EdgeTracks


def hull(x=10., y=2., width=2., height=1.):
    return np.array([[x, y], [x+width, y], [x+width, y+height], [x, y+height]])


def test_first_observation_is_unready_not_stationary():
    motion = OccupancyMotion()
    assert motion.update(1., {'one': hull()}, (0., 0.))
    region, = motion.regions
    assert region.velocity is None and region.sigma is None and not region.ready
    ranges = region.displacement_range(2.)
    assert ranges[0][0] < 0. < ranges[0][1]
    assert region.polygon == tuple(map(tuple, hull()))


def test_stationary_noisy_outline_converges_to_zero_motion():
    motion = OccupancyMotion()
    rng = np.random.default_rng(4)
    for k in range(50):
        motion.update(1.+k*.1, {'one': hull()+rng.normal(0., .025, (4, 2))}, (0., 0.))
    region, = motion.regions
    assert region.ready
    assert np.linalg.norm(region.velocity) < .12
    assert all(0. < sd < EdgeTracks.READY_SIGMA for sd in region.sigma)


@pytest.mark.parametrize('velocity', [(0., -1.2), (-1., .7)])
def test_translation_and_id_churn_keep_planar_motion(velocity):
    motion = OccupancyMotion()
    for k in range(35):
        p = hull()+np.array(velocity)*k*.1
        motion.update(1.+k*.1, {f'new:{k}': p}, (0., 0.))
    region, = motion.regions
    assert region.ready
    assert np.allclose(region.velocity, velocity, atol=.12)
    assert np.allclose(np.asarray(region.at(region.stamp+1.)), p+velocity, atol=.12)
    assert len(motion.regions) == 1


def test_nearby_regions_do_not_share_motion_or_depend_on_input_order():
    motion = OccupancyMotion()
    for k in range(30):
        rows = [(f'a{k}', hull(y=2.-k*.025)), (f'b{k}', hull(y=4.+k*.025))]
        motion.update(1.+k*.1, dict(rows[::-1] if k % 2 else rows), (0., 0.))
    regions = sorted(motion.regions, key=lambda r: np.mean(r.polygon, axis=0)[1])
    assert len(regions) == 2
    assert regions[0].velocity[1] == pytest.approx(-.25, abs=.08)
    assert regions[1].velocity[1] == pytest.approx(.25, abs=.08)


def test_growing_outline_does_not_become_translation():
    motion = OccupancyMotion()
    for k in range(40):
        motion.update(1.+k*.1, {'piece': hull(height=1.+k*.1)}, (0., 0.))
    region, = motion.regions
    assert region.velocity is None or abs(region.velocity[1]) < .1


def test_missing_scan_coasts_at_motion_without_stationary_trail_then_expires():
    motion = OccupancyMotion()
    for k in range(30):
        motion.update(1.+k*.1, {'body': hull(y=2.+k*.1)}, (0., 0.))
    before, = motion.regions
    motion.update(before.stamp+.2, {}, (0., 0.))
    after, = motion.regions
    assert after.coasted == pytest.approx(.2)
    assert np.allclose(np.asarray(after.at(motion.stamp)), np.asarray(before.polygon)+[0., .2], atol=.03)
    motion.update(before.stamp+EdgeTracks.KEEP_S+.01, {}, (0., 0.))
    assert motion.regions == []


def test_old_duplicate_and_invalid_scans_do_not_change_motion():
    motion = OccupancyMotion()
    motion.update(2., {'a': hull()}, (0., 0.))
    before = motion.regions.copy()
    assert not motion.update(1., {'a': hull(x=100.)}, (0., 0.))
    assert not motion.update(2., {}, (0., 0.))
    assert not motion.update(float('nan'), {}, (0., 0.))
    assert motion.regions == before


def test_occluded_edge_changes_hold_motion_and_preserve_last_outline():
    motion = OccupancyMotion()
    for k in range(30):
        motion.update(1.+k*.1, {'body': hull(y=2.+k*.02)}, (0., 0.))
    before, = motion.regions
    cropped = hull(x=11., y=2.6, width=1., height=.5)
    motion.update(4., {'fragment': cropped, 'cover': hull(x=4., y=0., width=2., height=4.)}, (0., 0.))
    region = max(motion.regions, key=lambda r: r.center[0])
    assert region.ready
    assert region.velocity == pytest.approx(before.velocity, abs=.05)
    assert region.observed_polygon == tuple(map(tuple, cropped))
    assert np.ptp(np.asarray(region.polygon)[:, 0]) >= 2.-1e-6
    assert np.ptp(np.asarray(region.polygon)[:, 1]) >= 1.-1e-6


def test_hidden_edges_stop_claiming_ready_after_hold_limit():
    motion = OccupancyMotion()
    for k in range(30):
        motion.update(1.+k*.1, {'body': hull()}, (0., 0.))
    for k in range(12):
        motion.update(4.+k*.1, {'body': hull(), 'cover': hull(x=4., y=0., width=2., height=4.)}, (0., 0.))
    region = max(motion.regions, key=lambda r: r.center[0])
    assert not region.ready
    assert region.velocity is None


def test_growth_after_measured_stationarity_holds_measured_motion_with_uncertainty():
    motion = OccupancyMotion()
    for k in range(35):
        motion.update(1.+k*.1, {'body': hull()}, (0., 0.))
    before, = motion.regions
    assert before.ready
    for k in range(1, 8):
        motion.update(before.stamp+k*.1, {'changed': hull(height=1.+k*.4)}, (0., 0.))
        region, = motion.regions
        assert region.ready
        assert region.velocity == pytest.approx(before.velocity, abs=.1)
    assert region.sigma[1] > before.sigma[1]


def test_coherent_acceleration_updates_motion_instead_of_holding_old_speed():
    motion = OccupancyMotion()
    for k in range(35):
        motion.update(1.+k*.1, {'body': hull()}, (0., 0.))
    before, = motion.regions
    for k in range(1, 16):
        dt = k*.1
        motion.update(before.stamp+dt, {'changed': hull(y=2.+.5*dt*dt)}, (0., 0.))
    region, = motion.regions
    assert region.ready
    assert region.velocity[1] > .8


def test_shape_contradiction_hold_expires_from_last_coherent_measurement():
    motion = OccupancyMotion()
    for k in range(35):
        motion.update(1.+k*.1, {'body': hull()}, (0., 0.))
    before, = motion.regions
    for k in range(1, 21):
        motion.update(before.stamp+k*.1, {'changed': hull(height=1.+k*.4)}, (0., 0.))
    region, = motion.regions
    assert not region.ready


def test_newly_visible_opposite_edge_must_confirm_apparent_translation():
    from ctrl_planning.ctrl_bev_motion import _RegionTrack
    p=hull(height=8.)
    track=_RegionTrack(p,1.)
    for k in range(35):
        track.observe(1.+k*.1,p,[[True,True],[True,False]],np.array([0.,0.]))
    for k in range(1,5):
        cropped=hull(y=2.+k*.4,height=8.-k*.4)
        track.observe(4.4+k*.1,cropped,[[True,True],[True,True]],np.array([0.,0.]))
        assert not track.region.ready or abs(track.region.velocity[1])<.1
