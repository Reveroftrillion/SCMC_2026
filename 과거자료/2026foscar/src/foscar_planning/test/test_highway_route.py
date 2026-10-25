from pathlib import Path
import numpy as np
import pytest
from foscar_planning.highway_route import HighwayRoute

MAPS = Path(__file__).resolve().parents[2] / 'foscar_bringup'

@pytest.fixture
def highway():
    return HighwayRoute.from_files(MAPS/'maps/link_set.json',
        MAPS/'routes/global_path.txt',
        MAPS/'routes/checkpoints.txt', start_checkpoint=10,
        hold_checkpoint=None, rejoin_checkpoint=12, end_checkpoint=13, excluded_lanes=[1])

def test_actual_highway_topology(highway):
    assert highway.start_link == 'A2256W000411'
    assert highway.left_link('A2256W000411') == 'A2256W000409'
    assert highway.left_link('A2256W000418') is None
    assert highway.next_link('A2256W000420') == 'A2256W000408'
    assert highway.next_link('A2256W000423') == 'A2256W000153'
    assert highway.lanes == (5,4,3,2)

def test_a_checkpoint_cannot_be_claimed_by_progress_only(highway):
    assert not highway.checkpoint_passed(12, (80.,-213.), (80.,-222.))
    assert highway.checkpoint_passed(12, (75.41,-213.), (75.41,-222.))
    assert not highway.checkpoint_passed(12,(75.41,-213.),(75.41,-222.),continuous=False)

def test_lane_centers_follow_map_and_rejoin_original(highway):
    p = highway.centerline(2, 1300.,1500.,step=.5)
    assert p.shape[1] == 3 and np.all(np.diff(p[:,0])>0)
    assert 73 < p[0,1] < 76
    assert 11 not in highway.checkpoints            # CP11 dropped from the rules (2026-09-30)
    assert highway.entry_end_s == pytest.approx(1285.8, abs=.1)   # lane 5 merges into lane 4 where CP11 was
    assert highway.rejoin_s == pytest.approx(1593.2, abs=.1)
    np.testing.assert_allclose(highway.at(2,highway.rejoin_s),[75.4142037,-217.855054],atol=.01)

def test_projection_uses_lane_geometry_and_refuses_outside(highway):
    xy=highway.at(2,1400.)
    lane, error=highway.match(*xy,1400.)
    assert lane==2 and error<.01
    with pytest.raises(ValueError): highway.at(1,1400.)
    with pytest.raises(ValueError): highway.centerline(2,800.,900.)

def test_rear_observation_follows_real_predecessor_links(highway):
    # CP10 is the ego ramp entry, not the start of the adjacent highway lane.
    xy=highway.observation_at(4,highway.start_s-30.)
    assert xy[1]>280. and 63.<xy[0]<68.
    assert 'A2256W000406' in highway.observation_chains[4]

def test_rejoin_and_highway_end_are_distinct_checkpoints(highway):
    assert highway.rejoin_s==pytest.approx(1593.200617,abs=.001)
    assert highway.end_s==pytest.approx(1741.335218,abs=.001)
    assert highway.rejoin_s<highway.end_s
    np.testing.assert_allclose(highway.original.at(highway.end_s),highway.checkpoints[13]['xy'])
