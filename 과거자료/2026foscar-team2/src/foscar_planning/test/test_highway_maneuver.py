import numpy as np
from foscar_planning.highway_maneuver import blend_centers, build_maneuver
from highway_fixtures import calibrated_config
from test_highway_route import highway

def test_blend_starts_and_finishes_on_lane_centers():
    u=np.linspace(0,1,101);source=np.c_[40*u,np.zeros_like(u)];target=source+[0,3.5]
    xy=blend_centers(source,target,u)
    np.testing.assert_allclose(xy[0],source[0]);np.testing.assert_allclose(xy[-1],target[-1])
    assert np.all(np.diff(xy[:,1])>=0)

def test_actual_map_change_uses_target_center_after_end(highway,tmp_path):
    cfg=calibrated_config(tmp_path)
    m=build_maneuver(highway,5,4,1130.,30.,5.,cfg)
    np.testing.assert_allclose(m.xy[-1],highway.at(4,m.global_s[-1]),atol=1e-6)
    assert m.end_global_s==1160.
    assert np.all(np.diff(m.global_s)>0)

def test_preview_covers_current_ego_and_rear_before_entry(highway,tmp_path):
    cfg=calibrated_config(tmp_path)
    m=build_maneuver(highway,5,4,1169.75,50.,15.,cfg)
    assert m.global_s[0]<=1150.-cfg.policy.reference_back_m+1e-6
    assert m.geometry_ok
