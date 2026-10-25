from pathlib import Path
import pytest
from foscar_planning.highway_config import HighwayConfig, PlanningBounds

CONFIG=Path(__file__).resolve().parents[1]/'config/highway.yaml'

def test_default_is_control_with_explicit_planning_assumptions():
    cfg=HighwayConfig.load(CONFIG)
    assert cfg.mode=='control'
    assert cfg.bounds.min_brake_mps2>0
    assert not hasattr(cfg.bounds,'verified')


def test_invalid_bound_is_rejected():
    for v in (None,0,-1,float('nan'),True):
        with pytest.raises(ValueError):PlanningBounds.from_dict({'latency_s':v})


def test_evidence_file_is_not_a_control_gate():
    with pytest.raises(ValueError):PlanningBounds.from_dict({'verified':True})
    assert PlanningBounds.from_dict({}).latency_s>0


def test_rejects_unknown_policy_and_impossible_settings():
    import yaml
    data=yaml.safe_load(CONFIG.read_text())
    data['policy']['history_s']=-1.
    with pytest.raises(ValueError):HighwayConfig.from_dict(data)
    data=yaml.safe_load(CONFIG.read_text());data['policy']['typo']=1
    with pytest.raises(ValueError):HighwayConfig.from_dict(data)


def test_controller_profile_is_loaded_without_certifying_physical_bounds(tmp_path):
    control=tmp_path/'control.yaml'
    control.write_text('/**:\n  ros__parameters:\n    a_lat_mps2: 1.2\n    decel_mps2: 1.1\n    accel_mps2: 0.8\n/path_follower:\n  ros__parameters:\n    a_lat_mps2: 0.9\n')
    cfg=HighwayConfig.load(CONFIG,control_config=control)
    assert cfg.control_profile.a_lat_mps2==.9
    assert cfg.control_profile.decel_mps2==1.1
    assert cfg.control_profile.accel_mps2==.8
    assert cfg.control_profile.source==str(control.resolve())
    assert cfg.stopping_decel_limit==1.1
    assert cfg.lateral_accel_limit==.9



@pytest.mark.parametrize('value', [None, 0, -1, True, float('nan')])
def test_invalid_controller_profile_fails_instead_of_using_guessed_defaults(tmp_path,value):
    import yaml
    control=tmp_path/'control.yaml'
    control.write_text(yaml.safe_dump({'path_follower':{'ros__parameters':dict(a_lat_mps2=value,decel_mps2=1.5,accel_mps2=1.)}}))
    with pytest.raises(ValueError,match='control profile'):
        HighwayConfig.load(CONFIG,control_config=control)
