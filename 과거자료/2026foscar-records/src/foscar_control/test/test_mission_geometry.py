from pathlib import Path
import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[3]

def test_competition_highway_end_is_cp13_and_general_before_tunnel():
    from foscar_control.mission_config import load_mission_config
    from foscar_control.speed_policy import uncapped_highway_zones
    config=ROOT/'src/foscar_planning/config/missions.yaml'
    rows=load_mission_config(config)['zones']
    highway=next(z for z in rows if z['name']=='HIGHWAY')
    tunnel=next(z for z in rows if z['name']=='TUNNEL')
    # Stations on the user's global path of 2026-10-01 (17:21); the old path had CP10 at 1118.36 and CP13 at 1741.34.
    # CP10: the 60 km/h exemption. The FSM itself takes over at approach_s (curve end, 2026-09-28).
    assert highway['limit_start_s']==pytest.approx(1108.718041,abs=.001)
    assert highway['start_s']==pytest.approx(1100.36,abs=.001)
    assert highway['end_s']==pytest.approx(1731.661656,abs=.001)
    assert highway['end_s']<tunnel['start_s']
    assert uncapped_highway_zones(config)==[(highway['limit_start_s'],highway['end_s'])]

def test_checkpoint_config_is_derived_from_geometry_not_fixed_station(tmp_path):
    from foscar_control.mission_config import load_mission_config
    np.savetxt(tmp_path/'route.txt',np.c_[np.arange(101.),np.zeros(101)])
    (tmp_path/'cp.txt').write_text('반경 3m\n[10]\n20, 0, 0\n0\n[13]\n80, 0, 0\n0\n')
    data=dict(geometry=dict(route='route.txt',checkpoints='cp.txt'),default={},zones=[dict(name='HIGHWAY',start_checkpoint=10,end_checkpoint=13,speed_kmh=None)])
    f=tmp_path/'missions.yaml';f.write_text(yaml.safe_dump(data))
    assert load_mission_config(f)['zones'][0]['end_s']==80
    (tmp_path/'cp.txt').write_text('반경 3m\n[10]\n20, 0, 0\n0\n[13]\n70, 0, 0\n0\n')
    assert load_mission_config(f)['zones'][0]['end_s']==70
    data['zones'][0]['end_s']=90;f.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_mission_config(f)
