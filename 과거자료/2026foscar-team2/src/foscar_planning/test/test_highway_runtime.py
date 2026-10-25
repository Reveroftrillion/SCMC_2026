from pathlib import Path
from dataclasses import replace
from foscar_planning.highway_runtime import HighwayRuntime
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.state import Target
from test_highway_route import highway
from test_highway_policy import ctx

CONFIG=Path(__file__).resolve().parents[1]/'config/highway.yaml'

def test_shadow_preserves_target_and_control_uses_valid_scan(highway):
    cfg=HighwayConfig.load(CONFIG)
    shadow=HighwayRuntime(replace(cfg,mode='shadow'),highway)
    upstream=Target(v_max=13.)
    c=ctx(highway,1150,lane=5,v=12.)
    assert shadow.apply(upstream,c) is upstream
    assert shadow.diagnostic['reason']=='SCAN_UNKNOWN'
    control=HighwayRuntime(replace(cfg,mode='control'),highway)
    control.accept(dict(stamp=1.,frame='map',scan_valid=True,objects=[]),1.)
    out=control.apply(upstream,c)
    assert 0<out.v_max<=13. and out.reference is not None


def test_other_missions_never_get_highway_output(highway):
    cfg=replace(HighwayConfig.load(CONFIG),mode='control')
    run=HighwayRuntime(cfg,highway)
    upstream=Target(v_max=7.)
    c=ctx(highway,1150,lane=5);c.mission_name='ROUNDABOUT'
    assert run.apply(upstream,c) is upstream
    assert run.diagnostic=={}


def test_leaving_the_highway_hands_the_reference_back(highway):
    # 2026-09-27 s1759: at CP13 the rejoin was not yet within 0.3 m, the mission ended and
    # the local reference simply vanished; the controller stopped with REFERENCE_MISSING.
    cfg=replace(HighwayConfig.load(CONFIG),mode='control')
    run=HighwayRuntime(cfg,highway)
    run.accept(dict(stamp=1.,frame='map',scan_valid=True,objects=[]),1.)
    c=ctx(highway,1150,lane=5,v=12.)
    assert run.apply(Target(v_max=13.),c).reference is not None
    c.mission_name='CRUISE';c.now+=.1
    out=run.apply(Target(v_max=13.),c)
    assert out.reference is None and out.reference_release
    c.now+=5.
    assert not run.apply(Target(v_max=13.),c).reference_release   # only around the handoff


def test_reference_is_handed_back_when_the_node_resets_first(highway):
    # behavior_node calls reset() on the mission change before apply(); the first fix only
    # handled apply() and the car still stopped at CP13 (2026-09-27, s1758).
    cfg=replace(HighwayConfig.load(CONFIG),mode='control')
    run=HighwayRuntime(cfg,highway)
    run.accept(dict(stamp=1.,frame='map',scan_valid=True,objects=[]),1.)
    c=ctx(highway,1150,lane=5,v=12.)
    assert run.apply(Target(v_max=13.),c).reference is not None
    run.reset()
    c.mission_name='CRUISE';c.now+=.1
    assert run.apply(Target(v_max=13.),c).reference_release
