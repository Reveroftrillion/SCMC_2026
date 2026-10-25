import numpy as np
from pathlib import Path
from foscar_control.core import Route
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_rejoin import build_rejoin
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.state import Context
from test_highway_route import highway


def config():return HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')


def test_rejoin_passes_checkpoint_and_reduces_curvature_without_mutating_map(highway):
    original=highway.original.points.copy()
    curve=build_rejoin(highway,config())
    assert curve is not None
    assert np.all(np.diff(curve.stations)>0)
    cp=highway.checkpoints[12]
    assert np.min(np.linalg.norm(curve.xy-cp['xy'],axis=1))<1.   # 0.59 m on the 2026-10-01 path (rule: 3 m)
    assert max(Route(curve.xy).curvature)<.01
    np.testing.assert_allclose(curve.xy[-1],highway.original.at(curve.end_s),atol=1e-6)
    np.testing.assert_array_equal(highway.original.points,original)
    p=HighwayPolicy(highway,config());p.current_lane=2
    s=highway.rejoin_s-10.;x,y=curve.at(s)
    ref=p._reference(Context(s=s,x=x,y=y,yaw=0.,v=12.,now=1.))
    # Check both joins including neighboring untouched map points.
    mask=(ref.global_s>curve.stations[0]-5)&(ref.global_s<curve.end_s+5)
    assert max(ref.route.curvature[mask])<.012


def test_candidate_outside_checkpoint_acceptance_is_not_installed(highway):
    highway.checkpoints[12]['radius']=.01
    assert build_rejoin(highway,config()) is None


def test_no_paved_body_clearance_keeps_existing_rejoin(highway,monkeypatch):
    from foscar_planning.lane_prediction import LaneMap
    monkeypatch.setattr(LaneMap,'contains_road',lambda self,points:np.zeros(len(points),dtype=bool))
    policy=HighwayPolicy(highway,config())
    assert policy.rejoin is None
    policy.current_lane=2
    s=highway.rejoin_s-10.;x,y=highway.at(2,s)
    ref=policy._reference(Context(s=s,x=x,y=y,yaw=0.,v=12.,now=1.))
    np.testing.assert_allclose(ref.at_q(ref.q_at_global(s)),[x,y],atol=.01)
