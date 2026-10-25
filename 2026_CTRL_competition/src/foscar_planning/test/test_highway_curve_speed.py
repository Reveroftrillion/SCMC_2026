"""Curve speed must depend on distance, and ignore already passed bends."""
from pathlib import Path
import numpy as np
from foscar_control.reference import ReferenceLine
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_observation import ObservationSnapshot
from foscar_planning.state import Context


def bend_line(name):
    """Straight 0..90 m, a 25 m radius bend 90..111 m, straight after (real geometry: the policy rates curvature
    from the heading change over +-10 m, like the tracker, not from route.curvature)."""
    ss=np.arange(0.,201.,.5)
    k=np.where((ss>=90.)&(ss<111.),.04,0.)
    heading=np.r_[0.,np.cumsum(k[:-1]*np.diff(ss))]
    xy=np.c_[np.r_[0.,np.cumsum(np.cos(heading[:-1])*np.diff(ss))],np.r_[0.,np.cumsum(np.sin(heading[:-1])*np.diff(ss))]]
    return ReferenceLine(name,ss,xy),xy


def test_curve_braking_is_delayed_until_needed_and_released_after_passing():
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')
    policy=HighwayPolicy(None,cfg)
    ref,xy=bend_line('curve-speed')     # a 25 m radius bend at 90..111 m
    snap=ObservationSnapshot(1.,True,'OK',{})
    caps=[]
    for s in (10.,80.,100.,135.):
        x,y=xy[int(s*2)]
        cap,*_=policy._follow_cap(Context(s=s,x=x,y=y,yaw=0.,v=12.,now=1.),snap,ref)
        caps.append(cap)
    assert caps[0]>12.,caps
    assert caps[0]>caps[1]>caps[2],caps
    assert caps[2]<=np.sqrt(cfg.lateral_accel_limit/.04)+1e-9
    assert caps[3]>12.,caps
    # Braking plus latency must reach the curve at its allowed speed. The +-10 m window and +-10 m dilation
    # ramp the rating in from 70 m and rate the whole bend (90..111 m) at its curvature.
    x,y=xy[100]
    early,*_=policy._follow_cap(Context(s=50.,x=x,y=y,yaw=0.,v=12.,now=1.),snap,ref)
    distance=90.-50.
    assert early**2<=cfg.lateral_accel_limit/.04+2*cfg.boxes.bend_brake_mps2*(distance-early*cfg.bounds.latency_s)+1e-8


def test_cp12_approach_keeps_straight_speed_but_limits_the_actual_bend():
    from test_highway_route import highway
    road=highway.__wrapped__()
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')
    policy=HighwayPolicy(road,cfg);policy.current_lane=2
    policy.rejoin=None  # Exercise distance-aware speed independently of rejoin smoothing.
    caps=[]
    for s in (1520.35,1550.34,1570.34,1580.36):   # old-path s1530-1590 on the 2026-10-01 path
        x,y=road.at(2,s)
        ctx=Context(s=s,x=x,y=y,yaw=0.,v=12.,now=1.)
        ref=policy._reference(ctx)
        cap,*_=policy._follow_cap(ctx,ObservationSnapshot(1.,True,'OK',{}),ref)
        caps.append(cap)
    assert caps[0]>12.
    assert caps[0]>caps[1]>caps[2]>caps[3]
    # The CP12 lane-link joint is rated by its heading change over +-10 m, like the tracker does on the
    # highway: dilated +-4 m curvature read it as an R 85 m arc and braked to 70 km/h on an empty road
    # (2026-09-29). Still a bend, not a straight.
    assert 16.<caps[3]<40.


def test_committed_change_brakes_only_for_bends_ahead_not_its_worst_point():
    # The change used to hold its worst-point speed for the whole change and pinned
    # the car near its entry speed (~45 km/h, 2026-09-27). A bend 80 m ahead now only
    # limits speed through its braking distance, exactly as outside a change.
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')
    policy=HighwayPolicy(None,cfg);policy.committed=object()
    ref,xy=bend_line('committed-speed')
    snap=ObservationSnapshot(1.,True,'OK',{})
    far,*_=policy._follow_cap(Context(s=10.,x=xy[20][0],y=xy[20][1],yaw=0.,v=12.,now=1.),snap,ref)
    inside,*_=policy._follow_cap(Context(s=100.,x=xy[200][0],y=xy[200][1],yaw=0.,v=8.,now=1.),snap,ref)
    from foscar_planning.highway_config import change_curve_speed      # a committed change is driven at change_lat(v)
    fit=max(np.sqrt(cfg.lateral_accel_limit/.04),float(change_curve_speed(cfg.boxes,.04)))
    assert far>fit+1.
    assert inside<=fit+1e-9
def test_empty_straight_is_capped_only_at_the_highway_max_speed():
    # no sensor-range cruise cap; the highway max speed (130 km/h, user 2026-10-04) only
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')
    policy=HighwayPolicy(None,cfg)
    ss=np.arange(201.,dtype=float)
    ref=ReferenceLine('empty-highway',ss,np.c_[ss,np.zeros_like(ss)])
    cap,*_=policy._follow_cap(Context(s=10.,x=10.,y=0.,yaw=0.,v=20.,now=1.),
                             ObservationSnapshot(1.,True,'OK',{}),ref)
    assert cap==cfg.boxes.max_speed_mps


def test_change_lateral_limit_depends_on_speed():
    # user 2026-09-29: 4.5 m/s2 at 40 km/h, 3.0 at 100 km/h, linear between, held beyond
    from types import SimpleNamespace
    from foscar_planning.highway_config import change_lat, change_curve_speed
    b=SimpleNamespace(change_lat_slow_kmh=40.,change_lat_slow_mps2=4.5,change_lat_fast_kmh=100.,change_lat_fast_mps2=3.)
    assert abs(change_lat(b,30/3.6)-4.5)<1e-9 and abs(change_lat(b,40/3.6)-4.5)<1e-9
    assert abs(change_lat(b,70/3.6)-3.75)<1e-9
    assert abs(change_lat(b,100/3.6)-3.)<1e-9 and abs(change_lat(b,130/3.6)-3.)<1e-9
    for k in (.2,.02,.008,.003,.001):
        v=float(change_curve_speed(b,k))
        assert abs(v*v*k-change_lat(b,v))<1e-6,(k,v)
