"""Highway settings and existing controller limits; no evidence-file runtime gate."""
from dataclasses import dataclass, fields, replace
import math
from pathlib import Path
from types import SimpleNamespace
import yaml


@dataclass(frozen=True)
class PlanningBounds:
    """Tunable planning assumptions, not certified physical measurements."""
    min_brake_mps2: float=1.5
    lat_accel_mps2: float=2.5
    latency_s: float=0.25
    front_range_m: float=60.
    rear_range_m: float=40.
    npc_speed_bound_mps: float=20.
    follow_brake_mps2: float=4.
    position_error_m: float=0.2
    velocity_error_mps: float=0.5

    @classmethod
    def from_dict(cls,data):
        if not isinstance(data,dict) or set(data)-{f.name for f in fields(cls)}:
            raise ValueError('unknown planning bound')
        c=cls(**data)
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0
               for v in vars(c).values()):raise ValueError('planning bounds must be finite positive')
        return c


POLICY_KEYS=frozenset('history_s min_history_s max_scan_age_s max_observation_gap_s velocity_error_mps clear_hold_s settle_s settle_window_s settle_error_m settle_heading_rad margin_m standstill_gap_m min_change_length_m min_change_speed_mps min_merge_speed_mps change_time_s path_step_m lookahead_m reference_back_m reference_ahead_m projection_jump_margin_m front_roi_m rear_roi_m lateral_roi_m cut_in_min_sep_m safe_headway_s sensor_lead_in_m'.split())


BOX_KEYS=frozenset('length_m rear_m front_m intrusion_m sensor_forward_m shadow_pad_rad max_length_m angular_res_rad lateral_ref_m edge_window_s closing_error_mps closing_error_per_m rear_min_gap_m pass_rear_min_gap_m pass_front_gap_m accel_mps2 yield_brake_mps2 change_lat_slow_kmh change_lat_slow_mps2 change_lat_fast_kmh change_lat_fast_mps2 change_lat_max_mps2 rejoin_lat_mps2 brake_jerk_mps3 brake_max_mps2 unknown_speed_mps rear_accel_mps2 rear_accel_near_m tail_s start_step_s max_start_s max_enter_s speed_cost_s progress_horizon_s max_speed_mps'.split())


FOLLOWING_PHASES=('cruise','scout','yield','pass','change','settle')   # highway phases, each with its own gap values
FOLLOWING_KEYS=frozenset('time_gap_s approach_decel_mps2 gain_hz'.split())


def change_lat(boxes, v):
    """Lane-change lateral acceleration at speed v [m/s]: change_lat_slow at change_lat_slow_kmh, change_lat_fast at
    change_lat_fast_kmh, linear between and held beyond (user 2026-09-29: 4.5 at 40 km/h, 3.0 at 100 km/h)."""
    lo,hi=boxes.change_lat_slow_kmh/3.6,boxes.change_lat_fast_kmh/3.6
    w=min(1.,max(0.,(v-lo)/max(hi-lo,1e-9)))
    lat=boxes.change_lat_slow_mps2+(boxes.change_lat_fast_mps2-boxes.change_lat_slow_mps2)*w
    return min(lat,getattr(boxes,'change_lat_max_mps2',math.inf))


def change_curve_speed(boxes, curvature):
    """Highest speed v with v^2 * curvature <= change_lat(v) (array or scalar curvature > 0)."""
    import numpy as np
    k=np.maximum(np.asarray(curvature,dtype=float),1e-9)
    lo,hi=boxes.change_lat_slow_kmh/3.6,boxes.change_lat_fast_kmh/3.6
    a_lo,a_hi=boxes.change_lat_slow_mps2,boxes.change_lat_fast_mps2
    slope=(a_lo-a_hi)/max(hi-lo,1e-9)                      # lat = a_lo - slope*(v-lo) between lo and hi
    c=a_lo+slope*lo
    mid=(-slope+np.sqrt(slope*slope+4.*k*c))/(2.*k)
    v=np.where(np.sqrt(a_lo/k)<=lo,np.sqrt(a_lo/k),np.where(np.sqrt(a_hi/k)>=hi,np.sqrt(a_hi/k),mid))
    return np.minimum(v,np.sqrt(getattr(boxes,'change_lat_max_mps2',math.inf)/k))


@dataclass(frozen=True)
class ControlProfile:
    """Existing follower planning limits, not measured actuator bounds."""
    source: str
    a_lat_mps2: float
    decel_mps2: float
    accel_mps2: float

    @classmethod
    def load(cls, filename):
        path=Path(filename).expanduser().resolve()
        data=yaml.safe_load(path.read_text())
        if not isinstance(data,dict):raise ValueError('invalid control profile file')
        params={}
        for name in ('/**','path_follower','/path_follower'):
            section=data.get(name,{})
            if not isinstance(section,dict):raise ValueError('invalid control profile node')
            values=section.get('ros__parameters',{})
            if not isinstance(values,dict):raise ValueError('invalid control profile parameters')
            params.update(values)
        values={key:params.get(key) for key in ('a_lat_mps2','decel_mps2','accel_mps2')}
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0
               for v in values.values()):raise ValueError('missing/invalid control profile limits')
        return cls(str(path),**{key:float(value) for key,value in values.items()})


@dataclass(frozen=True)
class HighwayConfig:
    mode: str
    mission: str
    route: dict
    vehicle: SimpleNamespace
    policy: SimpleNamespace
    bounds: PlanningBounds
    control_profile: ControlProfile=None
    boxes: SimpleNamespace=None     # target-lane boxes and the slot planner
    following: SimpleNamespace=None # gap to the car ahead (see highway.yaml following:)

    @classmethod
    def load(cls,filename,*,control_config=None):
        cfg=cls.from_dict(yaml.safe_load(Path(filename).read_text()))
        return replace(cfg,control_profile=ControlProfile.load(control_config)) if control_config else cfg

    @property
    def lateral_accel_limit(self):
        limit=self.bounds.lat_accel_mps2
        return min(limit,self.control_profile.a_lat_mps2) if self.control_profile and limit is not None else limit

    @property
    def stopping_decel_limit(self):
        # Share the follower's configured braking profile. These are planning
        # settings, not a claim about measured physical deceleration.
        limit=self.bounds.min_brake_mps2
        return min(limit,self.control_profile.decel_mps2) if self.control_profile and limit is not None else limit

    @classmethod
    def from_dict(cls,data):
        if not isinstance(data,dict) or set(data)!={'mode','mission','route','vehicle','policy','bounds','boxes','following'}:raise ValueError('invalid highway config sections')
        if data['mode'] not in ('off','shadow','control'):raise ValueError('invalid highway mode')
        p=data['policy']
        if set(p)!=POLICY_KEYS:raise ValueError('missing/unknown highway policy')
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<=0 for v in p.values()):raise ValueError('policy must be finite positive')
        if p['min_history_s']>p['history_s'] or p['path_step_m']>1.:raise ValueError('inconsistent highway policy')
        vehicle=data['vehicle']
        if set(vehicle)!={'source','length_m','width_m','rear_axle_to_front_m'} or not vehicle['source']:raise ValueError('vehicle geometry source required')
        if any(not isinstance(vehicle[k],(float,int)) or not math.isfinite(vehicle[k]) or vehicle[k]<=0 for k in ('length_m','width_m','rear_axle_to_front_m')):raise ValueError('invalid vehicle geometry')
        if vehicle['rear_axle_to_front_m']>=vehicle['length_m']:raise ValueError('invalid axle position')
        route=data['route']
        if set(route)!={'start_checkpoint','hold_checkpoint','rejoin_checkpoint','end_checkpoint','excluded_lanes'}:raise ValueError('invalid route configuration')
        hold=route['hold_checkpoint']    # null: no checkpoint to pass in lane 4 first (CP11 dropped, 2026-09-30)
        if not route['start_checkpoint']<(route['start_checkpoint']+.5 if hold is None else hold)<route['rejoin_checkpoint']<route['end_checkpoint']:raise ValueError('invalid checkpoints')
        boxes=data['boxes']
        if not isinstance(boxes,dict) or set(boxes)!=BOX_KEYS:raise ValueError('invalid highway boxes')
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<=0 for v in boxes.values()):raise ValueError('boxes must be finite positive')
        boxes=SimpleNamespace(**boxes)
        following=data['following']
        if not isinstance(following,dict) or set(following)!={'standstill_m',*FOLLOWING_PHASES}:raise ValueError('invalid highway following')
        def positive(v):return not isinstance(v,bool) and isinstance(v,(float,int)) and math.isfinite(v) and v>0
        if not positive(following['standstill_m']):raise ValueError('following must be finite positive')
        phases={}
        for name in FOLLOWING_PHASES:
            row=following[name]
            if not isinstance(row,dict) or set(row)!=FOLLOWING_KEYS or not all(positive(v) for v in row.values()):
                raise ValueError(f'invalid highway following.{name}')
            phases[name]=SimpleNamespace(**row)
        following=dict(standstill_m=following['standstill_m'],**phases)
        return cls(data['mode'],data['mission'],dict(route),SimpleNamespace(**vehicle),SimpleNamespace(**p),PlanningBounds.from_dict(data['bounds']),boxes=boxes,
                   following=SimpleNamespace(**following))
