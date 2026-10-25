"""One map-lane transition: the change curve between two lane centres."""
from dataclasses import dataclass
import hashlib
import math
import numpy as np
from foscar_control.reference import ReferenceLine


@dataclass(frozen=True)
class Maneuver:
    id: str
    global_s: np.ndarray
    xy: np.ndarray
    q: np.ndarray
    start_global_s: float
    end_global_s: float
    duration_s: float
    source_lane: int
    target_lane: int
    reference: ReferenceLine
    geometry_ok: bool
    road: object


def blend_centers(source_xy,target_xy,u):
    u=np.clip(np.asarray(u),0.,1.);w=10*u**3-15*u**4+6*u**5
    return np.asarray(source_xy)*(1-w[:,None])+np.asarray(target_xy)*w[:,None]


def center_at(highway,lane,s):
    if s>highway.rejoin_s:return np.asarray(highway.original.at(s))
    return highway.at(lane,s)


def centers_at(highway,lane,ss):
    """center_at for many stations at once (same coverage error, same original route past rejoin)."""
    ss=np.asarray(ss,dtype=float);out=np.empty((len(ss),2))
    far=ss>highway.rejoin_s;near=~far
    if near.any():
        if lane not in highway.tables:raise ValueError('excluded or unknown lane')
        t=highway.tables[lane];sub=ss[near]
        if sub.min()<t[0,0]-1e-5 or sub.max()>t[-1,0]+1e-5:raise ValueError('outside lane coverage')
        out[near]=np.c_[np.interp(sub,t[:,0],t[:,1]),np.interp(sub,t[:,0],t[:,2])]
    if far.any():out[far]=np.array([np.asarray(highway.original.at(s)) for s in ss[far]])
    return out


def heading_at(highway,lane,s,step):
    lo=max(highway.tables[lane][0,0],s-step)
    hi=min(highway.tables[lane][-1,0],s+step)
    if s>=highway.rejoin_s:return highway.original.heading_at(s)
    delta=highway.at(lane,hi)-highway.at(lane,lo)
    return math.atan2(delta[1],delta[0])


def build_maneuver(highway,source_link,target_link,start_global_s,length_m,speed_mps,config):
    # source_link/target_link accept lane numbers or IDs; no fixed IDs in the algorithm.
    source=highway.links[source_link]['ego_lane'] if isinstance(source_link,str) else source_link
    target=highway.links[target_link]['ego_lane'] if isinstance(target_link,str) else target_link
    if source!=target and highway.lanes.index(target)!=highway.lanes.index(source)+1:raise ValueError('only one left lane at a time')
    p=config.policy;c=config.bounds;car=config.vehicle
    if min(length_m,speed_mps)<=0:raise ValueError('positive maneuver length/speed required')
    finish=start_global_s+length_m
    if source!=target and finish>min(highway.tables[source][-1,0],highway.tables[target][-1,0]):raise ValueError('change exceeds lane coverage')
    preview=p.lookahead_m+speed_mps*((c.latency_s or 0.)+p.clear_hold_s+p.max_scan_age_s)
    observation_start=start_global_s-preview
    lo=max(highway.tables[source][0,0],observation_start-p.reference_back_m)
    # Run past the change by a braking distance: at 68 km/h the curve ended before the car
    # settled and the controller stopped it at the reference end (2026-09-27).
    tail=p.lookahead_m+speed_mps**2/(2*config.stopping_decel_limit)
    hi=min(highway.original.s[-1],max(finish+tail,start_global_s+p.reference_ahead_m))
    if source==target and source!=highway.lanes[-1]:hi=min(hi,highway.tables[source][-1,0])
    ss=np.linspace(lo,hi,int(np.ceil((hi-lo)/p.path_step_m))+1)
    # Vectorised: per-point center_at made build_maneuver ~15 ms and a 7-wait pass preparation
    # ~0.7 s, which stalled behavior_node and dropped the reference at highway entry (2026-09-27).
    geometry=True;points=np.empty((len(ss),2))
    before=ss<=start_global_s;after=(ss>=finish)&~before;middle=~before&~after
    if before.any():points[before]=centers_at(highway,source,ss[before])
    if after.any():points[after]=centers_at(highway,target,ss[after])
    if middle.any():
        points[middle]=blend_centers(centers_at(highway,source,ss[middle]),centers_at(highway,target,ss[middle]),
                                     (ss[middle]-start_global_s)/length_m)
    ident=hashlib.sha256(ss.tobytes()+points.tobytes()).hexdigest()
    ref=ReferenceLine(ident,ss,points)
    # Batch all body corners: one route projection rather than hundreds of KD calls.
    front=car.rear_axle_to_front_m;rear=car.length_m-front
    if source!=target:
        indices=np.flatnonzero((ss>=start_global_s)&(ss<=finish))
        headings=np.array([ref.heading_at(ref.q[i]) for i in indices])
        unit=np.c_[np.cos(headings),np.sin(headings)]
        normal=np.c_[-np.sin(headings),np.cos(headings)]
        corners=np.concatenate([points[indices]+a*unit+b*normal
            for a in (-rear,front) for b in (-car.width_m/2-p.margin_m,car.width_m/2+p.margin_m)])
        station=highway.project_many(corners)
        station=np.clip(station,highway.tables[source][0,0],min(highway.tables[source][-1,0],highway.tables[target][-1,0]))
        def lane_xy(lane,values):
            table=highway.tables[lane]
            return np.c_[np.interp(values,table[:,0],table[:,1]),np.interp(values,table[:,0],table[:,2])]
        src=lane_xy(source,station);dst=lane_xy(target,station)
        direction=lane_xy(source,station+p.path_step_m)-lane_xy(source,station-p.path_step_m)
        direction/=np.maximum(np.linalg.norm(direction,axis=1)[:,None],np.finfo(float).eps)
        normal=np.c_[-direction[:,1],direction[:,0]]
        offset=np.sum((dst-src)*normal,axis=1);lateral=np.sum((corners-src)*normal,axis=1)
        def widths(lane,values):
            keys,inverse=np.unique(np.round(values,1),return_inverse=True)
            return np.array([highway.width(lane,float(k)) for k in keys])[inverse]
        sw=widths(source,station)/2;tw=widths(target,station)/2
        geometry=bool(np.all((np.minimum(-sw,offset-tw)<=lateral)&(lateral<=np.maximum(sw,offset+tw))))
    duration=(finish-observation_start)/speed_mps+p.settle_s
    return Maneuver(ident,ss,points,ref.q,start_global_s,finish,duration,source,target,ref,geometry,highway)


def change_curvature(maneuver,margin=1.):
    """Peak |curvature| of the change itself (start..finish). The reference runs on for a braking
    distance past the finish; at 83 km/h that tail (~186 m) reached the CP12 link joint and its bend was
    charged to the change: no curve 'fitted' and the car eased 84 -> 51 km/h with lane 2 empty (2026-09-28
    11:34 runs 3 and 7). Bends ahead are the bend cap's business."""
    k=np.abs(np.asarray(maneuver.reference.route.curvature))
    gs=np.asarray(maneuver.global_s)
    mask=(gs>=maneuver.start_global_s-margin)&(gs<=maneuver.end_global_s+margin)
    return float(k[mask].max()) if mask.any() else float(k.max())


def motion(obj):
    if not obj.speed_ready:return np.zeros(2)
    a,b,c,d=obj.velocity_bounds
    return np.array([(a+b)/2,(c+d)/2])
