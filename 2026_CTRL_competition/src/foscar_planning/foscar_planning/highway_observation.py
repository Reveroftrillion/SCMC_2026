"""The current complete scan owns objects; a measured car that drops out of it coasts for coast_s.
Motion history dies with visibility."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ObservedObject:
    id: str
    stamp: float
    xy: np.ndarray
    speed_ready: bool
    velocity_bounds: tuple
    sample_count: int
    velocity_stamp: float=-math.inf
    motion_source: str="unknown"
    height: float=math.nan    # top of the hull above the road (lidar z_max, clipped at its ROI top)


@dataclass(frozen=True)
class ObservationSnapshot:
    stamp: float
    valid: bool
    reason: str
    objects: dict
    removed_count: int=0
    sight_m: float=math.inf   # how far ahead (base_link x) a car body is met by a ring this scan (foscar_lidar sight.py)


class HighwayObservations:
    def __init__(self,history_s,max_age_s,min_history_s,velocity_error_mps,max_gap_s):
        if min(history_s,max_age_s,min_history_s,velocity_error_mps,max_gap_s)<=0:raise ValueError('invalid observation settings')
        self.history_s,self.max_age_s,self.min_history_s=history_s,max_age_s,min_history_s
        self.velocity_error,self.max_gap=velocity_error_mps,max_gap_s
        self.reset()

    motion_memory_s=1.
    # A car being passed vanished from the lidar 45 m ahead in lane 4 (s1137-1164, 12:11 run 4): the pass plan
    # lost its target, the fallback cut in, and following/boxes would not have seen it. A measured car that
    # drops out keeps moving at its last speed for coast_s unless a new body takes its place (an id change).
    coast_s=2.

    def reset(self):
        self.stamp=-math.inf;self.valid=False;self.objects={};self.histories={};self.removed=0;self.sight=math.inf
        self.motion={}   # id -> (stamp, velocity bounds) of its last measured motion

    def accept(self,data,now):
        if not isinstance(data,dict) or data.get('frame')!='map':return False
        stamp=data.get('stamp');rows=data.get('objects')
        if isinstance(stamp,bool) or not isinstance(stamp,(float,int)) or not math.isfinite(stamp) or stamp<=self.stamp or stamp>now:return False
        if not isinstance(rows,list) or not isinstance(data.get('scan_valid'),bool):return False
        parsed={};heights={}
        try:
            for row in rows:
                key=row['id'];xy=np.asarray(row['xy'],dtype=float)
                if not isinstance(key,str) or not key or key in parsed or xy.ndim!=2 or xy.shape[1]!=2 or len(xy)<2 or not np.isfinite(xy).all():return False
                parsed[key]=xy.copy()
                h=row.get('height');heights[key]=float(h) if isinstance(h,(int,float)) and not isinstance(h,bool) and math.isfinite(h) else math.nan
        except (KeyError,ValueError,TypeError):return False
        sight=data.get('sight')
        reach=sight.get('range_m') if isinstance(sight,dict) else None
        self.sight=float(reach) if isinstance(reach,(int,float)) and not isinstance(reach,bool) and math.isfinite(reach) and reach>0 else math.inf
        if not data['scan_valid']:
            self.histories={};self.objects={};self.stamp=stamp;self.valid=False
            return True
        self.removed=len(set(self.objects)-set(parsed))
        continuous=self.valid and stamp-self.stamp<=self.max_gap
        histories={};objects={}
        matches={}
        if continuous:
            for key,xy in parsed.items():
                candidates=[]
                for oldkey,obj in self.objects.items():
                    if oldkey!=key and oldkey in parsed:continue
                    if not obj.speed_ready or stamp-obj.velocity_stamp>self.max_gap:continue
                    a,b,c,d=obj.velocity_bounds;velocity=np.array([(a+b)/2,(c+d)/2])
                    predicted=obj.xy+velocity*(stamp-obj.stamp)
                    # Both observed extents must remain compatible. Ambiguous
                    # split/merge observations do not inherit motion.
                    if np.max(abs(xy.min(axis=0)-predicted.min(axis=0)))<=1. and np.max(abs(xy.max(axis=0)-predicted.max(axis=0)))<=1.:
                        candidates.append(oldkey)
                if len(candidates)==1:matches[key]=candidates[0]
            counts={old:list(matches.values()).count(old) for old in matches.values()}
            matches={key:old for key,old in matches.items() if counts[old]==1}
        for key,xy in parsed.items():
            old=self.histories.get(key,[]) if continuous else []
            # Use both bounding ends. A changing visible centroid alone is not vehicle motion.
            features=np.r_[xy.mean(axis=0),xy.min(axis=0),xy.max(axis=0)]
            rows=[r for r in old if stamp-r[0]<=self.history_s]+[(stamp,features)]
            ready=len(rows)>=3 and stamp-rows[0][0]>=self.min_history_s-1e-9
            bounds=(-math.inf,math.inf,-math.inf,math.inf)
            if ready:
                ts=np.array([r[0]-stamp for r in rows]);positions=np.array([r[1] for r in rows])
                fit=np.polyfit(ts,positions,1);pred=ts[:,None]*fit[0]+fit[1]
                residual=np.max(abs(positions-pred),axis=0)/(stamp-rows[0][0])
                # Pick the continuously moving face with the smallest fit
                # residual on each axis. A newly revealed end must not turn
                # into a one-frame speed spike for the whole vehicle.
                indexes=[axis+2*int(np.argmin(residual[axis::2])) for axis in (0,1)]
                velocity=fit[0,indexes]
                error=self.velocity_error+np.minimum(residual[indexes],self.velocity_error)
                bounds=(float(velocity[0]-error[0]),float(velocity[0]+error[0]),
                        float(velocity[1]-error[1]),float(velocity[1]+error[1]))

            measured_stamp=stamp if ready else -math.inf
            source='measured' if ready else 'unknown'
            if not ready and key in matches:
                previous=self.objects[matches[key]]
                bounds=previous.velocity_bounds;ready=True
                measured_stamp=previous.velocity_stamp;source='recent_geometry_match'
            elif not ready and key in self.motion and stamp-self.motion[key][0]<=self.motion_memory_s:
                # Same track, history briefly thinned by missed scans: keep its last measured
                # motion instead of treating the lead as stopped (hard brake 50 m behind a
                # 40 km/h lead, 2026-09-27).
                measured_stamp,bounds=self.motion[key];ready=True;source='recent_id'
            if ready and source=='measured':self.motion[key]=(stamp,bounds)
            histories[key]=rows;objects[key]=ObservedObject(key,stamp,xy,ready,bounds,len(rows),measured_stamp,source,heights[key])
        if continuous:
            inherited=set(matches.values())
            for oldkey,obj in self.objects.items():
                if oldkey in objects or oldkey in inherited or not obj.speed_ready or stamp-obj.velocity_stamp>self.coast_s:continue
                a,b,c,d=obj.velocity_bounds;velocity=np.array([(a+b)/2,(c+d)/2])
                xy=obj.xy+velocity*(stamp-obj.stamp)
                if any(np.linalg.norm(o.xy.mean(axis=0)-xy.mean(axis=0))<3. for o in objects.values()):continue
                objects[oldkey]=ObservedObject(oldkey,stamp,xy,True,obj.velocity_bounds,obj.sample_count,obj.velocity_stamp,'coasting',obj.height)
        self.motion={k:v for k,v in self.motion.items() if stamp-v[0]<=self.motion_memory_s}
        self.objects,self.histories=objects,histories
        self.stamp,self.valid=stamp,True
        return True

    def snapshot(self,now):
        valid=self.valid and 0<=now-self.stamp<=self.max_age_s
        return ObservationSnapshot(self.stamp,valid,'OK' if valid else 'SCAN_UNKNOWN',
                                   dict(self.objects) if valid else {},self.removed,self.sight if valid else math.inf)
