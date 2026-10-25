"""Fixed map-checked C2 connection through CP12; no online curve switching."""
from dataclasses import dataclass
import numpy as np
from .lane_prediction import LaneMap
from foscar_control.core import Route


@dataclass(frozen=True)
class RejoinCurve:
    stations: np.ndarray
    xy: np.ndarray

    @property
    def end_s(self):return float(self.stations[-1])

    def at(self,s):
        return np.array([np.interp(s,self.stations,self.xy[:,i]) for i in (0,1)])


def build_rejoin(road,config):
    """Return None on invalid geometry, retaining the existing mapped fallback."""
    car=config.vehicle
    key=(car.length_m,car.width_m,car.rear_axle_to_front_m,config.bounds.position_error_m)
    cache=getattr(road,'_rejoin_cache',{})
    if key in cache:return cache[key]
    curve=None
    lo=road.rejoin_s-40.;hi=road.rejoin_s+20.;length=hi-lo
    if lo>=road.tables[road.lanes[-1]][0,0] and hi<road.end_s:
        def base(s):
            return road.at(road.lanes[-1],s) if s<=road.rejoin_s else np.asarray(road.original.at(s))
        coeff=np.zeros((6,2));coeff[0]=base(lo)
        coeff[1]=(base(lo+.5)-base(lo-.5))*length
        coeff[2]=(base(lo+.5)-2*base(lo)+base(lo-.5))/.25*length**2/2
        rhs=np.array([base(hi)-coeff[:3].sum(axis=0),
            (base(hi+.5)-base(hi-.5))*length-coeff[1]-2*coeff[2],
            (base(hi+.5)-2*base(hi)+base(hi-.5))/.25*length**2-2*coeff[2]])
        coeff[3:]=np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],rhs)
        u=np.linspace(0.,1.,int(length/.25)+1)
        xy=sum(u[:,None]**i*coeff[i] for i in range(6))
        # Station labels must remain the original localization coordinate.
        stations=np.array([road.original.project(*point)[0] for point in xy])
        heading=np.arctan2(np.gradient(xy[:,1]),np.gradient(xy[:,0]))
        unit=np.c_[np.cos(heading),np.sin(heading)];normal=np.c_[-unit[:,1],unit[:,0]]
        front=car.rear_axle_to_front_m;rear=car.length_m-front
        half=car.width_m/2+config.bounds.position_error_m
        body=np.concatenate([xy+a*unit+b*normal for a in np.linspace(-rear,front,11)
                             for b in np.linspace(-half,half,7)])
        cp=road.checkpoints[road.rejoin_checkpoint]
        original=np.array([base(s) for s in np.linspace(lo,hi,len(xy))])
        if (np.isfinite(xy).all() and np.all(np.diff(stations)>0)
                and np.min(np.linalg.norm(xy-cp['xy'],axis=1))<=cp['radius']
                and max(Route(xy).curvature)<max(Route(original).curvature)
                and LaneMap(list(road.links.values())).contains_road(body).all()):
            curve=RejoinCurve(stations,xy)
    cache[key]=curve;road._rejoin_cache=cache
    return curve
