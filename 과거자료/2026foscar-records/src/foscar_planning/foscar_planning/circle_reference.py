"""Fit a closed map loop for display. This reference does not authorize motion."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class CircularReference:
    center: np.ndarray
    radius: float
    rms: float
    max_error: float
    direction: int

    @classmethod
    def from_lanes(cls, lanes, link_ids):
        if lanes is None or not link_ids or any(k not in lanes.links for k in link_ids):
            return None
        pieces=[lanes.links[k][0] for k in link_ids]
        if any(np.linalg.norm(a[-1]-b[0])>.5 for a,b in zip(pieces,pieces[1:]+pieces[:1])):
            return None
        points=np.vstack([p[:-1] for p in pieces])
        origin=points.mean(axis=0);q=points-origin
        fit=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],(q*q).sum(axis=1),rcond=None)[0]
        center=origin+fit[:2];radius2=fit[2]+fit[:2]@fit[:2]
        if radius2<=0:return None
        radius=math.sqrt(radius2)
        angles=np.unwrap(np.arctan2(points[:,1]-center[1],points[:,0]-center[0]))
        error=np.linalg.norm(points-center,axis=1)-radius
        if abs(angles[-1]-angles[0])<5. or not 5.<radius<80. or max(abs(error))>1.:
            return None
        return cls(center,radius,float(np.sqrt(np.mean(error**2))),float(max(abs(error))),
                   1 if angles[-1]>angles[0] else -1)

    def arc(self, start=0.,end=2*math.pi,count=120):
        a=np.linspace(start,end,count)
        return np.c_[self.center[0]+self.radius*np.cos(a),self.center[1]+self.radius*np.sin(a)]
