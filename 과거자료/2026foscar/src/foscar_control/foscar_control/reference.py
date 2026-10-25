"""Immutable local geometry carrying original global route station labels."""
import hashlib
import math
import numpy as np
from .core import Route


class ReferenceLine:
    def __init__(self,reference_id,global_s,xy):
        gs=np.array(global_s,dtype=float,copy=True);pts=np.array(xy,dtype=float,copy=True)
        if not isinstance(reference_id,str) or not reference_id or gs.ndim!=1 or pts.shape!=(len(gs),2) or len(gs)<2:raise ValueError('reference shape/id')
        if not np.isfinite(gs).all() or not np.isfinite(pts).all() or np.any(np.diff(gs)<=0):raise ValueError('reference station order')
        lengths=np.linalg.norm(np.diff(pts,axis=0),axis=1)
        if np.any(lengths<=1e-6) or np.any(lengths>1.+1e-6):raise ValueError('reference duplicate/gap')
        self.id=reference_id;self.global_s=gs;self.xy=pts
        self.route=Route(pts);self.q=self.route.s
        self.signature=hashlib.sha256(gs.tobytes()+pts.tobytes()).hexdigest()
        self.global_s.flags.writeable=False;self.xy.flags.writeable=False

    def at_q(self,q):return self.route.at(q)
    def global_at_q(self,q):return float(np.interp(q,self.q,self.global_s))
    def q_at_global(self,s):return float(np.interp(s,self.global_s,self.q))
    def project(self,x,y,previous_q=None):return self.route.project(x,y,previous_q)
    def heading_at(self,q):return self.route.heading_at(q)
    def curvature_at(self,q):return float(np.interp(q,self.q,self.route.curvature))

    def valid_for(self,x,y,global_s,lookahead):
        q,_,_=self.project(x,y)
        return bool(self.global_s[0]<=global_s<=self.global_s[-1] and q+lookahead<=self.q[-1])
