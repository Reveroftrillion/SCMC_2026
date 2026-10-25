"""Immutable local geometry carrying original global route station labels."""
import hashlib
import math
import numpy as np
from .ctrl_core import Route


class ReferenceLine:
    def __init__(self,reference_id,global_s,xy):
        global_stations=np.array(global_s,dtype=float,copy=True);reference_points=np.array(xy,dtype=float,copy=True)
        if not isinstance(reference_id,str) or not reference_id or global_stations.ndim!=1 or reference_points.shape!=(len(global_stations),2) or len(global_stations)<2:raise ValueError('reference shape/id')
        if not np.isfinite(global_stations).all() or not np.isfinite(reference_points).all() or np.any(np.diff(global_stations)<=0):raise ValueError('reference station order')
        lengths=np.linalg.norm(np.diff(reference_points,axis=0),axis=1)
        if np.any(lengths<=1e-6) or np.any(lengths>1.+1e-6):raise ValueError('reference duplicate/gap')
        self.id=reference_id;self.global_s=global_stations;self.xy=reference_points
        self.route=Route(reference_points);self.q=self.route.s
        # 서명은 전역 station과 점 좌표의 기존 바이트 순서를 그대로 사용한다.
        self.signature=hashlib.sha256(global_stations.tobytes()+reference_points.tobytes()).hexdigest()
        self.global_s.flags.writeable=False;self.xy.flags.writeable=False

    def at_q(self,q):return self.route.at(q)
    def global_at_q(self,q):return float(np.interp(q,self.q,self.global_s))
    def q_at_global(self,s):return float(np.interp(s,self.global_s,self.q))
    def project(self,x,y,previous_q=None):return self.route.project(x,y,previous_q)
    def heading_at(self,q):return self.route.heading_at(q)
    def curvature_at(self,q):return float(np.interp(q,self.q,self.route.curvature))

    def valid_for(self,x,y,global_s,lookahead):
        local_station,_,_=self.project(x,y)
        return bool(self.global_s[0]<=global_s<=self.global_s[-1] and local_station+lookahead<=self.q[-1])
