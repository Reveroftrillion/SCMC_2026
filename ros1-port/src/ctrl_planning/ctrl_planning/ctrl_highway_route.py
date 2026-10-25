"""Map-derived lane chains. Original route station is never lane arc length."""
import json
import math
import re
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from ctrl_control.ctrl_core import Route


from ctrl_control.ctrl_mission_config import read_checkpoints


class HighwayRoute:
    @classmethod
    def from_files(cls,link_file,route_file,checkpoint_file,*,start_checkpoint,
                   hold_checkpoint,rejoin_checkpoint,end_checkpoint,excluded_lanes):
        return cls(json.loads(Path(link_file).read_text()),np.loadtxt(route_file),
                   read_checkpoints(checkpoint_file),start_checkpoint,hold_checkpoint,
                   rejoin_checkpoint,end_checkpoint,excluded_lanes)

    def __init__(self,links,points,checkpoints,start_checkpoint,hold_checkpoint,rejoin_checkpoint,end_checkpoint,excluded_lanes):
        self.original=Route(points)
        self.checkpoints={k:dict(v) for k,v in checkpoints.items()}
        self.start_checkpoint,self.hold_checkpoint,self.rejoin_checkpoint=start_checkpoint,hold_checkpoint,rejoin_checkpoint
        for cp in self.checkpoints.values():
            cp['s']=float(self.original.project(*cp['xy'])[0])
        self.start_s=self.checkpoints[start_checkpoint]['s']
        # hold_checkpoint None: no checkpoint to pass in lane 4 before 4->3 (CP11 dropped from the rules, 2026-09-30).
        self.hold_s=None if hold_checkpoint is None else self.checkpoints[hold_checkpoint]['s']
        self.rejoin_s=self.checkpoints[rejoin_checkpoint]['s']
        self.end_checkpoint=end_checkpoint
        self.end_s=self.checkpoints[end_checkpoint]['s']
        if not self.start_s<(self.start_s+1e-9 if self.hold_s is None else self.hold_s)<self.rejoin_s<self.end_s:
            raise ValueError('checkpoint order')
        self.links={r['idx']:r for r in links}
        self.excluded=frozenset(excluded_lanes)
        self.outgoing={}
        for r in links:self.outgoing.setdefault(r['from_node_idx'],[]).append(r['idx'])
        anchor=self.checkpoints[start_checkpoint]['xy']
        starts=sorted((np.linalg.norm(np.asarray(r['points'][0])[:2]-anchor),r['idx'])
                      for r in links if r.get('ego_lane') not in self.excluded)
        if not starts or starts[0][0]>self.checkpoints[start_checkpoint]['radius']:raise ValueError('no start link')
        self.start_link=starts[0][1]
        seeds=[];key=self.start_link
        while key is not None:
            if key in seeds: raise ValueError('cyclic lateral links')
            seeds.append(key);key=self.left_link(key)
        self.lanes=tuple(self.links[k]['ego_lane'] for k in seeds)
        if len(set(self.lanes))!=len(seeds) or len(seeds)<2: raise ValueError('invalid lateral lane ordering')
        self._projection_lo=max(0,np.searchsorted(self.original.s,self.start_s)-1)
        self._projection_hi=min(len(self.original.points),np.searchsorted(self.original.s,self.rejoin_s)+2)
        self._projection_tree=cKDTree(self.original.points[self._projection_lo:self._projection_hi])
        self._bounds={}
        self._deadlines={}
        self.chains={};self.tables={}
        for seed in seeds:
            lane=self.links[seed]['ego_lane'];chain=[];key=seed
            while key is not None and self.links[key]['ego_lane']==lane:
                if key in chain:raise ValueError('cyclic highway')
                chain.append(key)
                self._bounds[key]=self.project_many(np.asarray(self.links[key]['points'])[[0,-1],:2])
                end=np.asarray(self.links[key]['points'][-1])[:2]
                if np.linalg.norm(end-self.checkpoints[rejoin_checkpoint]['xy'])<1e-5:break
                key=self.next_link(key)
            pts=np.vstack([np.asarray(self.links[k]['points'])[:,:2] for k in chain])
            station=self.project_many(pts)
            if np.any(np.diff(station)<-1.):raise ValueError('backwards lane projection')
            station=np.maximum.accumulate(station)
            keep=np.r_[True,np.diff(station)>1e-6]
            self.chains[lane]=tuple(chain)
            self.tables[lane]=np.c_[station[keep],pts[keep]]
        self.chain_ids=frozenset(k for chain in self.chains.values() for k in chain)
        self.entry_end_s=float(self.tables[self.lanes[0]][-1,0])   # where the entry lane (5) merges into lane 4
        # Adjacent traffic exists upstream of the ego ramp's checkpoint. Use actual
        # predecessor links for observations, without changing mission station.
        self.observation_tables={};self.observation_chains={};self._observation_bounds={}
        for lane,chain in self.chains.items():
            prefix=[];node=self.links[chain[0]]['from_node_idx'];seen=set(chain)
            while True:
                incoming=[row['idx'] for row in links if row['to_node_idx']==node and row['ego_lane']==lane and row['idx'] not in seen]
                if len(incoming)!=1:break
                key=incoming[0];prefix.insert(0,key);seen.add(key);node=self.links[key]['from_node_idx']
            self.observation_chains[lane]=tuple(prefix)+chain
            if not prefix:
                self.observation_tables[lane]=self.tables[lane];continue
            pts=np.vstack([np.asarray(self.links[k]['points'])[:,:2] for k in prefix])
            q=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]
            station=q-q[-1]+self.tables[lane][0,0]
            keep=np.r_[True,np.diff(station)>1e-6]
            self.observation_tables[lane]=np.vstack([np.c_[station[keep],pts[keep]][:-1],self.tables[lane]])
            start=float(station[0])
            for key in prefix:
                pp=np.asarray(self.links[key]['points'])[:,:2]
                end=start+float(np.linalg.norm(np.diff(pp,axis=0),axis=1).sum())
                self._observation_bounds[key]=(start,end);start=end


    def project_many(self,xy):
        """Exact projection onto the route segment nearest a KD-tree seed."""
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        lo,hi=self._projection_lo,self._projection_hi
        _,near=self._projection_tree.query(xy)
        idx=np.clip(near[:,None]+lo+np.array([-1,0]),lo,min(hi-2,len(self.original.delta)-1))
        a=self.original.points[idx];v=self.original.delta[idx]
        t=np.clip(np.sum((xy[:,None]-a)*v,axis=2)/np.sum(v*v,axis=2),0.,1.)
        d=np.linalg.norm(xy[:,None]-a-t[:,:,None]*v,axis=2)
        j=np.argmin(d,axis=1);i=np.arange(len(xy))
        return self.original.s[idx[i,j]]+t[i,j]*self.original.lengths[idx[i,j]]

    def left_link(self,key):
        left=self.links[key].get('left_lane_change_dst_link_idx')
        if left is None:return None
        if left not in self.links:raise ValueError('missing adjacent link')
        return left if self.links[left]['ego_lane'] not in self.excluded else None

    def next_link(self,key):
        row=self.links[key];candidates=self.outgoing.get(row['to_node_idx'],[])
        if not candidates:return None
        end=np.asarray(row['points'][-1])[:2]
        at_rejoin=np.linalg.norm(end-self.checkpoints[self.rejoin_checkpoint]['xy'])<1e-5
        if not at_rejoin:
            same=[k for k in candidates if self.links[k]['ego_lane']==row['ego_lane']]
            if len(same)==1:return same[0]
            if not same:return None
            candidates=same
        scores=[]
        tree=cKDTree(self.original.points)
        for k in candidates:
            p=np.asarray(self.links[k]['points'])[:,:2]
            scores.append((float(np.mean(tree.query(p)[0])),k))
        scores.sort()
        if len(scores)>1 and abs(scores[0][0]-scores[1][0])<1e-6:raise ValueError('ambiguous successor')
        return scores[0][1]

    def at(self,lane,s):
        if lane not in self.tables:raise ValueError('excluded or unknown lane')
        t=self.tables[lane]
        if s<t[0,0]-1e-5 or s>t[-1,0]+1e-5:raise ValueError('outside lane coverage')
        return np.array([np.interp(s,t[:,0],t[:,j]) for j in (1,2)])

    def centerline(self,lane,start_global_s,end_global_s,step=.5):
        if step<=0 or end_global_s<=start_global_s:raise ValueError('invalid sampling')
        ss=np.linspace(start_global_s,end_global_s,int(np.ceil((end_global_s-start_global_s)/step))+1)
        return np.c_[ss,np.array([self.at(lane,s) for s in ss])]

    def link_at(self,lane,s):
        for k in self.chains[lane]:
            bounds=self._bounds[k]
            if bounds[0]-1e-5<=s<=bounds[1]+1e-5:return k
        return None

    def observation_at(self,lane,s):
        t=self.observation_tables[lane]
        if not t[0,0]<=s<=t[-1,0]:raise ValueError('outside observation map coverage')
        return np.array([np.interp(s,t[:,0],t[:,j]) for j in (1,2)])

    def width(self,lane,s):
        # Memoised per 0.1 m: build_maneuver asks for ~40k corner widths per gap preparation.
        cache=self.__dict__.setdefault('_width_cache',{});memo=(lane,round(float(s),1))
        if memo not in cache:cache[memo]=self._width(lane,s)
        return cache[memo]

    def _width(self,lane,s):
        key=self.link_at(lane,s)
        if key is None:
            key=next((k for k in self.observation_chains[lane] if k in self._observation_bounds
                      and self._observation_bounds[k][0]<=s<=self._observation_bounds[k][1]),None)
        if key is None:raise ValueError('no lane width')
        row=self.links[key]
        values=[row.get('width_start'),row.get('width_end')]
        if any(v is None or not math.isfinite(v) or v<=0 for v in values):raise ValueError('map lane width unavailable')
        return min(values)

    def match(self,x,y,s):
        candidates=[]
        for lane,t in self.tables.items():
            if t[0,0]-1e-5<=s<=t[-1,0]+1e-5:
                candidates.append((float(np.linalg.norm(self.at(lane,s)-[x,y])),lane))
        if not candidates:raise ValueError('no highway lane at station')
        error,lane=min(candidates)
        return lane,error

    def merges_into(self,lane,target,tolerance_m=.3):
        """Does `lane` end on `target`'s centre (lane 5 tapering into lane 4)? Past its end a change into `target`
        just follows `target`."""
        if lane not in self.tables or target not in self.tables:return False
        a=self.tables[lane];b=self.tables[target];s=a[-1,0]
        if s>=b[-1,0]:return False
        return bool(np.hypot(a[-1,1]-np.interp(s,b[:,0],b[:,1]),a[-1,2]-np.interp(s,b[:,0],b[:,2]))<tolerance_m)

    def separation(self,lane,target,s):
        """Distance between the two lanes' centres at station s (clamped to `lane`'s coverage)."""
        a=self.tables[lane];b=self.tables[target];s=min(max(s,a[0,0]),a[-1,0])
        return float(np.hypot(np.interp(s,a[:,0],a[:,1])-np.interp(s,b[:,0],b[:,1]),np.interp(s,a[:,0],a[:,2])-np.interp(s,b[:,0],b[:,2])))

    def coverage_end(self,lane,target):
        """Last station a change from `lane` into `target` may run to."""
        a=self.tables[lane][-1,0];b=self.tables[target][-1,0]
        return float(b if self.merges_into(lane,target) else min(a,b))

    def change_deadline(self,lane,minimum_separation_m):
        """Last station before source and destination lose required separation."""
        key=(lane,minimum_separation_m)
        if key in self._deadlines:return self._deadlines[key]
        i=self.lanes.index(lane)
        if i+1>=len(self.lanes):return self.rejoin_s
        dst=self.lanes[i+1];a=self.tables[lane];b=self.tables[dst]
        ss=a[(a[:,0]>=b[0,0])&(a[:,0]<=b[-1,0]),0]
        src=np.c_[np.interp(ss,a[:,0],a[:,1]),np.interp(ss,a[:,0],a[:,2])]
        target=np.c_[np.interp(ss,b[:,0],b[:,1]),np.interp(ss,b[:,0],b[:,2])]
        close=np.flatnonzero(np.linalg.norm(src-target,axis=1)<minimum_separation_m)
        value=float(ss[close[0]]) if len(close) else float(min(a[-1,0],b[-1,0]))
        self._deadlines[key]=value
        return value

    def checkpoint_passed(self,number,previous_xy,current_xy,continuous=True):
        if not continuous:return False
        a=np.asarray(previous_xy,dtype=float);b=np.asarray(current_xy,dtype=float)
        if a.shape!=(2,) or b.shape!=(2,) or not np.isfinite([a,b]).all():return False
        cp=self.checkpoints[number];v=b-a
        t=np.clip(np.dot(cp['xy']-a,v)/max(np.dot(v,v),1e-12),0.,1.)
        return bool(np.linalg.norm(a+t*v-cp['xy'])<=cp['radius'])
