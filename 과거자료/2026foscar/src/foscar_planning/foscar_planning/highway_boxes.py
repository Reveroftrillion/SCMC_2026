"""Target-lane boxes from the current scan only: OCCUPIED / FREE / UNKNOWN (not visible).

A box is occupied when any current hull point lies in it, unknown when the sensor's line of sight to it passes
behind another body or it is out of sensor range. target_bodies turns the occupied and unknown runs into the
bodies the slot planner plans around, each with the speed it is assumed to move at.
"""
import math
import numpy as np
from .highway_slot_planner import Body

FREE,OCCUPIED,UNKNOWN='FREE','OCCUPIED','UNKNOWN'


def _wrap(a):return (a+math.pi)%(2*math.pi)-math.pi


def box_length(config,distance):
    b=config.boxes
    return min(b.max_length_m,b.length_m+uncertainty(config,distance))


def uncertainty(config,distance):
    """Along-lane position error of a body edge seen at this distance in a neighbouring lane."""
    b=config.boxes
    return config.bounds.position_error_m+b.angular_res_rad*distance**2/b.lateral_ref_m


def stations(road,pts):
    """Route stations of points, extended along the route before its start: the projection clamps a body behind
    CP10 onto CP10, and at highway entry a lane-4 car 5 m behind us read as beside us or not at all (the lane
    table starts there; the observation table runs on upstream)."""
    ss=road.project_many(np.asarray(pts,dtype=float))
    for i,(pt,s) in enumerate(zip(np.asarray(pts,dtype=float).reshape(-1,2),ss)):
        h=road.original.heading_at(s);ss[i]+=float((pt-np.asarray(road.original.at(s)))@np.array([math.cos(h),math.sin(h)]))
    return ss


def lane_boxes(road,config,ctx,snapshot,lane,extents=None,members=None,table=None,width=None):
    """Boxes along `lane` from rear_m behind to front_m ahead of the ego station (upstream of CP10 too), or along any
    (global station, x, y) `table` of the given `width` (our own path).
    extents / members (lists, filled per box): station span of the hulls in an occupied box and which hulls they are
    (this scan only: runs are built from them, speeds come from how their edges move, never from hull ids)."""
    b=config.boxes;table=road.observation_tables[lane] if table is None else table
    lo=max(float(table[0,0]),ctx.s-b.rear_m);hi=min(float(table[-1,0]),ctx.s+b.front_m)
    # Longer boxes further away: along a neighbouring lane the lidar's point spacing grows with
    # distance squared (~0.1 m at 10 m, ~1.6 m at 40 m for 0.2 deg), so far boxes flicker.
    edges=[ctx.s]
    while edges[-1]<hi:edges.append(edges[-1]+box_length(config,edges[-1]-ctx.s))
    back=[ctx.s]
    while back[-1]>lo:back.append(back[-1]-box_length(config,ctx.s-back[-1]))
    edges=np.array([e for e in back[::-1][:-1]+edges if lo-1e-9<=e<=hi+1e-9])
    n=len(edges)-1
    if n<1:return []
    s0=edges[:-1];lengths=np.diff(edges);mid=s0+lengths/2
    def xy(values):return np.column_stack([np.interp(values,table[:,0],table[:,j]) for j in (1,2)])
    centers=xy(mid);d=xy(mid+.5)-xy(mid-.5)
    unit=d/np.maximum(np.linalg.norm(d,axis=1)[:,None],1e-9);normal=np.column_stack((-unit[:,1],unit[:,0]))
    half=np.array([(road.width(lane,float(s)) if width is None else width)/2 for s in mid])
    state=[FREE]*n;by=['']*n;ext=[None]*n;mem=[[] for _ in range(n)]
    # Occupancy: every current body, whatever its track or speed state. A hull spans every box
    # between its end stations (its points are only the outline), laterally by interval overlap.
    for obj in snapshot.objects.values():
        pts=np.asarray(obj.xy,dtype=float)
        if len(pts)==0:continue
        st=stations(road,pts)
        k0=max(0,int(np.searchsorted(edges,st.min(),'right'))-1);k1=min(n-1,int(np.searchsorted(edges,st.max(),'right'))-1)
        if st.max()<edges[0] or st.min()>edges[-1]:continue
        for j in range(k0,k1+1):
            lat=(pts-centers[j])@normal[j]
            # A body counts once it reaches intrusion_m into the lane: a lane-2 car whose edge sat on
            # the 2/3 line filled lane-3 boxes and held a clear 4->3 change for 190 m (00:53 lap 3).
            if lat.min()<=half[j]-b.intrusion_m and lat.max()>=-half[j]+b.intrusion_m:
                if state[j]==FREE:state[j]=OCCUPIED;by[j]=obj.id
                lo_j,hi_j=max(float(st.min()),float(edges[j])),min(float(st.max()),float(edges[j+1]))
                ext[j]=(lo_j,hi_j) if ext[j] is None else (min(ext[j][0],lo_j),max(ext[j][1],hi_j));mem[j].append(obj.id)
    # Visibility: a box centre behind another body, seen from the sensor, is not known to be free.
    u=np.array([math.cos(ctx.yaw),math.sin(ctx.yaw)]);sensor=np.array([ctx.x,ctx.y])+u*b.sensor_forward_m
    shadows=[]
    for obj in snapshot.objects.values():
        rel=np.asarray(obj.xy,dtype=float)-sensor;r=np.linalg.norm(rel,axis=1)
        if len(rel)==0 or r.min()<.5:continue
        c=math.atan2(*rel.mean(axis=0)[::-1]);a=np.array([_wrap(math.atan2(y,x)-c) for x,y in rel])
        shadows.append((obj.id,c,a.min()-b.shadow_pad_rad,a.max()+b.shadow_pad_rad,float(r.min())))
    for j in range(n):
        if state[j]!=FREE:continue
        rel=centers[j]-sensor;rc=float(np.linalg.norm(rel));ang=math.atan2(rel[1],rel[0])
        ahead=float(rel@u)>=0
        if rc>(config.bounds.front_range_m if ahead else config.bounds.rear_range_m):
            state[j]=UNKNOWN;by[j]='RANGE';continue
        for key,c,a0,a1,rmin in shadows:
            if a0<=_wrap(ang-c)<=a1 and rc>rmin+lengths[j]/2:
                state[j]=UNKNOWN;by[j]=key;break
    if extents is not None:extents[:]=ext
    if members is not None:members[:]=mem
    boxes=[]
    for j in range(n):
        e=normal[j]*half[j];f=unit[j]*lengths[j]/2;c=centers[j]
        poly=[(c-f-e).tolist(),(c+f-e).tolist(),(c+f+e).tolist(),(c-f+e).tolist()]
        boxes.append((float(s0[j]),float(edges[j+1]),state[j],by[j],poly))
    return boxes


def run_bodies(config,ctx,boxes,runs,speeds,caster_speed):
    """Bodies in a lane for the slot planner, from its occupied runs (no object identity).

    measured   a run whose edge speed is known: that speed;
    unmeasured a run seen for under ~0.2 s: unknown_speed_mps faster than us behind, slower ahead;
    a run's edge next to hidden boxes extends over them (what is there moves with it); hidden boxes on their own
    move with whatever casts the shadow (caster_speed(key), our speed if unknown).
    Boxes beyond sensor range are not bodies: 40 m behind, a car closing at 8 m/s is seen 5 s before it reaches
    us, longer than a change takes (as a 20 m/s phantom it vetoed every change below ~60 km/h)."""
    b=config.boxes;out=[];owned=set()
    for (lo,hi,lo_hidden,hi_hidden,i,j),v in zip(runs,speeds):
        # A hidden edge: the run reaches over the adjacent hidden boxes.
        k=i
        while lo_hidden and k>0 and boxes[k-1][2]==UNKNOWN and boxes[k-1][3]!='RANGE':k-=1;owned.add(k);lo=min(lo,boxes[k][0])
        k=j
        while hi_hidden and k<len(boxes)-1 and boxes[k+1][2]==UNKNOWN and boxes[k+1][3]!='RANGE':k+=1;owned.add(k);hi=max(hi,boxes[k][1])
        err=uncertainty(config,abs((lo+hi)/2-ctx.s))
        if v is not None:kind='measured'
        else:v=ctx.v+b.unknown_speed_mps if hi<ctx.s else max(0.,ctx.v-b.unknown_speed_mps);kind='unmeasured'
        out.append(Body(f'run@{lo:.1f}',lo,hi,v,err,kind))
    run=None
    for idx,(s0,s1,state,key,_) in enumerate(list(boxes)+[(math.inf,math.inf,FREE,'',None)]):
        tag='shadow:'+key if state==UNKNOWN and key!='RANGE' and idx not in owned else None
        if run is not None and (tag!=run[2] or s0>run[1]+1e-6):
            lo,hi,tag0,caster=run
            v=caster_speed(caster);v=ctx.v if v is None else v
            out.append(Body(f'shadow@{lo:.1f}',lo,hi,v,0.,'shadow'));run=None
        if tag is not None:
            if run is None:run=[s0,s1,tag,key]
            else:run[1]=s1
    return out
