"""Read-only RViz geometry. A visible gap is not an independent GO decision."""
import math
import numpy as np
from .highway_maneuver import motion


def highway_scene(policy,ctx,snapshot):
    scene=dict(lanes=[],objects={},gap=None)
    road=policy.route;lane=policy.current_lane;p=policy.config.policy
    if lane is None or ctx.s>=road.rejoin_s:return scene
    maneuver=policy.committed or policy.candidate
    target=maneuver.target_lane if maneuver else road.lanes[min(road.lanes.index(lane)+1,len(road.lanes)-1)]
    def station(points):
        ss=road.project_many(points)
        # Preserve longitudinal position for upstream returns beyond the ego ramp.
        for i,(point,s) in enumerate(zip(points,ss)):
            h=road.original.heading_at(s)
            ss[i]+=float((point-np.array(road.original.at(s)))@np.array([math.cos(h),math.sin(h)]))
        return ss
    def sections(lane,stations):
        table=road.observation_tables[lane]
        ss=np.clip(np.asarray(stations,dtype=float),table[0,0],table[-1,0])
        def xy(values):return np.column_stack([np.interp(values,table[:,0],table[:,j]) for j in (1,2)])
        centers=xy(ss);delta=xy(np.clip(ss+.5,table[0,0],table[-1,0]))-xy(np.clip(ss-.5,table[0,0],table[-1,0]))
        unit=delta/np.maximum(np.linalg.norm(delta,axis=1)[:,None],1e-9)
        normals=np.column_stack((-unit[:,1],unit[:,0]))
        widths=np.array([road.width(lane,float(s)) for s in ss])
        return centers,normals,widths
    def cross_section(lane,s):
        xy,n,width=sections(lane,[s])
        return xy[0],n[0],width[0]
    lo=max(ctx.s-p.rear_roi_m,road.observation_tables[target][0,0])
    hi=min(ctx.s+p.front_roi_m,road.tables[target][-1,0])
    for number in dict.fromkeys((lane,target)):
        table=road.observation_tables[number]
        a=max(lo,table[0,0]);b=min(hi,table[-1,0])
        if a>=b:continue
        xy,n,width=sections(number,np.linspace(a,b,max(2,int((b-a)/2)+1)))
        edges=[(xy+sign*n*width[:,None]/2).tolist() for sign in (-1,1)]
        scene['lanes'].append(dict(lane=number,target=number==target,edges=edges))
    front=rear=None;overlap=False
    for obj in snapshot.objects.values():
        ss=station(obj.xy);mid=float(np.mean(ss));choices=[];target_overlap=False
        for number in road.lanes:
            xy,n,width=cross_section(number,mid);d=(obj.xy-xy)@n
            choices.append((abs(float(np.mean(d))),number))
            if number==target:target_overlap=bool(d.min()<=width/2 and d.max()>=-width/2)
        number=min(choices)[1]
        role='OTHER'
        if target_overlap:
            role='FRONT' if ss.min()>ctx.s else 'REAR' if ss.max()<ctx.s else 'SIDE'
            if role=='FRONT' and (front is None or ss.min()<front[0]):front=(float(ss.min()),obj.id)
            if role=='REAR' and (rear is None or ss.max()>rear[0]):rear=(float(ss.max()),obj.id)
            overlap=overlap or role=='SIDE'
        speed=float(np.linalg.norm(motion(obj)))*3.6 if obj.speed_ready else None
        scene['objects'][obj.id]=dict(lane=number,role=role,speed_kmh=speed)
    # The slot judgement (user 2026-10-08: which box we aim at, visible in RViz): the box aimed at, the fallback, the
    # target car, a pass just given up. Stations now, on the target lane.
    view=getattr(policy,'pursuit_view',None)
    if view:
        t0,t1=float(road.observation_tables[target][0,0]),float(road.observation_tables[target][-1,0])
        def band(r,frac):
            if not r:return None
            a=max(r[0],t0,lo);b=min(r[1],t1,hi)
            if b<=a:return None
            xy,n,width=sections(target,np.linspace(a,b,max(2,int((b-a)/2)+1)))
            return (xy+n*width[:,None]*frac).tolist()+(xy-n*width[:,None]*frac)[::-1].tolist()
        scene['pursuit']=dict(state=view['state'],kind=view['kind'],commit_scans=view['commit_scans'],need=view['need'],
            slot=band(view['slot'],.42),alt=band(view['alt'],.3),target=band(view['target'],.5),aborted=band(view['aborted'],.36))
    if not snapshot.valid or not 0<=ctx.now-snapshot.stamp<=p.max_scan_age_s or hi<=lo:return scene
    start=max(lo,rear[0] if rear else lo);end=min(hi,front[0] if front else hi)
    assessment=policy.last_assessment
    status=('COMMITTED' if policy.committed else
            'CLEAR_CONFIRM' if assessment is not None and assessment.permitted else
            'BLOCKED' if assessment is not None else 'UNASSESSED')
    if overlap:end=start
    polygon=[]
    if end>start:
        xy,n,width=sections(target,np.linspace(start,end,max(2,int((end-start)/2)+1)))
        left=(xy+n*width[:,None]*.42).tolist();right=(xy-n*width[:,None]*.42).tolist()
        polygon=left+right[::-1]
    scene['gap']=dict(polygon=polygon,length_m=max(0.,end-start),status=status,
        front_id=front[1] if front else None,rear_id=rear[1] if rear else None,
        bounded=front is not None and rear is not None,occupied=overlap)
    return scene
