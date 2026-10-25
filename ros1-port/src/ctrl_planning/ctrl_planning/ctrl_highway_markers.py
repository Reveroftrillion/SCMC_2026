"""Current hulls, proposed curve and the actual blocking objects; compact HUD."""
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from ctrl_ros1.ctrl_messages import Marker


def highway_markers(runtime,stamp):
    arr=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
    if not runtime.active:return arr
    data=runtime.diagnostic
    def line(xy,color,ns,closed=False,width=.12):
        if not xy:return None
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns=ns;m.id=len(arr.markers)
        m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=width
        m.color.r,m.color.g,m.color.b=color;m.color.a=.95
        m.points=[Point(x=float(x),y=float(y),z=.3) for x,y in xy+([xy[0]] if closed else [])]
        arr.markers.append(m);return m
    def text_at(xy,value,ns,color=(1.,1.,1.),size=1.):
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns=ns;m.id=len(arr.markers)
        m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.
        m.pose.position.x=float(xy[0]);m.pose.position.y=float(xy[1]);m.pose.position.z=1.4
        m.scale.z=size;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=value
        arr.markers.append(m)
    scene=data.get('scene',{});gap=scene.get('gap')
    for lane in scene.get('lanes',[]):
        for edge in lane['edges']:
            line(edge,(.1,.65,1.) if lane['target'] else (.45,.45,.45),'highway_lane_edges',width=.07)
    if gap and gap['polygon']:
        polygon=gap['polygon'];n=len(polygon)
        fill=line(polygon,(.1,.65,1.),'highway_gap');fill.type=Marker.TRIANGLE_LIST
        fill.scale.x=fill.scale.y=fill.scale.z=1.;fill.color.a=.22;fill.points=[]
        for i in range(n//2-1):
            for j in (i,i+1,n-1-i,i+1,n-2-i,n-1-i):
                x,y=polygon[j];fill.points.append(Point(x=float(x),y=float(y),z=.12))
        color={'COMMITTED':(0.,1.,.3),'CLEAR_CONFIRM':(1.,.8,0.),'BLOCKED':(1.,.1,.1)}.get(gap['status'],(.5,.55,.65))
        line(polygon,color,'highway_gap_border',closed=True,width=.12)
        anchor=data.get('ego_xy',polygon[0]);h=data.get('ego_yaw',0.)
        anchor=[anchor[0]+10*math.cos(h),anchor[1]+10*math.sin(h)]
        j=min(range(n//2),key=lambda j:sum((polygon[j][i]-anchor[i])**2 for i in (0,1)))
        xy=[(polygon[j][i]+polygon[n-1-j][i])/2 for i in (0,1)]
        value=f"GAP NOW {gap['length_m']:.0f}m" if gap['bounded'] else 'GAP NOW / OPEN END'
        text_at(xy,value,'highway_gap_label',size=1.1)
    # The slot judgement: orange fill the box we close in on (PURSUE), green once committed, blue an ordinary target box;
    # thin white the fallback box (landing behind / between), red outline a pass just given up, magenta the target car.
    pursuit=scene.get('pursuit')
    if pursuit:
        fills={'PURSUE':(1.,.55,0.),'COMMIT':(0.,1.,.3),'ABORT':(1.,.1,.1),'TARGET':(.1,.65,1.)}
        poly=pursuit.get('slot')
        if poly:
            n=len(poly);fill=line(poly,fills.get(pursuit['state'],(.5,.5,.5)),'highway_slot');fill.type=Marker.TRIANGLE_LIST
            fill.scale.x=fill.scale.y=fill.scale.z=1.;fill.color.a=.45;fill.points=[]
            for i in range(n//2-1):
                for j in (i,i+1,n-1-i,i+1,n-2-i,n-1-i):
                    x,y=poly[j];fill.points.append(Point(x=float(x),y=float(y),z=.15))
        if pursuit.get('alt'):line(pursuit['alt'],(1.,1.,1.),'highway_slot_fallback',closed=True,width=.08)
        if pursuit.get('aborted'):line(pursuit['aborted'],(1.,.1,.1),'highway_slot_aborted',closed=True,width=.2)
        if pursuit.get('target'):line(pursuit['target'],(1.,0.,1.),'highway_slot_target',closed=True,width=.25)
        ego=data.get('ego_xy')
        if ego:
            extra=f" {pursuit['commit_scans']}/{pursuit['need']}" if pursuit['state']=='PURSUE' else ''
            text_at([ego[0],ego[1]+4.],f"{pursuit['state']} {pursuit['kind']}{extra}",'highway_slot_label',
                    fills.get(pursuit['state'],(1.,1.,1.)),size=1.3)
    boxes=data.get('boxes')
    if boxes:
        # Target-lane boxes: red occupied, green free, yellow not visible; white outline = gate region.
        colors={'OCCUPIED':(1.,.1,.1),'FREE':(0.,.9,.3),'UNKNOWN':(1.,.85,0.)}
        lo,hi=boxes['region']
        for state,color in colors.items():
            fill=Marker();fill.header.frame_id='map';fill.header.stamp=stamp;fill.ns='highway_boxes';fill.id=len(arr.markers)
            fill.type=Marker.TRIANGLE_LIST;fill.action=Marker.ADD;fill.pose.orientation.w=1.
            fill.scale.x=fill.scale.y=fill.scale.z=1.;fill.color.r,fill.color.g,fill.color.b=color;fill.color.a=.35
            for s0,s1,st,_,poly in boxes['boxes']:
                if st!=state:continue
                for j in (0,1,2,0,2,3):fill.points.append(Point(x=float(poly[j][0]),y=float(poly[j][1]),z=.08))
            if fill.points:arr.markers.append(fill)
        for s0,s1,st,_,poly in boxes['boxes']:
            if s1>lo and s0<hi:line(poly,(1.,1.,1.) if boxes['permitted'] else (1.,.1,.1),'highway_box_gate',closed=True,width=.06)
    line(data.get('fallback_path',[]),(.1,.9,1.),'highway_fallback_path',width=.28)
    committed=data.get('state') in ('CHANGE','SETTLE')
    line(data.get('change_path',[]),(0.,1.,.3) if committed else (1.,.8,0.),'highway_change_path',width=.22)
    blockers={r.split(':',1)[1] for r in data.get('blocked_by',[]) if ':' in r}
    ego=data.get('ego_xy');follow=data.get('follow_object')
    for obj in data.get('objects',[]):
        blocked=obj['id'] in blockers;leading=obj['id']==follow
        info=scene.get('objects',{}).get(obj['id'],{})
        role=info.get('role','OTHER')
        nearest=gap is not None and obj['id'] in (gap['front_id'],gap['rear_id'])
        color=(1.,.1,.1) if blocked else (1.,.55,0.) if leading else (.2,.65,1.) if nearest and role=='FRONT' else (.8,.35,1.) if nearest else (.5,.55,.65)
        xy=obj['xy'];line(xy,color,'highway_current_objects',closed=True)
        if not xy:continue
        center=[sum(p[i] for p in xy)/len(xy) for i in (0,1)]
        if info:
            speed=info.get('speed_kmh');suffix=f" {speed:.0f}km/h" if speed is not None else ' ?'
            label_role=role if nearest or blocked or leading or role=='SIDE' else ''
            text_at([center[0],center[1]+1.8],f"L{info['lane']} {label_role}{suffix}",'highway_object_role',color,size=1.2 if label_role else .8)
        if not (blocked or leading or nearest):continue
        if ego and (blocked or leading):line([ego,center],color,'highway_blocker' if blocked else 'highway_follow',width=.15)
        velocity=obj.get('velocity',[0.,0.]);speed=math.hypot(*velocity)
        if speed>.1:
            end=[center[i]+velocity[i]*min(1.,6./speed) for i in (0,1)]
            arrow=line([center,end],color,'highway_motion');arrow.type=Marker.ARROW
            arrow.scale.x=.2;arrow.scale.y=.45;arrow.scale.z=.5
    if data.get('state'):
        mode=data.get('mode','unknown').upper()
        label=Marker();label.header.frame_id='base_link';label.header.stamp=stamp
        label.ns='highway_status';label.id=0;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
        label.pose.orientation.w=1.;label.pose.position.x=-14.;label.pose.position.y=16.;label.pose.position.z=2.
        label.scale.z=1.8;label.color.r=1.;label.color.g=1.;label.color.b=1.;label.color.a=1.
        reason=data.get('reason','unknown').split(':',1)[0]
        lane=data.get('current_lane');target=data.get('target_lane')
        change=f' {lane} -> {target}' if lane is not None and target is not None else ''
        label.text=f"HIGHWAY {data['state']}{change} [{mode}]\n{reason}"
        control=data.get('control',{})
        if mode=='CONTROL' and control.get('state') in ('FAULT','HOLD'):
            label.text=f"HIGHWAY CONTROL {control['state']}\n{control.get('reason','')}"
            label.color.g=.2;label.color.b=.2
        arr.markers.append(label)
    return arr
