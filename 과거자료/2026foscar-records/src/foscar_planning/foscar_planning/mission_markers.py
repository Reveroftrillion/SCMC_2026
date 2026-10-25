"""Mission-scoped, read-only decision display. Never invent clearance evidence."""
import math
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from .state import event_key
from .frenet import to_xy
from .measured_corridor import STOP_CLEARANCE_M

GREEN=(.2,1.,.4)
RED=(1.,.15,.1)
AMBER=(1.,.65,.1)
WHITE=(.9,.95,1.)


def checkpoint_markers(route, zones, s_offset, stamp):
    """Draw the actual FSM boundaries on the currently selected map route."""
    arr=MarkerArray()
    if route is None or s_offset is None or not math.isfinite(s_offset):return arr
    names={'STATIC_OBSTACLE':'BOX','PEDESTRIAN':'PED','ROUNDABOUT':'ROUNDABOUT','SIGNAL_1':'SIG1','SIGNAL_4':'SIG4',
           'HIGHWAY':'HIGHWAY','TUNNEL':'TUNNEL'}
    boundaries={}
    for zone in zones:
        for station,kind in ((zone.start_s,'IN'),(zone.end_s,'OUT')):
            boundaries.setdefault(station,[]).append((names.get(zone.name,zone.name),kind))
    for number,(station,entries) in enumerate(sorted(boundaries.items()),1):
        local=station+s_offset
        # Route.at clamps; an off-route checkpoint must not pile up at an end.
        if not 0<=local<=route.s[-1]:continue
        color=(.85,.35,1.)
        line=Marker();line.header.frame_id='map';line.header.stamp=stamp
        line.ns='mission_checkpoints';line.id=number;line.type=Marker.LINE_STRIP
        line.pose.orientation.w=1.;line.scale.x=.45
        line.color=ColorRGBA(r=color[0],g=color[1],b=color[2],a=1.)
        line.lifetime=Duration(sec=1)
        line.points=[Point(x=float(x),y=float(y),z=.35)
                     for x,y in (to_xy(route,local,-5.),to_xy(route,local,5.))]
        label=Marker();label.header=line.header;label.ns='mission_checkpoint_labels';label.id=number
        label.type=Marker.TEXT_VIEW_FACING;label.pose.orientation.w=1.
        x,y=to_xy(route,local,7.)
        label.pose.position=Point(x=float(x),y=float(y),z=1.8)
        label.scale.z=1.8;label.color=line.color;label.lifetime=line.lifetime
        label.text='FSM '+ ' / '.join(name+' '+kind for name,kind in entries)
        arr.markers.extend([line,label])
    return arr


def required_checkpoint_markers(checkpoints, stamp):
    """Official world coordinates and acceptance radius, independent of route projection."""
    arr=MarkerArray()
    for number,cp in sorted(checkpoints.items()):
        x,y=map(float,cp['xy']);radius=float(cp['radius'])
        if not all(math.isfinite(v) for v in (x,y,radius)) or radius<=0:continue
        ring=Marker();ring.header.frame_id='map';ring.header.stamp=stamp
        ring.ns='required_checkpoints';ring.id=int(number);ring.type=Marker.LINE_STRIP
        ring.pose.orientation.w=1.;ring.scale.x=.22
        ring.color=ColorRGBA(r=.1,g=.95,b=1.,a=1.)
        ring.lifetime=Duration(sec=1)
        ring.points=[Point(x=x+radius*math.cos(i*math.tau/48),
                           y=y+radius*math.sin(i*math.tau/48),z=.45) for i in range(49)]
        label=Marker();label.header=ring.header;label.ns='required_checkpoint_labels';label.id=int(number)
        label.type=Marker.TEXT_VIEW_FACING;label.pose.orientation.w=1.
        label.pose.position=Point(x=x,y=y,z=2.5)
        label.scale.z=1.5;label.color=ring.color;label.lifetime=ring.lifetime
        label.text=f'CP{number:02d}'
        arr.markers.extend([ring,label])
    return arr


def yield_status(behavior, ctx):
    y=next((s for s in behavior.states if s.name=='YIELD' and behavior.engaged[s.name]),None)
    if y is None:
        return dict(kind='NONE',phase='INACTIVE',reason='',target_type=None,progress=None)
    kind=('PEDESTRIAN' if y.target_type=='PEDESTRIAN' else
          'OBJECT_GUARD' if y.target_type=='OBSTACLE' else 'VEHICLE_GUARD')
    observed=[e for e in ctx.events+ctx.observations
              if (event_key(e)==y.target_key if y.target_key is not None else abs(e.s-y.target_s)<5.)]
    phase='OCCUPIED' if observed else 'OBSERVATION_MISSING'
    progress=None
    if y.clear_since is not None:
        phase='CLEAR_CONFIRMATION'
        # Use successive observation stamps, exactly as Yield.check_exit does.
        progress=min(1.,max(0.,((y.last_seen or y.clear_since)-y.clear_since)/max(.001,y.clear_s)))
    return dict(kind=kind,phase=phase,reason=y.reason,target_type=y.target_type,
                target_s=y.target_s,progress=progress,
                object_ids=[e.value for e in observed])


def mission_markers(ctx, target, behavior, result, stamp, control=None, gate_reason=''):
    arr=MarkerArray(); clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
    if not all(math.isfinite(v) for v in (ctx.x,ctx.y,ctx.yaw,ctx.s)):
        return arr
    def marker(ns, kind, color):
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp
        m.ns=ns;m.id=len(arr.markers);m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color=ColorRGBA(r=color[0],g=color[1],b=color[2],a=1.)
        m.lifetime=Duration(sec=1);arr.markers.append(m);return m
    def line(ns, points, color, width=.12):
        m=marker(ns,Marker.LINE_STRIP,color);m.scale.x=float(width)
        m.points=[Point(x=float(x),y=float(y),z=.25) for x,y in points]
    y=yield_status(behavior,ctx)
    max_age=result.get('display_max_age_s',.45)
    fresh=all(0<=ctx.now-result.get(k,-math.inf)<=max_age for k in ('evidence_stamp','evaluated_at'))
    state=target.state
    color=RED if target.v_max<=0. or math.isfinite(target.stop_s) else GREEN
    detail=''
    if y['kind']!='NONE':
        detail=y['kind'].replace('_',' ')+' / '+y['phase'].replace('_',' ')
        if y['phase']!='OCCUPIED':color=AMBER
        for e in ctx.events+ctx.observations:
            if e.value not in y.get('object_ids',[]):continue
            h=e.box_yaw if math.isfinite(e.box_yaw) else 0.
            c,s=math.cos(h),math.sin(h);l,w=max(.1,e.size_x)/2,max(.1,e.size_y)/2
            pts=[(e.x+u*c-v*s,e.y+u*s+v*c) for u,v in ((-l,-w),(l,-w),(l,w),(-l,w),(-l,-w))]
            line('yield_object',pts,RED)
            line('yield_pointer',[(ctx.x,ctx.y),(e.x,e.y)],RED,.08)
            if ctx.mission_name=='PEDESTRIAN' and e.type=='PEDESTRIAN':
                ystate=next(s for s in behavior.states if s.name=='YIELD')
                line('pedestrian_motion',[(e.x,e.y),(e.x+e.vx*ystate.predict_s,e.y+e.vy*ystate.predict_s)],AMBER,.08)
        if ctx.mission_name=='PEDESTRIAN' and ctx.route is not None and math.isfinite(y.get('target_s',math.nan)):
            # This is the ego path's conflict corridor, not a free-space certificate.
            s0=y['target_s']+ctx.s_offset
            line('pedestrian_conflict_corridor',[to_xy(ctx.route,s0+ds,d)
                 for ds,d in ((-1.,-1.3),(1.,-1.3),(1.,1.3),(-1.,1.3),(-1.,-1.3))],color)
        if y['progress'] is not None:
            c,s=math.cos(ctx.yaw),math.sin(ctx.yaw)
            a=(ctx.x-3*c-2*s,ctx.y-3*s+2*c)
            b=(a[0]+4*c,a[1]+4*s)
            line('pedestrian_clear_progress_base',[a,b],AMBER,.25)
            line('pedestrian_clear_progress',[a,(a[0]+4*c*y['progress'],a[1]+4*s*y['progress'])],GREEN,.3)
    elif ctx.mission_name=='ROUNDABOUT':
        if result.get('display_gate_enabled') is False:
            detail='ENTRY CONTROL OFF';color=AMBER
        elif ctx.vehicle_following_active:
            gap=result.get('following_gap_m')
            reference=result.get('following_reference_s')
            if gap is not None and reference is not None:
                gap=max(0.,reference+gap-ctx.s)
            detail='FRONT '+(f'{gap:.1f}m' if gap is not None else 'NO MEASURED OBJECT')
            if ctx.route is not None:
                boundary=ctx.s+ctx.s_offset+3.8+STOP_CLEARANCE_M
                line('following_stop_boundary',[to_xy(ctx.route,boundary,d) for d in (-1.196,1.196)],AMBER)
                if gap is not None and reference is not None:
                    surface=reference+ctx.s_offset+3.8+result['following_gap_m']
                    line('following_surface',[to_xy(ctx.route,surface,d) for d in (-1.196,1.196)],AMBER if not fresh else RED if gap<=STOP_CLEARANCE_M else GREEN)
        elif target.state in ('ROUNDABOUT_ENTER','ROUNDABOUT_FOLLOW'):
            detail='GO'
        elif gate_reason=='exited':detail='EXIT COMPLETE'
        else:
            side=result.get('conflict_side')
            detail='WAIT / '+({'rear':'REAR','front':'FRONT','other':'OBJECT'}.get(side,'VERIFYING'))
            if result.get('decision')=='UNKNOWN':color=AMBER
        if gate_reason=='imminent conflict':
            detail='COLLISION GUARD' if fresh else 'OBSERVATION STALE'
            color=RED if fresh else AMBER
    elif ctx.mission_name=='TUNNEL':
        detail='LOCALIZATION '+ctx.loc_mode
        if target.state=='AVOID':
            detail+=' / MAP AVOID '+('STOP' if math.isfinite(target.stop_s) or target.v_max<=0 else 'LEFT' if target.lat_offset>0 else 'RIGHT' if target.lat_offset<0 else 'PREPARE')
    elif ctx.mission_name=='STATIC_OBSTACLE':
        a=next((s for s in behavior.states if s.name=='AVOID' and behavior.engaged[s.name]),None)
        if a is not None:
            detail='RETURN' if math.isfinite(target.return_s) else 'AVOID'
            for station,ns in ((target.entry_s,'avoid_entry'),(target.return_s,'avoid_return')):
                if math.isfinite(station) and ctx.route is not None:
                    line(ns,[to_xy(ctx.route,station+ctx.s_offset,d) for d in (-1.5,1.5)],GREEN)
        else:detail='PATH FOLLOWING'
    elif ctx.mission_name=='PEDESTRIAN':detail='NO YIELD REQUEST'
    cap='NONE' if math.isinf(target.v_max) else f'{target.v_max*3.6:.0f}'
    control=control or {}
    if 0<=ctx.now-control.get('stamp',-math.inf)<=.5:
        speed=f'V {ctx.v*3.6:.0f} -> {control["target_speed_mps"]*3.6:.0f} km/h / {control.get("limit_source","CONTROL")}'
    else:speed=f'V {ctx.v*3.6:.0f} km/h / CAP {cap}'
    panel=marker('mission_status',Marker.TEXT_VIEW_FACING,color)
    panel.pose.position.x=float(ctx.x-5*math.sin(ctx.yaw))
    panel.pose.position.y=float(ctx.y+5*math.cos(ctx.yaw));panel.pose.position.z=2.
    panel.scale.z=1.05
    panel.text='\n'.join(x for x in (f'{ctx.mission_name or "LEGACY"} / {state}',speed,detail) if x)
    return arr


def bev_stop_markers(ctx, target, diagnostic, stamp):
    """Display the exact guard decision; geometry is evidence, not a new check."""
    if not diagnostic:
        return []
    reason=diagnostic['reason']
    colors={'occupied':RED,'predicted occupancy':(1.,.2,1.),
            'unobserved':(.3,.6,1.),'stale scan':AMBER,'planner hold':AMBER,'clear':GREEN}
    color=colors[reason]
    labels={'occupied':'OCCUPIED','predicted occupancy':'PREDICTED',
            'unobserved':'UNOBSERVED','stale scan':'STALE SCAN','planner hold':'PLANNER HOLD','clear':'CLEAR'}
    out=[]
    def marker(ns,kind,rgb):
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp
        m.ns=ns;m.id=0;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.
        m.lifetime=Duration(sec=1);m.color=ColorRGBA(r=float(rgb[0]),g=float(rgb[1]),b=float(rgb[2]),a=.95)
        out.append(m);return m
    status='STOP' if diagnostic['cap']==0. else 'LIMIT' if reason!='clear' else 'CLEAR'
    m=marker('bev_stop_reason',Marker.TEXT_VIEW_FACING,color)
    m.pose.position=Point(x=float(ctx.x),y=float(ctx.y),z=3.5);m.scale.z=.65
    m.text=f"BEV {status}: {labels[reason]}\nFSM: {target.state}\ncap {diagnostic['cap']*3.6:.1f} km/h | scan age {diagnostic['scan_age']:.2f}s"
    if 'distance' in diagnostic:m.text+=f"\nconflict {diagnostic['distance']:.2f}m along path"
    if 'velocity' in diagnostic:
        vx,vy=diagnostic['velocity'];sx,sy=diagnostic['sigma']
        dx,dy=diagnostic['displacement']
        m.text+=f"\nobject {math.hypot(vx,vy)*3.6:.2f} km/h | sigma xy {sx*3.6:.2f}, {sy*3.6:.2f} km/h"
        m.text+=f"\nprediction dx [{dx[0]:+.2f},{dx[1]:+.2f}] dy [{dy[0]:+.2f},{dy[1]:+.2f}]m"
    for name,key,rgb in [('body','body',WHITE),('observed','region',AMBER),('prediction','prediction',color)]:
        polygon=diagnostic.get(key)
        if polygon:
            line=marker('bev_stop_'+name,Marker.LINE_STRIP,rgb);line.scale.x=.12
            line.points=[Point(x=float(x),y=float(y),z=.3) for x,y in (*polygon,polygon[0])]
    if diagnostic.get('cells'):
        cells=marker('bev_stop_cells',Marker.CUBE_LIST,color)
        cells.scale.x=cells.scale.y=float(diagnostic['resolution']);cells.scale.z=.2
        cells.points=[Point(x=float(x),y=float(y),z=.3) for x,y in diagnostic['cells']]
    return out
