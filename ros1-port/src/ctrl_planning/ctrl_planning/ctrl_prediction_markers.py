"""Read-only RViz diagnostics of measured boxes and actual prediction hypotheses."""
import math
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray
from ctrl_ros1.ctrl_messages import Marker
from rospy import Duration


COLORS={'OBSTACLE':(1.,.65,.1),'PEDESTRIAN':(1.,.2,.3),
        'VEHICLE':(.25,.6,1.),'DYNAMIC_UNKNOWN':(.8,.3,1.)}


def blocking_ids(result):
    if result.get('decision') not in ('WAIT','UNKNOWN'):return []
    if result.get('track'):return [result['track']]
    pair=result.get('gap_pair',[]);side=result.get('conflict_side')
    if len(pair)==2 and side in ('rear','front'):return [pair[0 if side=='rear' else 1]]
    return result.get('uncertain_ids',[]) or result.get('off_circle_ids',[])


def decision_label(ctx,result,target,gate_reason):
    reason=result.get('reason','')
    titles={
        'rear following clearance exhausted after handoff':'REAR TOO CLOSE AFTER MERGE',
        'entry corridor visibility or joining interval not verified':'EMPTY ROAD / JOINING TIME NOT VERIFIED',
        'single vehicle motion not fresh and ready':'VEHICLE SPEED NOT CONFIRMED',
        'no fresh bounded gap in observed arc':'GAP BOUNDARIES NOT CONFIRMED',
        'another observed or occluded body occupies selected gap':'OTHER OBJECT OCCUPIES GAP',
        'vehicle outside circular model':'OBJECT OUTSIDE RING MODEL',
        'entry does not reach stable lane alignment':'CANNOT FINISH ALIGNMENT IN PREVIEW',
        'lidar observation updates unavailable':'LIDAR DATA MISSING',
    }
    title=titles.get(reason,reason.upper() or 'NO DECISION DATA')
    if reason=='ego does not fit between circulating bounds throughout entry':
        title={'rear':'REAR ARRIVES BEFORE WE CLEAR','front':'TOO CLOSE TO LEAD'}.get(result.get('conflict_side'),'INSUFFICIENT GAP')
    decision=result.get('decision','UNKNOWN')
    color=(1.,.2,.15) if decision=='WAIT' else (1.,.7,.1)
    if result.get('upstream_stop'):
        title='OTHER STOP: '+result.get('upstream_state','UNKNOWN');color=(1.,.2,.15)
    elif gate_reason=='unmodeled lateral path':title='AVOIDANCE PATH NOT COVERED BY ENTRY MODEL'
    elif target.state in ('ROUNDABOUT_ENTER','ROUNDABOUT_FOLLOW'):
        title='ENTERING / FOLLOW LEAD';color=(.2,1.,.3)
    elif gate_reason=='exited':title='EXIT COMPLETE';color=(.2,1.,.3)
    elif decision=='CANDIDATE_GAP':
        title='GAP AVAILABLE' if gate_reason=='fresh gap confirmed' else 'GAP CANDIDATE / GATE NOT CONFIRMED'
        if gate_reason=='fresh gap confirmed':color=(.2,1.,.3)
        elif result.get('admission_model')=='three_vehicle_entry' and (
                ctx.now-result.get('evidence_stamp',-math.inf)>.25):
            title='GAP REJECTED / OBSERVATION TOO OLD'
    if target.state=='ROUNDABOUT_BRAKE':
        title='BRAKING / '+title;color=(1.,.2,.15)
    if result.get('gate_enabled') is False:title='MONITOR ONLY / '+title
    ids=blocking_ids(result)
    lines=[title,f'{target.state} | predictor: {decision}',f'gate: {gate_reason or "-"}']
    if ids:lines.append('BLOCKER '+', '.join('#'+i.split(':')[-1] for i in ids))
    for key,label,unit in (('minimum_gap_margin_m','Entry clearance','m'),
                           ('handoff_rear_reserve_m','Rear clearance after merge','m'),
                           ('conflict_in_s','Conflict in','s'),('following_gap_m','Lead gap','m')):
        if result.get(key) is not None:lines.append(f'{label}: {result[key]:+.2f}{unit}')
    if result.get('following_model')=='measured_route_clearance':
        lines.append('CONTROL: OBSERVED ROUTE DISTANCE')
        if result.get('boundary_speed_mps') is not None:
            lines.append(f'Surface motion: {result["boundary_speed_mps"]*3.6:.1f}km/h')
    if 'evaluated_at' in result:lines.append(f'Prediction age: {max(0.,ctx.now-result["evaluated_at"]):.2f}s')
    if math.isfinite(result.get('evidence_stamp',-math.inf)):
        lines.append(f'Observation age: {ctx.now-result["evidence_stamp"]:.2f}s')
    lines.append(f'Now {ctx.v*3.6:.1f} | command cap {target.v_max*3.6:.1f} km/h')
    lines.append(reason)
    return '\n'.join(lines),color


def prediction_markers(predictor,ctx,result,target,stamp,gate_reason='',circle=None,show_future=False,show_details=False):
    arr=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
    if ctx.mission_name and (ctx.mission_name != 'ROUNDABOUT' or result.get('gate_enabled') is False) and not show_details:
        return arr
    def marker(ns,kind,color,alpha=1.):
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp
        m.ns=ns;m.id=len(arr.markers);m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color=ColorRGBA(r=float(color[0]),g=float(color[1]),b=float(color[2]),a=float(alpha))
        m.lifetime=Duration(secs=1);arr.markers.append(m);return m
    def line(ns,points,color,width=.05,alpha=.6):
        m=marker(ns,Marker.LINE_STRIP,color,alpha);m.scale.x=float(width)
        m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2]) if len(p)>2 else .2) for p in points]
        return m
    def box(ns,x,y,h,length,width,color,alpha=.15,z=.2):
        m=marker(ns,Marker.CUBE,color,alpha)
        m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=float(z)
        m.pose.orientation.z=math.sin(h/2);m.pose.orientation.w=math.cos(h/2)
        m.scale.x=float(max(.05,length));m.scale.y=float(max(.05,width));m.scale.z=.08
        return m
    def text(ns,x,y,label,color=(1.,1.,1.),size=.5):
        if not show_details:
            if ns not in ('decision','rear_decision_panel'):return
            size=1.05
            if ns=='decision':
                if result.get('upstream_stop'):label='OTHER STOP'
                elif target.state=='ROUNDABOUT_BRAKE':label='BRAKE'
                elif target.state in ('ROUNDABOUT_ENTER','ROUNDABOUT_FOLLOW'):label='GO'
                elif result.get('decision')=='EXITED':label='EXIT'
                elif result.get('decision')=='CANDIDATE_GAP':label='GAP OK'
                else:
                    side=result.get('conflict_side')
                    label='WAIT: '+({'rear':'REAR','front':'FRONT','other':'OBJECT'}.get(side,'CHECKING'))
            else:
                rear=result.get('rear_check')
                speed=rear['speed_mps'] if rear else result.get('single_rear_speed_mps',0.)
                lines=[f'REAR {speed*3.6:.0f} km/h']
                if 'rear_conflict_in_s' in result:lines.append(f'HIT IN {result["rear_conflict_in_s"]:.1f}s')
                if rear:
                    room=min(rear['clearance_m'],result.get('handoff_rear_reserve_m',math.inf))
                    after=room<rear['clearance_m'] and room<=0.
                    lines.append(f'{"AFTER SHORT" if after else "SHORT" if room<=0 else "SPARE"} {abs(room):.1f}m')
                label='\n'.join(lines)
        m=marker(ns,Marker.TEXT_VIEW_FACING,color)
        m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=1.5
        m.scale.z=float(size);m.text=label
    blockers=set(blocking_ids(result))
    # Every currently observed road user, including observation-only tracks.
    for e in ctx.events+ctx.observations:
        if e.type not in COLORS:continue
        h=e.box_yaw if math.isfinite(e.box_yaw) else 0.
        age=max(0.,ctx.now-e.stamp)
        observed_color=(.5,.5,.5) if age>.3 else COLORS[e.type]
        ring_row=next((r for r in result.get('occupied_intervals',[]) if r['id']==e.value and r.get('polygon_xy')),None)
        if ring_row is None and result.get('model')!='circular_gap':
            box('observed_box',e.x,e.y,h,e.size_x,e.size_y,observed_color,.1 if age>.3 else .28)
        if e.value in blockers:
            if result.get('model')=='circular_gap':
                line('blocking_object',[(e.x-.5,e.y-.5),(e.x+.5,e.y+.5)],(1.,0.,0.),.15,1.)
            else:box('blocking_object',e.x,e.y,h,e.size_x+.3,e.size_y+.3,(1.,0.,0.),.5)
            line('blocker_pointer',[(ctx.x,ctx.y),(e.x,e.y)],(1.,.1,.1),.12,1.)
        mode='CLASS UNCONFIRMED' if e.type=='OBSTACLE' and e.speed>.5 else 'STATIC CANDIDATE' if e.type=='OBSTACLE' else 'DYNAMIC'
        text('observed_label',e.x,e.y+2.,
             f'#{e.value.split(":")[-1]} {e.type} [{mode}]\n'
             +('VISIBLE OUTLINE / ENDS\n' if result.get('model')=='circular_gap' else
               f'MEASURED AABB {e.size_x:.2f}x{e.size_y:.2f}m  {e.speed*3.6:.1f}km/h\n')+
             f'{"PATH" if e.path_relevant else "OBS ONLY"}  age={max(0.,ctx.now-e.stamp):.2f}s',COLORS[e.type])
    bridged=set(result.get('bridged_ids',[]))
    circular = result.get('model') == 'circular_gap'
    for tr in ([] if circular else predictor.tracks.values()):
        e=tr.event;age=max(0.,ctx.now-e.stamp)
        color=(1.,.15,.15) if result.get('track')==e.value else COLORS.get(e.type,(.8,.3,1.))
        positions=list(tr.poses(age))
        if not positions:continue
        x,y,h=positions[0];margin=predictor.body_margin(tr,0.,age,e.value in bridged)
        box('prediction_body',x,y,h,tr.length+2*margin,tr.width+2*margin,color,.12,z=.35)
        reasons=','.join(result.get('uncertainties',{}).get(e.value,[])) or ('ready' if tr.ready else 'cold')
        text('model_label',x,y-2.,f'#{e.value.split(":")[-1]} MODEL {tr.length:.2f}x{tr.width:.2f}m\n'
             f'+margin {margin:.2f}m/side  {reasons}'+(' [BRIDGED]' if e.value in bridged else ''),color,.42)
        # All hypotheses are displayed; the first map candidate is not a unique prediction.
        paths={}
        for dt in (np.arange(0.,predictor.horizon+.01,.5) if show_future else ()):
            for kind,key,p in tr.pose_hypotheses(float(dt)+age):
                paths.setdefault((kind,key),[]).append(p[:2])
        for (kind,key),points in paths.items():
            line('lane_hypotheses' if kind=='lane' else 'motion_hypotheses',points,
                 color if kind=='lane' else (.9,.55,.15),.045,.45)
        for dt in ((1.,2.,3.) if show_future else ()):
            margin=predictor.body_margin(tr,dt,age,e.value in bridged)
            for k,(px,py,ph) in enumerate(tr.poses(age+dt)):
                box('object_future',px,py,ph,tr.length+2*margin,tr.width+2*margin,color,.07,z=.3)
                if k==0:text('future_label',px,py,f'#{e.value.split(":")[-1]} +{dt:.0f}s [candidate 1]',color,.38)
        if circle is not None:
            distance=math.hypot(x-circle.center[0],y-circle.center[1])
            if abs(distance-circle.radius)<=3.5:
                a=math.atan2(y-circle.center[1],x-circle.center[0])
                span=(tr.radius+predictor.body_margin(tr,0.,age,e.value in bridged))/circle.radius
                line('ring_occupied',circle.arc(a-span,a+span,20),color,.3,.8)
    if circular and circle is not None:
        reference=result.get('reference_angle',0.)
        def ring_arc(lo,hi):
            return circle.arc(circle.direction*(reference+lo/circle.radius),
                              circle.direction*(reference+hi/circle.radius),30)
        rear=result.get('rear_check')
        if rear:
            room=rear['clearance_m'];blocked=room<=0.
            after_blocked=result.get('handoff_rear_reserve_m',math.inf)<=0.
            color=(1.,.15,.1) if blocked or after_blocked else (.2,1.,.4)
            label=(f'REAR #{rear["id"].split(":")[-1]}: {rear["speed_mps"]*3.6:.1f} km/h\n'
                   f'ENTRY CHECK: {"SHORT BY" if blocked else "SPARE"} {abs(room):.2f} m\n'
                   f'Worst point +{rear["at_s"]:.1f}s | ego accel {rear["ego_accel_mps2"]:.1f}m/s2\n'
                   f'Body gap {rear["body_gap_m"]:.2f}m / required margin {rear["required_margin_m"]:.2f}m\n'
                   f'Margin: base {rear["base_margin_m"]:.2f} + future {rear["future_margin_m"]:.2f}\n'
                   f'+ age {rear["age_margin_m"]:.2f} + ego {rear["ego_margin_m"]:.2f} m\n'
                   f'Front end: {rear["boundary_source"]}, age {rear["age_s"]:.2f}s')
            if rear.get('hidden_extension_m') is not None:
                label+=f'\nHidden length beyond visible end: {rear["hidden_extension_m"]:.2f}m'
            if 'rear_conflict_in_s' in result:label+=f'\nFirst rear overlap: +{result["rear_conflict_in_s"]:.1f}s'
            if 'handoff_rear_reserve_m' in result:
                label+=f'\nAFTER MERGE {"BLOCKED" if after_blocked else "CLEAR"}: {result["handoff_rear_reserve_m"]:+.2f}m'
            label+=f'\nRadial occupied width: {rear["radial_width_m"]:.2f}m'
            label+='\nGREEN=visible end  PURPLE=hidden extension\nWHITE=model end  ORANGE=margin  CYAN=ego rear\nAll marks projected to the SAME worst-point time'
            text('rear_decision_panel',ctx.x+7.*math.sin(ctx.yaw),ctx.y-7.*math.cos(ctx.yaw),label,color,.62)
            line('rear_safety_margin',ring_arc(rear['model_front_s'],rear['guard_front_s']),(1.,.55,.05),.55,1.)
            if rear.get('observed_front_s') is not None:
                line('rear_hidden_extension',ring_arc(rear['observed_front_s'],rear['model_front_s']),(.8,.2,1.),.3,.9)
                a=circle.direction*(reference+rear['observed_front_s']/circle.radius)
                direction=np.array([math.cos(a),math.sin(a)])
                line('rear_visible_end',[circle.center+(circle.radius+v)*direction for v in (-1.4,1.4)],(.2,1.,.4),.13,1.)
            for key,ns,c in (('model_front_s','rear_body_end',(1.,1.,1.)),
                             ('guard_front_s','rear_guard_end',(1.,.55,.05)),
                             ('ego_rear_s','ego_rear_at_check',(0.,1.,1.))):
                a=circle.direction*(reference+rear[key]/circle.radius)
                direction=np.array([math.cos(a),math.sin(a)])
                line(ns,[circle.center+(circle.radius+v)*direction for v in (-1.4,1.4)],c,.15,1.)
            line('rear_clearance_at_check',ring_arc(rear['guard_front_s'],rear['ego_guard_rear_s']),color,.2,1.)
        elif result.get('rear_only') and result.get('single_rear_speed_mps') is not None:
            text('rear_decision_panel',ctx.x+7.*math.sin(ctx.yaw),ctx.y-7.*math.cos(ctx.yaw),
                 f'REAR ONLY: {result["single_rear_speed_mps"]*3.6:.1f} km/h\n'
                 f'Observation age: {result["single_rear_age_s"]:.2f}s\n'
                 f'{result["decision"]}: {result.get("reason", "")}\n'
                 f'Rear margin: {result.get("single_rear_margin_m",.35):.2f} + {result.get("single_rear_growth_mps",.1):.2f} x future + 0.35 x age m\n'
                 'Single-car + visible-front-space check; no front/rear pair',
                 (1.,.65,.1) if result['decision']!='CANDIDATE_GAP' else (.2,1.,.4),.6)
        for row in result.get('occupied_intervals',[]):
            color=(.5,.5,.5) if row['age']>.45 else (.2,.6,1.) if row['ready'] else (.8,.3,1.)
            line('ring_occupied',ring_arc(row['lo'],row['hi']),color,.35,.9)
            polygon=row.get('polygon_xy',[])
            if polygon and row['age']<=.45:line('observed_polygon',polygon+[polygon[0]],color,.08,1.)
            if len(polygon)>=2 and row['age']<=.45:
                pts=np.asarray(polygon);center=pts.mean(axis=0)
                heading=math.atan2(center[1]-circle.center[1],center[0]-circle.center[0])+circle.direction*math.pi/2
                c,s=math.cos(heading),math.sin(heading);axes=np.array([[c,-s],[s,c]])
                uv=pts@axes;lo=uv.min(axis=0);hi=uv.max(axis=0);mid=((lo+hi)/2)@axes.T
                for u,side in ((lo[0],'rear'),(hi[0],'front')):
                    edge=np.array([[u,lo[1]],[u,hi[1]]])@axes.T
                    confirmed=row.get(f'{side}_observed',False)
                    estimated=side=='rear' and row.get('rear_estimated',False)
                    edge_color=(.2,1.,.7) if confirmed else (1.,.75,.2) if estimated else (.55,.55,.55)
                    line('measured_boundary',edge,edge_color,.13,1.)
            if row['id'] in blockers:
                line('blocking_object',ring_arc(row['lo'],row['hi']),(1.,0.,0.),.55,1.)
            a=circle.direction*(reference+row['s']/circle.radius)
            x=circle.center[0]+circle.radius*math.cos(a)
            y=circle.center[1]+circle.radius*math.sin(a)
            text('ring_model',x,y-2.,f"#{row['id'].split(':')[-1]} L={row['length']:.2f}m "
                 f"v={row['speed']*3.6:.1f}km/h age={row['age']:.2f}s",color,.42)
            for dt in ((1.,2.,3.) if show_future else ()):
                shift=row['speed']*dt;grow=.35*dt
                line('ring_future',ring_arc(row['lo']+shift-grow,row['hi']+shift+grow),color,.16,.4)
        if len(result.get('gap_pair',[]))==2 and 'gap_rear_s' in result:
            # This is the whole available slot, not rear travel or safety padding.
            color=(.2,1.,.3) if result['decision']=='CANDIDATE_GAP' else (.6,.6,.6)
            line('selected_gap',ring_arc(result['gap_rear_s'],result['gap_front_s']),color,.15,.9)
            for dt in ((1.,2.,3.) if show_future else ()):
                lo=result['gap_rear_s']+result.get('gap_rear_speed',0.)*dt+result.get('rear_uncertainty_mps',.35)*dt
                hi=result['gap_front_s']+result.get('gap_front_speed',0.)*dt-.35*dt
                if hi>lo:line('selected_gap_future',ring_arc(lo,hi),color,.08,.35)
            response=result.get('rear_response',[])
            if response and show_future:
                points=[]
                for p in response:
                    a=circle.direction*(reference+p['s']/circle.radius)
                    points.append((circle.center[0]+circle.radius*math.cos(a),circle.center[1]+circle.radius*math.sin(a)))
                line('npc_following_assumption',points,(1.,.2,1.),.1,.7)
            for key,label in (('rear','APPROACH FRONT'),('front','LEAD REAR')):
                boundary=result[f'gap_{key}_s']
                source=result.get(f'gap_{key}_boundary_source','estimated')
                a=circle.direction*(reference+boundary/circle.radius)
                direction=np.array([math.cos(a),math.sin(a)])
                points=[circle.center+(circle.radius+offset)*direction for offset in (-1.2,1.2)]
                edge_color=(.2,1.,.5) if source=='observed_face' else (.8,.3,1.)
                line('gap_boundary',points,edge_color,.14,1.)
                x,y=circle.center+(circle.radius+2.)*direction
                text('gap_boundary_label',x,y,f'{label} [{source}]',edge_color,.4)
            rear,front=result['gap_pair']
            text('gap_status',circle.center[0],circle.center[1]-4.,
                 f"GAP #{rear.split(':')[-1]} -> #{front.split(':')[-1]}\n"
                 f"minimum margin {result['minimum_gap_margin_m']:.2f}m\n"
                 f"entry interaction +{result['interaction_start_s']:.1f}..{result['interaction_end_s']:.1f}s\n"
                 +('NPC following ASSUMED; verify response' if result.get('cooperative_rear') else 'constant-speed rear'),
                 color,.5)
    if ctx.route is not None:
        if 'staging_s' in result:
            x,y=ctx.route.at(result['staging_s']+ctx.s_offset)
            text('staging_position',x,y,'APPROACH LIMIT',(0.,1.,1.),.5)
        if circular and show_future:
            for path in result.get('ego_paths',[]):line('ego_departure_hypotheses',path['xy'],(0.,1.,1.),.09,.65)
        for accel in (() if circular or not show_future else (.7,1.,1.3)):
            line('ego_departure_hypotheses',[predictor.ego_pose(ctx,float(dt),accel)[:2]
                 for dt in np.arange(0.,predictor.horizon+.01,.5)],(0.,1.,1.),.09,.65)
            for dt in (1.,2.,3.):
                x,y,h,l,w=predictor.ego_pose(ctx,dt,accel)
                box('ego_future',x,y,h,l,w,(0.,1.,1.),.08,z=.4)
                if accel==1.:text('ego_future_label',x,y,f'EGO +{dt:.0f}s IF DEPART',(0.,1.,1.),.4)
    if result.get('decision') in ('WAIT','UNKNOWN') and 'conflict_xy' in result:
        x,y=result['conflict_xy']
        line('conflict',[(x-1,y-1),(x+1,y+1)],(1.,0.,0.),.2,1.)
        line('conflict',[(x-1,y+1),(x+1,y-1)],(1.,0.,0.),.2,1.)
        track=next((r for r in result.get('occupied_intervals',[]) if r['id']==result.get('track')),None)
        detail=(f"\nage={track['age']:.2f}s observed={track['observed_length']:.2f}m" if track else '')
        text('conflict_label',x,y-3.,f'CONFLICT +{result["conflict_in_s"]:.1f}s\n'
             f'{result.get("conflict_side", "?")} #{result.get("track", "").split(":")[-1]}'+detail,(1.,.15,.15),.55)
        if show_future and 'conflict_ego' in result:
            box('conflict_ego',*result['conflict_ego'],(1.,0.,0.),.25,z=.5)
    label,color=decision_label(ctx,result,target,gate_reason)
    text('decision',ctx.x-6.*math.sin(ctx.yaw),ctx.y+6.*math.cos(ctx.yaw),label,color,size=.65)
    for row in result.get('observed_corridor_polygons',[]):
        pts=row['xy']
        if len(pts)>=2:
            line('corridor_observation',pts+[pts[0]],(1.,.6,.1) if row['id']==result.get('following_lead_id') else (.2,.6,1.),.1,1.)
    if result.get('following_model')=='measured_route_clearance' and ctx.route is not None:
        distance=result.get('following_gap_m')
        length=min(30.,distance+3.8) if distance is not None else 30.
        for side in (-1.,1.):
            points=[]
            for ds in np.arange(0.,length+.1,.25):
                station=ctx.s+ctx.s_offset+ds;x,y=ctx.route.at(station);h=ctx.route.heading_at(station)
                points.append((x-side*1.196*math.sin(h),y+side*1.196*math.cos(h)))
            line('measured_corridor',points,(.2,1.,.4),.08,.8)
    if circle is not None and np.linalg.norm(np.array([ctx.x,ctx.y])-circle.center)<80.:
        line('ring_reference',circle.arc(),(.65,.65,.65),.1,.7)
        text('ring_label',circle.center[0],circle.center[1],
             f'MAP RING R={circle.radius:.2f}m RMS={circle.rms:.2f}m\n'
             'REFERENCE ONLY; gray = UNVERIFIED',(.8,.8,.8),.55)
    if ctx.mission_name and not show_details:
        if ctx.vehicle_following_active:
            keep={'corridor_observation','measured_corridor','conflict','blocking_object','blocker_pointer'}
            arr.markers=[m for m in arr.markers if m.action==Marker.DELETEALL or m.ns in keep]
        else:
            arr.markers=[m for m in arr.markers if m.ns!='decision']
    return arr
