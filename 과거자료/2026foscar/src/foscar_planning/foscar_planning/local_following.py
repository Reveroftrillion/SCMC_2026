"""Post-admission front following; never re-runs the departure gap decision."""
import math
from .following_policy import following_speed
import numpy as np
from .circular_gap import wrap
from .prediction import rectangles_overlap
from .traffic_position import rear_traffic
from .measured_corridor import corridor_distance, distance_speed


def begin_following(gap,result):
    gap.distance_history=[]
    pair=result.get('gap_pair',[])
    gap.following_lead=gap.tracks.get(pair[1]) if len(pair)==2 else None
    gap.following_rear=gap.tracks.get(pair[0]) if len(pair)==2 else None


def refresh_exit_prediction(result,ctx,circle,entered_ring=False):
    """Cheap current-pose exit check, independent of the slower NPC update."""
    if entered_ring and circle is not None:
        c,s=math.cos(ctx.yaw),math.sin(ctx.yaw)
        center=np.array([ctx.x+1.4825*c,ctx.y+1.4825*s])-circle.center
        corners=np.array([center+[u*c-v*s,u*s+v*c]
                          for u,v in ((-2.3175,-.946),(2.3175,-.946),(2.3175,.946),(-2.3175,.946))])
        edges=np.roll(corners,-1,axis=0)-corners
        ratios=np.clip(-np.sum(corners*edges,axis=1)/np.sum(edges*edges,axis=1),0.,1.)
        inner=float(np.min(np.linalg.norm(corners+edges*ratios[:,None],axis=1)))
        outward=float(center@np.array([c,s]))/max(.01,float(np.linalg.norm(center)))
        # Clear the full modeled annulus (3.5m), plus a half-metre margin.
        if inner>circle.radius+4. and outward>.5:
            return dict(result,model='circular_gap',admission_model='three_vehicle_entry',
                        decision='EXITED',reason='whole body clear of ring and heading outward',
                        evaluated_at=ctx.now,evidence_stamp=ctx.now,phase='exited')
    return result


def evaluate_following(gap,ctx,predictor,entered_ring=False):
    fresh=[tr for tr in gap.tracks.values() if 0<=ctx.now-tr.event.stamp<=.45]
    base=dict(model='circular_gap',admission_model='three_vehicle_entry',phase='following',
        following_reference_s=ctx.s,evaluated_at=ctx.now,evidence_stamp=max(gap.scan_stamp,max((t.event.stamp for t in fresh),default=-math.inf)),
        tracks=len(fresh),lane_tracks=len(fresh),occupied_intervals=[],cooperative_rear=False,
        cleared_tracks=gap.cleared_count,ego_overlaps_ring=False)
    base['following_lead_id']=getattr(getattr(getattr(gap,'following_lead',None),'event',None),'value','')
    base['following_rear_id']=getattr(getattr(getattr(gap,'following_rear',None),'event',None),'value','')
    exited=refresh_exit_prediction(base,ctx,gap.circle,entered_ring)
    if exited is not base:return exited
    if gap.circle is None or ctx.route is None:
        return dict(base,decision='UNKNOWN',reason='circle or route unavailable',conflict_in_s=0.,conflict_side='other')
    if not 0<=ctx.now-base['evidence_stamp']<=.45:
        return dict(base,decision='UNKNOWN',reason='lidar observation updates unavailable')
    circle=gap.circle;R=circle.radius
    for tr in fresh:tr.observe_boundaries((ctx.x,ctx.y),circle)
    ego=predictor.ego_pose(ctx,0.,gap.departure_accels[0])
    angle=math.atan2(ego[1]-circle.center[1],ego[0]-circle.center[0])
    tangent=angle+circle.direction*math.pi/2
    half=abs(math.cos(ego[2]-tangent))*ego[3]/2+abs(math.sin(ego[2]-tangent))*ego[4]/2
    base['ego_aligned']=abs(wrap(ctx.yaw-tangent))<math.radians(25.)
    base['ego_overlaps_ring']=bool(gap.body_inner_radius(ctx.route,ctx.s+ctx.s_offset)<=R+max((t.radial_hi for t in fresh),default=1.15))
    if 0<=ctx.now-gap.scan_stamp<=.45:
        # Live footprint pipeline: control actual forward route occupancy.
        # Rear objects never revoke a committed admission; no hidden body length
        # or unstable tracked vehicle velocity enters this clearance control.
        sources={t.event.value:t.event for t in fresh}
        sources.update({k:t.event for k,t in predictor.tracks.items()
                        if k in gap.off_circle and 0<=ctx.now-t.event.stamp<=.45})
        nearest=None;nearest_id='';polygons=[]
        for key,event in sources.items():
            if rear_traffic(event,ctx,circle):continue
            footprint=gap.footprints.get(key)
            if footprint is not None and 0<=ctx.now-footprint[0]<=.45:
                polygon=footprint[1]
            else:
                # Missing outline: only the observed AABB, never a guessed car.
                h=event.box_yaw if math.isfinite(event.box_yaw) else 0.
                c,s=math.cos(h),math.sin(h)
                polygon=[[event.x+u*c-v*s,event.y+u*s+v*c] for u,v in
                         ((-event.size_x/2,-event.size_y/2),(event.size_x/2,-event.size_y/2),
                          (event.size_x/2,event.size_y/2),(-event.size_x/2,event.size_y/2))]
            polygons.append(dict(id=key,xy=polygon))
            distance=corridor_distance(ctx,polygon)
            if distance is not None and (nearest is None or distance<nearest):nearest=distance;nearest_id=key
        history=getattr(gap,'distance_history',[]);gap.distance_history=history
        # No ring speed limit of its own (user 2026-09-29): the lead and the road's curvature set it; max_speed is
        # the admission model's departure speed only.
        known=getattr(gap,'tracks',{}).get(nearest_id)
        known=known.speed if known is not None and getattr(known,'ready',False) else None
        cap,rate=distance_speed(history,ctx,nearest,math.inf,known)
        base.update(following_speed_mps=cap,following_gap_m=nearest,
                    boundary_speed_mps=rate,following_lead_id=nearest_id,
                    observed_corridor_polygons=polygons,following_model='measured_route_clearance')
        if nearest is not None and nearest<=.8:
            return dict(base,decision='WAIT',reason='observed surface too close on route',
                        conflict_side='front',conflict_in_s=0.,track=nearest_id)
        return dict(base,decision='FOLLOW',reason='measured route clearance; admission latched')
    previous=getattr(gap,'following_lead',None)
    if previous is not None and ctx.now-previous.event.stamp>.45:
        gap.following_lead=None
        base['following_lead_id']=''
    cap=math.inf;bodies=[];leads=[]     # (no ring speed limit of its own: see above)
    for tr in fresh:
        e=tr.event;age=ctx.now-e.stamp
        arc=wrap(tr.samples[-1][1]/R-angle*circle.direction)*R+tr.speed*age
        if rear_traffic(e,ctx,circle):
            # Admission is complete. Rear approach/overlap never revokes it,
            # including when perception assigns a new identity to that car.
            continue
        h=math.atan2(e.y-circle.center[1],e.x-circle.center[0])+circle.direction*math.pi/2
        view=((ctx.x-e.x)*math.cos(h)+(ctx.y-e.y)*math.sin(h))/max(.01,math.hypot(ctx.x-e.x,ctx.y-e.y))
        lo,hi=gap.extents(tr,0.,view)
        # Fully trailing traffic no longer controls ego braking after admission.
        # Side/overlapping and leading bodies are still checked below.
        if arc+hi < -half:continue
        bodies.append(tr)
        distance=arc+lo-half
        if 0<arc<35. and tr.ready:
            leads.append((distance,tr))
            cap=min(cap,following_speed(tr.speed,ctx.v,distance,getattr(ctx,'ring_tuck',0.)))
    if leads:gap.following_lead=min(leads,key=lambda row:row[0])[1]
    base['following_speed_mps']=cap
    horizon=max(2.,max(0.,ctx.v)/1.5+.4);base['horizon_s']=horizon
    # Outside-circle classification is not a stop reason. Only a fresh body
    # intersecting the short actual route rollout can command braking here.
    for source in predictor.tracks.values():
        e=source.event
        if e.value not in gap.off_circle or not 0<=ctx.now-e.stamp<=.45:continue
        if rear_traffic(e,ctx,circle):continue
        for dt in np.arange(0.,horizon+gap.step/2,gap.step):
            elapsed=ctx.now-e.stamp+dt
            body=(e.x+e.vx*elapsed,e.y+e.vy*elapsed,
                  e.box_yaw if math.isfinite(e.box_yaw) else 0.,
                  max(.1,e.size_x)+.6,max(.1,e.size_y)+.6)
            for accel in gap.departure_accels:
                vehicle=predictor.ego_pose(ctx,float(dt),accel)
                if rectangles_overlap(vehicle,body):
                    return dict(base,decision='WAIT',reason='observed body conflicts with local path',
                                conflict_side='front',conflict_in_s=round(float(dt),2),track=e.value,
                                conflict_xy=list(vehicle[:2]),conflict_ego=list(vehicle))
    for dt in np.arange(0.,horizon+gap.step/2,gap.step):
        for tr in bodies:
            age=ctx.now-tr.event.stamp
            a=(tr.samples[-1][1]+tr.speed*(age+dt))/R*circle.direction
            lo,hi=gap.extents(tr,float(dt)+age)
            h=a+circle.direction*math.pi/2
            radial=(tr.radial_lo+tr.radial_hi)/2
            x,y=circle.center+(R+radial)*np.array([math.cos(a),math.sin(a)])
            x+=(lo+hi)/2*math.cos(h);y+=(lo+hi)/2*math.sin(h)
            body=(float(x),float(y),h,hi-lo,tr.radial_hi-tr.radial_lo+.4)
            for accel in gap.departure_accels:
                vehicle=predictor.ego_pose(ctx,float(dt),accel)
                if rectangles_overlap(vehicle,body):
                    gap.following_lead=tr
                    return dict(base,decision='WAIT',reason='local front or side conflict',
                        conflict_side='front',conflict_in_s=round(float(dt),2),track=tr.event.value,
                        conflict_xy=list(vehicle[:2]),conflict_ego=list(vehicle))
    return dict(base,decision='FOLLOW',reason='local front following; departure decision latched')
