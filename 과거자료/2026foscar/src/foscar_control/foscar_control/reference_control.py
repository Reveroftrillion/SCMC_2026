"""Reuse the verified controller on local geometry without changing global progress."""
from dataclasses import replace
import math
import numpy as np
from .core import Controller, Target, Output, FRONT_AXLE, wrap


_TRANSFER=('steering','integral','lateral_integral','previous_accel','previous_brake','state','reason','since_ok')


def step_reference(owner,x,y,yaw,speed,mode,gear,healthy,dt,target):
    reference=target.reference
    old=owner.active_reference
    blocked=None
    global_local,global_error,global_heading=owner.route.project(x,y,owner.previous)
    global_s=global_local-owner.s_offset
    if old is not None and reference is None and target.reference_release:
        if global_error<=owner.max_error and abs(wrap(yaw-global_heading))<=math.atan2(owner.max_error,owner.lookahead):
            if owner.reference_controller is not None:
                for key in _TRANSFER:setattr(owner,key,getattr(owner.reference_controller,key))
            # Start future acceleration from the speed actually delivered by
            # the replacement path, not from a bend that this path bypassed.
            i=max(0,int(np.searchsorted(owner.route.s,global_local))-1)
            owner.profile[i:]=owner.braking_profile[i:]
            owner.profile[i]=min(owner.profile[i],max(0.,speed))
            for j in range(i+1,len(owner.profile)):
                owner.profile[j]=min(owner.profile[j],math.sqrt(owner.profile[j-1]**2+
                    2*owner.reference_options['a_acc']*owner.route.lengths[j-1]))
            owner.active_reference=None;owner.reference_controller=None
            owner.previous=global_local
            return owner.step(x,y,yaw,speed,mode,gear,healthy,dt,replace(target,reference_release=False))
        blocked='REFERENCE_RELEASE_UNALIGNED'
    if reference is not None and old is not None and reference.id==old.id and reference.signature!=old.signature:
        reference=old;blocked='REFERENCE_ID_CHANGED_GEOMETRY'
    if reference is not None and reference.project(x,y)[1]>owner.max_error:
        reference=old;blocked='REFERENCE_DISCONNECTED'
    if reference is not None and (target.lat_offset!=0 or target.entry_length_m or target.return_length_m):
        reference=old;blocked='REFERENCE_OFFSET_CONFLICT'
    if reference is None:
        reference=old;blocked=blocked or 'REFERENCE_MISSING'
    if reference is None:return Output('HOLD',blocked or 'REFERENCE_INVALID',steering=owner.steering,progress=global_s)
    if not reference.valid_for(x,y,global_s,min(owner.lookahead_max,max(owner.lookahead,owner.lookahead_gain*speed))):blocked='REFERENCE_COVERAGE'
    if owner.previous is not None and abs(global_local-owner.previous)>max(2.,abs(speed)*max(.5,owner.since_ok)):
        blocked='REFERENCE_POSITION_JUMP'
    if old is None or old.id!=reference.id:
        inner=Controller(reference.route,**owner.reference_options,
                         uncapped_zones=owner.uncapped_zones,profile_global_s=reference.global_s)
        # Where this geometry rejoins the original route, inherit its speed
        # profile too. Resampling may smooth away a curvature peak; releasing
        # the reference must not suddenly trigger the original overspeed guard.
        stations=reference.global_s+owner.s_offset
        original_xy=np.array([owner.route.at(s) for s in stations])
        same=np.linalg.norm(reference.xy-original_xy,axis=1)<=.05
        # An isolated crossing is not a handoff. Only the matching suffix
        # inherits future original-route limits; local geometry governs earlier.
        same=np.logical_and.accumulate(same[::-1])[::-1]
        original_cap=np.interp(stations,owner.route.s,owner.braking_profile)
        inner.profile[same]=np.minimum(inner.profile[same],original_cap[same])
        for i in range(len(inner.profile)-2,-1,-1):
            inner.profile[i]=min(inner.profile[i],math.sqrt(inner.profile[i+1]**2+
                2*owner.reference_options['a_dec']*reference.route.lengths[i]))
        for i in range(1,len(inner.profile)):
            inner.profile[i]=min(inner.profile[i],math.sqrt(inner.profile[i-1]**2+
                2*owner.reference_options['a_acc']*reference.route.lengths[i-1]))
        source=owner.reference_controller or owner
        for key in _TRANSFER:setattr(inner,key,getattr(source,key))
        inner.a_stop=owner.a_stop;inner.a_finish=owner.a_finish
        inner.previous=None;inner.stop_s=None
        # A rolling local window is not the destination. Its coverage is
        # checked every tick above; explicit stop lines still use stop_q.
        # Only the final window retains the route-end braking ramp.
        if owner.continuous and reference.global_s[-1]<owner.route.s[-1]-owner.s_offset-1e-6:
            inner.stop_s=math.inf
        owner.reference_controller=inner;owner.active_reference=reference
    inner=owner.reference_controller
    # stop_s denotes front wheels. Convert station to q before applying axle offset.
    stop_q=reference.q_at_global(target.stop_s) if math.isfinite(target.stop_s) else math.nan
    if math.isfinite(target.stop_s) and target.stop_s<reference.global_s[0]:stop_q=math.nan
    command=Target(v_max=0. if blocked else target.v_max,stop_s=stop_q,state=target.state)
    out=inner.step(x,y,yaw,speed,mode,gear,healthy,dt,command)
    owner.previous=global_local;owner.since_ok=inner.since_ok
    owner.steering=inner.steering;owner.previous_accel=inner.previous_accel;owner.previous_brake=inner.previous_brake
    owner.state=out.state
    if out.state=='FINISH':
        # A local window ending is not a lap finish. Hold until another window arrives.
        inner.state='CRUISE';owner.state='CRUISE'
        out=replace(out,state='CRUISE',reason='REFERENCE_END',accel=0.,brake=1.)
    return replace(out,progress=global_s,reason=blocked or out.reason)
