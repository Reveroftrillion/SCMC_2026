"""Reuse the verified controller on local geometry without changing global progress."""
from dataclasses import replace
import math
import numpy as np
from .ctrl_core import Controller, Target, Output, FRONT_AXLE, wrap


_TRANSFER=('steering','integral','lateral_integral','previous_accel','previous_brake','state','reason','since_ok')


def step_reference(owner,x,y,yaw,speed,mode,gear,healthy,dt,target):
    reference=target.reference
    previous_reference=owner.active_reference
    blocked=None
    route_station,global_lateral_error,global_heading=owner.route.project(x,y,owner.previous)
    global_s=route_station-owner.s_offset
    if previous_reference is not None and reference is None and target.reference_release:
        if global_lateral_error<=owner.max_error and abs(wrap(yaw-global_heading))<=math.atan2(owner.max_error,owner.lookahead):
            if owner.reference_controller is not None:
                for key in _TRANSFER:setattr(owner,key,getattr(owner.reference_controller,key))
            # Start future acceleration from the speed actually delivered by
            # the replacement path, not from a bend that this path bypassed.
            profile_index=max(0,int(np.searchsorted(owner.route.s,route_station))-1)
            owner.profile[profile_index:]=owner.braking_profile[profile_index:]
            owner.profile[profile_index]=min(owner.profile[profile_index],max(0.,speed))
            for next_profile_index in range(profile_index+1,len(owner.profile)):
                owner.profile[next_profile_index]=min(owner.profile[next_profile_index],math.sqrt(owner.profile[next_profile_index-1]**2+
                    2*owner.reference_options['a_acc']*owner.route.lengths[next_profile_index-1]))
            owner.active_reference=None;owner.reference_controller=None
            owner.previous=route_station
            return owner.step(x,y,yaw,speed,mode,gear,healthy,dt,replace(target,reference_release=False))
        blocked='REFERENCE_RELEASE_UNALIGNED'
    if reference is not None and previous_reference is not None and reference.id==previous_reference.id and reference.signature!=previous_reference.signature:
        reference=previous_reference;blocked='REFERENCE_ID_CHANGED_GEOMETRY'
    if reference is not None and reference.project(x,y)[1]>owner.max_error:
        reference=previous_reference;blocked='REFERENCE_DISCONNECTED'
    if reference is not None and (target.lat_offset!=0 or target.entry_length_m or target.return_length_m):
        reference=previous_reference;blocked='REFERENCE_OFFSET_CONFLICT'
    if reference is None:
        reference=previous_reference;blocked=blocked or 'REFERENCE_MISSING'
    if reference is None:return Output('HOLD',blocked or 'REFERENCE_INVALID',steering=owner.steering,progress=global_s)
    max_lookahead=owner.reference_options['lookahead_max']     # the highway's own steering preview (reference_lookahead_max_m)
    if not reference.valid_for(x,y,global_s,min(max_lookahead,max(owner.lookahead,owner.lookahead_gain*speed))):blocked='REFERENCE_COVERAGE'
    if owner.previous is not None and abs(route_station-owner.previous)>max(2.,abs(speed)*max(.5,owner.since_ok)):
        blocked='REFERENCE_POSITION_JUMP'
    if previous_reference is None or previous_reference.id!=reference.id:
        reference_controller=Controller(reference.route,**owner.reference_options,
                         uncapped_zones=owner.uncapped_zones,profile_global_s=reference.global_s)
        # Where this geometry rejoins the original route, inherit its speed
        # profile too. Resampling may smooth away a curvature peak; releasing
        # the reference must not suddenly trigger the original overspeed guard.
        route_stations=reference.global_s+owner.s_offset
        original_xy=np.array([owner.route.at(s) for s in route_stations])
        matching_suffix=np.linalg.norm(reference.xy-original_xy,axis=1)<=.05
        # An isolated crossing is not a handoff. Only the matching suffix
        # inherits future original-route limits; local geometry governs earlier.
        matching_suffix=np.logical_and.accumulate(matching_suffix[::-1])[::-1]
        original_speed_cap=np.interp(route_stations,owner.route.s,owner.braking_profile)
        reference_controller.profile[matching_suffix]=np.minimum(reference_controller.profile[matching_suffix],original_speed_cap[matching_suffix])
        deceleration=np.interp(route_stations,owner.route.s,owner.decel)     # the original route's per-station braking (highway_decel)
        for profile_index in range(len(reference_controller.profile)-2,-1,-1):
            reference_controller.profile[profile_index]=min(reference_controller.profile[profile_index],math.sqrt(reference_controller.profile[profile_index+1]**2+
                2*deceleration[profile_index]*reference.route.lengths[profile_index]))
        for profile_index in range(1,len(reference_controller.profile)):
            reference_controller.profile[profile_index]=min(reference_controller.profile[profile_index],math.sqrt(reference_controller.profile[profile_index-1]**2+
                2*owner.reference_options['a_acc']*reference.route.lengths[profile_index-1]))
        state_source=owner.reference_controller or owner
        for key in _TRANSFER:setattr(reference_controller,key,getattr(state_source,key))
        reference_controller.a_stop=owner.a_stop;reference_controller.a_finish=owner.a_finish
        reference_controller.previous=None;reference_controller.stop_s=None
        # A rolling local window is not the destination. Its coverage is
        # checked every tick above; explicit stop lines still use stop_q.
        # Only the final window retains the route-end braking ramp.
        if owner.continuous and reference.global_s[-1]<owner.route.s[-1]-owner.s_offset-1e-6:
            reference_controller.stop_s=math.inf
        owner.reference_controller=reference_controller;owner.active_reference=reference
    reference_controller=owner.reference_controller
    # target.stop_s는 앞축 기준 전역 station이다. 후속 제어기의 축 보정 전에 reference의 q로 변환한다.
    stop_q=reference.q_at_global(target.stop_s) if math.isfinite(target.stop_s) else math.nan
    if math.isfinite(target.stop_s) and target.stop_s<reference.global_s[0]:stop_q=math.nan
    command=Target(v_max=0. if blocked else target.v_max,stop_s=stop_q,state=target.state)
    output=reference_controller.step(x,y,yaw,speed,mode,gear,healthy,dt,command)
    owner.previous=route_station;owner.since_ok=reference_controller.since_ok
    owner.steering=reference_controller.steering;owner.previous_accel=reference_controller.previous_accel;owner.previous_brake=reference_controller.previous_brake
    owner.state=output.state
    if output.state=='FINISH':
        # A local window ending is not a lap finish. Hold until another window arrives.
        reference_controller.state='CRUISE';owner.state='CRUISE'
        output=replace(output,state='CRUISE',reason='REFERENCE_END',accel=0.,brake=1.)
    return replace(output,progress=global_s,reason=blocked or output.reason)
