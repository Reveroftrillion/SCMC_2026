"""Route departure rollout with acceleration then measured front-gap following."""
import math
from .ctrl_circular_gap import CircularGap
from .ctrl_traffic_position import rear_traffic
from .ctrl_following_policy import following_speed
from .ctrl_departure_profile import ACCELERATIONS, launch_delay


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def conservative_braking(speed,target):
    # Same brake command law as core.Controller; use a weaker plant response
    # than the measured ~15 m/s² per unit brake and 0.32 m/s² coasting.
    horizon=max(speed,1.)*.5
    ff=min(.3,max(0.,(speed*speed-target*target)/(2*horizon)-.32)/15.)
    feedback=min(.5,(speed-target)*.08) if speed-target>.4 else 0.
    return .2+12.*max(ff,feedback)


def braking_response(speed,target,accel):
    """Progress bounds: prompt strong braking for rear, delayed weak for front.

    The 9 m/s² lower-progress bound is deliberately stronger than the measured
    response; it is an uncertainty bound, not a requested controller command.
    """
    weak=conservative_braking(speed,target)
    if accel<=1.5 or accel in (2.5,ACCELERATIONS[0]):return 9.,0.
    if accel<=1.8 or accel in (3.,ACCELERATIONS[1]):return .32+(weak-.2)*15./12.,.1
    return weak,.15


# Entry speed strategies (user 2026-09-29): until the ring is verified we ease off toward the entry; from then on both
# are tried every evaluation, speeding up first (it scores higher) -- the first whose departure clears the gap is taken:
#   accel  speed up to what the route's curvature allows (a_lat ACCEL_A_LAT, at most the 60 km/h rule speed)
#   keep   go on at no more than our current speed (curvature permitting)
# If neither clears we keep easing off ('decel', the gate's approach profile). The 'decel' departure (ease to the
# ring speed, max_speed) is no admission candidate any more; it stays for the committed ring following.
STRATEGIES=('accel','keep')
ACCEL_A_LAT=4.5
RULE_SPEED=60/3.6


class FollowingDeparture:
    def __init__(self,predictor,circle,tracks,follow_before_contact=False,mode='decel'):
        self.predictor=predictor;self.circle=circle;self.ring_tracks=tracks
        self.follow_before_contact=follow_before_contact;self.mode=mode
        self.tracks=getattr(predictor,'tracks',{})
        self.max_speed=predictor.max_speed
        self.cache={}

    def route_speed(self,route,s):
        # Reuse the controller's Route implementation, including its backward
        # braking and forward acceleration passes. Cache across decision ticks.
        if not hasattr(route,'speed_profile'):return self.max_speed if self.mode=='decel' else RULE_SPEED
        if self.mode!='decel':
            key=('free',RULE_SPEED,ACCEL_A_LAT)
            cached=getattr(self.predictor,'departure_free_profile',None)
            if cached is None or cached[0] is not route:
                cached=(route,key,route.speed_profile(RULE_SPEED,ACCEL_A_LAT,1.5,1.))
                self.predictor.departure_free_profile=cached
            return route.speed_at(s,cached[2])
        limits=getattr(self.predictor,'departure_profile_limits',(2.5,1.5,1.))
        cruise=getattr(self.predictor,'departure_cruise_speed',self.max_speed)
        key=(cruise,*limits)
        cached=getattr(self.predictor,'departure_route_profile',None)
        if cached is None or cached[0] is not route or cached[1]!=key:
            cached=(route,key,route.speed_profile(cruise,*limits))
            self.predictor.departure_route_profile=cached
        return min(self.max_speed,route.speed_at(s,cached[2]))

    def ego_pose(self,ctx,dt,accel=1.):
        # Cache one forward rollout per acceleration hypothesis.
        key=(ctx.now,accel)
        history=self.cache.setdefault(key,[(0.,ctx.s+ctx.s_offset,max(0.,ctx.v),self.max_speed)])
        while history[-1][0]<dt-1e-8:
            t,s,v,_=history[-1];step=min(.1,dt-t)
            delay=launch_delay(accel)
            if t<delay-1e-8:step=min(step,delay-t)
            x,y=ctx.route.at(s);h=ctx.route.heading_at(s)
            x+=1.4825*math.cos(h);y+=1.4825*math.sin(h)
            dx,dy=x-self.circle.center[0],y-self.circle.center[1]
            angle=math.atan2(dy,dx);radius=math.hypot(dx,dy)
            half_radial=abs(math.cos(h-angle))*2.3175+abs(math.sin(h-angle))*.946
            wanted=self.route_speed(ctx.route,s)
            if self.mode=='keep':wanted=min(wanted,max(history[0][2],.5))
            # Local control follows the lead on the first post-launch update,
            # not only after radial lane contact. Allow one .25s decision cycle
            # before this response rather than assuming instantaneous braking.
            if (self.follow_before_contact and t>=.25) or radius-half_radial<=self.circle.radius+1.5:
                ego_half=abs(math.cos(h-angle-math.pi/2))*2.3175+abs(math.sin(h-angle-math.pi/2))*.946
                for tr in self.ring_tracks:
                    # Preview the committed controller: a current rear vehicle
                    # does not become a hypothetical lead/brake command. Its
                    # collision envelope remains checked by CircularGap.
                    if (ctx.vehicle_following_active or self.follow_before_contact) and rear_traffic(tr.event,ctx,self.circle):continue
                    age=ctx.now-tr.event.stamp
                    end_ready=(tr.has_polygon and (tr.rear_observed or tr.rear_estimated) and tr.rear_boundary_motion.ready
                               and tr.rear_boundary_motion.stamp==tr.event.stamp)
                    if not (end_ready or tr.ready) or not 0<=age<=.45:continue
                    speed=tr.rear_boundary_motion.speed if end_ready else tr.speed
                    future=tr.samples[-1][1]+speed*(age+t)
                    distance=wrap(future/self.circle.radius-angle*self.circle.direction)*self.circle.radius
                    if 0<distance<30.:
                        # Use the same measured/anchored rear boundary as
                        # CircularGap's live following_speed_mps calculation.
                        back,_=CircularGap.extents(tr,0.)
                        gap=distance+back-ego_half
                        wanted=min(wanted,following_speed(speed,v,gap,1.))   # entering: tucked in close (user 2026-09-29)
            _,reaction=braking_response(v,wanted,accel)
            delayed=wanted if reaction==0. else next((r[3] for r in reversed(history) if r[0]<=t-reaction),history[0][3])
            decel,_=braking_response(v,delayed,accel)
            if t<delay-1e-8:new_v=v
            else:new_v=max(0.,min(v+accel*step,max(v-decel*step,delayed)))
            history.append((t+step,s+(v+new_v)*.5*step,new_v,wanted))
        # Most calls use monotonic dt; lower dt requests occur during pair scans.
        row=min(history,key=lambda r:abs(r[0]-dt))
        x,y=ctx.route.at(row[1]);h=ctx.route.heading_at(row[1])
        return (x+1.4825*math.cos(h),y+1.4825*math.sin(h),h,4.635,1.892)
