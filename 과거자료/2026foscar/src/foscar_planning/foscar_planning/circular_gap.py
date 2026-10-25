"""Low-cost circular occupancy, restricted to the calibrated circulating lane.

A gap requires two observed bounding vehicles, not an absence of returns.
The result remains a finite-horizon prediction, not a visibility certificate.
"""
from collections import deque
from types import SimpleNamespace
import math
import numpy as np
from .rear_response import predict_rear_response
from .following_policy import following_speed
from .ring_contact import contact_arc
from .state import EventInfo
from .prediction import rectangles_overlap
from .empty_entry import EmptyEntry
from .boundary_motion import BoundaryMotion
from .departure_profile import ACCELERATIONS
from .observed_boundary import endpoint_queries, side_supported


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inspect_unselected_body(local_entry,ctx,age,dt):
    """Old third-body hypotheses guard the braking horizon, not a whole lap.

    Fresh front/rear bounds still cover insertion and following handoff. Current
    third objects remain full-horizon vetoes; missing data is not deleted or
    declared free, and immediate stale-body conflicts continue to stop entry.
    """
    return not local_entry or age<=1. or dt<=max(2.,max(0.,ctx.v)/1.5+.4)


class RingTrack:
    def __init__(self):
        self.front_motion=BoundaryMotion();self.rear_motion=BoundaryMotion()
        self.estimated_rear_motion=BoundaryMotion()
        self.side_proofs={}
        self.samples = deque(maxlen=24)
        self.sizes = deque(maxlen=24)
        self.length = 4.8
        self.speed = 0.
        self.observed_length = 0.
        self.has_polygon = False
        self.polygon_stamp = None
        self.arc_lo = self.arc_hi = 0.
        self.polygon_xy=[]
        self.raw_radial_width=0.
        self.raw_radial_lo=self.raw_radial_hi=0.
        self.front_face_width=self.rear_face_width=0.
        self.body_bounds=None
        self.rear_observed=self.front_observed=False
        self.rear_estimated=False;self.rear_uncertainty=0.
        self.radial_lo,self.radial_hi = -1.15,1.15
        self.ready = False
        self.ever_ready = False
        self.event = None

    def update(self, event, circle):
        if self.event is not None and event.stamp <= self.event.stamp:
            return
        dx,dy=event.x-circle.center[0],event.y-circle.center[1]
        angle=math.atan2(dy,dx)*circle.direction
        if self.samples:
            angle=self.samples[-1][1]/circle.radius+wrap(angle-self.samples[-1][1]/circle.radius)
        self.samples.append((event.stamp, angle*circle.radius))
        self.event=event
        self.has_polygon=False
        self.body_bounds=None
        self.rear_observed=self.front_observed=False
        self.rear_estimated=False;self.rear_uncertainty=0.
        h=math.atan2(dy,dx)+circle.direction*math.pi/2
        delta=h-event.box_yaw if math.isfinite(event.box_yaw) else 0.
        measured=(abs(math.cos(delta))*event.size_x+abs(math.sin(delta))*event.size_y
                  if math.isfinite(event.box_yaw) else math.hypot(event.size_x,event.size_y))
        self.observed_length=measured
        radial=math.hypot(dx,dy)-circle.radius
        observed_width=(abs(math.sin(delta))*event.size_x+abs(math.cos(delta))*event.size_y
                        if math.isfinite(event.box_yaw) else math.hypot(event.size_x,event.size_y))
        self.radial_lo=min(-1.15,radial-observed_width/2-.2)
        self.radial_hi=max(1.15,radial+observed_width/2+.2)
        self.sizes.append((event.stamp,measured))
        # Retain a robust upper size within this track, not a forever maximum.
        recent=[v for t,v in self.sizes if event.stamp-t<=3.]
        if len(recent)>=3:
            estimate=max(4.8,float(np.percentile(recent,90)))
            self.length=max(estimate,self.length-.03)
        elif measured>self.length:
            self.length=measured
        points=np.asarray([(t,s) for t,s in self.samples if event.stamp-t<=.8])
        self.ready=False
        self.speed=max(0.,event.vx*math.cos(h)+event.vy*math.sin(h))
        if len(points)>=4 and points[-1,0]-points[0,0]>=.45:
            ts=points[:,0]-points[-1,0]
            speed,intercept=np.polyfit(ts,points[:,1],1)
            residual=float(np.max(abs(points[:,1]-(speed*ts+intercept))))
            self.speed=float(speed)
            tangent_v=event.vx*math.cos(h)+event.vy*math.sin(h)
            self.ready=bool(.8<speed<15. and residual<.8 and tangent_v>.5
                        and tangent_v/max(.01,event.speed)>.75)
        self.ever_ready=self.ever_ready or self.ready


    def apply_polygon(self,stamp,points,circle):
        if abs(stamp-self.event.stamp)>.001 or self.polygon_stamp==stamp:
            return
        xy=np.asarray(points,dtype=float)
        if xy.ndim!=2 or xy.shape[1]!=2 or not 2<=len(xy)<=512 or not np.isfinite(xy).all():
            return
        delta=xy-circle.center
        mean_angle=self.samples[-1][1]/circle.radius
        a=np.arctan2(delta[:,1],delta[:,0])*circle.direction
        arc=np.arctan2(np.sin(a-mean_angle),np.cos(a-mean_angle))*circle.radius
        self.arc_lo,self.arc_hi=float(min(arc)),float(max(arc))
        self.observed_length=self.arc_hi-self.arc_lo
        edges=np.roll(delta,-1,axis=0)-delta
        norm=np.sum(edges*edges,axis=1)
        ratios=np.clip(-np.sum(delta*edges,axis=1)/np.maximum(norm,1e-9),0.,1.)
        inner=float(np.min(np.linalg.norm(delta+edges*ratios[:,None],axis=1)))-circle.radius
        outer=float(np.max(np.linalg.norm(delta,axis=1)))-circle.radius
        self.raw_radial_width=outer-inner
        self.raw_radial_lo,self.raw_radial_hi=inner,outer
        radii=np.linalg.norm(delta,axis=1)
        self.front_face_width=float(np.ptp(radii[arc>=max(arc)-.5]))
        self.rear_face_width=float(np.ptp(radii[arc<=min(arc)+.5]))
        self.body_bounds=None
        self.rear_observed=self.front_observed=False
        self.rear_estimated=False;self.rear_uncertainty=0.
        self.radial_lo=min(-1.15,inner-.2);self.radial_hi=max(1.15,outer+.2)
        if self.sizes and self.sizes[-1][0]==stamp:self.sizes[-1]=(stamp,self.observed_length)
        self.length=max(4.8,self.length-.03,float(np.percentile([v for t,v in self.sizes],90)))
        self.has_polygon=True;self.polygon_stamp=stamp;self.polygon_xy=xy.tolist()


    def observe_boundaries(self,sensor,circle):
        """Identify the visible end independently of the hidden vehicle length.

        A hull edge near an extreme must span an end face, approximately normal
        to lane motion. The sensor must be beyond that end plane: an L shape
        alone (or merely looking at a side) does not identify a rear bumper.
        A stable side may additionally supply a provisional rear with a hidden
        length/error allowance. This is distinct from confirmed end evidence;
        neither survives a new scan without a polygon.
        """
        self.rear_observed=self.front_observed=False
        self.rear_estimated=False;self.rear_uncertainty=0.
        if not self.has_polygon or not self.polygon_xy:
            self.rear_motion.observe(self.event.stamp,None)
            self.estimated_rear_motion.observe(self.event.stamp,None)
            self.front_motion.observe(self.event.stamp,None)
            return
        center=np.array([self.event.x,self.event.y])
        offset=center-circle.center
        angle=math.atan2(offset[1],offset[0])
        h=angle+circle.direction*math.pi/2
        tangent=np.array([math.cos(h),math.sin(h)])
        normal=np.array([-math.sin(h),math.cos(h)])
        delta=np.asarray(self.polygon_xy)-center
        u=delta@tangent;v=delta@normal
        sensor_u=float((np.asarray(sensor)-center)@tangent)
        du=np.roll(u,-1)-u;dv=np.roll(v,-1)-v
        face=(abs(dv)>1e-6)&(abs(du)<=.35*abs(dv))
        near_rear=np.maximum(u,np.roll(u,-1))<=min(u)+.5
        near_front=np.minimum(u,np.roll(u,-1))>=max(u)-.5
        def supported_width(mask):
            indices=np.flatnonzero(face&mask)
            return float(np.ptp(np.r_[v[indices],v[(indices+1)%len(v)]])) if len(indices) else 0.
        self.rear_observed=bool(sensor_u<min(u)-.2 and supported_width(near_rear)>=.8)
        self.front_observed=bool(sensor_u>max(u)+.2 and supported_width(near_front)>=.8)
        self.rear_observed=self.rear_observed or side_supported(self,'rear')
        self.front_observed=self.front_observed or side_supported(self,'front')
        # Warm up endpoint motion while the longitudinal side is visible. End
        # confirmation must not start an additional motion-estimation delay.
        side=bool(self.observed_length>=2. and np.any((abs(du)>=1.2)&(abs(dv)<=.25*abs(du))))
        self.rear_motion.observe(self.event.stamp,self.samples[-1][1]+self.arc_lo if self.rear_observed or side else None)
        self.rear_estimated=bool(side and not self.rear_observed and self.rear_motion.ready
            and self.event.stamp-self.rear_motion.samples[0][0]>=.3)
        if self.rear_estimated:
            # Keep the hidden-length allowance; a clipped side is not proof of
            # a bumper. Add fit error rather than shortening NPC travel.
            self.rear_uncertainty=max(0.,self.length-self.observed_length)+.5+self.rear_motion.residual
        estimate_error=max(0.,self.length-self.observed_length)+.5+self.rear_motion.residual
        self.estimated_rear_motion.observe(self.event.stamp,
            self.samples[-1][1]+self.arc_lo-estimate_error if side else None)
        self.rear_estimated=self.rear_estimated and self.estimated_rear_motion.ready
        self.front_motion.observe(self.event.stamp,self.samples[-1][1]+self.arc_hi if self.front_observed else None)
        self.ever_ready=self.ever_ready or self.rear_motion.ready or self.front_motion.ready
        lo,hi=self.arc_lo,self.arc_hi
        if self.rear_observed:
            self.body_bounds=(lo-.2,max(hi,lo+self.length)+.2)
        elif self.front_observed:
            self.body_bounds=(min(lo,hi-self.length)-.2,hi+.2)


    @property
    def rear_boundary_motion(self):
        if not self.rear_estimated:return self.rear_motion
        # Visibility changes can move either endpoint or its uncertainty bound.
        # Credit only forward motion supported by both estimates.
        return SimpleNamespace(stamp=self.estimated_rear_motion.stamp,
            ready=self.estimated_rear_motion.ready and self.rear_motion.ready,
            speed=min(self.estimated_rear_motion.speed,self.rear_motion.speed))


class CircularGap:
    def __init__(self,circle,horizon=12.,step=.2,cooperative_rear=False,rear_uncertainty_mps=.175,rear_margin_m=.3):
        self.circle=circle
        self.cooperative_rear=cooperative_rear
        self.local_entry=False
        if not math.isfinite(rear_uncertainty_mps) or rear_uncertainty_mps<0.:
            raise ValueError('rear uncertainty growth must be finite and nonnegative')
        self.rear_uncertainty_mps=rear_uncertainty_mps
        if not math.isfinite(rear_margin_m) or rear_margin_m<0.:raise ValueError('invalid rear margin')
        self.rear_margin_m=rear_margin_m
        self.horizon,self.step=horizon,step
        self.departure_accels=ACCELERATIONS
        self.tracks={}
        self.off_circle=[]
        self.footprints={}
        self.retired={}
        self.raw_events={}
        self.boundary_proofs={}
        self.cleared_count=0
        self.scan_stamp=-math.inf
        self.current_scan_ids=None
        self.empty_entry=EmptyEntry()

    def on_footprints(self,data):
        if not isinstance(data,dict) or data.get('frame')!='map':return
        stamp=data.get('stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):return
        if stamp <= self.scan_stamp:return
        objects=data.get('objects',[])
        if not isinstance(objects,list):return
        if self.local_entry:
            if 'objects' not in data:return
            # Only a complete, valid scan is authoritative. Malformed data is
            # not an empty road; leave the previous stamp to expire normally.
            try:
                for row in objects:
                    points=np.asarray(row['xy'],dtype=float)
                    if (not isinstance(row['id'],str) or points.ndim!=2 or points.shape[1]!=2
                            or not 2<=len(points)<=512 or not np.isfinite(points).all()):return
            except (KeyError,TypeError,ValueError):return
            self.current_scan_ids={row['id'] for row in objects}
            for key in set(self.tracks)-self.current_scan_ids:
                old=self.tracks.pop(key)
                for role in ('following_lead','following_rear'):
                    if getattr(self,role,None) is old:setattr(self,role,None)
                self.cleared_count+=1
            self.raw_events={k:e for k,e in self.raw_events.items() if k in self.current_scan_ids}
            self.footprints={k:p for k,p in self.footprints.items() if k in self.current_scan_ids}
        self.scan_stamp=stamp
        self.empty_entry.observe(data)
        # Certificates belong to this complete sweep. A failed/newly occupied
        # endpoint patch revokes the previous positive evidence immediately.
        self.boundary_proofs={}
        for proof in data.get('boundary_evidence',[]):
            try:
                if (proof['side'] in ('front','rear') and isinstance(proof['id'],str)
                    and all(math.isfinite(proof[k]) for k in ('stamp','edge_s','speed'))
                    and abs(proof['stamp']-stamp)<.001):
                    self.boundary_proofs[(proof['id'],proof['side'])]=dict(proof)
            except (KeyError,TypeError,ValueError):continue
        self.boundary_proofs={k:p for k,p in self.boundary_proofs.items() if 0<=stamp-p['stamp']<=.35}
        observed_ids={r.get('id') for r in data.get('objects',[]) if isinstance(r,dict) and isinstance(r.get('id'),str)}
        for row in data.get('cleared',[]):
            if not isinstance(row,dict) or not isinstance(row.get('id'),str):continue
            key=row['id'];last=row.get('last_seen')
            if key in observed_ids:continue
            if not isinstance(last,(int,float)) or not math.isfinite(last) or last>=stamp:continue
            track=self.tracks.get(key)
            if track is not None and track.event.stamp<=last:
                if getattr(self,'following_lead',None) is track:self.following_lead=None
                self.tracks.pop(key);self.raw_events.pop(key,None);self.footprints.pop(key,None)
                self.retired[key]=stamp;self.cleared_count+=1
        for row in data.get('objects',[]):
            if isinstance(row,dict) and isinstance(row.get('id'),str):
                old=self.footprints.get(row['id'])
                if old is None or stamp>old[0]:
                    self.footprints[row['id']]=(stamp,row.get('xy',[]))
                    if row.get('on_road') and self.circle is not None:
                        try:
                            x,y=map(float,row['center']);vx=float(row['vx']);vy=float(row['vy'])
                            pts=np.asarray(row['xy'],dtype=float)
                            if (pts.ndim!=2 or pts.shape[1]!=2 or not 2<=len(pts)<=512
                                or not np.isfinite(pts).all() or not all(math.isfinite(v) for v in (x,y,vx,vy))):continue
                            if abs(math.hypot(x-self.circle.center[0],y-self.circle.center[1])-self.circle.radius)>3.5:continue
                            h=math.atan2(y-self.circle.center[1],x-self.circle.center[0])+self.circle.direction*math.pi/2
                            axes=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
                            size=np.ptp(pts@axes,axis=0)
                            self.raw_events[row['id']]=EventInfo('DYNAMIC_UNKNOWN',stamp,x=x,y=y,vx=vx,vy=vy,
                                size_x=float(size[0]),size_y=float(size[1]),box_yaw=h,value=row['id'])
                        except (KeyError,ValueError,TypeError):continue

    def observe(self,predictor,now):
        if self.circle is None:
            return
        active={k:t for k,t in self.tracks.items() if 0<=now-t.event.stamp<=self.horizon}
        self.off_circle=[]
        incoming={source.event.value:source.event for source in predictor.tracks.values()}
        for key,event in self.raw_events.items():
            # Position events and hulls travel on separate ROS topics. Prefer
            # the latest complete scan for a bounded one-scan skew; mixing a
            # newer AABB with a missing hull invents a closer rear bumper.
            newer=incoming.get(key)
            paired=(self.local_entry and newer is not None and
                    (self.current_scan_ids is not None or 0<=newer.stamp-event.stamp<=.15))
            if 0<=now-event.stamp<=.5 and (newer is None or event.stamp>=newer.stamp or paired):
                incoming[key]=event
        if self.local_entry and self.current_scan_ids is not None:
            active={k:t for k,t in active.items() if k in self.current_scan_ids}
            incoming={k:e for k,e in incoming.items() if k in self.current_scan_ids
                      and abs(e.stamp-self.scan_stamp)<.001}
        refreshed={k for k,t in active.items() if k in incoming and incoming[k].stamp>t.event.stamp}
        candidates={}
        for source in incoming:
            e=incoming[source]
            if source in active or e.stamp<=self.retired.get(source,-math.inf):continue
            a=math.atan2(e.y-self.circle.center[1],e.x-self.circle.center[0])*self.circle.direction
            possible=[]
            for old,tr in active.items():
                dt=e.stamp-tr.event.stamp
                if old in refreshed or not .05<=dt<=.8 or tr.speed<.8 or e.speed<.8:continue
                predicted=tr.samples[-1][1]+tr.speed*dt
                error=abs(wrap(a-predicted/self.circle.radius)*self.circle.radius)
                if error<=1.5+.3*dt and abs(e.speed-tr.speed)<2.:possible.append(old)
            candidates[source]=possible
        claims={old:sum(old in p for p in candidates.values()) for p in candidates.values() for old in p}
        for source,possible in candidates.items():
            if len(possible)==1 and claims[possible[0]]==1:
                old=possible[0];tr=active.pop(old)
                self.retired[old]=tr.event.stamp
                active[source]=tr
        for source in incoming:
            e=incoming[source]
            if e.stamp<=self.retired.get(source,-math.inf):continue
            if not all(math.isfinite(v) for v in (e.x,e.y,e.size_x,e.size_y,e.stamp)):
                continue
            radial=math.hypot(e.x-self.circle.center[0],e.y-self.circle.center[1])-self.circle.radius
            if abs(radial)>3.5:
                self.off_circle.append(e.value)
                continue
            # Source object survives the existing mutually unique ID reassociation.
            key=source
            track=active.get(key,RingTrack())
            track.update(e,self.circle)
            footprint=self.footprints.get(e.value)
            if footprint is not None:track.apply_polygon(*footprint,self.circle)
            track.side_proofs={side:self.boundary_proofs[(e.value,side)] for side in ('front','rear') if (e.value,side) in self.boundary_proofs}
            if track.has_polygon and getattr(self,'sensor_xy',None) is not None:track.observe_boundaries(self.sensor_xy,self.circle)
            active[key]=track
        self.tracks=active
        self.retired={k:t for k,t in self.retired.items() if now-t<=2.}
        self.footprints={k:p for k,p in self.footprints.items() if 0<=now-p[0]<=1.}
        self.raw_events={k:e for k,e in self.raw_events.items() if 0<=now-e.stamp<=1.}

    def clearance_queries(self,now,ctx=None,waiting_s=None):
        rows=[]
        if self.circle is None:return dict(frame='map',stamp=now,objects=rows)
        if self.local_entry and ctx is not None and not ctx.vehicle_following_active:
            rows.extend(self.empty_entry.queries(ctx,self.circle,waiting_s))
        for key,tr in self.tracks.items():
            if self.local_entry and ctx is not None and not ctx.vehicle_following_active:
                rows.extend(endpoint_queries(tr,self.circle,now))
            age=now-tr.event.stamp
            if not .25<=age<=self.horizon:continue
            angle=(tr.samples[-1][1]+tr.speed*age)/self.circle.radius*self.circle.direction
            radius=math.hypot(tr.event.x-self.circle.center[0],tr.event.y-self.circle.center[1])
            x,y=self.circle.center+radius*np.array([math.cos(angle),math.sin(angle)])
            heading=angle+self.circle.direction*math.pi/2
            lo,hi=self.extents(tr,age);radial_offset=radius-self.circle.radius
            half_width=max(abs(radial_offset-tr.radial_lo),abs(tr.radial_hi-radial_offset))+.2
            length=2*max(-lo,hi);width=2*half_width
            if not tr.ever_ready and tr.has_polygon:
                # Unconfirmed fragments are not already established car bodies.
                # Test their measured footprint plus drift error, not an invented
                # full vehicle extending from both ends of a short return.
                drift=.25+.35*age
                length=max(.3,tr.observed_length)+2*drift
                width=max(.3,tr.raw_radial_width)+2*drift
                radial_shift=(tr.raw_radial_lo+tr.raw_radial_hi)/2-radial_offset
                x+=radial_shift*math.cos(angle);y+=radial_shift*math.sin(angle)
                offset=(tr.arc_lo+tr.arc_hi)/2
                x+=offset*math.cos(heading);y+=offset*math.sin(heading)
            rows.append(dict(id=key,last_seen=tr.event.stamp,x=float(x),y=float(y),yaw=heading,
                             purpose='ring_vehicle' if self.local_entry else 'body',
                             length=length,width=width,
                             vx=tr.speed*math.cos(heading),vy=tr.speed*math.sin(heading)))
        return dict(frame='map',stamp=now,objects=rows)

    def body_inner_radius(self,route,s):
        x,y=route.at(s);h=route.heading_at(s)
        c,sn=math.cos(h),math.sin(h)
        center=np.array([x+1.4825*c,y+1.4825*sn])-self.circle.center
        corners=np.array([center+[u*c-v*sn,u*sn+v*c]
                          for u,v in ((-2.3175,-.946),(2.3175,-.946),(2.3175,.946),(-2.3175,.946))])
        edges=np.roll(corners,-1,axis=0)-corners
        ratios=np.clip(-np.sum(corners*edges,axis=1)/np.sum(edges*edges,axis=1),0.,1.)
        return float(np.min(np.linalg.norm(corners+edges*ratios[:,None],axis=1)))

    def staging_s(self,ctx):
        if self.circle is None or ctx.route is None or self.off_circle:
            return None
        fresh=[t for t in self.tracks.values() if t.ready and 0<=ctx.now-t.event.stamp<=.45]
        if len(fresh)<2 or ctx.s>888.:
            return None
        outer=self.circle.radius+max(t.radial_hi for t in self.tracks.values())
        # At 0.8 m/s: allow command latency and stopping travel, in addition
        # to static clearance. Verify the full prefix, not just its endpoint.
        speed=max(.8,ctx.v)
        clearance=.6+speed*.4+speed**2/6.
        safe=None
        for station in np.arange(max(880.,ctx.s),888.001,.1):
            if self.body_inner_radius(ctx.route,float(station)+ctx.s_offset)<=outer+clearance:
                break
            safe=min(888.,float(station))
        return safe

    @staticmethod
    def extents(track,dt,view=None):
        # The mean of visible returns is not the body center. If only an end
        # is seen, cover the hidden length in either direction.
        margin=.6+.35*dt
        missing=max(0.,track.length-track.observed_length)
        if track.has_polygon:
            lo,hi=track.arc_lo,track.arc_hi
            # A broad, thin observed end face plus viewing direction anchors
            # an estimated body. Direction alone never supplies this evidence.
            thin_end=track.raw_radial_width>=1.2 and track.observed_length<=.8
            if view is not None and not (track.rear_observed or track.front_observed):
                if view>.6 and (thin_end or track.front_face_width>=1.2):
                    track.body_bounds=(min(lo,hi-track.length)-.2,hi+.2)
                elif view<-.6 and (thin_end or track.rear_face_width>=1.2):
                    track.body_bounds=(lo-.2,max(hi,lo+track.length)+.2)
            if track.body_bounds is not None:lo,hi=track.body_bounds
            elif track.ever_ready:
                lo-=missing;hi+=missing
            # A never-established fragment is measured occupancy, not evidence
            # for a full hidden car extending from BOTH ends. This matches the
            # existing clearance-query model. Future motion uncertainty and
            # independent body collision checks still apply.
            # A supported end face is already the relevant measured boundary.
            # Hidden body length may extend only away from that observed end.
            if track.rear_observed:lo=track.arc_lo
            elif track.rear_estimated:lo=track.arc_lo-track.rear_uncertainty
            if track.front_observed:hi=track.arc_hi
            return lo-margin,hi+margin
        half=track.length/2+missing/2+margin
        return -half,half

    def independent_conflict(self,ctx,predictor):
        """A missing gap pair must not disable checking individual bodies."""
        if ctx.route is None:return None
        horizon=max(2.,max(0.,ctx.v)/1.5+.4)
        for dt in np.arange(0.,horizon+self.step,self.step):
            bodies=[]
            for tr in self.tracks.values():
                age=max(0.,ctx.now-tr.event.stamp)
                angle=(tr.samples[-1][1]+tr.speed*(age+dt))/self.circle.radius*self.circle.direction
                radius=math.hypot(tr.event.x-self.circle.center[0],tr.event.y-self.circle.center[1])
                x,y=self.circle.center+radius*np.array([math.cos(angle),math.sin(angle)])
                lo,hi=self.extents(tr,float(dt)+age)
                extra=2.*dt if not tr.ready else 0.
                radial_offset=radius-self.circle.radius
                half_width=max(abs(radial_offset-tr.radial_lo),abs(tr.radial_hi-radial_offset))+.2
                bodies.append((float(x),float(y),angle+self.circle.direction*math.pi/2,
                               2*max(-lo,hi)+2*extra,2*half_width))
            for source in predictor.tracks.values():
                e=source.event
                if e.value not in self.off_circle:continue
                elapsed=max(0.,ctx.now-e.stamp)+dt
                bodies.append((e.x+e.vx*elapsed,e.y+e.vy*elapsed,
                               e.box_yaw if math.isfinite(e.box_yaw) else 0.,
                               max(4.8,e.size_x)+1.,max(2.,e.size_y)+1.))
            for accel in self.departure_accels:
                ego=predictor.ego_pose(ctx,float(dt),accel)
                if any(rectangles_overlap(ego,body) for body in bodies):
                    return dict(conflict_in_s=round(float(dt),2),conflict_side='other',
                                conflict_model='independent_body',conflict_xy=list(ego[:2]),
                                conflict_ego=list(ego))
        return None

    def evaluate(self,ctx,predictor):
        self.sensor_xy=(ctx.x,ctx.y)
        tracks=[t for t in self.tracks.values() if not self.local_entry or 0<=ctx.now-t.event.stamp<=.45 or t.ever_ready]
        base=dict(model='circular_gap',evaluated_at=ctx.now,horizon_s=self.horizon,
                  evidence_stamp=max((t.event.stamp for t in tracks),default=-math.inf),
                  tracks=len(tracks),lane_tracks=len(tracks),occupied_intervals=[],
                  cooperative_rear=self.cooperative_rear,cleared_tracks=self.cleared_count)
        cooperative=self.cooperative_rear and not self.local_entry
        rear_growth=self.rear_uncertainty_mps if self.local_entry else .35
        rear_margin=self.rear_margin_m if self.local_entry else .6
        base['rear_uncertainty_mps']=rear_growth
        base['cooperative_rear']=cooperative
        if self.local_entry:base['admission_model']='three_vehicle_entry'
        def result(decision,reason,**kw):
            if decision=='UNKNOWN' and 'conflict_in_s' not in kw:
                conflict=self.independent_conflict(ctx,predictor) if self.circle is not None else None
                if conflict:kw.update(conflict)
            return dict(base,decision=decision,reason=reason,**kw)
        if (self.local_entry and self.current_scan_ids is not None
                and not 0<=ctx.now-self.scan_stamp<=.45):
            return dict(base,decision='UNKNOWN',reason='latest lidar scan unavailable')
        if self.circle is None or ctx.route is None:
            return result('UNKNOWN','circle or route unavailable')
        # A second upstream car is not a lead. Permit the existing open-front
        # path when fresh traffic is wholly upstream, retaining every other
        # track as a collision guard during insertion.
        entry_tracks=tracks
        extra_tracks=[]
        open_front=False
        if self.local_entry and len(tracks)>=2 and not self.off_circle:
            reference=math.atan2(ctx.y-self.circle.center[1],ctx.x-self.circle.center[0])*self.circle.direction
            fresh=[tr for tr in tracks if 0<=ctx.now-tr.event.stamp<=.45]
            ends=[]
            for tr in fresh:
                tr.observe_boundaries((ctx.x,ctx.y),self.circle)
                motion=tr.front_motion
                speed=getattr(motion,'approach_speed',motion.speed) if tr.front_observed and motion.ready else tr.speed
                arc=wrap(tr.samples[-1][1]/self.circle.radius-reference)*self.circle.radius
                _,front=self.extents(tr,0.)
                ends.append((arc+speed*(ctx.now-tr.event.stamp)+front,tr))
            if ends and max(end for end,_ in ends)<0.:
                nearest=max(ends,key=lambda row:row[0])[1]
                entry_tracks=[nearest]
                extra_tracks=[tr for tr in tracks if tr is not nearest]
                open_front=True
        if self.local_entry and (len(tracks)<2 or open_front) and not self.off_circle:
            body_at=None
            rear_only=False
            if entry_tracks:
                tr=entry_tracks[0]
                tr.observe_boundaries((ctx.x,ctx.y),self.circle)
                from .traffic_position import rear_traffic
                rear_only=rear_traffic(tr.event,ctx,self.circle)
                motion=tr.front_motion if rear_only else tr.rear_boundary_motion
                bounded=(tr.has_polygon and (tr.front_observed if rear_only else (tr.rear_observed or tr.rear_estimated))
                         and motion.ready and motion.stamp==tr.event.stamp)
                if not bounded or not 0<=ctx.now-tr.event.stamp<=.25:
                    return result('UNKNOWN','single vehicle boundary motion not fresh and ready')
                boundary_speed=motion.speed if tr.has_polygon else tr.speed
                if rear_only:boundary_speed=getattr(motion,'approach_speed',boundary_speed)
                def body_at(dt):
                    age=max(0.,ctx.now-tr.event.stamp)+dt
                    travel_age=max(0.,ctx.now-tr.event.stamp)+dt
                    a=(tr.samples[-1][1]+boundary_speed*travel_age)/self.circle.radius*self.circle.direction
                    h=a+self.circle.direction*math.pi/2
                    lo,hi=self.extents(tr,age)
                    if rear_only:
                        hi-=.6+.35*age-(rear_margin+rear_growth*dt+.35*max(0.,ctx.now-tr.event.stamp))
                    radius=self.circle.radius+(tr.radial_lo+tr.radial_hi)/2
                    x,y=self.circle.center+radius*np.array([math.cos(a),math.sin(a)])
                    return (x+(lo+hi)/2*math.cos(h),y+(lo+hi)/2*math.sin(h),h,
                            hi-lo,tr.radial_hi-tr.radial_lo+.4)
            def other_bodies_at(dt):
                bodies=[]
                for other in extra_tracks:
                    age=max(0.,ctx.now-other.event.stamp)+dt
                    a=(other.samples[-1][1]+other.speed*age)/self.circle.radius*self.circle.direction
                    h=a+self.circle.direction*math.pi/2
                    lo,hi=self.extents(other,age)
                    if not other.ready:lo-=2.*dt;hi+=2.*dt
                    radius=self.circle.radius+(other.radial_lo+other.radial_hi)/2
                    x,y=self.circle.center+radius*np.array([math.cos(a),math.sin(a)])
                    bodies.append((x+(lo+hi)/2*math.cos(h),y+(lo+hi)/2*math.sin(h),h,
                                   hi-lo,other.radial_hi-other.radial_lo+.4))
                return bodies
            clear=self.empty_entry.permitted(ctx,self.circle,predictor,body_at,rear_only=rear_only,
                                              other_bodies_at=other_bodies_at if extra_tracks else None)
            return result('CANDIDATE_GAP' if clear else 'UNKNOWN',
                          ('rear-only entry; observed clear road ahead' if rear_only else
                           'single vehicle entry checked' if tracks else 'observed empty entry corridor') if clear
                          else 'entry corridor visibility or joining interval not verified',
                          rear_only=rear_only,
                          single_rear_speed_mps=boundary_speed if rear_only else None,
                          single_rear_age_s=ctx.now-tr.event.stamp if rear_only else None,
                          single_rear_margin_m=rear_margin,single_rear_growth_mps=rear_growth,
                          empty_lane=clear and not tracks,gap_pair=[t.event.value for t in entry_tracks],
                          additional_guard_ids=[t.event.value for t in extra_tracks],
                          evidence_stamp=self.empty_entry.scan)
        if self.off_circle:
            return result('UNKNOWN','vehicle outside circular model',off_circle_ids=self.off_circle)
        stage=self.staging_s(ctx)
        if stage is not None:base['staging_s']=stage
        if len(tracks)<2:
            return result('UNKNOWN','two bounding vehicles not observed')
        circle=self.circle;R=circle.radius; circumference=2*math.pi*R
        evaluation_horizon=max(self.horizon,20.) if hasattr(predictor,'ring_tracks') else self.horizon
        base['horizon_s']=evaluation_horizon
        reference=math.atan2(ctx.y-circle.center[1],ctx.x-circle.center[0])*circle.direction
        base['reference_angle']=reference
        tangent=math.atan2(ctx.y-circle.center[1],ctx.x-circle.center[0])+circle.direction*math.pi/2
        base['ego_aligned']=abs(wrap(ctx.yaw-tangent))<math.radians(25.)
        snapshots=[];uncertain=[]
        if self.local_entry:
            for tr in tracks:
                if 0<=ctx.now-tr.event.stamp<=.45:
                    tr.observe_boundaries((ctx.x,ctx.y),circle)
        for tr in tracks:
            e=tr.event;age=ctx.now-e.stamp
            s=wrap(tr.samples[-1][1]/R-reference)*R+tr.speed*max(0.,age)
            fresh=0<=age<=.45
            if not tr.ready or not fresh:uncertain.append(e.value)
            angle=math.atan2(e.y-circle.center[1],e.x-circle.center[0])+circle.direction*math.pi/2
            dx,dy=ctx.x-e.x,ctx.y-e.y
            view=(dx*math.cos(angle)+dy*math.sin(angle))/max(.01,math.hypot(dx,dy))
            view=view if fresh else None
            back,front=self.extents(tr,0.,view)
            row=dict(id=e.value,s=s,speed=tr.speed,length=tr.length,age=age,stamp=e.stamp,ready=tr.ready and fresh,
                     hidden=max(0.,tr.length-tr.observed_length)/2,
                     radial_lo=tr.radial_lo,radial_hi=tr.radial_hi,motion_ready=tr.ready,
                     back_extent=-back-.6,front_extent=front-.6,polygon=tr.has_polygon,
                     body_anchored=tr.body_bounds is not None,observed_length=tr.observed_length,
                     ever_ready=tr.ever_ready,
                     rear_observed=tr.rear_observed,front_observed=tr.front_observed,
                     rear_estimated=tr.rear_estimated,rear_uncertainty_m=tr.rear_uncertainty,
                     observed_rear_s=s+tr.arc_lo if tr.has_polygon else None,
                     observed_front_s=s+tr.arc_hi if tr.has_polygon else None,
                     polygon_xy=tr.polygon_xy if tr.has_polygon else [])
            if self.local_entry and tr.has_polygon:
                front_ready=tr.front_observed and tr.front_motion.ready and tr.front_motion.stamp==e.stamp
                approach_speed=getattr(tr.front_motion,'approach_speed',tr.front_motion.speed)
                rear_ready=(tr.rear_observed or tr.rear_estimated) and tr.rear_boundary_motion.ready and tr.rear_boundary_motion.stamp==e.stamp
                row.update(front_boundary_ready=front_ready,rear_boundary_ready=rear_ready,
                           front_boundary_speed=approach_speed,front_boundary_fit_speed=tr.front_motion.speed,
                           rear_boundary_speed=tr.rear_boundary_motion.speed,
                           ready=fresh and (front_ready or rear_ready))
                if front_ready:row['front_extent']=tr.arc_hi+(approach_speed-tr.speed)*max(0.,age)
                if rear_ready:row['back_extent']=-tr.arc_lo+tr.rear_uncertainty-(tr.rear_boundary_motion.speed-tr.speed)*max(0.,age)
            elif self.local_entry:
                row.update(front_boundary_ready=False,rear_boundary_ready=False,ready=False)
            snapshots.append(row)
            # Display the same boundary-refined body used by admission; the
            # older centre-based extents can disagree after end-speed correction.
            padding=.6+.35*max(0.,age)
            base['occupied_intervals'].append(dict(row,lo=s-row['back_extent']-padding,
                                                   hi=s+row['front_extent']+padding))
        # Project full ego corners; only times with radial overlap count.
        interactions=[]
        radial_inner=min(t.radial_lo for t in tracks)
        radial_outer=max(t.radial_hi for t in tracks)
        handoffs=[];handoff_by_accel={}
        for accel in self.departure_accels:
            aligned_since=None;handoff_recorded=False
            for dt in np.arange(0.,evaluation_horizon+self.step/2,self.step):
                x,y,h,l,w=predictor.ego_pose(ctx,float(dt),accel)
                dx,dy=x-circle.center[0],y-circle.center[1]
                a=math.atan2(dy,dx); radius=math.hypot(dx,dy)
                radial_half=abs(math.cos(h-a))*l/2+abs(math.sin(h-a))*w/2
                if radius-radial_half>R+radial_outer or radius+radial_half<R+radial_inner:
                    continue
                corners=[]
                for u,v in ((-l/2,-w/2),(-l/2,w/2),(l/2,-w/2),(l/2,w/2)):
                    px=dx+u*math.cos(h)-v*math.sin(h);py=dy+u*math.sin(h)+v*math.cos(h)
                    corners.append(wrap(math.atan2(py,px)*circle.direction-reference)*R)
                mid=wrap(a*circle.direction-reference)*R
                corners=[mid+wrap((v-mid)/R)*R for v in corners]
                lo,hi=min(corners),max(corners)
                if self.local_entry:
                    contact=contact_arc((x,y,h,l,w),circle.center,R,radial_inner,radial_outer,reference,circle.direction)
                    if contact is None:continue
                    lo,hi=contact
                interactions.append((float(dt),lo-.25,hi+.25,accel,(x,y,h,l,w),
                                     radius-radial_half-R,radius+radial_half-R))
                if self.cooperative_rear or self.local_entry:
                    aligned=(abs(wrap(h-a-circle.direction*math.pi/2))<math.radians(25.)
                             and abs(radius-R)<1.5)
                    aligned_since=(float(dt) if aligned_since is None else aligned_since) if aligned else None
                    if not handoff_recorded and aligned_since is not None and dt-aligned_since>=.6-1e-6:
                        handoffs.append(float(dt));handoff_by_accel[accel]=float(dt);handoff_recorded=True
                # Extra samples protect FRONT/third bodies only; rear checks
                # stop at handoff and must not use this following preview.
                if self.local_entry and handoff_recorded and dt>=handoff_by_accel[accel]+2.-1e-6:
                    break
        if not interactions:
            return result('UNKNOWN','no ring interaction within departure horizon')
        first=min(i[0] for i in interactions);last=max(i[0] for i in interactions)
        base['ego_paths']=[dict(accel=a,xy=[list(predictor.ego_pose(ctx,float(t),a)[:2]) for t in np.arange(0.,min(last+1.,evaluation_horizon)+.01,.5)]) for a in self.departure_accels]
        base.update(interaction_start_s=round(first,2),interaction_end_s=round(last,2),
                    ego_overlaps_ring=first==0.,
                    following_handoff_in_s=max(handoffs) if len(handoffs)==len(self.departure_accels) else None)
        if self.local_entry and len(handoffs)!=len(self.departure_accels):
            return result('UNKNOWN','entry does not reach stable lane alignment')
        if not self.local_entry and last>=evaluation_horizon-self.step/2:
            return result('UNKNOWN','departure horizon does not cover exit')
        # Enumerate adjacent circulating pairs at first interaction, including wrap.
        ordered=sorted((r for r in snapshots if r['ready']),key=lambda r:r['s']+r['speed']*first)
        best=None
        for rear0,front0 in zip(ordered,ordered[1:]+ordered[:1]):
            rear=dict(rear0);front=dict(front0)
            if self.local_entry:
                if not rear.get('front_boundary_ready',False) or not front.get('rear_boundary_ready',False):continue
                rear['speed']=rear.get('front_boundary_speed',rear['speed'])
                front['speed']=front.get('rear_boundary_speed',front['speed'])
            if front['s']+front['speed']*first<=rear['s']+rear['speed']*first:
                # Local insertion cannot choose the complementary slot via an
                # unobserved lap around the back of the ring.
                if self.local_entry:continue
                front['s']+=circumference
            separation=front['s']-rear['s']
            # Fresh measured boundaries on the local minor arc can be >35m
            # apart. The old fixed cap rejected the roomiest observed slots.
            max_span=circumference/2 if self.local_entry else 35.
            if not rear['ready'] or not front['ready'] or not 0<separation<=max_span:
                continue
            for shift in ((0.,) if self.local_entry else (-circumference,0.,circumference)):
                minimum=math.inf; bottleneck=None
                responses={}
                if cooperative:
                    for acceleration in self.departure_accels:
                        trace=[(dt,lo) for dt,lo,hi,a,body,rlo,rhi in interactions
                               if a==acceleration and rlo<=rear['radial_hi'] and rhi>=rear['radial_lo']]
                        responses[acceleration]=predict_rear_response(rear['s']+shift,rear['speed'],rear['front_extent']+.6,trace)
                for dt,lo,hi,accel,ego,rlo,rhi in interactions:
                    margin=.6+.35*dt
                    rear_position=responses.get(accel,{}).get(dt,{}).get('s',rear['s']+shift+rear['speed']*dt)
                    rear_end=rear_position+rear['front_extent']+rear_margin+rear_growth*dt+.35*max(0.,rear['age'])
                    front_end=front['s']+shift+front['speed']*dt-front['back_extent']-margin-.35*max(0.,front['age'])
                    rear_room=lo-rear_end if rlo<=rear['radial_hi'] and rhi>=rear['radial_lo'] else math.inf
                    front_room=front_end-hi if rlo<=front['radial_hi'] and rhi>=front['radial_lo'] else math.inf
                    if self.local_entry and dt>handoff_by_accel[accel]+1e-6:rear_room=math.inf
                    room=min(rear_room,front_room)
                    if room<minimum:minimum=room;bottleneck=(dt,ego)
                if best is None or minimum>best[0]:
                    best=(minimum,rear,front,shift,bottleneck,responses)
        if best is None:
            return result('UNKNOWN','no fresh bounded gap in observed arc',uncertain_ids=uncertain)
        minimum,rear,front,shift,bottleneck,responses=best
        base.update(gap_pair=[rear['id'],front['id']],evidence_stamp=min(rear['stamp'],front['stamp']),
                    minimum_gap_margin_m=round(minimum,3),
                    gap_rear_s=rear['s']+shift+rear['front_extent']+rear_margin,
                    gap_front_s=front['s']+shift-front['back_extent']-.6,
                    gap_rear_speed=rear['speed'],gap_front_speed=front['speed'],
                    gap_rear_boundary_source='observed_face' if rear['front_observed'] else 'estimated',
                    gap_front_boundary_source='observed_face' if front['rear_observed'] else 'estimated',
                    rear_response=[dict(t=t,**v) for t,v in responses.get(self.departure_accels[1],{}).items()])
        current=[i for i in interactions if i[0]==0.]
        if current:
            ego_front=max(i[2] for i in current)
            leads=[r for r in snapshots if r['ready'] and 0<r['s']-r['back_extent']-ego_front<35.]
            if leads:
                lead=min(leads,key=lambda r:r['s']-r['back_extent'])
                distance=lead['s']-lead['back_extent']-ego_front
                base['following_gap_m']=distance
                base['following_speed_mps']=following_speed(lead['speed'],ctx.v,distance,getattr(ctx,'ring_tuck',0.))   # no ring speed limit of its own
        # Rear cooperation must never mask a front/third-body collision.
        # Keep the full interaction horizon, including after tangent alignment.
        conflicts=[]
        seen_sides=set()
        rear_check=None
        for when,lo,hi,accel,body,rlo,rhi in sorted(interactions):
            margin=.6+.35*when
            rear_position=responses.get(accel,{}).get(when,{}).get('s',rear['s']+shift+rear['speed']*when)
            end=rear_position+rear['front_extent']+rear_margin+rear_growth*when+.35*max(0.,rear['age'])
            if (rlo<=rear['radial_hi'] and rhi>=rear['radial_lo'] and
                    (not self.local_entry or when<=handoff_by_accel[accel]+1e-6)):
                clearance=lo-end
                if rear_check is None or clearance<rear_check['clearance_m']:
                    physical_end=rear_position+rear['front_extent']
                    observed=rear.get('observed_front_s')
                    observed_end=(rear_position+observed-rear['s']) if observed is not None else None
                    padding=rear_margin+rear_growth*when+.35*max(0.,rear['age'])+.25
                    rear_check=dict(id=rear['id'],speed_mps=rear['speed'],at_s=when,
                        radial_width_m=rear['radial_hi']-rear['radial_lo'],
                        age_s=rear['age'],ego_accel_mps2=accel,
                        observed_front_s=observed_end,model_front_s=physical_end,
                        guard_front_s=end,ego_rear_s=lo+.25,ego_guard_rear_s=lo,
                        body_gap_m=lo+.25-physical_end,required_margin_m=padding,
                        clearance_m=clearance,base_margin_m=rear_margin,future_margin_m=rear_growth*when,
                        age_margin_m=.35*max(0.,rear['age']),ego_margin_m=.25,
                        hidden_extension_m=max(0.,physical_end-observed_end) if observed_end is not None else None,
                        boundary_source='observed face' if rear['front_observed'] else 'estimated end')
            start=front['s']+shift+front['speed']*when-front['back_extent']-margin-.35*max(0.,front['age'])
            for side,hit in (
                ('rear',lo<=end and rlo<=rear['radial_hi'] and rhi>=rear['radial_lo']
                 and (not self.local_entry or when<=handoff_by_accel[accel]+1e-6)),
                ('front',hi>=start and rlo<=front['radial_hi'] and rhi>=front['radial_lo'])):
                if hit and side not in seen_sides:
                    conflicts.append((when,side,body,None))
                    seen_sides.add(side)
        if rear_check is not None:base['rear_check']=rear_check
        # Lost or unclassified bodies are still obstacles, including bodies
        # whose center is outside the selected pair but whose extent overlaps.
        for dt,lo,hi,accel,ego,rlo,rhi in sorted(interactions):
            for row in snapshots:
                if row['id'] in base['gap_pair']:continue
                if not inspect_unselected_body(self.local_entry,ctx,row['age'],dt):continue
                if rlo>row['radial_hi'] or rhi<row['radial_lo']:continue
                uncertainty=.6+.35*(dt+max(0.,row['age']))
                if not row['motion_ready']:uncertainty+=2.*dt
                back=row['back_extent']+uncertainty
                front_extent=row['front_extent']+uncertainty
                center=row['s']+row['speed']*dt
                for offset in (-circumference,0.,circumference):
                    if center+offset+front_extent>=lo and center+offset-back<=hi:
                        conflicts.append((dt,'other',ego,row));break
            if any(c[1]=='other' for c in conflicts):break
        if conflicts:
            relevant=[c for c in conflicts if c[1]!='rear'] if cooperative else conflicts
            dt,side,ego,row=min(relevant or conflicts,key=lambda c:c[0])
            base['conflict_side']=side
            rear_times=[c[0] for c in conflicts if c[1]=='rear']
            if rear_times:base['rear_conflict_in_s']=round(min(rear_times),2)
            if row is not None:base['track']=row['id']
            return result('UNKNOWN' if row is not None and not row['ready'] else 'WAIT',
                          'another observed or occluded body occupies selected gap' if row is not None else
                          'ego does not fit between circulating bounds throughout entry',
                          conflict_in_s=round(dt,2),conflict_model='circular_gap',
                          conflict_xy=list(ego[:2]),conflict_ego=list(ego))
        if self.local_entry:
            # In this simulator, NPC following owns rear separation after
            # insertion. Only the pre-handoff rear checks above can veto entry.
            return result('CANDIDATE_GAP','fresh front/rear bound entry; handoff to local following')
        return result('CANDIDATE_GAP',
                      'entry fits with delayed NPC following response' if cooperative else
                      'same observed pair bounds full departure; visibility not certified')
