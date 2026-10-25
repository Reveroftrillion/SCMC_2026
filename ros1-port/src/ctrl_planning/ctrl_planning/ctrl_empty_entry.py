"""Observed-empty alternative to a two-car entry gap; no detection is not proof."""
import math
import numpy as np
from .ctrl_prediction import rectangles_overlap
from .ctrl_departure_profile import ACCELERATIONS


class EmptyEntry:
    def __init__(self):
        self.anchor=None;self.rows=[];self.stamp=-math.inf;self.scan=-math.inf;self.proofs={}

    def queries(self,ctx,circle,waiting_s=None):
        if circle is None or (waiting_s is not None and ctx.route is None):return []
        anchor=ctx.route.at(waiting_s+ctx.s_offset) if waiting_s is not None else (ctx.x,ctx.y)
        if self.anchor is None or math.hypot(anchor[0]-self.anchor[0],anchor[1]-self.anchor[1])>.5:
            self.anchor=tuple(anchor);self.rows=[];self.stamp=-math.inf
            angle=math.atan2(anchor[1]-circle.center[1],anchor[0]-circle.center[0])
            for offset in range(-30,19,4):
                a=angle+offset/circle.radius*circle.direction
                x,y=circle.center+circle.radius*np.array([math.cos(a),math.sin(a)])
                self.rows.append(dict(id=f'empty-entry:{ctx.now}:{offset}',last_seen=ctx.now-.3,
                    x=float(x),y=float(y),yaw=a+circle.direction*math.pi/2,
                    length=4.4,width=3.5,vx=0.,vy=0.,purpose='empty_entry'))
        return self.rows

    def observe(self,data):
        stamp=data['stamp']
        if stamp<=self.scan:return
        self.scan=stamp;self.stamp=-math.inf
        proofs={r.get('id'):r.get('last_seen') for r in data.get('cleared',[]) if isinstance(r,dict)}
        self.proofs=proofs
        if self.rows and all(proofs.get(r['id'])==r['last_seen'] for r in self.rows):
            self.stamp=stamp

    def permitted(self,ctx,circle,predictor,body_at=None,rear_only=False,other_bodies_at=None):
        if self.anchor is None or not 0<=ctx.now-self.scan<=.25:return False
        body=body_at(0.) if body_at else None
        reference=math.atan2(self.anchor[1]-circle.center[1],self.anchor[0]-circle.center[0])
        def arc(x,y):
            delta=(math.atan2(y-circle.center[1],x-circle.center[0])-reference)*circle.direction
            return math.atan2(math.sin(delta),math.cos(delta))*circle.radius
        rear_front=None
        if rear_only and body is not None:
            x,y,h,l,w=body;c,s=math.cos(h),math.sin(h)
            rear_front=max(arc(x+u*c-v*s,y+u*s+v*c)
                           for u,v in ((-l/2,-w/2),(l/2,-w/2),(l/2,w/2),(-l/2,w/2)))
        for row in self.rows:
            patch=(row['x'],row['y'],row['yaw'],row['length'],row['width'])
            clear=self.proofs.get(row['id'])==row['last_seen']
            # The known approaching car bounds this slot. The road behind it
            # is not part of the slot and may be occluded by the car itself.
            # Applies only to the single-direction circular-lane model.
            if rear_front is not None and arc(row['x'],row['y'])+row['length']/2<=rear_front:
                continue
            if not clear and not (body is not None and rectangles_overlap(patch,body)):
                return False
        # The observed upstream 30m must cover the complete joining interval,
        # assuming an unseen vehicle approaches at the configured speed bound.
        deadline=(30.-3.)/max(20./3.6,predictor.max_speed)-.4
        for accel in ACCELERATIONS:
            aligned_since=None;joined=False
            for t in np.arange(0.,deadline+.001,.2):
                ego=predictor.ego_pose(ctx,float(t),accel)
                x,y,h,_,_=ego
                if body_at is not None and rectangles_overlap(ego,body_at(float(t))):return False
                if other_bodies_at is not None and any(rectangles_overlap(ego,b) for b in other_bodies_at(float(t))):return False
                a=math.atan2(y-circle.center[1],x-circle.center[0])
                delta=(a-reference)*circle.direction
                arc=math.atan2(math.sin(delta),math.cos(delta))*circle.radius
                if not -27.<arc<17.:return False
                error=h-a-circle.direction*math.pi/2
                aligned=(abs(math.atan2(math.sin(error),math.cos(error)))<math.radians(25.)
                         and abs(math.hypot(x-circle.center[0],y-circle.center[1])-circle.radius)<1.5)
                aligned_since=(float(t) if aligned_since is None else aligned_since) if aligned else None
                if aligned_since is not None and t-aligned_since>=.6-1e-6:
                    joined=True;break
            if not joined:return False
        return True
