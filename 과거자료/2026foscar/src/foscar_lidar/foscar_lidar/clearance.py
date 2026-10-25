"""Repeated raw-ray checks for explicitly requested stale track volumes."""
import math
import numpy as np
from .free_space import body_clearance, lane_patch_clearance


class ClearanceVerifier:
    def __init__(self):
        self.queries={};self.confirmations={}

    def accept(self,data):
        if not isinstance(data,dict) or data.get('frame')!='map':return
        stamp=data.get('stamp');rows=data.get('objects')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not isinstance(rows,list):return
        accepted={}
        for row in rows[:64]:
            try:
                values=[float(row[k]) for k in ('x','y','yaw','length','width','vx','vy','last_seen')]
                if not isinstance(row['id'],str) or not all(map(math.isfinite,values)):continue
                if not 0<row['length']<=30 or not 0<row['width']<=12 or row['last_seen']>stamp:continue
                if row.get('purpose')=='ring_boundary':
                    if (not isinstance(row.get('track_id'),str) or row.get('side') not in ('front','rear')
                            or not all(math.isfinite(float(row[k])) for k in ('edge_s','speed'))):continue
                if 'circle_center' in row:
                    center=np.asarray(row['circle_center'],dtype=float)
                    if (center.shape!=(2,) or not np.isfinite(center).all()
                            or not math.isfinite(row['circle_radius']) or row['circle_radius']<=0.
                            or row['circle_direction'] not in (-1,1)):continue
                accepted[row['id']]=dict(row,stamp=stamp)
            except (KeyError,TypeError,ValueError):continue
        self.queries=accepted

    @staticmethod
    def current_endpoint(q,footprints,stamp):
        """Bind the ray patch to this sweep's endpoint, never a moving centroid."""
        obj=next((o for o in footprints if o['id']==q['track_id']),None)
        if obj is None:return None
        center=np.asarray(q['circle_center']);radius=q['circle_radius'];direction=q['circle_direction']
        pts=np.asarray(obj['xy'])-center
        ref=math.atan2(obj['center'][1]-center[1],obj['center'][0]-center[0])*direction
        arcs=np.arctan2(pts[:,1],pts[:,0])*direction
        rel=np.arctan2(np.sin(arcs-ref),np.cos(arcs-ref))*radius
        if len(rel)<2 or np.ptp(rel)<1.2:return None
        edge=ref*radius+(float(min(rel)) if q['side']=='rear' else float(max(rel)))
        edge=q['edge_s']+math.atan2(math.sin((edge-q['edge_s'])/radius),math.cos((edge-q['edge_s'])/radius))*radius
        angle=(edge+(-.6 if q['side']=='rear' else .6))/radius*direction
        radii=np.linalg.norm(pts,axis=1)-radius
        lo=min(-1.15,float(min(radii))-.2);hi=max(1.15,float(max(radii))+.2)
        x,y=center+(radius+(lo+hi)/2)*np.array([math.cos(angle),math.sin(angle)])
        return dict(q,x=float(x),y=float(y),yaw=angle+direction*math.pi/2,
                    width=hi-lo,edge_s=edge,stamp=stamp)

    def check(self,stamp,points,origin,ego,footprints=None):
        self.boundaries=[]
        if ego is None:return []
        ex,ey,yaw=ego;c,s=math.cos(yaw),math.sin(yaw)
        cleared=[];next_confirm={}
        for key,q in self.queries.items():
            dt=stamp-q['stamp']
            boundary=q.get('purpose')=='ring_boundary'
            if not 0<=dt<=.35 or (not boundary and stamp-q['last_seen']<.25):continue
            if boundary and 'circle_center' in q:
                q=self.current_endpoint(q,footprints or [],stamp)
                if q is None:continue
                dt=0.
            x=q['x']+q['vx']*dt-ex;y=q['y']+q['vy']*dt-ey
            center=[c*x+s*y,-s*x+c*y]
            empty=q.get('purpose')=='empty_entry'
            check=lane_patch_clearance if empty or boundary or q.get('purpose')=='ring_vehicle' else body_clearance
            pad=0. if empty or boundary else .4
            if not check(origin,points,center,q['length']+pad,q['width']+pad,q['yaw']-yaw):continue
            old=self.confirmations.get(key)
            consistent=old and (old[0]==q['last_seen'] or (boundary and len(old)>3 and
                math.hypot(x+ex-(old[3]+q['vx']*(stamp-old[1])),y+ey-(old[4]+q['vy']*(stamp-old[1])))<.5))
            count=old[2]+1 if consistent and 0<stamp-old[1]<=.25 else 1
            next_confirm[key]=(q['last_seen'],stamp,count,x+ex,y+ey)
            if count>=2:
                if boundary:
                    self.boundaries.append(dict(id=q['track_id'],side=q['side'],stamp=stamp,
                        edge_s=q['edge_s']+q['speed']*dt,speed=q['speed']))
                else:cleared.append(dict(id=key,last_seen=q['last_seen']))
        self.confirmations=next_confirm
        return cleared
