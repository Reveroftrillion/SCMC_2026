"""Map-aligned ray queries just beyond visible cluster endpoints."""
import math
import numpy as np


def endpoint_queries(track,circle,now):
    if not track.has_polygon or not 0<=now-track.event.stamp<=.25:return []
    # Tiny isolated returns are not evidence of a vehicle side.
    if track.observed_length<1.2:return []
    rows=[];age=now-track.event.stamp
    for side,offset,sign in (('rear',track.arc_lo,-1.),('front',track.arc_hi,1.)):
        edge=track.samples[-1][1]+offset+track.speed*age
        angle=(edge+sign*.6)/circle.radius*circle.direction
        radial=(track.radial_lo+track.radial_hi)/2
        x,y=circle.center+(circle.radius+radial)*np.array([math.cos(angle),math.sin(angle)])
        heading=angle+circle.direction*math.pi/2
        rows.append(dict(id=f'boundary:{track.event.value}:{side}',track_id=track.event.value,
            side=side,last_seen=track.event.stamp,x=float(x),y=float(y),yaw=heading,
            length=.8,width=track.radial_hi-track.radial_lo,purpose='ring_boundary',
            vx=track.speed*math.cos(heading),vy=track.speed*math.sin(heading),
            edge_s=edge,speed=track.speed,circle_center=circle.center.tolist(),
            circle_radius=circle.radius,circle_direction=circle.direction))
    return rows


def side_supported(track,side):
    proof=track.side_proofs.get(side)
    if proof is None or not 0<=track.event.stamp-proof['stamp']<=.25:return False
    actual=track.samples[-1][1]+(track.arc_lo if side=='rear' else track.arc_hi)
    expected=proof['edge_s']+proof['speed']*(track.event.stamp-proof['stamp'])
    return abs(actual-expected)<=.25
