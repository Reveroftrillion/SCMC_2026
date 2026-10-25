"""Diagnostic ray evidence, not a complete free-space or driving permission test.

All inputs must share a frame. Use raw returns before ground/ROI filtering.
Missing returns and rays ending before a volume give no clearance evidence.
Counts alone do not prove that the entire volume is empty.
"""

import numpy as np


def ray_box_evidence(origin, points, lower, upper, beyond_m=0.2):
    origin = np.asarray(origin, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (origin.shape != (3,) or lower.shape != (3,) or upper.shape != (3,)
            or not np.isfinite([origin, lower, upper]).all()
            or np.any(upper <= lower) or not np.isfinite(beyond_m) or beyond_m < 0):
        raise ValueError('finite origin and nonempty box required')
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    direction = points - origin
    parallel = np.abs(direction) < 1e-10
    safe = np.where(parallel, 1., direction)
    a, b = (lower - origin) / safe, (upper - origin) / safe
    near = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=1)
    far = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=1)
    outside_parallel = (parallel & ((origin < lower) | (origin > upper))).any(axis=1)
    intersects = ~outside_parallel & (far > np.maximum(near, 0.))
    occupied = ((points >= lower) & (points <= upper)).all(axis=1)
    remaining = (1. - far) * np.linalg.norm(direction, axis=1)
    through = intersects & (far > 0.) & (remaining > beyond_m) & ~occupied
    return {'through': int(through.sum()), 'occupied': int(occupied.sum())}


def body_clearance(origin, points, center, length, width, yaw, top=1.8):
    """Positive ray coverage of a stale vehicle hypothesis, not unseen space.

    Require traversals in all 2x2 footprint cells at both torso height bands.
    Any raw occupied return vetoes removal. Ground/absent/occluded rays do not
    clear a track. Caller must require repeated fresh sweeps.
    """
    origin=np.asarray(origin,dtype=float)
    points=np.asarray(points,dtype=float).reshape(-1,3)
    center=np.asarray(center,dtype=float)
    if (origin.shape!=(3,) or center.shape!=(2,) or not np.isfinite(origin).all()
            or not np.isfinite(center).all() or not all(np.isfinite([length,width,yaw]))
            or length<=0 or width<=0):return False
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<8:return False
    c,s=np.cos(yaw),np.sin(yaw)
    rotation=np.array([[c,-s],[s,c]])
    local=points.copy();local[:,:2]=(local[:,:2]-center)@rotation
    o=origin.copy();o[:2]=(o[:2]-center)@rotation
    # The torso bands end at the body's own top when it is lower (user 2026-10-09: rays over a 0.5-0.7 m tunnel
    # barricade, 0.8-1.1 m up at 20-26 m, cleared it while it stood there). A low body's bands start at half its
    # height, so rays through its lower half that land on the ground behind it clear it (a 0.44 m one-frame ghost
    # stayed forever with a 0.4 m floor, user 2026-10-09); under 0.4 m: never cleared
    top=min(1.8,float(top)) if top is not None and np.isfinite(top) else 1.8
    floor=min(.4,top/2)
    if top-floor<.2:return False
    lower=np.array([-length/2,-width/2,floor]);upper=np.array([length/2,width/2,top])
    if np.any(np.all((local>=lower)&(local<=upper),axis=1)):return False
    # Filter angularly unrelated rays before testing the eight cells.
    direction=local-o
    distance=np.linalg.norm(o[:2]);radius=np.hypot(length,width)/2
    if distance<=radius:return False
    cosine=(direction[:,:2]@(-o[:2]))/np.maximum(1e-9,np.linalg.norm(direction[:,:2],axis=1)*distance)
    local=local[cosine>=np.sqrt(max(0.,1-(radius/distance)**2))]
    for xi in range(2):
        for yi in range(2):
            for zi in range(2):
                lo=lower+(upper-lower)*np.array([xi,yi,zi])/2
                hi=lo+(upper-lower)/2
                evidence=ray_box_evidence(o,local,lo,hi)
                if evidence['through']<2:return False
    return True


def lane_patch_clearance(origin, points, center, length, width, yaw):
    """Repeated low-body ray coverage for a mapped empty traffic-lane patch.

    Four horizontal cells must each have through returns at vehicle body height.
    Raw occupied returns anywhere up to 2.2m veto clearance, including unclustered
    objects. Missing, ground-only and occluded returns do not count as free rays.
    """
    points=np.asarray(points,dtype=float).reshape(-1,3)
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<8:return False
    c,s=np.cos(yaw),np.sin(yaw);rotation=np.array([[c,-s],[s,c]])
    local=points.copy();local[:,:2]=(local[:,:2]-center)@rotation
    o=np.asarray(origin,dtype=float).copy();o[:2]=(o[:2]-center)@rotation
    inside=(abs(local[:,0])<=length/2)&(abs(local[:,1])<=width/2)
    if np.any(inside&(local[:,2]>=.35)&(local[:,2]<=2.2)):return False
    # Only rays whose XY direction can intersect this patch need slab tests.
    distance2=float(o[0]**2+o[1]**2);radius2=(length**2+width**2)/4
    if distance2<=radius2:return False
    direction=local[:,:2]-o[:2]
    dot=-(direction[:,0]*o[0]+direction[:,1]*o[1])
    cross=direction[:,0]*o[1]-direction[:,1]*o[0]
    local=local[(dot>0)&(cross**2<=dot**2*radius2/(distance2-radius2))]
    for u in (-length/2,0.):
        for v in (-width/2,0.):
            if ray_box_evidence(o,local,[u,v,.35],[u+length/2,v+width/2,1.25])['through']<2:
                return False
    return True
