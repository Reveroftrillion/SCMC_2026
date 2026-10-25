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
    safe_direction = np.where(parallel, 1., direction)
    # 교차 위치는 origin + t * direction의 t다. 실제 잔여 거리는 뒤에서 ray 길이를 곱한다.
    lower_intersections, upper_intersections = (lower - origin) / safe_direction, (upper - origin) / safe_direction
    entry_parameter = np.where(parallel, -np.inf, np.minimum(lower_intersections, upper_intersections)).max(axis=1)
    exit_parameter = np.where(parallel, np.inf, np.maximum(lower_intersections, upper_intersections)).min(axis=1)
    outside_parallel = (parallel & ((origin < lower) | (origin > upper))).any(axis=1)
    intersects = ~outside_parallel & (exit_parameter > np.maximum(entry_parameter, 0.))
    occupied = ((points >= lower) & (points <= upper)).all(axis=1)
    distance_beyond_box = (1. - exit_parameter) * np.linalg.norm(direction, axis=1)
    through = intersects & (exit_parameter > 0.) & (distance_beyond_box > beyond_m) & ~occupied
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
    cos_yaw,sin_yaw=np.cos(yaw),np.sin(yaw)
    rotation=np.array([[cos_yaw,-sin_yaw],[sin_yaw,cos_yaw]])
    local_points=points.copy();local_points[:,:2]=(local_points[:,:2]-center)@rotation
    local_origin=origin.copy();local_origin[:2]=(local_origin[:2]-center)@rotation
    # The torso bands end at the body's own top when it is lower (user 2026-10-09: rays over a 0.5-0.7 m tunnel
    # barricade, 0.8-1.1 m up at 20-26 m, cleared it while it stood there). A low body's bands start at half its
    # height, so rays through its lower half that land on the ground behind it clear it (a 0.44 m one-frame ghost
    # stayed forever with a 0.4 m floor, user 2026-10-09); under 0.4 m: never cleared
    top=min(1.8,float(top)) if top is not None and np.isfinite(top) else 1.8
    floor=min(.4,top/2)
    if top-floor<.2:return False
    lower=np.array([-length/2,-width/2,floor]);upper=np.array([length/2,width/2,top])
    if np.any(np.all((local_points>=lower)&(local_points<=upper),axis=1)):return False
    # Filter angularly unrelated rays before testing the eight cells.
    direction=local_points-local_origin
    distance=np.linalg.norm(local_origin[:2]);radius=np.hypot(length,width)/2
    if distance<=radius:return False
    cosine=(direction[:,:2]@(-local_origin[:2]))/np.maximum(1e-9,np.linalg.norm(direction[:,:2],axis=1)*distance)
    local_points=local_points[cosine>=np.sqrt(max(0.,1-(radius/distance)**2))]
    for x_cell in range(2):
        for y_cell in range(2):
            for z_cell in range(2):
                cell_lower=lower+(upper-lower)*np.array([x_cell,y_cell,z_cell])/2
                cell_upper=cell_lower+(upper-lower)/2
                evidence=ray_box_evidence(local_origin,local_points,cell_lower,cell_upper)
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
    cos_yaw,sin_yaw=np.cos(yaw),np.sin(yaw);rotation=np.array([[cos_yaw,-sin_yaw],[sin_yaw,cos_yaw]])
    local_points=points.copy();local_points[:,:2]=(local_points[:,:2]-center)@rotation
    local_origin=np.asarray(origin,dtype=float).copy();local_origin[:2]=(local_origin[:2]-center)@rotation
    inside=(abs(local_points[:,0])<=length/2)&(abs(local_points[:,1])<=width/2)
    if np.any(inside&(local_points[:,2]>=.35)&(local_points[:,2]<=2.2)):return False
    # Only rays whose XY direction can intersect this patch need slab tests.
    distance_squared=float(local_origin[0]**2+local_origin[1]**2);radius_squared=(length**2+width**2)/4
    if distance_squared<=radius_squared:return False
    direction=local_points[:,:2]-local_origin[:2]
    dot=-(direction[:,0]*local_origin[0]+direction[:,1]*local_origin[1])
    cross=direction[:,0]*local_origin[1]-direction[:,1]*local_origin[0]
    local_points=local_points[(dot>0)&(cross**2<=dot**2*radius_squared/(distance_squared-radius_squared))]
    for cell_x in (-length/2,0.):
        for cell_y in (-width/2,0.):
            if ray_box_evidence(local_origin,local_points,[cell_x,cell_y,.35],[cell_x+length/2,cell_y+width/2,1.25])['through']<2:
                return False
    return True
