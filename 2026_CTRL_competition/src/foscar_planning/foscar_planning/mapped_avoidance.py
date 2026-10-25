"""Map-only pavement checks for a tunnel offset manoeuvre. No sensor correction."""
import math
import numpy as np
from .avoid_planner import Plan
from .frenet import to_frenet


def deep_on_lanes(ctx, points, depth):
    """Tunnel: points more than `depth` inside the mapped lanes' union, across the route. A negative seam tolerance
    shrank each lane on its own and left a 1 m hole on the line between them (2026-10-10): a person crossing it read
    off our road, and an obstacle on the lane line could have been no box."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    lanes = getattr(ctx.road_map, 'road_map', ctx.road_map)
    if lanes is None or ctx.route is None or not len(pts):
        return False
    s = to_frenet(ctx.route, *pts.mean(axis=0), ctx.s + ctx.s_offset)[0]
    h = ctx.route.heading_at(s)
    # contains_road closes surveyed seams with 0.15 m (its default), which also widens the outer edge: without it
    # back, 0.5 m was 0.35 m and a deer standing beside the lanes (read 0.38 m on) was a box again (2026-10-10)
    n = np.array([-math.sin(h), math.cos(h)])*(depth + .15)
    return bool(np.any(np.asarray(lanes.contains_road(pts)) & np.asarray(lanes.contains_road(pts + n))
                       & np.asarray(lanes.contains_road(pts - n))))


def road_contains(ctx, points):
    if ctx.mission_name == 'STATIC_OBSTACLE' and hasattr(ctx.road_map, 'contains_intersection_road'):
        return ctx.road_map.contains_intersection_road(points)
    return ctx.road_map.contains_road(points)


def paved_sweep(ctx, d, start, length, return_start, return_length, front, rear, half, d0=0., from_s=None):
    """Body sweep of the entry (d0 -> d) and return (d -> 0) curves over mapped/walled road.

    Only the part still ahead of the rear bumper (from_s) is checked, from the offset
    the car actually starts at (d0). Checking the already-driven stretch from offset 0
    rejected a second tunnel barricade's valid swerve and stopped the car (2026-09-26).
    """
    if ctx.road_map is None or ctx.route is None:
        return False
    end = return_start+return_length
    lo = max(0., start-front-rear, (from_s-rear) if from_s is not None else -math.inf)
    ss=np.arange(lo, end+.5, 1.)
    a,w=np.meshgrid(np.linspace(-rear,front,7),np.linspace(-half,half,5))
    a,w=a.ravel(),w.ravel()
    points=[]
    for s in ss:
        u=np.clip((s-start)/length,0.,1.)
        off=d0+(d-d0)*u**3*(10-15*u+6*u*u)
        slope=(d-d0)*30*u*u*(1-u)**2/length
        if s>=return_start:
            u=np.clip((s-return_start)/return_length,0.,1.)
            off=d*(1-u**3*(10-15*u+6*u*u))
            slope=-d*30*u*u*(1-u)**2/return_length
        local=s+ctx.s_offset
        if local<0 or local>ctx.route.s[-1]:return False
        x,y=ctx.route.at(local);h=ctx.route.heading_at(local)
        x-=math.sin(h)*off;y+=math.cos(h)*off
        yaw=h+math.atan(slope);co,si=math.cos(yaw),math.sin(yaw)
        # Sample the whole inflated body, including interior, so disconnected
        # pavement on each side cannot make a median look traversable.
        points.extend(np.column_stack((x+a*co-w*si,y+a*si+w*co)))
    return bool(road_contains(ctx, points).all())


def paved_offset(ctx, d, s0, s1, half):
    """Constant offset d from s0 to s1 (global) stays on mapped/walled pavement, body +- half."""
    if ctx.road_map is None or ctx.route is None:
        return False
    points=[]
    for s in np.arange(s0,s1+.5,.5):
        local=s+ctx.s_offset
        if local<0 or local>ctx.route.s[-1]:return False
        x,y=ctx.route.at(local);h=ctx.route.heading_at(local)
        for w in (-half,0.,half):
            points.append((x-math.sin(h)*(d+w),y+math.cos(h)*(d+w)))
    return bool(road_contains(ctx, points).all())


def paved_moves(ctx, d0, moves, s_from, s_to, front, rear, half):
    """The body following a chain of quintic moves [(d, start, length)] from offset d0 stays on the
    mapped/walled pavement between rear-axle s_from and s_to."""
    if ctx.road_map is None or ctx.route is None:
        return False
    if s_to < s_from:
        return True
    # Vectorized form of Route.at / heading_at; include the exact endpoint.
    ss = np.unique(np.r_[np.arange(s_from,s_to,1.),s_to])
    local = ss + ctx.s_offset
    if np.any(local < 0.) or np.any(local > ctx.route.s[-1]):
        return False
    off, slope = np.full(len(ss),d0,dtype=float), np.zeros(len(ss))
    prev = d0
    for d,start,length in moves:
        active = ss >= start
        u = np.clip((ss-start)/max(length,1e-6),0.,1.)
        off[active] = (prev+(d-prev)*u**3*(10-15*u+6*u*u))[active]
        slope[active] = ((d-prev)*30*u*u*(1-u)**2/max(length,1e-6))[active]
        prev = d
    x = np.interp(local,ctx.route.s,ctx.route.points[:,0])
    y = np.interp(local,ctx.route.s,ctx.route.points[:,1])
    idx = np.clip(np.searchsorted(ctx.route.s,local,side='right')-1,0,len(ctx.route.delta)-1)
    heading = np.arctan2(ctx.route.delta[idx,1],ctx.route.delta[idx,0])
    x -= np.sin(heading)*off
    y += np.cos(heading)*off
    yaw = heading+np.arctan(slope)
    a,w = np.meshgrid(np.linspace(-rear,front,7),np.linspace(-half,half,5))
    a,w = a.ravel()[None,:],w.ravel()[None,:]
    co,si = np.cos(yaw)[:,None],np.sin(yaw)[:,None]
    points = np.stack((x[:,None]+a*co-w*si,y[:,None]+a*si+w*co),axis=-1).reshape(-1,2)
    return bool(road_contains(ctx, points).all())


def mapped_plan(ctx, planner, boxes, prev_d, start, length, return_start, return_length, front, rear, d0=0.):
    plan=planner.plan(boxes,ctx.s,prev_d)
    if not hasattr(planner,'map_cache'):planner.map_cache={}
    if len(planner.map_cache)>256:planner.map_cache.clear()
    def paved(d):
        key=(getattr(ctx.road_map,'cache_key',id(ctx.road_map)),id(ctx.route),ctx.s_offset,d,start,length,return_start)
        if key not in planner.map_cache:
            # Cheap constant-offset check rejects the wrong roadside before
            # evaluating the entry/return sweep with the full vehicle body.
            points=[]
            for s in np.arange(start+length,return_start+.5,.5):
                x,y=ctx.route.at(s+ctx.s_offset);h=ctx.route.heading_at(s+ctx.s_offset)
                for w in (-planner.half,0.,planner.half):
                    points.append((x-math.sin(h)*(d+w),y+math.cos(h)*(d+w)))
            valid=(ctx.road_map is not None and bool(points)
                   and road_contains(ctx, points).all())
            planner.map_cache[key]=valid
        return planner.map_cache[key]
    for c in plan.candidates:
        if not c.feasible:continue
        if prev_d and c.d*prev_d<=0:
            c.feasible=False;c.reason='keep committed side'
        elif ctx.route is None or not paved(c.d):
            c.feasible=False;c.reason='outside mapped pavement or map unavailable'
    while True:
        ok=[c for c in plan.candidates if c.feasible]
        if not ok:return Plan(prev_d,0.,True,plan.candidates,plan.nearest_s)
        # Smallest lateral move from where the car is (d0) that clears the obstacle.
        # The old rule (centre of the widest free band) swung +4-4.75 m in the wide
        # tunnel and, on a barricade seen only partly, picked the side that later
        # closed, forcing a 4 m re-plan in 7 m (FAULT, 2026-09-26). Feasibility
        # already includes car half-width + margin and the obstacle pad.
        best=min(ok,key=lambda c:(abs(c.d-d0),abs(c.d)))
        if prev_d:
            # Keep the committed side, but move to the nearest feasible offset on it
            # when the held one is now blocked. Both ways: a box growing toward us
            # needs more, a wall measured closer than first assumed needs less
            # (2026-09-26 sim loop: -1.25 blocked, -1.0 free -> the car stopped).
            # Feasibility already includes the box and wall margins.
            same=[c for c in ok if c.d*prev_d>0]
            if not same:return Plan(prev_d,0.,True,plan.candidates,plan.nearest_s)
            best=min(same,key=lambda c:(abs(c.d-prev_d),-abs(c.d)))
        candidate_return_length = return_length(best.d) if callable(return_length) else return_length
        # From the rear bumper, quantised to 5 m so the cache survives between ticks.
        from_s=5.*math.floor(ctx.s/5.)
        key=('sweep',getattr(ctx.road_map,'cache_key',id(ctx.road_map)),id(ctx.route),ctx.s_offset,best.d,start,length,
             return_start,candidate_return_length,d0,from_s)
        if key not in planner.map_cache:
            planner.map_cache[key]=paved_sweep(ctx,best.d,start,length,return_start,candidate_return_length,front,rear,
                                               planner.half,d0,from_s)
        if planner.map_cache[key]:
            return Plan(best.d,math.inf,False,plan.candidates,plan.nearest_s)
        best.feasible=False;best.reason='turning body outside mapped pavement'
