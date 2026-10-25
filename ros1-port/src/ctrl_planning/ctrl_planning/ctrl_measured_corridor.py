"""Observed surface clearance along ego's route, without inferred vehicle boxes."""
import math
import numpy as np
from .ctrl_following_policy import following_speed


def corridor_distance(ctx,polygon):
    pts=np.asarray(polygon,dtype=float)
    if pts.ndim!=2 or pts.shape[1]!=2 or len(pts)<2 or not np.isfinite(pts).all():return None
    stations=np.arange(ctx.s+ctx.s_offset-.8,ctx.s+ctx.s_offset+34.,.2)
    path=np.asarray([ctx.route.at(float(s)) for s in stations])
    edges=np.roll(pts,-1,axis=0)-pts
    outline=np.vstack([p+np.linspace(0.,1.,max(2,int(np.linalg.norm(e)/.2)+1))[:,None]*e
                       for p,e in zip(pts,edges)])
    distances=np.sum((outline[:,None,:]-path[None,:,:])**2,axis=2)
    nearest=np.argmin(distances,axis=1)
    hits=nearest[distances[np.arange(len(outline)),nearest]<=(.946+.25)**2]
    # A wide hull can cover the route while every visible edge is outside it.
    area=abs(float(np.sum(pts[:,0]*np.roll(pts[:,1],-1)-pts[:,1]*np.roll(pts[:,0],-1))))
    if len(pts)>=3 and area>1e-6:
        rel=path[:,None,:]-pts[None,:,:]
        cross=edges[None,:,0]*rel[:,:,1]-edges[None,:,1]*rel[:,:,0]
        inside=np.all(cross>=-1e-8,axis=1)|np.all(cross<=1e-8,axis=1)
        hits=np.r_[hits,np.flatnonzero(inside)]
    if not len(hits):return None
    # Rear axle to front bumper: 1.4825 + 4.635/2 = 3.8m.
    return max(0.,float(stations[int(min(hits))]-ctx.s-ctx.s_offset-3.8))


STOP_CLEARANCE_M = .8

def distance_speed(history,ctx,distance,max_speed,fallback_rate=None):
    if distance is None:
        history.clear();return max_speed,None
    sample=(ctx.now,ctx.s,distance)
    if history:
        dt=ctx.now-history[-1][0]
        if dt<=0:return min(max_speed,math.sqrt(5.*max(0.,distance-STOP_CLEARANCE_M))),None
        apparent=(distance-history[-1][2]+ctx.s-history[-1][1])/dt
        # A surface jump / object replacement starts a new distance-rate window.
        if dt>.45 or not -2.<=apparent<=9.:history.clear()
    history.append(sample)
    history[:]=[r for r in history if ctx.now-r[0]<=.7]
    rate=None
    if len(history)>=3 and ctx.now-history[0][0]>=.3:
        rates=[(b[2]-a[2]+b[1]-a[1])/(b[0]-a[0]) for a,b in zip(history,history[1:])]
        rate=max(0.,float(np.median(rates)))
    # Before 0.3 s of distance history exists, the speed the ring tracker already measured for that body (if any)
    # stands in: taken as standing, it cut the command to ~8 km/h right after every launch behind a moving ring car
    # and braked before speeding up again (user 2026-09-29: the launch is still a little slow).
    if rate is None and fallback_rate is not None:rate=max(0.,fallback_rate)
    if distance<=STOP_CLEARANCE_M:return 0.,rate
    # Before a stable rate exists, a stationary surface defines stopping speed.
    # Once measured, regulate closing distance instead of object identity/speed.
    stopping=math.sqrt(2.*2.5*max(0.,distance-STOP_CLEARANCE_M))
    cap=stopping if rate is None else min(following_speed(rate,ctx.v,distance,getattr(ctx,'ring_tuck',0.)),rate+stopping)
    return min(max_speed,cap),rate
