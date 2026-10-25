"""Time-coherent occupancy constraints for local obstacle avoidance."""
import math
from dataclasses import replace
import numpy as np

from .avoid_planner import polygon_box
from .bev_occupancy import BevOccupancy, FREE, OCCUPIED, UNKNOWN
from .bev_motion import OccupancyMotion
from .highway_box_tracks import EdgeTracks
from .frenet import to_frenet
from .mapped_avoidance import road_contains


class BevAvoidance:
    def __init__(self, resolution, sight):
        self.field = BevOccupancy(resolution, sight)
        self.motion = OccupancyMotion()
        self.boxes = []
        self.last_reason = ''
        self.diagnostic = {}
        self.remembered = {}

    def update(self, ctx):
        if ctx.occupancy_stamp <= self.field.stamp:
            return
        hulls = ctx.occupancy_hulls
        if hulls is None or ctx.route is None:
            return
        # A person is no box to steer around (user 2026-10-10): we keep our line, YIELD stops us when the person
        # comes onto our road and lets us go once they are off it. Steering around a person at the roadside, and
        # re-planning after every step of a crossing one, made the slalom swing and stop.
        people = {key for key,kind in getattr(ctx,'occupancy_kinds',{}).items() if kind == 'PEDESTRIAN'}
        if people:
            hulls = {key:points for key,points in hulls.items() if key not in people}
            for key in people:
                self.remembered.pop(key,None)
        for proof in ctx.occupancy_cleared:
            old = self.remembered.get(proof.get('id'))
            if old is not None and proof.get('last_seen') == old[0]:
                del self.remembered[proof['id']]
        for key,points in hulls.items():
            self.remembered[key] = (ctx.occupancy_stamp,points,getattr(ctx,'occupancy_heights',{}).get(key))
        self.remembered = {key:row for key,row in self.remembered.items()
                           if np.linalg.norm(np.asarray(row[1]).mean(axis=0)-np.asarray(ctx.occupancy_sensor))
                           <= self.field.sight_m}
        self.field.update(ctx.occupancy_stamp, hulls, ctx.occupancy_sensor)
        self.motion.update(ctx.occupancy_stamp, self.motion_hulls(), ctx.occupancy_sensor)
        coasting = {('coast',i):r.at(ctx.occupancy_stamp) for i,r in enumerate(self.motion.regions) if r.coasted>0.}
        missing = {key:row[1] for key,row in self.remembered.items() if key not in hulls}
        if coasting or missing:
            combined = dict(hulls); combined.update(coasting); combined.update(missing)
            self.field = BevOccupancy(self.field.resolution,self.field.sight_m)
            self.field.update(ctx.occupancy_stamp,combined,ctx.occupancy_sensor)
        self.boxes = []
        for indices in self.field.region_cells.values():
            # Offset proposals and the swept-body check must cover the same cells.
            # Keep each region separate; this hull only seeds candidate offsets.
            points = self.field._polygon(self.field.cell_polygons(indices).reshape(-1,2))
            # Off the pavement entirely: no avoidance path can reach it, so it is no box to plan around --
            # whatever its distance from the global path (user 2026-10-09: a structure beside the tunnel
            # portal, +6.5 m off the road, blocked the slalom). Partly on the pavement it stays.
            if ctx.road_map is not None and not np.asarray(road_contains(ctx, points)).any():
                continue
            sd = [to_frenet(ctx.route, x, y, ctx.s+ctx.s_offset) for x,y in points]
            box = polygon_box([(s-ctx.s_offset,d) for s,d in sd])
            if box.s_max >= ctx.s-self.field.sight_m and box.s_min <= ctx.s+self.field.sight_m:
                self.boxes.append(box)

    def measured_motion(self, key, ctx):
        if not key or key[0] != 'track':
            return None
        polygon = self.field.hulls.get(key[1])
        if polygon is None:
            return None
        center = np.asarray(polygon).mean(axis=0)
        h = ctx.route.heading_at(ctx.s+ctx.s_offset)
        normal = np.array([-math.sin(h),math.cos(h)])
        for region in self.motion.regions:
            if not region.ready or region.velocity is None or region.sigma is None:
                continue
            p = np.asarray(region.observed_polygon or region.polygon)
            edges = np.roll(p,-1,axis=0)-p
            delta = center-p
            cross = edges[:,0]*delta[:,1]-edges[:,1]*delta[:,0]
            if not (np.all(cross>=-1e-9) or np.all(cross<=1e-9)):
                continue
            lateral = float(np.dot(region.velocity,normal))
            sigma = float(np.linalg.norm(np.asarray(region.sigma)*normal))
            if abs(lateral) > EdgeTracks.GATE_SIGMA*sigma:
                return 'crossing'
            if np.all(np.abs(region.velocity) <= EdgeTracks.GATE_SIGMA*np.asarray(region.sigma)):
                return 'stationary'
            return 'moving'
        return None

    def clearance_queries(self, now):
        rows = []
        for key,(stamp,points,height) in self.remembered.items():
            if stamp >= self.field.stamp:
                continue
            p = np.asarray(points);lo,hi = p.min(axis=0),p.max(axis=0)
            center = (lo+hi)/2.
            rows.append(dict(id=key,last_seen=stamp,x=float(center[0]),y=float(center[1]),
                             yaw=0.,length=float(hi[0]-lo[0]),width=float(hi[1]-lo[1]),vx=0.,vy=0.,
                             **({} if height is None else dict(height=float(height)))))
        return dict(stamp=now,frame='map',objects=rows)

    def motion_hulls(self):
        # Adjacent noisy cell runs share a motion estimate, not their collision occupancy.
        keys = list(self.field.region_cells)
        parents = list(range(len(keys)))
        def find(i):
            while parents[i]!=i:
                parents[i]=parents[parents[i]];i=parents[i]
            return i
        owners = {}
        reach = 1+math.ceil(EdgeTracks.POS_SIGMA/self.field.resolution)
        for i,key in enumerate(keys):
            for x,y in self.field.region_cells[key]:
                for dx in range(-reach,reach+1):
                    for dy in range(-reach,reach+1):
                        other = owners.get((x+dx,y+dy))
                        if other is not None:
                            parents[find(i)] = find(other)
                owners[(x,y)] = i
        groups = {}
        for i,key in enumerate(keys):
            groups.setdefault(find(i),[]).extend(self.field.hulls[key])
        return {str(i):polygon_box(points).vertices for i,points in groups.items()}

    @staticmethod
    def bodies(ctx, s, d, slope, front, rear, half):
        """Body corners (n, 4, 2) at stations s; to_xy / heading_at of the route, all stations at once."""
        route = ctx.route
        local = np.asarray(s,dtype=float)+ctx.s_offset
        clipped = np.clip(local,0.,route.s[-1])
        x = np.interp(clipped,route.s,route.points[:,0])
        y = np.interp(clipped,route.s,route.points[:,1])
        idx = np.clip(np.searchsorted(route.s,local,side='right')-1,0,len(route.delta)-1)
        heading = np.arctan2(route.delta[idx,1],route.delta[idx,0])
        x,y = x-np.sin(heading)*d,y+np.cos(heading)*d
        yaw = heading+np.arctan(slope)
        co,si = np.cos(yaw)[:,None],np.sin(yaw)[:,None]
        a = np.array([-rear,front,front,-rear]);w = np.array([-half,-half,half,half])
        return np.stack((x[:,None]+a*co-w*si,y[:,None]+a*si+w*co),axis=2)

    def segment_clear(self, ctx, d0, d1, start, length, s_a, s_b, front, rear, half):
        ss = np.unique(np.r_[np.arange(s_a,s_b,self.field.resolution),s_b])
        u = np.clip((ss-start)/max(length,1e-6),0.,1.)
        d = d0+(d1-d0)*u**3*(10-15*u+6*u*u)
        slope = (d1-d0)*30*u*u*(1-u)**2/max(length,1e-6)
        local = ss+ctx.s_offset
        x = np.interp(local,ctx.route.s,ctx.route.points[:,0])
        y = np.interp(local,ctx.route.s,ctx.route.points[:,1])
        idx = np.clip(np.searchsorted(ctx.route.s,local,side='right')-1,0,len(ctx.route.delta)-1)
        heading = np.arctan2(ctx.route.delta[idx,1],ctx.route.delta[idx,0])
        positions = np.c_[x-np.sin(heading)*d,y+np.cos(heading)*d]
        return not self.field.hits_bodies(positions,heading+np.arctan(slope),front,rear,half)

    def guard(self, ctx, target, avoid):
        self.last_reason = 'clear'
        self.diagnostic = dict(reason='clear', scan_age=max(0.,ctx.now-self.field.stamp),
                               cap=target.v_max, resolution=self.field.resolution)
        if ctx.now-self.field.stamp > avoid.tunnel_unseen_scans*avoid.scan_period_s:
            self.last_reason = 'stale scan'
            self.diagnostic.update(reason=self.last_reason,cap=0.)
            return replace(target,v_max=0.)
        # A stopped target already holds its lateral plan. No additional geometry is published.
        if target.v_max == 0.:
            self.last_reason = 'planner hold'
            self.diagnostic.update(reason=self.last_reason)
            return target
        chain = tuple(avoid.slalom_moves(avoid.slalom,ctx.s)) if avoid.slalom is not None else target.chain
        origin = avoid.slalom_origin if chain else target.lat_offset
        stop_distance = max(ctx.v,0.)**2/(2.*avoid.slalom_decel_mps2)
        horizon = min(self.field.sight_m, max(avoid.front_m, stop_distance+avoid.entry_preview_m))
        # Every station at once (one cell query for the union of the bodies' cells); walked in order below as
        # before. Station by station this was a third of behavior_node's time and overran the 20 Hz tick (2026-10-10).
        stations = np.unique(np.r_[np.arange(ctx.s,ctx.s+horizon,self.field.resolution),ctx.s+horizon])
        d = np.full(len(stations),float(origin));slope = np.zeros(len(stations));previous = origin
        for offset,start,length in chain:
            on = stations>=start
            u = np.clip((stations[on]-start)/max(length,1e-6),0.,1.)
            d[on] = previous+(offset-previous)*u**3*(10-15*u+6*u*u)
            slope[on] = (offset-previous)*30*u*u*(1-u)**2/max(length,1e-6)
            previous = offset
        bodies = self.bodies(ctx,stations,d,slope,avoid.front_m,avoid.rear_overhang_m,avoid.path_planner(ctx).half)
        owner,all_cells = self.field.intersected_cells_many(bodies)
        unique,inverse = np.unique(all_cells,axis=0,return_inverse=True)
        all_states = self.field.query((unique+.5)*self.field.resolution)[inverse.reshape(-1)]
        occupied_at = np.bincount(owner[all_states==OCCUPIED],minlength=len(stations))>0
        unknown_at = np.bincount(owner[all_states==UNKNOWN],minlength=len(stations))>0
        for i,s in enumerate(stations):
            body = [tuple(map(float,corner)) for corner in bodies[i]]
            reason = 'occupied' if occupied_at[i] else 'unobserved' if unknown_at[i] else ''
            distance=max(0.,s-ctx.s)
            # Check a reachable arrival interval within the stopping-time horizon; newly
            # observed scans re-evaluate it before motion beyond that horizon is committed.
            acceleration=avoid.return_plan_accel_mps2
            arrival=2.*distance/max(1e-6,max(ctx.v,0.)+math.sqrt(max(ctx.v,0.)**2+2.*acceleration*distance))
            prediction_time = max(ctx.v,0.)/avoid.slalom_decel_mps2 + avoid.tunnel_unseen_scans*avoid.scan_period_s
            if not reason and 0. < arrival <= prediction_time:
                for region in self.motion.regions:
                    # The uninitialized filter prior is not observed motion.
                    # Its current occupied cells are already checked above.
                    if not region.ready:
                        continue
                    # A velocity compatible with zero is not evidence of motion.
                    # Its measured occupancy remains in the current-cell check.
                    if region.velocity is not None and region.sigma is not None and all(
                            abs(v) <= EdgeTracks.GATE_SIGMA*sd
                            for v,sd in zip(region.velocity,region.sigma)):
                        continue
                    speed = min(max(ctx.v,0.),target.v_max)
                    latest = prediction_time if speed <= 0. else min(prediction_time,distance/speed)
                    times = (ctx.now+arrival,ctx.now+max(arrival,latest))
                    bounds = [region.displacement_range(t) for t in times]
                    limits = tuple((min(b[axis][0] for b in bounds),max(b[axis][1] for b in bounds))
                                   for axis in (0,1))
                    points=[(x+dx,y+dy) for x,y in region.polygon
                            for dx in limits[0] for dy in limits[1]]
                    # Skip far predictions by bounds before rasterizing.
                    p=np.asarray(points);b=np.asarray(body)
                    if np.any(p.max(axis=0)<b.min(axis=0)) or np.any(p.min(axis=0)>b.max(axis=0)):
                        continue
                    p = self.field._polygon(points)
                    edges = np.r_[np.roll(p,-1,axis=0)-p, np.roll(b,-1,axis=0)-b]
                    axes = np.c_[-edges[:,1],edges[:,0]]
                    pp,bp = p@axes.T,b@axes.T
                    if not np.any((pp.max(axis=0)<bp.min(axis=0)) | (bp.max(axis=0)<pp.min(axis=0))):
                        reason='predicted occupancy';break
            if reason:
                self.last_reason=reason
                free=max(0.,distance-self.field.resolution)
                stop=ctx.s+free+avoid.front_axle_m
                if math.isfinite(target.stop_s):stop=min(stop,target.stop_s)
                cap=math.sqrt(2.*avoid.slalom_decel_mps2*free)
                self.diagnostic.update(reason=reason,cap=min(target.v_max,cap),
                                       distance=float(distance),body=tuple(body))
                if reason=='predicted occupancy':
                    self.diagnostic.update(region=region.polygon,prediction=tuple(map(tuple,p)),
                                           velocity=region.velocity,sigma=region.sigma,
                                           displacement=limits)
                else:
                    mine = owner==i
                    selected = all_cells[mine][all_states[mine]==(OCCUPIED if reason=='occupied' else UNKNOWN)]
                    self.diagnostic['cells'] = tuple(map(tuple,(selected+.5)*self.field.resolution))
                return replace(target,stop_s=stop,v_max=min(target.v_max,cap))
        self.last_reason='clear'
        return target
