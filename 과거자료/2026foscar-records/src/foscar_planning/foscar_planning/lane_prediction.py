"""Directed road-link candidates for shadow vehicle prediction.

Lane matching is a hypothesis, not a guarantee that a road user follows a lane.
Paths end at missing connectivity; they never extrapolate across a map gap.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import binary_fill_holes


@dataclass(frozen=True)
class LaneMatch:
    link_id: str
    s: float
    offset: float
    heading: float
    score: float


class LanePath:
    def __init__(self, points, offset, link_ids):
        self.points = np.asarray(points, dtype=float)
        self.s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.offset = offset
        self.link_ids = tuple(link_ids)

    def at(self, distance):
        if distance < 0 or distance > self.s[-1]+1e-6:
            return None
        i = min(len(self.s)-2, max(0, int(np.searchsorted(self.s, distance, side='right')-1)))
        delta = self.points[i+1]-self.points[i]
        heading = math.atan2(delta[1], delta[0])
        p = self.points[i] + delta * ((distance-self.s[i])/(self.s[i+1]-self.s[i]))
        return (float(p[0]-self.offset*math.sin(heading)),
                float(p[1]+self.offset*math.cos(heading)), heading)


class LaneMap:
    def __init__(self, links, seams_cache=None):
        self.links = {}
        self.outgoing = {}
        starts, vectors, lengths, owners, offsets = [], [], [], [], []
        widths = []
        rows = {}
        for row in links:
            points = np.asarray(row.get('points', []), dtype=float)
            if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 2:
                continue
            points = points[:, :2]
            if not np.isfinite(points).all():
                continue
            points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1)>1e-6]]
            if len(points)<2:
                continue
            key = row['idx']
            distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
            cumulative = np.r_[0., np.cumsum(distance)]
            self.links[key] = (points, cumulative, row['to_node_idx'])
            rows[key] = row
            self.outgoing.setdefault(row['from_node_idx'], []).append(key)
            starts.extend(points[:-1]); vectors.extend(np.diff(points, axis=0))
            lengths.extend(distance); owners.extend([key]*len(distance)); offsets.extend(cumulative[:-1])
            width = min(float(row.get('width_start') or 3.5), float(row.get('width_end') or 3.5))
            widths.extend([width/2]*len(distance))
        if not starts:
            raise ValueError('no valid directed road links')
        self.road_half_widths = np.asarray(widths)
        self.starts = np.asarray(starts)
        self.vectors = np.asarray(vectors)
        self.lengths = np.asarray(lengths)
        self.owners = owners
        self.offsets = np.asarray(offsets)
        self.tree = cKDTree(self.starts+self.vectors/2)
        self.max_half_segment = float(max(self.lengths))/2
        # (key, file): the seams saved for this same map are read back, else built and saved (user 2026-10-09: 1.2 s of
        # the start-up while the steering was already live)
        quads = None
        if seams_cache is not None:
            try:
                with np.load(seams_cache[1], allow_pickle=False) as f:
                    if str(f['key']) == seams_cache[0]:
                        quads = f['quads']
            except (OSError, KeyError, ValueError):
                pass
        if quads is None:
            self._build_internal_seams(rows)
            if seams_cache is not None and 'pytest' not in sys.modules:
                try:
                    np.savez(seams_cache[1], key=np.array(seams_cache[0]), quads=self.seam_quads)
                except OSError:
                    pass
        else:
            self._set_seams(quads)
        self.intersection_bounds = {
            key: (self.links[key][0].min(axis=0), self.links[key][0].max(axis=0))
            for key, row in rows.items()
            if str(row.get('link_type')) == '1' and row.get('related_signal')}
        self._intersection_patches = {}
        self._intersection_keys = tuple(self.intersection_bounds)
        self._intersection_bounds_array = np.asarray(list(self.intersection_bounds.values())).reshape(-1,2,2)

    def _build_internal_seams(self, rows):
        """Only explicit neighbours sharing a boundary enclose internal pavement."""
        quads, seen = [], set()
        for key, row in rows.items():
            for side, opposite in (('left', 'right'), ('right', 'left')):
                other = row.get(f'{side}_lane_change_dst_link_idx')
                if other not in rows or other == key:
                    continue
                pair = tuple(sorted((key, other)))
                marks = set(row.get(f'lane_mark_{side}') or [])
                if pair in seen or not marks.intersection(rows[other].get(f'lane_mark_{opposite}') or []):
                    continue
                seen.add(pair)
                a, b = self.links[key][0], self.links[other][0]
                bv = np.diff(b, axis=0)
                bu = bv/np.linalg.norm(bv, axis=1)[:, None]
                for start, end in zip(a[:-1], a[1:]):
                    av = end-start
                    au = av/np.linalg.norm(av)
                    same = bu @ au > 0.
                    axis = bu+au
                    axis /= np.maximum(np.linalg.norm(axis, axis=1)[:, None], 1e-12)
                    a0, a1 = axis @ start, axis @ end
                    b0 = np.sum(axis*b[:-1], axis=1)
                    b1 = np.sum(axis*b[1:], axis=1)
                    lo, hi = np.maximum(a0, b0), np.minimum(a1, b1)
                    valid = same & (hi-lo > 1e-6)
                    if not np.any(valid):
                        continue
                    # Clip both segments to their shared longitudinal interval;
                    # neither end cap may extend beyond the other lane's end.
                    axis_a = a1[valid]-a0[valid]
                    axis_b = b1[valid]-b0[valid]
                    alo = start+(lo[valid]-a0[valid])[:, None]/axis_a[:, None]*av
                    ahi = start+(hi[valid]-a0[valid])[:, None]/axis_a[:, None]*av
                    blo = b[:-1][valid]+(lo[valid]-b0[valid])[:, None]/axis_b[:, None]*bv[valid]
                    bhi = b[:-1][valid]+(hi[valid]-b0[valid])[:, None]/axis_b[:, None]*bv[valid]
                    quads.extend(np.stack((alo, ahi, bhi, blo), axis=1))
        self._set_seams(quads)

    def _set_seams(self, quads):
        self.seam_quads = np.asarray(quads, dtype=float).reshape(-1, 4, 2)
        self.seam_tree = None
        if len(self.seam_quads):
            centres = self.seam_quads.mean(axis=1)
            self.seam_tree = cKDTree(centres)
            self.seam_radius = float(np.max(np.linalg.norm(self.seam_quads-centres[:, None], axis=2)))
            self.seam_edges = np.roll(self.seam_quads, -1, axis=1)-self.seam_quads

    def _contains_internal_seam(self, points):
        inside = np.zeros(len(points), dtype=bool)
        if self.seam_tree is None or not len(points):
            return inside
        candidates = self.seam_tree.query_ball_point(points, self.seam_radius)
        counts = np.fromiter((len(ids) for ids in candidates), dtype=int, count=len(points))
        if not counts.sum():
            return inside
        point_ids = np.repeat(np.arange(len(points)), counts)
        quad_ids = np.concatenate(candidates).astype(int)
        delta = points[point_ids, None]-self.seam_quads[quad_ids]
        edges = self.seam_edges[quad_ids]
        cross = edges[:, :, 0]*delta[:, :, 1]-edges[:, :, 1]*delta[:, :, 0]
        contained = np.all(cross >= -1e-9, axis=1) | np.all(cross <= 1e-9, axis=1)
        inside[point_ids[contained]] = True
        return inside

    def contains_road(self, points, seam_tolerance=.15):
        """Mapped lane union, including opposite lanes; no centre-line penalty.

        A 0.15 m tolerance closes surveyed lane seams (vehicle sweep is
        inflated by 0.30 m). It is not permission to cross wider medians.
        Candidate segments are queried spatially, then exact segment distances
        checked. This is map geometry, not lidar free-space evidence.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not len(points):
            return np.zeros(0,dtype=bool)
        # The nearest segment is included in the existing 32-segment union query.
        # Accept its interior cheaply, and run the full query only for the remainder.
        _, nearest = self.tree.query(points,k=1)
        delta = points-self.starts[nearest]
        ratio = np.clip(np.sum(delta*self.vectors[nearest],axis=1)/self.lengths[nearest]**2,0.,1.)
        distance = np.linalg.norm(delta-ratio[:,None]*self.vectors[nearest],axis=1)
        inside = distance <= self.road_half_widths[nearest]+seam_tolerance
        pending = np.flatnonzero(~inside)
        if len(pending):
            _, ids = self.tree.query(points[pending],k=min(32,len(self.starts)))
            ids = np.asarray(ids).reshape(len(pending),-1)
            delta = points[pending,None,:]-self.starts[ids]
            ratio = np.clip(np.sum(delta*self.vectors[ids],axis=2)/self.lengths[ids]**2,0.,1.)
            distance = np.linalg.norm(delta-ratio[...,None]*self.vectors[ids],axis=2)
            inside[pending] = np.any(distance <= self.road_half_widths[ids]+seam_tolerance,axis=1)
        pending = np.flatnonzero(~inside)
        if len(pending):
            inside[pending] = self._contains_internal_seam(points[pending])
        return inside

    def contains_intersection_road(self, points):
        """Fill enclosed lane-corridor holes inside marked intersections only.

        Open exterior space stays outside. Ordinary lane/median queries continue
        to use contains_road; this supplement is for intersection avoidance.
        """
        points = np.asarray(points,dtype=float).reshape(-1,2)
        inside = self.contains_road(points)
        if inside.all() or not self.intersection_bounds:
            return inside
        padding = 2.*float(self.road_half_widths.max())
        lo,hi = points.min(axis=0)-padding,points.max(axis=0)+padding
        bounds = self._intersection_bounds_array
        matches = np.all(bounds[:,1]>=lo,axis=1) & np.all(bounds[:,0]<=hi,axis=1)
        keys = tuple(self._intersection_keys[i] for i in np.flatnonzero(matches))
        if not keys:
            return inside
        if keys not in self._intersection_patches:
            resolution = .25
            origin = np.floor((np.min([self.intersection_bounds[k][0] for k in keys],axis=0)-padding)/resolution)*resolution
            end = np.max([self.intersection_bounds[k][1] for k in keys],axis=0)+padding
            xs,ys = np.arange(origin[0],end[0]+resolution,resolution),np.arange(origin[1],end[1]+resolution,resolution)
            xx,yy = np.meshgrid(xs,ys)
            road = self.contains_road(np.c_[xx.ravel(),yy.ravel()]).reshape(xx.shape)
            filled = binary_fill_holes(road)
            # Require enclosed pavement at every corner and an internal-hole corner.
            holes = filled & ~road
            cells = (filled[:-1,:-1]&filled[1:,:-1]&filled[:-1,1:]&filled[1:,1:]
                     &(holes[:-1,:-1]|holes[1:,:-1]|holes[:-1,1:]|holes[1:,1:]))
            self._intersection_patches[keys] = (origin,resolution,cells)
        origin,resolution,cells = self._intersection_patches[keys]
        pending = np.flatnonzero(~inside)
        ij = np.floor((points[pending]-origin)/resolution).astype(int)
        valid = (ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<cells.shape[1])&(ij[:,1]<cells.shape[0])
        inside[pending[valid]] = cells[ij[valid,1],ij[valid,0]]
        return inside

    @classmethod
    def from_file(cls, filename):
        path = Path(filename).expanduser().resolve()
        text = path.read_text()
        return cls(json.loads(text), (hashlib.sha1(text.encode()).hexdigest(), str(path.with_suffix('.seams.npz'))))

    def match(self, x, y, heading, max_distance=3.):
        matches = self.matches(x,y,heading,max_distance)
        return matches[0] if matches else None

    def matches(self, x, y, heading, max_distance=3., score_slack=1.5):
        if not all(math.isfinite(v) for v in (x,y,heading)):
            return []
        ids = self.tree.query_ball_point([x,y], max_distance+self.max_half_segment)
        if not ids:
            return []
        ids = np.asarray(ids)
        delta = np.array([x,y])-self.starts[ids]
        ratio = np.clip(np.sum(delta*self.vectors[ids],axis=1)/self.lengths[ids]**2, 0., 1.)
        residual = delta-ratio[:,None]*self.vectors[ids]
        distance = np.linalg.norm(residual,axis=1)
        unit = self.vectors[ids]/self.lengths[ids,None]
        alignment = unit @ np.array([math.cos(heading),math.sin(heading)])
        valid = (distance <= max_distance) & (alignment >= math.cos(math.pi/3))
        if not np.any(valid):
            return []
        score = np.where(valid,distance+3*(1-alignment),np.inf)
        candidates = []
        seen = set()
        for j in np.argsort(score):
            if not np.isfinite(score[j]) or score[j] > np.min(score)+score_slack:
                break
            i = ids[j]
            if self.owners[i] in seen:
                continue
            seen.add(self.owners[i])
            candidates.append(LaneMatch(self.owners[i],float(self.offsets[i]+ratio[j]*self.lengths[i]),
                              float(unit[j,0]*residual[j,1]-unit[j,1]*residual[j,0]),
                              math.atan2(unit[j,1],unit[j,0]),float(score[j])))
        return candidates

    def paths(self, match, distance, max_paths=16):
        """Enumerate connected alternatives in deterministic order, including cycles.

        A candidate ending before the requested distance signals incomplete map
        coverage. Callers must check coverage, including candidate-limit cases.
        """
        points, cumulative, end = self.links[match.link_id]
        i = min(len(points)-2, int(np.searchsorted(cumulative,match.s,side='right')-1))
        ratio = (match.s-cumulative[i])/(cumulative[i+1]-cumulative[i])
        start = points[i]+ratio*(points[i+1]-points[i])
        tail = points[i+1:]
        if np.linalg.norm(tail[0]-start)<1e-6:
            tail = tail[1:]
        initial = np.vstack([start,tail])
        stack = [(initial,[match.link_id],end,float(cumulative[-1]-match.s))]
        results = []
        while stack:
            pts, keys, end, length = stack.pop(0)
            successors = self.outgoing.get(end,[])
            # Limit expansion, not just returned paths; truncation remains visible
            # as an incomplete path instead of silently dropping a branch.
            if length >= distance or not successors or len(keys)>=32 or len(stack)+len(results)+len(successors)>max_paths:
                if len(pts)>1:
                    results.append(LanePath(pts,match.offset,keys))
                continue
            for key in sorted(successors):
                extra, arc, next_end = self.links[key]
                if np.linalg.norm(pts[-1]-extra[0])>.5:
                    if len(pts)>1:
                        results.append(LanePath(pts,match.offset,keys))
                    continue
                stack.append((np.vstack([pts,extra[1:]]),keys+[key],next_end,length+float(arc[-1])))
        return results


class LaneMotion:
    """Causal speed smoothing and candidate ranking; no future samples used."""
    def __init__(self, lanes, horizon=9.):
        from collections import deque
        self.lanes = lanes
        self.horizon = horizon
        self.samples = deque()
        self.stamp = -math.inf
        self.speed = 0.
        self.paths = []
        self.matches = []

    def update(self, event, heading, turn_rate):
        if event.stamp <= self.stamp:
            return
        if not all(math.isfinite(v) for v in (event.stamp,event.x,event.y,event.speed,heading,turn_rate)):
            return
        self.stamp = event.stamp
        self.samples.append((event.stamp,event.speed))
        while self.samples and event.stamp-self.samples[0][0]>1.:
            self.samples.popleft()
        self.paths = []
        self.matches = []
        if event.speed < .5:
            self.samples.clear()
            self.speed = 0.
            return
        self.speed = float(np.median([v for _,v in self.samples]))
        self.matches = self.lanes.matches(event.x,event.y,heading)
        self.paths = [p for match in self.matches
                      for p in self.lanes.paths(match,max(event.speed,self.speed)*self.horizon)]
        def rank(path):
            score = 0.
            for dt in (1.,2.,3.):
                pose = path.at(self.speed*dt)
                if pose is None:
                    score += 10.
                    continue
                diff = pose[2]-heading-turn_rate*dt
                score += math.atan2(math.sin(diff),math.cos(diff))**2
            return score
        self.paths.sort(key=rank)

    def poses(self, dt):
        return [pose for path in self.paths if (pose := path.at(self.speed*dt)) is not None]

    def complete(self, dt):
        return bool(self.paths) and all(p.at(self.speed*dt) is not None for p in self.paths)
