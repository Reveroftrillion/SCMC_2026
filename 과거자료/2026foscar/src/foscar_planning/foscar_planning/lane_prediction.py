"""Directed road-link candidates for shadow vehicle prediction.

Lane matching is a hypothesis, not a guarantee that a road user follows a lane.
Paths end at missing connectivity; they never extrapolate across a map gap.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


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
    def __init__(self, links):
        self.links = {}
        self.outgoing = {}
        starts, vectors, lengths, owners, offsets = [], [], [], [], []
        widths = []
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

    def contains_road(self, points, seam_tolerance=.15):
        """Mapped lane union, including opposite lanes; no centre-line penalty.

        A 0.15 m tolerance closes surveyed lane seams (vehicle sweep is
        inflated by 0.30 m). It is not permission to cross wider medians.
        Candidate segments are queried spatially, then exact segment distances
        checked. This is map geometry, not lidar free-space evidence.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        _, ids = self.tree.query(points, k=min(32, len(self.starts)))
        ids = np.asarray(ids).reshape(len(points), -1)
        delta = points[:, None, :]-self.starts[ids]
        ratio = np.clip(np.sum(delta*self.vectors[ids], axis=2)/self.lengths[ids]**2, 0., 1.)
        distance = np.linalg.norm(delta-ratio[..., None]*self.vectors[ids], axis=2)
        return np.any(distance <= self.road_half_widths[ids]+seam_tolerance, axis=1)

    @classmethod
    def from_file(cls, filename):
        return cls(json.loads(Path(filename).expanduser().read_text()))

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
