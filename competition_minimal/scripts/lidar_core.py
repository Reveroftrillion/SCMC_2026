"""Small, ROS-independent LiDAR safety and trajectory helpers.

Coordinates in object dictionaries are path-relative metres (s, d), with
bounding half extents hs, hd. Speeds are metres/second.
"""
import math

import numpy as np
from scipy.spatial import cKDTree


def quintic(u):
    u = max(0.0, min(1.0, float(u)))
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def profile(s, start, target, shift, hold_until, return_length):
    """Path offset at s, including smooth transition back to the global path."""
    if s < start:
        return 0.0
    if s < start + shift:
        return target * quintic((s - start) / shift)
    if s < hold_until:
        return target
    return target * (1.0 - quintic((s - hold_until) / return_length))


def swept_half_width(s, plan, half_width=1.196):
    """Conservative lateral extent including yaw during a quintic shift."""
    start, target, shift, hold, ret = plan
    if shift <= 0 or ret <= 0:
        return float('inf')
    if start <= s < start+shift:
        u = (s-start)/shift
        slope = target*30*u*u*(1-u)*(1-u)/shift
    elif hold <= s < hold+ret:
        u = (s-hold)/ret
        slope = -target*30*u*u*(1-u)*(1-u)/ret
    else:
        slope = 0.0
    angle = math.atan(abs(slope))
    # The pose-to-front footprint used by collision checking is 3.5 m.
    return half_width*math.cos(angle) + 3.5*math.sin(angle)


def trajectory_fits_road(plan, road_half_width, half_width=1.196):
    if not math.isfinite(road_half_width) or road_half_width <= 0:
        return False
    start, _, _, hold, ret = plan
    return all(abs(profile(s, *plan)) + swept_half_width(s, plan, half_width) <= road_half_width
               for s in np.arange(start, hold+ret+0.25, 0.25))


def project(path, cum, points):
    """Project world points on piecewise linear path, returning s and signed d."""
    a, v = path[:-1, :2], path[1:, :2] - path[:-1, :2]
    vv = np.einsum('ij,ij->i', v, v)
    good = vv > 1e-12
    vv = np.where(good, vv, 1.0)
    result = []
    for p in np.asarray(points):
        w = p[:2] - a
        t = np.clip(np.einsum('ij,ij->i', w, v) / vv, 0.0, 1.0)
        delta = w - t[:, None] * v
        dsq = np.einsum('ij,ij->i', delta, delta)
        dsq[~good] = float('inf')
        i = int(np.argmin(dsq))
        length = math.sqrt(vv[i])
        cross = (v[i, 0] * delta[i, 1] - v[i, 1] * delta[i, 0]) / length
        result.append((float(cum[i] + t[i] * length), float(cross)))
    return result


def ground_and_clusters(raw, sensor_xyz=(2.0, 0.0, 2.11), eps=0.5, min_samples=5, with_ground=False):
    """Fit local ground from low samples per 2m cell, cluster nonground.

    Returns None if insufficient ground support; an empty obstacle list is not
    accepted as a healthy observation unless ground itself was observed.
    """
    p = np.asarray(raw, dtype=float)
    if p.ndim != 2 or p.shape[1] < 3:
        return None
    p = p[:, :3]
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(p[:, 0] > -25.0) & (p[:, 0] < 40.0) & (np.abs(p[:, 1]) < 20.0) & (p[:, 2] > -4.0) & (p[:, 2] < 3.0)]
    if len(p) < 50:
        return None
    # Ground is expected below the sensor.  Cell minima make the fit robust to
    # pedestrians and vehicles; a hard slope limit rejects walls.
    candidates = p[(p[:, 2] < -0.5) & (p[:, 2] > -3.8)]
    if len(candidates) < 30:
        return None
    cells = {}
    for x, y, z in candidates:
        key = (int(math.floor(x / 2)), int(math.floor(y / 2)))
        if key not in cells or z < cells[key][2]:
            cells[key] = (x, y, z)
    low = np.asarray(list(cells.values()))
    if len(low) < 12 or np.ptp(low[:, 0]) < 5 or np.ptp(low[:, 1]) < 3:
        return None
    # Iterative robust fit to avoid low curb returns corrupting the plane.
    keep = np.ones(len(low), dtype=bool)
    coef = None
    for _ in range(3):
        if keep.sum() < 10:
            return None
        coef = np.linalg.lstsq(np.column_stack((low[keep, 0], low[keep, 1], np.ones(keep.sum()))), low[keep, 2], rcond=None)[0]
        residual = low[:, 2] - (low[:, 0] * coef[0] + low[:, 1] * coef[1] + coef[2])
        keep = np.abs(residual) < 0.25
    if abs(coef[0]) > 0.2 or abs(coef[1]) > 0.2:
        return None
    height = p[:, 2] - (p[:, 0] * coef[0] + p[:, 1] * coef[1] + coef[2])
    if np.count_nonzero(np.abs(height) < 0.15) < 25:
        return None
    ground = p[np.abs(height) < 0.15, :2].copy()
    ground += np.asarray(sensor_xyz[:2])
    obj = p[(height > 0.15) & (height < 3.2)].copy()
    obj[:, 0] += sensor_xyz[0]
    obj[:, 1] += sensor_xyz[1]
    # Voxel reduction caps runtime and avoids treating an isolated return as
    # free space: residual unclustered voxels remain small obstacle boxes.
    if len(obj) == 0:
        return ([], ground) if with_ground else []
    vox = np.floor(obj / 0.15).astype(np.int32)
    _, indices = np.unique(vox, axis=0, return_index=True)
    obj = obj[np.sort(indices)]
    if len(obj) > 30000:
        return None
    tree = cKDTree(obj)
    neighbors = tree.query_ball_point(obj, eps)
    labels = np.full(len(obj), -1, dtype=np.int32)
    visited = np.zeros(len(obj), dtype=bool)
    cluster_id = 0
    for i in range(len(obj)):
        if visited[i]:
            continue
        visited[i] = True
        if len(neighbors[i]) < min_samples:
            continue
        labels[i] = cluster_id
        queue = list(neighbors[i])
        queued = np.zeros(len(obj), dtype=bool)
        queued[queue] = True
        j = 0
        while j < len(queue):
            n = queue[j]
            j += 1
            if not visited[n]:
                visited[n] = True
                if len(neighbors[n]) >= min_samples:
                    for neighbor in neighbors[n]:
                        if not queued[neighbor]:
                            queued[neighbor] = True
                            queue.append(neighbor)
            if labels[n] == -1:
                labels[n] = cluster_id
        cluster_id += 1
    result = []
    for label in range(cluster_id):
        q = obj[labels == label]
        lo, hi = q.min(axis=0), q.max(axis=0)
        result.append((lo, hi, len(q)))
    # Small sparse returns near the driving corridor are still safety hazards.
    noise = obj[labels < 0]
    for q in noise[(noise[:, 0] > -20) & (noise[:, 0] < 30) & (np.abs(noise[:, 1]) < 6)]:
        result.append((q - 0.15, q + 0.15, 1))
    return (result, ground) if with_ground else result


def trajectory_observed(ground, path, cum, pose, progress, plan, rear_check=0.0,
                        half_width=1.35, spacing=2.0, tolerance=1.35):
    """Require ground returns under a sampled swept path and rear target lane.

    Coordinates in ground are in base_link. This is a conservative visibility
    check, not a road-boundary detector. Occupied/occluded cells do not count as
    observed free space, even when no object cluster was produced.
    """
    ground = np.asarray(ground, dtype=float)
    path = np.asarray(path, dtype=float)
    if (plan is None or len(plan) != 5 or ground.ndim != 2 or ground.shape[1] != 2 or len(ground) < 20 or
            not np.isfinite(ground).all() or not np.isfinite(pose).all() or
            len(path) != len(cum) or cum[-1] <= 0 or spacing <= 0):
        return False
    start, target, shift, hold, ret = plan
    end = hold + ret
    if not all(math.isfinite(v) for v in (progress, start, target, shift, hold, ret, rear_check)):
        return False
    # Sample the future swept footprint using path normals, including overhang.
    lo = max(0.0, progress - 1.8)
    hi = min(float(cum[-1]), end + 3.5)
    if hi <= lo or hi - lo > 60:
        return False
    samples = []
    for s in np.arange(lo, hi + spacing / 2, spacing):
        idx = min(len(path)-2, max(0, int(np.searchsorted(cum, s, side='right') - 1)))
        seg = path[idx+1, :2] - path[idx, :2]
        length = float(np.linalg.norm(seg))
        if length < 1e-6:
            return False
        t = min(1.0, max(0.0, (s-cum[idx])/length))
        center = path[idx, :2] + t*seg
        normal = np.array([-seg[1], seg[0]]) / length
        offset = profile(s, *plan)
        width = swept_half_width(s, plan, half_width)
        for d in (offset-width, offset, offset+width):
            samples.append(center + normal*d)
    if rear_check > 0:
        # Observe the neighbouring lane behind the ego before moving into it.
        for s in np.arange(max(0.0, progress-rear_check), progress, spacing):
            idx = min(len(path)-2, max(0, int(np.searchsorted(cum, s, side='right')-1)))
            seg = path[idx+1, :2]-path[idx, :2]
            length = float(np.linalg.norm(seg))
            if length < 1e-6:
                return False
            center = path[idx, :2] + min(1., max(0., (s-cum[idx])/length))*seg
            normal = np.array([-seg[1], seg[0]])/length
            for d in (target-half_width, target, target+half_width):
                samples.append(center+normal*d)
    xy = np.asarray(samples) - np.asarray(pose[:2])
    c, sn = math.cos(pose[2]), math.sin(pose[2])
    body = np.column_stack((c*xy[:, 0]+sn*xy[:, 1], -sn*xy[:, 0]+c*xy[:, 1]))
    return bool(len(body) and np.all(cKDTree(ground).query(body, k=1)[0] <= tolerance))


def choose_highway_change(objects, progress, return_by, side_offset=3.2,
                          shift=12.0, return_length=12.0, road_half_width=0.0):
    """Choose a short pass that completes before CP12, or refuse to change."""
    if (not all(math.isfinite(v) for v in (progress, return_by, side_offset, shift,
                                            return_length, road_half_width)) or
            shift <= 0 or return_length <= 0 or road_half_width < abs(side_offset)+1.35):
        return None
    blocking = [o for o in objects if progress < o['s'] < progress+35 and
                abs(o['d']) <= 1.35+o['hd']]
    if not blocking:
        return None
    hold = max(progress+shift, max(o['s']+o['hs'] for o in blocking)+5.0)
    if hold+return_length > return_by:
        return None
    for target in (side_offset, -side_offset):
        # A fast vehicle approaching from behind can reach the target lane
        # during the transition. Lack of rear visibility is checked separately.
        if any(progress-20 < o['s'] < hold+return_length+5 and
               abs(o['d']-target) <= 1.35+o['hd'] for o in objects):
            continue
        plan = (progress, target, shift, hold, return_length)
        if (trajectory_fits_road(plan, road_half_width, half_width=1.35) and
                candidate_clear(objects, *plan, half_width=1.35, front=3.5, rear=1.8)):
            return plan
    return None


def candidate_clear(objects, start_s, target, shift, hold_until, return_length,
                    half_width=1.196, front=3.5, rear=1.8, from_s=None):
    """Check the entire candidate trajectory, including both quintic ramps."""
    if (not all(math.isfinite(v) for v in (start_s, target, shift, hold_until,
                                           return_length, half_width, front, rear)) or
            shift <= 0 or return_length <= 0 or hold_until < start_s + shift):
        return False
    end = hold_until + return_length
    begin = max(start_s, from_s) if from_s is not None else start_s
    if not math.isfinite(begin):
        return False
    for o in objects:
        if (not all(math.isfinite(o[k]) for k in ('s', 'd', 'hs', 'hd')) or
                o['hs'] < 0 or o['hd'] < 0):
            return False
        left = max(begin, o['s'] - o['hs'] - front)
        right = min(end, o['s'] + o['hs'] + rear)
        if left > right:
            continue
        for x in np.linspace(left, right, max(2, int((right - left) / 0.25) + 1)):
            plan = (start_s, target, shift, hold_until, return_length)
            if abs(profile(x, *plan) - o['d']) <= swept_half_width(x, plan, half_width) + o['hd']:
                return False
    return True


def choose_avoidance(objects, progress, side_offset=3.2, shift=10.0, return_length=12.0,
                     road_half_width=None, side_offsets=None):
    """Choose a collision-free left/right trajectory, or return None."""
    # The side corridor must be measured, not inferred from missing returns.
    if (road_half_width is not None and
            (not math.isfinite(road_half_width) or road_half_width < abs(side_offset) + 1.196)):
        return None
    blocking = [o for o in objects if progress < o['s'] < progress + 35 and abs(o['d']) <= 1.196 + o['hd']]
    if not blocking:
        return None
    last = max(o['s'] + o['hs'] for o in blocking)
    hold = last + 5.0
    for target in (side_offsets if side_offsets is not None else (side_offset, -side_offset)):
        if road_half_width is not None and abs(target) + 1.196 > road_half_width:
            continue
        plan = (progress, target, shift, hold, return_length)
        if road_half_width is not None and not trajectory_fits_road(plan, road_half_width):
            continue
        if candidate_clear(objects, *plan):
            return plan
    return None


def follow_speed(distance, lead_speed, minimum_gap=8.0, time_gap=2.0):
    """Conservative longitudinal speed ceiling; distance is bumper clearance."""
    if not all(math.isfinite(v) for v in (distance, lead_speed, minimum_gap, time_gap)) or time_gap <= 0:
        return 0.0
    if distance <= minimum_gap:
        return 0.0
    return max(0.0, max(0.0, lead_speed) + (distance - minimum_gap) / time_gap)


def retain_occupancy(history, boxes, now, pose, memory_seconds=1.0):
    """Retain short-lived missing objects as expanding conservative occupancy.

    Returned history holds original observations, so uncertainty does not grow
    cumulatively when the same observation is propagated over several frames.
    """
    ids = {b['id'] for b in boxes}
    x, y, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    ghosts, retained = [], []
    for old in history:
        age = now - old['time']
        if old['id'] in ids or not 0 <= age <= memory_seconds:
            continue
        retained.append(old)
        ghost = dict(old)
        dx, dy = ghost['wx'] - x, ghost['wy'] - y
        ghost['bx'], ghost['by'] = c*dx + sn*dy, -sn*dx + c*dy
        expansion = min(2.0, age * math.hypot(ghost['vx'], ghost['vy']))
        ghost['hs'] += expansion
        ghost['hd'] += expansion
        ghost['radius'] += expansion
        ghost['ghost'] = True
        ghosts.append(ghost)
    return retained + [dict(b) for b in boxes], boxes + ghosts


def update_pedestrian_latch(lock_id, observations, progress):
    """Release only after three consecutive observed positions outside corridor.

    A missing tracked pedestrian does not prove that the pedestrian exited.
    The association layer copies the clear counter into the next observation.
    """
    if lock_id is None:
        return None
    seen = next((b for b in observations if b['id'] == lock_id), None)
    if seen is None:
        return lock_id
    if abs(seen['d']) > 3.5 or seen['s'] < progress - 5.0:
        seen['clear'] = seen.get('clear', 0) + 1
        if seen['clear'] >= 3:
            return None
    else:
        seen['clear'] = 0
    return lock_id
