#!/usr/bin/env python3
"""Closed-loop tunnel obstacle sweep: many random barricade layouts, no simulator.

Real code: foscar_control Controller, foscar_planning Behavior (cruise/avoid/tunnel
from behavior.yaml), WallMemory/WallCorridor, and the obstacle_node output gates
(mapped-road centroid, tunnel wide-body rule, corridor edge, 3-frame confirm).
Modelled: kinematic bicycle with ~0.3 s steering lag and measured pedal maps,
a VLP-16 beam-height visibility model (low bodies appear only ~30 m ahead),
0.08 s scan-to-event delay projected with the latest pose (as behavior_node
does), and straight tunnel walls at fixed route offsets.

It checks logic over layouts; it is not a substitute for MORAI tyre/actuator
dynamics or real point clouds.

    python3 tools/tunnel_sweep.py --runs 300 --jobs 12
    python3 tools/tunnel_sweep.py --seed 17 --verbose      # replay one layout
"""
import argparse
import csv
import json
import math
import multiprocessing as mp
import random
import sys
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('foscar_control', 'foscar_planning', 'foscar_lidar'):
    sys.path.insert(0, str(ROOT/'src'/pkg))

from foscar_control.core import Controller, Route, Target as ControlTarget   # noqa: E402
from foscar_planning.frenet import to_frenet, to_xy                             # noqa: E402
from foscar_planning.lane_prediction import LaneMap                             # noqa: E402
from foscar_planning.state import Behavior, Context, EventInfo                  # noqa: E402
from foscar_planning.states.avoid import Avoid                                  # noqa: E402
from foscar_planning.states.cruise import Cruise                                # noqa: E402
from foscar_planning.states.tunnel import Tunnel                                # noqa: E402
from foscar_planning.tunnel_corridor import WallMemory                          # noqa: E402
from foscar_lidar.road_mask import RoadMask                                     # noqa: E402

LINK_SET = ROOT/'src/foscar_bringup/maps/link_set.json'
ROUTE_FILE = ROOT/'src/foscar_bringup/routes/global_path.txt'
FRONT, REAR, HALF_W, WHEELBASE = 3.8, 4.635-3.8, 1.892/2, 3.0
LIDAR_X, LIDAR_H = 2.0, 2.11
CHANNELS = np.radians(np.arange(-15, 16, 2))
DT, SCAN_DT, EVENT_DELAY = .05, .1, .08
ALLOWED = ['CRUISE', 'AVOID', 'YIELD', 'TUNNEL']


@dataclass
class Barricade:
    s: float
    d: float
    width: float           # across the road
    height: float
    length: float = .4     # along the road


def load_params():
    b = yaml.safe_load((ROOT/'src/foscar_planning/config/behavior.yaml').read_text())
    b = next(iter(b.values()))['ros__parameters']
    c = yaml.safe_load((ROOT/'src/foscar_bringup/config/low_speed.yaml').read_text())
    c = next(v['ros__parameters'] for v in c.values() if 'a_lat_mps2' in v.get('ros__parameters', {}))
    return b, c


def state_params(cls, flat, prefix):
    return {k: flat.get(f'{prefix}.{k}', v) for k, v in cls.defaults.items()}


def rect(cx, cy, yaw, front, rear, half):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(cx+a*c-w*s, cy+a*s+w*c) for a, w in ((front, half), (front, -half), (-rear, -half), (-rear, half))])


def overlap(p, q):
    """Separating-axis test for two convex quadrilaterals."""
    for poly in (p, q):
        for i in range(4):
            e = poly[(i+1) % 4]-poly[i]
            axis = np.array([-e[1], e[0]])
            a, b = p@axis, q@axis
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def box_polygon(route, b):
    return np.array([to_xy(route, s, d) for s, d in ((b.s-b.length/2, b.d-b.width/2), (b.s+b.length/2, b.d-b.width/2),
                                                      (b.s+b.length/2, b.d+b.width/2), (b.s-b.length/2, b.d+b.width/2))])


def visible_points(poly, height, pose):
    """Box perimeter samples a VLP-16 beam actually hits, in base_link."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = x+LIDAR_X*c, y+LIDAR_X*s
    centre = poly.mean(axis=0)
    out = []
    for i in range(4):
        a, b = poly[i], poly[(i+1) % 4]
        mid = (a+b)/2
        normal = np.array([b[1]-a[1], -(b[0]-a[0])])
        if normal@(mid-centre) < 0:
            normal = -normal
        if normal@(mid-np.array([lx, ly])) >= 0:
            continue                                  # face turned away from the sensor
        n = max(2, int(np.linalg.norm(b-a)/.1))
        for u in np.linspace(0., 1., n):
            p = a+(b-a)*u
            r = math.hypot(p[0]-lx, p[1]-ly)
            h = LIDAR_H+r*np.tan(CHANNELS)
            if np.any((h > .05) & (h < height)):
                dx, dy = p[0]-x, p[1]-y
                out.append((c*dx+s*dy, -s*dx+c*dy))
    return np.array(out)


class Sim:
    def __init__(self, layout, v0_kmh=40., start_s=1850., end_s=2000., walls=(-3.7, 7.4), use_walls=True, verbose=False):
        bparams, cp = load_params()
        self.route = Route(np.loadtxt(ROUTE_FILE))
        self.lane_map = LaneMap.from_file(str(LINK_SET))
        self.road = RoadMask.from_file(str(LINK_SET))
        self.layout, self.walls, self.use_walls, self.verbose = layout, walls, use_walls, verbose
        self.end_s = end_s
        self.behavior = Behavior([Cruise(**state_params(Cruise, bparams, 'cruise')),
                                  Avoid(**state_params(Avoid, bparams, 'avoid')),
                                  Tunnel(**state_params(Tunnel, bparams, 'tunnel'))])
        self.ctrl = Controller(self.route, 60., cp.get('test_distance_m', 20.), cp['lookahead_m'], cp['max_cross_track_m'], True,
                               cp['a_lat_mps2'], cp['decel_mps2'], cp['accel_max'], cp['lookahead_gain'], cp['lookahead_max_m'],
                               cp['accel_mps2'], cp['steer_tau'], cp['lateral_ki'], 0., cp['hold_brake'],
                               cp['transition_time_s'], cp['return_time_s'], cp['transition_min_m'],
                               cp['entry_accel_max'], cp['accel_rise_per_s'])
        x, y = self.route.at(start_s)
        self.pose = [x, y, self.route.heading_at(start_s)]
        # Entry speed cannot exceed the route profile there (it would FAULT at once).
        self.v, self.delta, self.t = min(v0_kmh/3.6, self.route.speed_at(start_s, self.ctrl.profile)), 0., 0.
        self.polys = [box_polygon(self.route, b) for b in layout]
        self.hits = [0]*len(layout)
        self.pending = []           # (deliver_at, base_link event, scan pose)
        self.events = {}
        self.memory = WallMemory()
        self.corridor = None
        self.target = ControlTarget()

    def s(self):
        return float(to_frenet(self.route, self.pose[0], self.pose[1], getattr(self, '_s', 1850.))[0])

    def scan(self):
        s_now = self._s
        for i, (b, poly) in enumerate(zip(self.layout, self.polys)):
            pts = visible_points(poly, b.height, self.pose)
            if len(pts):
                pts = pts[(pts[:, 0] >= 0.) & (pts[:, 0] <= 40.) & (np.abs(pts[:, 1]) < 20.)]
            if len(pts) < 5:
                self.hits[i] = 0
                continue
            self.hits[i] += 1
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            centre = (lo+hi)/2
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            corners_b = [(centre[0]+ox, centre[1]+oy) for ox in ((lo-hi)[0]/2, (hi-lo)[0]/2) for oy in ((lo-hi)[1]/2, (hi-lo)[1]/2)]
            corners = np.array([(self.pose[0]+c*bx-s*by, self.pose[1]+s*bx+c*by) for bx, by in corners_b])
            mx, my = self.pose[0]+c*centre[0]-s*centre[1], self.pose[1]+s*centre[0]+c*centre[1]
            # obstacle_node output gates
            if self.road.outside_one(mx, my) > 0.:
                continue
            dd = [to_frenet(self.route, px, py, s_now)[1] for px, py in corners]
            if max(dd)-min(dd) > 2.0 and not (self.road.outside(corners) <= 2.5).all():
                continue                                  # tunnel wide-body rule
            nearest = 0. if min(dd) < 0. < max(dd) else min(abs(min(dd)), abs(max(dd)))
            if nearest > 1.5 or self.hits[i] < 3:
                continue
            self.pending.append((self.t+EVENT_DELAY, i, centre, hi-lo, self.t))
        if self.use_walls:
            rows = []
            for ahead in np.arange(3., 45., 1.):
                for d, side in ((self.walls[0], 'R'), (self.walls[1], 'L')):
                    px, py = to_xy(self.route, s_now+ahead, d)
                    rows.append([px, py, side])
            if self.memory.add(rows, self.route, s_now, 0., to_frenet, self.lane_map) and (self.corridor is None or self.memory.due(self.t)):
                self.corridor = self.memory.corridor(self.lane_map, end_s=2000.)

    def deliver(self):
        keep = []
        for due, i, centre, size, stamp in self.pending:
            if due > self.t:
                keep.append((due, i, centre, size, stamp))
                continue
            x, y, yaw = self.pose                         # latest pose, as behavior_node.on_event
            c, s = math.cos(yaw), math.sin(yaw)
            mx, my = x+c*centre[0]-s*centre[1], y+s*centre[0]+c*centre[1]
            sl, d = to_frenet(self.route, mx, my, self._s)
            self.events[i] = EventInfo(type='OBSTACLE', stamp=stamp, s=sl, d=d, x=mx, y=my, size_x=float(size[0]),
                                       size_y=float(size[1]), value=f'lidar:sim:{i}', box_yaw=yaw)
        self.pending = keep
        self.events = {k: e for k, e in self.events.items() if -.05 <= self.t-e.stamp <= .5}

    def run(self, t_max=60.):
        next_scan, stopped_for, result = 0., 0., None
        max_lat, min_v, max_brake = 0., math.inf, 0.
        min_gap = math.inf
        log = []
        self._s = self.s()
        while self.t < t_max:
            self._s = self.s()
            if self._s >= self.end_s:
                result = 'PASS'
                break
            if self.t >= next_scan:
                self.scan()
                next_scan += SCAN_DT
            self.deliver()
            x, y, yaw = self.pose
            ctx = Context(s=self._s, x=x, y=y, yaw=yaw, v=self.v, now=self.t, events=list(self.events.values()),
                          loc_mode='DR', route=self.route, s_offset=0.,
                          road_map=self.corridor if self.corridor is not None else self.lane_map,
                          mission_name='TUNNEL', mission_speed_mps=math.inf)
            tgt = self.behavior.tick(ctx, allowed_states=ALLOWED)
            ct = ControlTarget(**{f.name: getattr(tgt, f.name) for f in fields(ControlTarget) if hasattr(tgt, f.name)})
            out = self.ctrl.step(x, y, yaw, self.v, 2, 4, True, DT, ct)
            if out.state == 'FAULT':
                result = 'FAULT:'+out.reason
                break
            accel = out.accel*4.3
            brake = (1.6+7.4*out.brake) if out.brake > .01 else 0.
            coast = .32 if out.accel <= .01 and brake == 0. else 0.
            self.v = max(0., self.v+(accel-brake-coast)*DT)
            self.delta += (out.steering-self.delta)*DT/.3
            self.pose[2] += self.v/WHEELBASE*math.tan(self.delta)*DT
            self.pose[0] += self.v*math.cos(self.pose[2])*DT
            self.pose[1] += self.v*math.sin(self.pose[2])*DT
            self.t += DT
            max_lat = max(max_lat, self.v**2*abs(math.tan(self.delta))/WHEELBASE)
            max_brake = max(max_brake, out.brake)
            if self._s > 1880:
                min_v = min(min_v, self.v)
            car = rect(*self.pose, FRONT, REAR, HALF_W)
            for i, poly in enumerate(self.polys):
                if overlap(car, poly):
                    result = f'COLLISION:box{i}'
            if result:
                break
            cd = [to_frenet(self.route, px, py, self._s)[1] for px, py in car]
            min_gap = min(min_gap, min(self.walls[1]-max(cd), min(cd)-self.walls[0]))
            if min(cd) < self.walls[0] or max(cd) > self.walls[1]:
                result = 'COLLISION:wall'
                break
            stopped_for = stopped_for+DT if self.v < .2 else 0.
            if stopped_for > 5.:
                result = 'STOPPED'
                break
            if self.verbose and int(self.t/DT) % 4 == 0:
                log.append(f't={self.t:5.2f} s={self._s:7.1f} d={to_frenet(self.route, x, y, self._s)[1]:+5.2f} '
                           f'v={self.v*3.6:4.1f} tgt={out.target_speed*3.6:4.1f} {tgt.state:6s} '
                           f'lat={tgt.lat_offset:+.2f} vmax={tgt.v_max*3.6 if math.isfinite(tgt.v_max) else -1:5.1f} '
                           f'stop={tgt.stop_s:6.1f} ev={len(self.events)} {out.reason[:30]}')
        return dict(result=result or 'TIMEOUT', s=round(self._s, 1), t=round(self.t, 1), max_lat=round(max_lat, 2),
                    min_v_kmh=round(min_v*3.6, 1) if math.isfinite(min_v) else None,
                    max_brake=round(max_brake, 2), min_wall_gap=round(min_gap, 2)), log


def sample_layout(rng, walls=(-3.7, 7.4)):
    n = rng.choice([1, 2, 2, 3])
    layout = []
    for _ in range(200):
        if len(layout) == n:
            break
        width, height = rng.uniform(2.5, 3.5), rng.uniform(.5, .9)
        s = rng.uniform(1890., 1985.)
        if any(abs(s-b.s) < 15. for b in layout):
            continue
        d = rng.uniform(-1.75, 5.25)                     # centred on the mapped lanes
        d = min(max(d, walls[0]+.3+width/2), walls[1]-.3-width/2)
        gaps = (d-width/2-walls[0], walls[1]-(d+width/2))
        if max(gaps) < 2*HALF_W+.6:                      # physically impassable: skip
            continue
        layout.append(Barricade(round(s, 1), round(d, 2), round(width, 2), round(height, 2)))
    return sorted(layout, key=lambda b: b.s)


def run_seed(args):
    seed, use_walls, v0 = args
    rng = random.Random(seed)
    layout = sample_layout(rng)
    speed = v0 if v0 else rng.uniform(30., 50.)
    res, _ = Sim(layout, v0_kmh=speed, use_walls=use_walls).run()
    res.update(seed=seed, v0_kmh=round(speed, 1), n=len(layout),
               layout=json.dumps([[b.s, b.d, b.width, b.height] for b in layout]))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=200)
    ap.add_argument('--jobs', type=int, default=max(1, mp.cpu_count()-1))
    ap.add_argument('--seed', type=int, help='run a single layout verbosely')
    ap.add_argument('--start-seed', type=int, default=0)
    ap.add_argument('--v0', type=float, default=0., help='entry speed km/h (default random 30-50)')
    ap.add_argument('--no-walls', action='store_true', help='map pavement only (previous behaviour)')
    ap.add_argument('--out', default='tunnel_sweep.csv')
    a = ap.parse_args()
    if a.seed is not None:
        rng = random.Random(a.seed)
        layout = sample_layout(rng)
        speed = a.v0 or rng.uniform(30., 50.)
        print('layout (s, d, width, height):', [(b.s, b.d, b.width, b.height) for b in layout], f'v0={speed:.1f}km/h')
        res, log = Sim(layout, v0_kmh=speed, use_walls=not a.no_walls, verbose=True).run()
        print('\n'.join(log))
        print(res)
        return
    seeds = [(s, not a.no_walls, a.v0) for s in range(a.start_seed, a.start_seed+a.runs)]
    with mp.Pool(a.jobs) as pool:
        rows = pool.map(run_seed, seeds, chunksize=1)
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    kinds = {}
    for r in rows:
        kinds.setdefault(r['result'].split(':')[0] if r['result'].startswith('FAULT') else r['result'], []).append(r)
    print(f'{len(rows)} layouts -> {a.out}')
    for k, v in sorted(kinds.items(), key=lambda kv: -len(kv[1])):
        print(f'  {k:18s} {len(v):4d}  seeds {[r["seed"] for r in v[:12]]}')
    passed = [r for r in rows if r['result'] == 'PASS']
    if passed:
        print('  PASS max lateral accel (m/s2) p50/p95/max: %.2f / %.2f / %.2f' % tuple(
            np.percentile([r['max_lat'] for r in passed], [50, 95, 100])))


if __name__ == '__main__':
    main()
