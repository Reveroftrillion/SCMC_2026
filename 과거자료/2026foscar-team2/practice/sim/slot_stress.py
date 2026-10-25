#!/usr/bin/env python3
"""PRACTICE ONLY: randomized closed-loop stress of the highway slot planner (exp/box-planner), no simulator.

NPCs keep their lane and speed and never react (the sim's NPCs do not brake for a cut-in); the ego follows the
policy's reference exactly and its speed cap at +3.5 / -4 m/s2. Each case: ego in lane 5 at entry (s1122) or
lane 4 past CP11 (s1300), random traffic in the ego's lane and the next one. Reports the lanes reached by s1540
(lanes 3 and 2 converge past it) and every overlap with an NPC:
  OURS       we moved into it: it was ahead of us, or in the lane we changed into and it reached us from behind
             within 3 s of the change start (we cut in in front of it);
  RAMMED     it ran into us from behind in the lane it kept (it does not brake);
  ARTIFACT   a lane-5 NPC kept on lane 5's centre through the taper (it would merge), or it was spawned on us.

    python3 practice/sim/slot_stress.py [--cases 200] [--seed 0]
"""
import argparse, math, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/foscar_planning/test'))
from test_highway_boxes import car, snap, highway_route, config          # noqa: E402
from foscar_planning.highway_policy import HighwayPolicy                  # noqa: E402
from foscar_planning.highway_maneuver import heading_at                   # noqa: E402
from foscar_planning.state import Context                                 # noqa: E402

FRONT, LENGTH, WIDTH = 3.8, 4.635, 1.892


def npc(road, lane, s, key, v):
    """A car on the lane's observation centre line (it runs on upstream of CP10, where the lane table starts)."""
    at = lambda q: np.asarray(road.observation_at(lane, q))
    pts = []
    for ds in (-2., 2.):
        c = at(s+ds);a = at(s+ds+.5)-at(s+ds-.5);u = a/np.linalg.norm(a);n = np.array([-u[1], u[0]])
        pts += [c+.9*n, c-.9*n]
    u = at(s+1.)-at(s-1.);u = u/np.linalg.norm(u)*v
    return type(car(road, 4, 1300., key, v=v))(id=key, xy=np.array(pts), speed_ready=True, velocity_bounds=(u[0], u[0], u[1], u[1]))


def traffic(rng, lanes, s0, v0):
    npcs = []
    for lane in lanes:
        v = rng.uniform(10., 20.); s = s0-rng.uniform(20., 60.)
        while s < s0+120.:
            if abs(s-s0) > 12. or lane != lanes[0]:npcs.append((lane, s, float(np.clip(v+rng.normal(0, 1.5), 8., 22.))))
            s += rng.uniform(9., 40.)
    return npcs


def run(road, cfg, lane, s0, v0, npcs, cp11):
    p = HighwayPolicy(road, cfg);p.current_lane = lane;p.cp11 = cp11
    s = s0;v = v0;ref = None;lanes = [lane];hits = [];ms = [];started = None;touched = set()
    for k in range(400):
        t = 1.+.1*k
        if s > 1540.:break
        if ref is None:x, y = road.at(lane, s);yaw = heading_at(road, lane, s, 1.)
        else:q = ref.q_at_global(s);x, y = ref.at_q(q);yaw = ref.heading_at(q)
        objs = [npc(road, l, s1+sv*.1*k, f'n{i}', sv) for i, (l, s1, sv) in enumerate(npcs)
                if road.observation_tables[l][0, 0]+3. < s1+sv*.1*k < min(1585., road.tables[l][-1, 0]-3.)]
        c0 = time.perf_counter()
        out = p.tick(Context(s=s, x=x, y=y, yaw=yaw, v=v, now=t, mission_name='HIGHWAY'), snap(*objs, t=t))
        ms.append((time.perf_counter()-c0)*1e3)
        if out.reference is not None:ref = out.reference
        if out.reason == 'CHANGE_STARTED':started = (t, p.committed.target_lane)
        if p.current_lane != lanes[-1]:lanes.append(p.current_lane)
        u = np.array([math.cos(yaw), math.sin(yaw)]);n = np.array([-u[1], u[0]])
        lane_of = {f'n{i}': l for i, (l, _, _) in enumerate(npcs)}
        for o in objs:
            rel = o.xy.mean(axis=0)-np.array([x, y]);a = float(rel@u)-(FRONT-(LENGTH-FRONT))/2;b = abs(float(rel@n))
            if b < WIDTH-.05 and abs(a) < (LENGTH+4.)/2 and o.id not in touched:   # NPC bodies are 4 m long
                touched.add(o.id)                              # judged at first contact only
                l = lane_of[o.id];stn = float(road.project_many(o.xy.mean(axis=0)[None, :])[0])
                if k == 0 or (l == 5 and stn > 1200.):kind = 'ARTIFACT'
                elif a < 0 and not (started and l == started[1] and t-started[0] < 3.):kind = 'RAMMED'
                else:kind = 'OURS'
                hits.append((kind, o.id, l, round(s, 1), round(a, 1), round(b, 2), out.reason, round(v*3.6)))
        v = float(np.clip(min(out.v_max, 40.), v-.4, v+.35));v = max(0., v);s += v*.1
    return lanes, hits, ms


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--cases', type=int, default=200); ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed);road = highway_route();cfg = config()
    stats = dict(cases=0, reached={}, ours=[], rammed=0, artifact=0);all_ms = []
    for c in range(a.cases):
        entry = c % 2 == 0
        lane, s0 = (5, 1122.) if entry else (4, 1300.)
        v0 = rng.uniform(11., 20.)
        npcs = traffic(rng, (lane, lane-1), s0, v0)
        lanes, hits, ms = run(road, cfg, lane, s0, v0, npcs, cp11=not entry)
        all_ms += ms;stats['cases'] += 1
        key = '->'.join(map(str, lanes));stats['reached'][key] = stats['reached'].get(key, 0)+1
        ours = [h for h in hits if h[0] == 'OURS']
        if ours:stats['ours'].append((c, lane, round(v0*3.6), ours[0]))
        stats['rammed'] += any(h[0] == 'RAMMED' for h in hits);stats['artifact'] += any(h[0] == 'ARTIFACT' for h in hits)
    print(f"{stats['cases']} cases; lanes reached by s1540: {dict(sorted(stats['reached'].items()))}")
    print(f"overlaps we caused: {len(stats['ours'])} cases; rammed from behind by a non-braking NPC: {stats['rammed']}; "
          f"test artifacts: {stats['artifact']}")
    for x in stats['ours'][:15]:print('   case', x)
    ms = np.array(all_ms);print(f'time per tick: median {np.median(ms):.1f} ms, p95 {np.percentile(ms, 95):.1f}, max {ms.max():.1f}')


if __name__ == '__main__':
    main()
