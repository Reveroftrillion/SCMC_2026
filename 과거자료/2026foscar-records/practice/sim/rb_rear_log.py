"""Roundabout entries vs the car behind: one record per entry, appended to rb_rear_records.jsonl.

Kept for the "the car behind will not let us in" rule (user 2026-10-02): what we saw and predicted at the launch,
what the car behind really did, and whether it hit us.

    python3 practice/sim/rb_rear_log.py practice/sim/runs/<run> [...]

Per run with an RB_TUCK_GO (or RB_GREEDY_GO, or the last RB_ROLL_GO when rolled in) line in launch_0.log:
  seen      at the launch: our station, the lead / rear runs (u, speed), the predicted merge time and rear gap
  rear car  the ring car right behind our tail once we are merged (hindsight)
  truth_go  at the launch: its nose and speed, its time to our entry
  truth_m   at our merge: rear gap (our tail - its nose along the ring) and its speed
  after     from the launch for 10 s: min body gap to the rear car, its lowest speed and hardest braking, contact
Runs already in the file are skipped.
"""
import json
import math
import sys
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT/'src'/'foscar_planning'), str(ROOT/'src'/'foscar_control')]
from foscar_control.core import Route                                   # noqa: E402
from foscar_planning.roundabout_boxes import RingLane                   # noqa: E402
from foscar_planning.roundabout_planner import RoundaboutGeometry, RoundaboutPlanner  # noqa: E402

OUT = HERE/'rb_rear_records.jsonl'
EGO_CENTRE, EGO_LEN, EGO_W = 1.4825, 4.635, 1.892      # truth pose is the rear axle (body box as loop_runner)
# The simulator's NPC position lies 1.48 m behind its body centre (lidar hulls seen whole, 2026-10-02)
NPC_AHEAD = 1.48


def ring():
    b = ROOT/'src'/'foscar_bringup'
    route = Route(np.loadtxt(b/'routes'/'global_path.txt'))
    links = {l['idx']: np.asarray(l['points'])[:, :2] for l in json.load(open(b/'maps'/'link_set.json'))}
    pts = np.vstack([links[k][:-1] for k in ('A2256W000132', 'A2256W000133', 'A2256W000134', 'A2256W000135')])
    o = pts.mean(0); q = pts-o
    fit = np.linalg.lstsq(np.c_[2*q, np.ones(len(q))], (q*q).sum(1), rcond=None)[0]
    c = types.SimpleNamespace(center=o+fit[:2], radius=math.sqrt(fit[2]+fit[:2]@fit[:2]), direction=1)
    g = RoundaboutGeometry(c, route, 0., 820.92, 908.49)
    lane = RingLane(c, g.origin)
    table = json.load(open(ROOT/'src'/'foscar_planning'/'config'/'roundabout_motion.json'))
    return route, c, g, lane, RoundaboutPlanner(g, lane, 876.6283, 60/3.6, motion_table=table)


def corners(x, y, yaw, l, w):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+a*c-b*s, y+a*s+b*c) for a, b in ((l/2, w/2), (l/2, -w/2), (-l/2, -w/2), (-l/2, w/2))]


def body_gap(A, B):
    """Separation of two convex quads (0 when they overlap)."""
    def axes(P):
        return [(P[(i+1) % 4][1]-P[i][1], P[i][0]-P[(i+1) % 4][0]) for i in range(4)]
    best = 0.
    for ax in axes(A)+axes(B):
        n = math.hypot(*ax)
        if n < 1e-9:
            continue
        ax = (ax[0]/n, ax[1]/n)
        pa = [p[0]*ax[0]+p[1]*ax[1] for p in A]; pb = [p[0]*ax[0]+p[1]*ax[1] for p in B]
        best = max(best, max(min(pb)-max(pa), min(pa)-max(pb)))
    return best


def record(run_dir, route, c, g, lane, rp):
    log = (run_dir/'launch_0.log').read_text(errors='ignore')
    tag = next((k for k in ('RB_TUCK_GO ', 'RB_GREEDY_GO ', 'RB_ROLL_GO ') if k in log), None)
    if tag is None:
        return None
    # rolled in: the last go counts (earlier ones were given up on the way, RB_ROLL_ABORT)
    lines = [l for l in log.splitlines() if tag in l]
    go = json.loads((lines[-1] if tag == 'RB_ROLL_GO ' else lines[0]).split(tag)[1])
    tgo = go['t']
    T = [json.loads(l) for l in open(run_dir/'truth_0.jsonl')]
    O = [json.loads(l) for l in open(run_dir/'objects_0.jsonl')]
    ot = np.array([f['t'] for f in O])

    def frame(t):
        return O[min(int(np.searchsorted(ot, t)), len(O)-1)]

    def cars(f):
        out = {}
        for ob in f['objects']:
            if ob['id'] == 0 or ob['type'] != 1:
                continue
            if abs(math.hypot(ob['x']-c.center[0], ob['y']-c.center[1])-c.radius) > 3.:
                continue
            u = lane.u_of(ob['x'], ob['y'])+NPC_AHEAD   # body centre, not the truth point
            out[ob['id']] = dict(tail=u-ob['l']/2, nose=u+ob['l']/2, v=ob['v']/3.6, ob=ob)
        return out
    # our station and merge time
    prev = None; s_go = None; tm = sm = None
    for x in T:
        s = route.project(x['x'], x['y'], prev); s = s[0] if isinstance(s, tuple) else s; prev = s
        if s_go is None and x['t'] >= tgo:
            s_go = s
        if x['t'] >= tgo and s >= g.s_merged:
            tm, sm = x['t'], s
            break
    c0 = cars(frame(tgo))
    nose_go = rp.front_arc(s_go) if s_go is not None else None
    # the rear car, in hindsight: the ring car right behind our tail once we are merged (a car behind the lead at
    # the launch often went in before us and became the one we follow)
    rear_id = None
    if tm is not None:
        cm0 = cars(frame(tm)); iv0 = g.interval(sm)
        if iv0 is not None:
            behind = [k for k, x in cm0.items() if x['nose'] <= iv0[0]+.5]
            rear_id = max(behind, key=lambda k: cm0[k]['nose']) if behind else None
    if rear_id is not None and rear_id not in c0:
        rear_id_seen_at_go = False
    else:
        rear_id_seen_at_go = True
    rec = dict(run=run_dir.name, scenario=json.loads((run_dir/'results.jsonl').read_text().splitlines()[-1]).get('scenario'),
               mode=tag.strip(), s_go=None if s_go is None else round(s_go, 2),
               seen=dict(lead=go.get('lead'), pred=go.get('pred'), tau=go.get('tau')),
               rear_id=rear_id)
    rec['rear_on_ring_at_go'] = rear_id is not None and rear_id_seen_at_go
    if rear_id is not None and rear_id in c0:
        r0 = c0[rear_id]
        rec['truth_go'] = dict(rear_nose=round(r0['nose'], 2), rear_v=round(r0['v'], 2),
                               our_nose=None if nose_go is None else round(nose_go, 2),
                               rear_to_entry_s=round(-r0['nose']/max(.1, r0['v']), 2) if r0['nose'] < 0 else 0.)
        if tm is not None:
            cm = cars(frame(tm)); iv = g.interval(sm)
            if rear_id in cm and iv is not None:
                rec['truth_m'] = dict(merge_in=round(tm-tgo, 2), rear_gap=round(iv[0]-cm[rear_id]['nose'], 2),
                                      rear_v=round(cm[rear_id]['v'], 2))
        # 10 s after the launch: the rear car's body gap to us, its speeds
        tt = np.array([x['t'] for x in T]); vs = []; gaps = []
        for f in O:
            if not tgo <= f['t'] <= tgo+10.:
                continue
            ob = next((o for o in f['objects'] if o['id'] == rear_id), None)
            if ob is None:
                continue
            e = T[min(int(np.searchsorted(tt, f['t'])), len(T)-1)]; h = e['yaw']
            E = corners(e['x']+EGO_CENTRE*math.cos(h), e['y']+EGO_CENTRE*math.sin(h), h, EGO_LEN, EGO_W)
            oh = math.radians(ob['yaw'])
            gaps.append(body_gap(E, corners(ob['x']+NPC_AHEAD*math.cos(oh), ob['y']+NPC_AHEAD*math.sin(oh), oh, ob['l'], ob['w'])))
            vs.append((f['t'], ob['v']/3.6))
        if vs:
            v = np.array([q[1] for q in vs]); t = np.array([q[0] for q in vs])
            dec = 0.
            for i in range(len(t)):
                j = int(np.searchsorted(t, t[i]+.5))
                if j < len(t):
                    dec = max(dec, (v[i]-v[j])/max(.1, t[j]-t[i]))
            rec['after'] = dict(min_body_gap=round(min(gaps), 2), rear_v_min=round(float(v.min()), 2),
                                rear_decel_max=round(dec, 2))
    res = json.loads((run_dir/'results.jsonl').read_text().splitlines()[-1])
    rec['contact_ids'] = sorted({x.get('id') for x in (res.get('collisions') or [])})   # the simulator's collision packets
    rec['rear_contact'] = rear_id is not None and rear_id in rec['contact_ids']
    rec['front_contact'] = bool(rec['contact_ids']) and not rec['rear_contact']
    return rec


def main(dirs):
    done = set()
    if OUT.exists():
        done = {json.loads(l)['run'] for l in OUT.read_text().splitlines() if l.strip()}
    parts = ring()
    added = 0
    with open(OUT, 'a') as f:
        for d in map(Path, dirs):
            if d.name in done or not (d/'launch_0.log').exists():
                continue
            try:
                rec = record(d, *parts)
            except Exception as exc:                     # a broken run must not stop the batch
                print(f'{d.name}: {exc}')
                continue
            if rec is not None:
                f.write(json.dumps(rec, ensure_ascii=False)+'\n'); added += 1
    print(f'rb_rear_log: {added} added, {OUT}')


if __name__ == '__main__':
    main(sys.argv[1:])
