#!/usr/bin/env python3
"""PRACTICE ONLY: a record of every highway lane change with the whole box picture it was started on.

User 2026-09-30: "every time you cut in, add the full box record, and keep noting which box states worked and which
did not, to find the limit of what can be passed." For every CHANGE_STARTED on /behavior/highway one JSON line:

  when     run, s, lanes, ego speed, the phase decided just before (SCOUT / YIELD / PASS / SETTLE), the admission
  plan     mode, goal speed, the body we land ahead of (rear) and behind (front), slot, start/end times
  own lane our lead: bumper gap and speed
  target   the target lane's boxes relative to our rear axle, merged into segments [from, to, FREE/OCCUPIED/UNKNOWN],
           and every body there [rear, front, speed km/h, kind] relative to our rear axle
  outcome  completed (CHANGE_COMPLETE seen), the smallest clearance between our body and any other hull while
           changing and settling (m, <= 0 is contact), the loop_runner collisions in that stretch, OK / CONTACT /
           INCOMPLETE

    python3 practice/sim/pass_log.py practice/sim/runs/<stamp> [--run N]    # append (loop_runner does this per run)
    python3 practice/sim/pass_log.py --summary                                # what worked, what did not
    python3 practice/sim/pass_log.py --label last 1330 possible [--note ..]   # the user's call on a moment (latest run)
    python3 practice/sim/pass_log.py --label 20260930_151745 1330 impossible

Besides every change started (kind "change"), a "decision" record is kept every second the car waits to change
(SCOUT / YIELD / PASS): the same box view and why it did not go. The user watches and says "that one was possible /
impossible" (2026-09-30); --label stores it on the nearest record, and precedents() finds labelled records like a
new view. src/ never reads this file: what the labels show goes into highway.yaml by hand.
"""
import argparse, glob, json, math, os, re
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE/'pass_records.jsonl'
FRONT, BACK, HALF = 3.8, 0.835, 0.946     # rear axle to front / rear bumper, half width (highway.yaml vehicle)


def read_highway(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/behavior/highway':
            out.append(json.loads(deserialize_message(data, get_message(types[topic])).data))
    return out


def read_track(bag):
    """(stamp, s, x, y, yaw, speed m/s, behavior state) along the run, for contacts outside the highway mission."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    s = v = None; state = ''; out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/localization/s':s = deserialize_message(data, get_message(types[topic])).data
        elif topic == '/behavior/target':state = deserialize_message(data, get_message(types[topic])).state
        elif topic == '/control/telemetry':
            try:v = json.loads(deserialize_message(data, get_message(types[topic])).data).get('speed_mps', v)
            except ValueError:pass
        elif topic == '/localization/odometry' and s is not None:
            m = deserialize_message(data, get_message(types[topic])); q = m.pose.pose.orientation
            out.append((t*1e-9, s, m.pose.pose.position.x, m.pose.pose.position.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), v, state))
    return out


def segments(boxes, s):
    """Consecutive boxes of one state merged: [[from, to, state]] relative to our rear axle."""
    out = []
    for lo, hi, state, *_ in boxes:
        if out and out[-1][2] == state and abs(out[-1][1]-round(lo-s, 1)) < .15:
            out[-1][1] = round(hi-s, 1)
        else:
            out.append([round(lo-s, 1), round(hi-s, 1), state])
    return out


def view(segs, bodies, lead, v_kmh):
    """The box coordinate view the user talks in (2026-09-30): our body is the origin (rear bumper -BACK, front bumper
    FRONT); on the target (left) side the states beside us, how much free box runs on ahead of our front bumper and
    behind our rear bumper, and the cars that end those runs; ahead of us the gap to our own lead."""
    def state_at(x):
        return next((st for a, b, st in segs if a <= x < b), 'NONE')
    beside = sorted({st for a, b, st in segs if b > -BACK and a < FRONT})
    def run_from(x, step):
        n = 0.
        while state_at(x+step*(n+.05)) == 'FREE' and n < 150:n += .1
        return round(n, 1)
    fronts = [b for b in bodies if b[0] >= FRONT-.5]; rears = [b for b in bodies if b[1] <= -BACK+.5]
    fb = min(fronts, key=lambda b: b[0], default=None); rb = max(rears, key=lambda b: b[1], default=None)
    return dict(left_beside=beside,
                left_front_free_m=run_from(FRONT, 1), left_front_car=None if fb is None else dict(gap_m=round(fb[0]-FRONT, 1), kmh=fb[2]),
                left_rear_free_m=run_from(-BACK, -1), left_rear_car=None if rb is None else dict(gap_m=round(-BACK-rb[1], 1), kmh=rb[2],
                                                                                             closing_kmh=round(rb[2]-v_kmh, 1)),
                my_front_gap_m=None if not lead else lead['gap_m'], my_front_kmh=None if not lead else lead['kmh'])


def clearance(d):
    """Smallest distance between our body rectangle and any observed hull (<= 0: overlap), from one tick."""
    ex, ey = d['ego_xy']; c, sn = math.cos(d['ego_yaw']), math.sin(d['ego_yaw'])
    best = math.inf
    for o in d.get('objects') or []:
        pts = [((x-ex)*c+(y-ey)*sn, -(x-ex)*sn+(y-ey)*c) for x, y in o['xy']]
        if not pts:continue
        a0, a1 = min(p[0] for p in pts), max(p[0] for p in pts)
        b0, b1 = min(p[1] for p in pts), max(p[1] for p in pts)
        dx = max(0., a0-FRONT, -BACK-a1); dy = max(0., b0-HALF, -HALF-b1)
        g = math.hypot(dx, dy) if dx > 0 or dy > 0 else -min(FRONT-a0, a1+BACK, HALF-b0, b1+HALF)
        best = min(best, g)
    return best


def body_rel(d, o):
    """An observed hull in our body frame: (lon min, lon max, lat min, lat max), lon from the rear axle, lat left +."""
    ex, ey = d['ego_xy']; c, sn = math.cos(d['ego_yaw']), math.sin(d['ego_yaw'])
    pts = [((x-ex)*c+(y-ey)*sn, -(x-ex)*sn+(y-ey)*c) for x, y in o['xy']]
    return min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts), max(p[1] for p in pts)


def along(d, o):
    v = o.get('velocity') or (0., 0.)
    return v[0]*math.cos(d['ego_yaw'])+v[1]*math.sin(d['ego_yaw'])


def contact(L, k, c, run_dir, t0):
    """One collision (loop_runner) analysed on the highway ticks: which part of our body, what, doing what, why."""
    base = dict(kind='contact', run_dir=os.path.basename(run_dir.rstrip('/')), run=k, s=round(c['s'], 1), other_id=c.get('id'),
                other_type=c.get('type'), runner_dist_m=c.get('dist'))
    idx = next((i for i, d in enumerate(L) if d['global_s'] >= c['s']-.5), None)
    if idx is None or abs(L[idx]['global_s']-c['s']) > 5.:
        return dict(base, **off_highway(c, run_dir, k))
    d = L[idx]
    objs = [(clearance(dict(d, objects=[o])), o) for o in d.get('objects') or []]
    if not objs:return dict(base, where='highway, no body observed at the moment')
    clr, o = min(objs, key=lambda x: x[0]); a0, a1, b0, b1 = body_rel(d, o)
    part = ('front' if a0 >= FRONT-.6 else 'rear' if a1 <= -BACK+.6 else 'left side' if b0 > 0 else 'right side' if b1 < 0 else 'overlap')
    if part in ('front', 'rear') and (b0 > HALF-.4 or b1 < -HALF+.4):part += '-left' if b0 > 0 else '-right'
    ov = along(d, o); ev = d['ego_speed']
    # the same body over the 3 s before (nearest hull to where it should have been, by position: ids churn)
    hist = []
    for q in L[max(0, idx-40):idx+1]:
        if d['stamp']-q['stamp'] > 3.:continue
        best = None
        for oo in q.get('objects') or []:
            if not oo.get('speed_ready'):continue
            r = body_rel(q, oo)
            dist = math.hypot((r[0]+r[1])/2-(a0+a1)/2, (r[2]+r[3])/2-(b0+b1)/2)
            if best is None or dist < best[0]:best = (dist, oo, r)
        if best and best[0] < 4.:hist.append((q['stamp'], along(q, best[1]), q['ego_speed'], best[2]))
    start = next((q for q in reversed(L[max(0, idx-200):idx+1]) if q.get('reason') == 'CHANGE_STARTED' and d['stamp']-q['stamp'] < 20.), None)
    plan = (start or {}).get('plan') or {}
    why = []
    if hist:
        dv_o = (ov-hist[0][1])*3.6; dv_e = (ev-hist[0][2])*3.6
        if dv_o > 5.:why.append(f'the other car sped up {dv_o:.0f} km/h in the last 3 s')
        if dv_e < -5.:why.append(f'we slowed {-dv_e:.0f} km/h in the last 3 s')
    if part.startswith('rear') and ov > ev+.5:why.append(f'a faster car ({ov*3.6:.0f} vs our {ev*3.6:.0f} km/h) ran into our back')
    if part.startswith('front') and ev > ov+.5:why.append(f'we closed on a slower car ({ov*3.6:.0f} km/h) and did not stop in time')
    if start is not None:
        sv = view(segments((start.get('boxes') or {}).get('boxes') or [], start['global_s']), [], None, start['ego_speed']*3.6)
        why.append(f"{part} contact after the change started at s{start['global_s']:.0f} ({plan.get('mode')}, landing ahead of "
                   f"{plan.get('rear') or '-'} / behind {plan.get('front') or '-'}); "
                   f"then L beside {'/'.join(sv['left_beside'])}, L rear free {sv['left_rear_free_m']} m, L front free {sv['left_front_free_m']} m")
    return dict(base, t_run=round(d['stamp']-t0, 1), phase=d.get('phase'), state=d.get('state'), reason=d.get('reason'),
                lane=d.get('current_lane'), part=part, clearance_m=round(clr, 2),
                other=dict(lon=[round(a0, 1), round(a1, 1)], lat=[round(b0, 1), round(b1, 1)], kmh=round(ov*3.6, 1)),
                v_kmh=round(ev*3.6, 1), closing_kmh=round((ev-ov)*3.6, 1),
                history=[dict(dt=round(h[0]-d['stamp'], 1), other_kmh=round(h[1]*3.6), ego_kmh=round(h[2]*3.6),
                              lon=[round(h[3][0], 1), round(h[3][1], 1)], lat=[round(h[3][2], 1), round(h[3][3], 1)]) for h in hist[::5]],
                change=None if start is None else dict(s=round(start['global_s'], 1), mode=plan.get('mode'), rear=plan.get('rear'),
                                                       front=plan.get('front'), admission=start.get('admission')),
                why=why or ['not classified: see history'])


_TRACKS = {}


def off_highway(c, run_dir, k):
    """A contact outside the highway ticks (tunnel, missions): the part of our body from our pose and the obstacle's
    position, our speed and state, and the tunnel planner's last word about that s (SLALOM lines)."""
    res = [json.loads(l) for l in open(os.path.join(run_dir, 'results.jsonl'))]
    obs = (res[k].get('obstacles') or {}).get(str(c.get('id'))) if k < len(res) else None
    log = os.path.join(run_dir, f'launch_{k}.log');text = open(log, errors='ignore').read() if os.path.exists(log) else ''
    m = re.search(r'(run_[0-9_]+)_0\.db3', text)
    out = dict(where='tunnel' if 1860 <= c['s'] <= 2000 else 'outside the highway')
    if m and obs:
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if bag not in _TRACKS:_TRACKS[bag] = read_track(bag) if os.path.exists(bag) else []
        tr = _TRACKS[bag]
        if tr:
            p = min(tr, key=lambda x: abs(x[1]-c['s']))
            dx, dy = obs['xy'][0]-p[2], obs['xy'][1]-p[3]
            lon = dx*math.cos(p[4])+dy*math.sin(p[4]); lat = -dx*math.sin(p[4])+dy*math.cos(p[4])
            part = 'front' if lon > FRONT-.5 else 'rear' if lon < -BACK+.5 else 'left side' if lat > 0 else 'right side'
            if part in ('front', 'rear') and abs(lat) > .6:part += '-left' if lat > 0 else '-right'
            out.update(part=part, other=dict(centre_lon=round(lon, 1), centre_lat=round(lat, 1), kmh=0.),
                       v_kmh=None if p[5] is None else round(p[5]*3.6, 1), state=p[6])
    first = re.search(r"SLALOM s=([\d.]+) [^\n]*groups=\[\[\(([\d.]+)", text)
    lines = [l for l in re.findall(r'SLALOM[^\n]*', text) if (mm := re.search(r's=([\d.]+)', l)) and c['s']-25 <= float(mm.group(1)) <= c['s']+1]
    why = []
    plans = [(float(mm.group(2)), float(mm.group(2))-float(mm.group(1)))
             for mm in re.finditer(r"SLALOM s=([\d.]+) d_now=\S+ v=\S+ groups=\[\[\(([\d.]+),", text)]
    box = [x for x in plans if -2 < x[0]-c['s'] < 12]
    if box:
        sb = min(box, key=lambda x: abs(x[0]-c['s']))[0]
        why.append(f"box first planned {max(d for b, d in plans if abs(b-sb) < 2.):.1f} m ahead")
    if any('BLOCKED' in l for l in lines):why.append('slalom BLOCKED (no offset passes) just before')
    if any('HIT' in l for l in lines):why.append('the curve being driven was found hitting a box (SLALOM HIT)')
    out.update(why=why or ['not classified'], slalom=[l[:200] for l in lines[-4:]])
    return out


def records(run_dir, index=None):
    out = []
    res = [json.loads(l) for l in open(os.path.join(run_dir, 'results.jsonl'))] if os.path.exists(os.path.join(run_dir, 'results.jsonl')) else []
    for log in sorted(glob.glob(os.path.join(run_dir, 'launch_*.log'))):
        k = int(re.search(r'launch_(\d+)\.log', log).group(1))
        if index is not None and k != index:continue
        m = re.search(r'(run_[0-9_]+)_0\.db3', open(log, errors='ignore').read())
        if not m:continue
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if not os.path.exists(bag):continue
        L = [d for d in read_highway(bag) if 'global_s' in d and d.get('ego_xy')]
        hits = [c for c in (res[k].get('collisions', []) if k < len(res) else []) if c.get('dist') is not None or c.get('unlisted')]
        t0 = L[0]['stamp'] if L else 0.
        last_decision = -math.inf
        i = 0
        while i < len(L):
            d = L[i]
            if d.get('reason') != 'CHANGE_STARTED':
                if (d.get('state') == 'WAIT_GAP' and d.get('boxes') and d['stamp']-last_decision >= 1.):
                    last_decision = d['stamp'];s = d['global_s'];box = d['boxes'];plan = d.get('plan') or {}
                    r = dict(kind='decision', run_dir=os.path.basename(run_dir.rstrip('/')), run=k, s=round(s, 1),
                             t_run=round(d['stamp']-t0, 1), lanes=f"{d.get('current_lane')}->{box.get('lane')}",
                             v_kmh=round(d['ego_speed']*3.6, 1), phase=d.get('phase'), reason=d.get('reason'),
                             blocked_by=(d.get('blocked_by') or [])[:4],
                             plan=dict(mode=plan.get('mode'), goal_kmh=plan.get('goal_kmh'), start_t=plan.get('start_t')),
                             lead=None if not d.get('lead') else dict(gap_m=round(d['lead']['lo']-s-FRONT, 1), kmh=round(d['lead']['v']*3.6, 1)),
                             boxes=segments(box.get('boxes') or [], s),
                             bodies=[[round(b['lo']-s, 1), round(b['hi']-s, 1), round(b['v']*3.6, 1), b.get('kind')]
                                     for b in d.get('bodies') or [] if -60. < b['lo']-s < 100.])
                    r['view'] = view(r['boxes'], r['bodies'], r['lead'], r['v_kmh'])
                    out.append(r)
                i += 1;continue
            prev = L[i-1] if i else {}
            s = d['global_s']; box = d.get('boxes') or prev.get('boxes') or {}
            plan = d.get('plan') or {}
            lead = d.get('lead') or prev.get('lead')
            bodies = d.get('bodies') or prev.get('bodies') or []
            j = i+1; completed = False; clear = math.inf
            while j < len(L) and L[j]['stamp']-d['stamp'] < 15.:
                if L[j].get('reason') == 'CHANGE_STARTED':
                    completed = True;break      # the next change started while settling: this one is done
                clear = min(clear, clearance(L[j]))
                if L[j].get('reason') == 'CHANGE_COMPLETE':
                    completed = True
                    for q in L[j+1:]:
                        if q['stamp']-L[j]['stamp'] > 2.:break
                        clear = min(clear, clearance(q))
                    break
                j += 1
            end_s = L[min(j, len(L)-1)]['global_s']
            hit = [dict(id=c['id'], s=round(c['s'], 1)) for c in hits if s-5. <= c['s'] <= end_s+30.]
            out.append(dict(
                kind='change', run_dir=os.path.basename(run_dir.rstrip('/')), run=k, s=round(s, 1), t_run=round(d['stamp']-t0, 1), lanes=f"{d.get('current_lane')}->{box.get('lane')}",
                v_kmh=round(d['ego_speed']*3.6, 1), phase_before=prev.get('phase'), admission=d.get('admission'),
                plan=dict(mode=plan.get('mode'), goal_kmh=plan.get('goal_kmh'), rear=plan.get('rear'), front=plan.get('front'),
                          start_t=plan.get('start_t'), end_t=plan.get('end_t'),
                          slot=[None if v is None else round(v-s, 1) for v in (plan.get('slot') or [None, None])]),
                lead=None if not lead else dict(gap_m=round(lead['lo']-s-FRONT, 1), kmh=round(lead['v']*3.6, 1)),
                boxes=segments(box.get('boxes') or [], s),
                bodies=[[round(b['lo']-s, 1), round(b['hi']-s, 1), round(b['v']*3.6, 1), b.get('kind')] for b in bodies
                        if -60. < b['lo']-s < 100.],
                view=None,
                completed=completed, min_clearance_m=None if not math.isfinite(clear) else round(clear, 2), collisions=hit,
                outcome='CONTACT' if hit or clear <= 0. else 'OK' if completed else 'INCOMPLETE'))
            r = out[-1];r['view'] = view(r['boxes'], r['bodies'], r['lead'], r['v_kmh'])
            i = j if j < len(L) and L[j].get('reason') == 'CHANGE_STARTED' else j+1
        done_s = []
        latched = {x['id'] for x in (res[k].get('collision_slots') or []) if x.get('t', 99) < 3.} if k < len(res) else set()
        for c in (res[k].get('collisions', []) if k < len(res) else []):
            if c.get('id') in latched:continue      # a flag already on at the start (loop_runner latch)
            # loop_runner before its latch fix: far packets and flags already on at the start are not contacts
            if (c.get('dist') is not None and c['dist'] > 5.) or (c.get('t') is not None and c['t'] < 3.):continue
            if any(abs(c['s']-x) < 3. for x in done_s):continue
            done_s.append(c['s']);out.append(contact(L, k, c, run_dir, t0))
    return out


def summary():
    """Per lane change: every record in box coordinates, tightest first (what worked and what did not)."""
    rows = [json.loads(l) for l in open(OUT)] if OUT.exists() else []
    ch = [r for r in rows if r.get('kind', 'change') == 'change'];lab = [r for r in rows if r.get('user_label')]
    print(f'{len(ch)} changes, {len(rows)-len(ch)} waiting decisions recorded, {len(lab)} labelled by the user'
          '  (box coordinates: our body is the origin; L = the target lane on our left)')
    for r in lab:print(f"  label {r['user_label']:10s} {r.get('kind')} {r['lanes']} s{r['s']} {r['run_dir']}: {r.get('user_note') or ''}")
    def fmt(r):
        v = r['view'];fc, rc = v['left_front_car'], v['left_rear_car']
        return (f"{r['outcome']:10s} v{r['v_kmh']:5.1f} | L beside {'/'.join(v['left_beside']):16s} L front free {v['left_front_free_m']:5.1f}"
                + (f" (car {fc['gap_m']:5.1f} m {fc['kmh']:4.0f})" if fc else ' (no car)        ')
                + f" | L rear free {v['left_rear_free_m']:5.1f}" + (f" (car {rc['gap_m']:5.1f} m, closing {rc['closing_kmh']:+5.1f})" if rc else ' (no car)               ')
                + f" | my front {'-' if v['my_front_gap_m'] is None else v['my_front_gap_m']} m {'' if v['my_front_kmh'] is None else v['my_front_kmh']}"
                + f" | {r['plan']['mode']} clr {r['min_clearance_m']}  s{r['s']} {r['run_dir']}")
    for r in [x for x in rows if x.get('kind') == 'contact']:
        print(f"  CONTACT s{r['s']} {r['run_dir']}: {r.get('part', '-')} with {r.get('other_type')}#{r.get('other_id')} "
              f"(v {r.get('v_kmh')} vs {(r.get('other') or {}).get('kmh')} km/h, phase {r.get('phase')}) -- {'; '.join(r.get('why') or [r.get('where', '')])}")
    for lanes in sorted({r['lanes'] for r in rows if 'lanes' in r}):
        sub = [r for r in rows if r.get('lanes') == lanes and r.get('view') and r.get('kind', 'change') == 'change']
        print(f"\n{lanes}: " + ', '.join(f"{k} {sum(r['outcome'] == k for r in sub)}" for k in sorted({r['outcome'] for r in sub})))
        tight = lambda r: min(r['view']['left_front_free_m'], r['view']['left_rear_free_m'], r['view']['my_front_gap_m'] if r['view']['my_front_gap_m'] is not None else 99)
        for r in sorted(sub, key=lambda r: (r['outcome'] == 'OK', tight(r)))[:8]:print('  '+fmt(r))


def label(run, s, value, note=''):
    rows = [json.loads(l) for l in open(OUT)]
    if run == 'last':run = max(r['run_dir'] for r in rows)
    cand = [r for r in rows if r['run_dir'] == run and abs(r['s']-s) <= 8.]
    if not cand:raise SystemExit(f'no record within 8 m of s{s} in {run}')
    r = min(cand, key=lambda r: (abs(r['s']-s), r.get('kind') != 'change'))
    r['user_label'] = value;r['user_note'] = note
    with open(OUT, 'w') as f:
        for x in rows:f.write(json.dumps(x, ensure_ascii=False)+'\n')
    print(f"labelled {value}: {r.get('kind')} {r['lanes']} s{r['s']} v{r['v_kmh']} {run}")


def precedents(v, lanes=None, label='possible', k=3):
    """Labelled records whose box view is most like `v` (a view() dict): distances on the free runs (capped at 30 m),
    the cars ending them (gap, speed) and our own front gap."""
    rows = [json.loads(l) for l in open(OUT)] if OUT.exists() else []
    rows = [r for r in rows if r.get('user_label') == label and r.get('view') and (lanes is None or r['lanes'] == lanes)]
    def key(w):
        c = lambda x: 30. if x is None else min(30., x)
        fc, rc = w['left_front_car'] or {}, w['left_rear_car'] or {}
        return (c(w['left_front_free_m']), c(w['left_rear_free_m']), c(fc.get('gap_m')), c(rc.get('gap_m')),
                (rc.get('closing_kmh') or 0.)/3.6, c(w['my_front_gap_m']))
    a = key(v)
    return sorted(rows, key=lambda r: sum((x-y)**2 for x, y in zip(a, key(r['view']))))[:k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*'); ap.add_argument('--run', type=int); ap.add_argument('--summary', action='store_true')
    ap.add_argument('--rebuild', action='store_true', help='start the record file over from these runs')
    ap.add_argument('--label', nargs=3, metavar=('RUN_DIR|last', 'S', 'possible|impossible'))
    ap.add_argument('--note', default='')
    a = ap.parse_args()
    if a.label:
        label(a.label[0], float(a.label[1]), a.label[2], a.note);return
    if a.summary:
        summary();return
    seen = set()
    if a.rebuild and OUT.exists():OUT.unlink()
    if OUT.exists():
        seen = {(r.get('kind', 'change'), r['run_dir'], r['run'], r['s']) for r in map(json.loads, open(OUT))}
    new = [r for d in a.runs for r in records(d, a.run) if (r['kind'], r['run_dir'], r['run'], r['s']) not in seen]
    with open(OUT, 'a') as f:
        for r in new:f.write(json.dumps(r, ensure_ascii=False)+'\n')
    ch = [r for r in new if r['kind'] == 'change'];ct = [r for r in new if r['kind'] == 'contact']
    print(f'pass_log: {len(ch)} changes, {len(new)-len(ch)-len(ct)} decisions, {len(ct)} contacts added ' + ' '.join(f"{r['lanes']}:{r['outcome']}" for r in ch))


if __name__ == '__main__':
    main()
