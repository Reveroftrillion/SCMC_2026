#!/usr/bin/env python3
"""PRACTICE ONLY: classify highway cut-ins from /behavior/highway dumps (hw_dump JSON: one list per run).

User 2026-09-28 (TuningPoint):
 1. precedents -- changes started with a car ahead in our lane and a target-lane car we had just passed
    beside us: what the judgement looked like when it went well.
 2. 5->4 entry -- did we pass and cut in ahead (PASS_CUT), go in behind (BEHIND_CUT), or ride the global
    path through the taper (TAPER)? For the yielding ones, could we have passed (at ~3 m/s2, the measured
    full-pedal acceleration at 60-80 km/h) before the lane-5 cut-in limit?

    python3 practice/sim/cut_in_cases.py <dump dir> [--out practice/sim/cut_in_cases.json]
"""
import argparse, glob, json, math, os
from pathlib import Path

FRONT, BACK, LENGTH = 3.8, 0.835, 4.635
ACCEL, VMAX = 3.0, 30.           # measured full pedal at 60-80 km/h; PASS_FIRST speed ceiling
PASS_GAP = 2.1                   # PASS-sequence rear gap in front of the passed car
CUT_LIMIT = 1285.77 - LENGTH     # fallback cut-in: lane-5 end minus a car length
CP10 = 1118.36


def rel(d, o):
    c, s = math.cos(d['ego_yaw']), math.sin(d['ego_yaw']); ex, ey = d['ego_xy']
    a = [(x-ex)*c+(y-ey)*s for x, y in o['xy']]
    return min(a), max(a)


def lane_cars(d, lane):
    objs = {o['id']: o for o in d.get('objects', [])}
    out = {}
    for key, info in ((d.get('scene') or {}).get('objects') or {}).items():
        if info.get('lane') == lane and key in objs:
            lo, hi = rel(d, objs[key])
            out[key] = dict(front_gap=lo-FRONT, rear_edge=hi+BACK, lo=lo, hi=hi, v=info.get('speed_kmh'))
    return out


def feasible_pass(v0, s0, car):
    """Could we get PASS_GAP ahead of this car before CUT_LIMIT at ACCEL?"""
    if car['v'] is None:return None
    v4 = car['v']/3.6
    need = car['hi']+BACK+PASS_GAP            # our rear axle must move this far relative to the car's front
    x = r = 0.; v = v0; dt = .05
    for _ in range(int(20/dt)):
        if r >= need:return dict(ok=s0+x+10. <= CUT_LIMIT, at_s=round(s0+x, 1), t=round(_*dt, 1), v_kmh=round(v*3.6))
        v = min(VMAX, v+ACCEL*dt); x += v*dt; r += (v-v4)*dt
    return dict(ok=False, at_s=None, t=None, v_kmh=round(v*3.6))


def entry(L):
    # The first tick that actually sees lane 4 (the first CP10 ticks carry no scan yet).
    first = next((d for d in L if d.get('current_lane') == 5 and d.get('global_s', 0) >= CP10 and d.get('scene')
                  and d['reason'] != 'SCAN_UNKNOWN' and lane_cars(d, 4)), None)
    if first is None:return None
    v0, s0 = first['ego_speed'], first['global_s']
    cars = lane_cars(first, 4)
    ahead = {k: c for k, c in cars.items() if c['front_gap'] > -LENGTH and c['lo'] < 80}
    for k, c in ahead.items():          # its speed is usually measured a few ticks later
        if c['v'] is None:
            c['v'] = next((lane_cars(d, 4).get(k, {}).get('v') for d in L if d['_t'] > first['_t']
                           and lane_cars(d, 4).get(k, {}).get('v') is not None), None)
    change = next((d for d in L if d['reason'] == 'CHANGE_STARTED' and d.get('current_lane') == 5), None)
    joined = next((d for d in L if d.get('current_lane') == 4), None)
    ref = change or joined
    if ref is None:return None
    now = lane_cars(ref, 4)
    # Track ids change mid-run: find the same car by where it should be now (its entry position + its speed x
    # elapsed time - the distance we drove); unseen and unmatched = beyond view ahead or passed behind.
    dt = ref['_t']-first['_t']; ds = ref['global_s']-s0
    def where(k):
        if k in now:return now[k]
        c = ahead[k]
        if c['v'] is None:return None
        pred = c['lo']+c['v']/3.6*dt-ds
        best = min(now.values(), key=lambda n: abs(n['lo']-pred), default=None)
        if best is not None and abs(best['lo']-pred) < 8.:return best
        return dict(lo=pred, hi=pred+LENGTH, predicted=True)
    passed = [k for k in ahead if where(k) is not None and where(k)['hi'] < -BACK]
    still = [k for k in ahead if where(k) is not None and where(k)['lo'] > FRONT - LENGTH]
    kind = ('PASS_CUT' if not still else 'BEHIND_CUT') if change else ('TAPER_AHEAD' if not still else 'TAPER_BEHIND')
    modes = []
    for d in L:
        if d.get('current_lane') == 5 and d['state'] in ('WAIT_GAP', 'ROUTE_FALLBACK'):
            m = d.get('gap_strategy') or d['reason']
            if not modes or modes[-1] != m:modes.append(m)
        if d is ref:break
    yield_cars = [dict(id=k[-4:], gap=round(ahead[k]['front_gap'], 1), v=ahead[k]['v'] and round(ahead[k]['v']),
                       could_pass=feasible_pass(v0, s0, ahead[k])) for k in still]
    return dict(kind=kind, entry_s=round(s0, 1), entry_kmh=round(v0*3.6), at_s=round(ref['global_s'], 1), at_kmh=round(ref['ego_speed']*3.6),
                ahead_at_entry=[dict(id=k[-4:], gap=round(c['front_gap'], 1), v=c['v'] and round(c['v'])) for k, c in sorted(ahead.items(), key=lambda kc: kc[1]['lo'])],
                passed=len(passed), still_ahead=yield_cars, modes=modes[:8])


def precedents(L):
    out = []
    for k, d in enumerate(L):
        if d['reason'] != 'CHANGE_STARTED':continue
        src, dst = d.get('current_lane'), d.get('target_lane')
        if dst is None:continue
        here = lane_cars(d, src); there = lane_cars(d, dst)
        lead = min((c for c in here.values() if c['front_gap'] > 0 and c['lo'] < 45), key=lambda c: c['lo'], default=None)
        if lead is None:continue
        beside_before = set()
        for e in L[max(0, k-100):k]:          # ~5 s
            for key, c in lane_cars(e, dst).items():
                if c['lo'] < FRONT and c['hi'] > -BACK:beside_before.add(key)
        passed = [key for key in beside_before if key in there and there[key]['hi'] < -BACK]
        if not passed:continue
        end = next((e for e in L[k+1:] if e['reason'] in ('CHANGE_COMPLETE',)), None)
        rear = ((d.get('boxes') or {}).get('rear')) or {}
        p = there[passed[0]]
        out.append(dict(s=round(d['global_s'], 1), lanes=f'{src}->{dst}', v_kmh=round(d['ego_speed']*3.6),
                        lead_gap=round(lead['front_gap'], 1), lead_kmh=lead['v'] and round(lead['v']),
                        passed_gap=round(-p['hi']-BACK, 1), passed_kmh=p['v'] and round(p['v']),
                        rear_mode=rear.get('mode'), rear_gap=rear.get('gap_m'), rear_closing=rear.get('closing_mps'), rear_need=rear.get('need_m'),
                        tail_in=d.get('tail_in'), completed=end is not None))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('dumps'); ap.add_argument('--out', default=None)
    a = ap.parse_args()
    entries, precs = [], []
    for f in sorted(glob.glob(os.path.join(a.dumps, '*.json'))):
        try:L = json.load(open(f))
        except Exception:continue
        if not L:continue
        e = entry(L)
        if e:e['run'] = os.path.basename(f)[:-5];entries.append(e)
        for p in precedents(L):p['run'] = os.path.basename(f)[:-5];precs.append(p)
    kinds = {}
    for e in entries:kinds[e['kind']] = kinds.get(e['kind'], 0)+1
    print('5->4 entries:', len(entries), kinds)
    for kind in ('PASS_CUT', 'TAPER_AHEAD', 'BEHIND_CUT', 'TAPER_BEHIND'):
        es = [e for e in entries if e['kind'] == kind]
        if not es:continue
        print(f'  {kind}: n={len(es)} entry {min(e["entry_kmh"] for e in es)}-{max(e["entry_kmh"] for e in es)} km/h')
    recent = [e for e in entries if e['run'] >= '20260928_1016']
    rk = {}
    for e in recent:rk[e['kind']] = rk.get(e['kind'], 0)+1
    print('  latest logic (batches from 10:16):', len(recent), rk)
    yielded = [e for e in entries if e['kind'] in ('BEHIND_CUT', 'TAPER_BEHIND')]
    could = [e for e in yielded if any(c['could_pass'] and c['could_pass']['ok'] for c in e['still_ahead'])]
    print(f'  yielded but could have passed at {ACCEL} m/s2: {len(could)} of {len(yielded)}')
    for e in could[:12]:
        print('   ', e['run'], e['kind'], 'entry', e['entry_s'], e['entry_kmh'], 'km/h ahead', e['ahead_at_entry'][:3], 'modes', e['modes'], '| stayed behind', e['still_ahead'])
    print('precedents (lead ahead + passed a target-lane car, change started):', len(precs), 'completed', sum(p['completed'] for p in precs))
    for p in precs[:12]:print('   ', p)
    if a.out:
        json.dump(dict(entries=entries, precedents=precs), open(a.out, 'w'), ensure_ascii=False, indent=1)
        print('->', a.out)


if __name__ == '__main__':
    main()
