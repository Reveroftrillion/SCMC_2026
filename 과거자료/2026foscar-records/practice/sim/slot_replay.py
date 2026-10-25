#!/usr/bin/env python3
"""PRACTICE ONLY: open-loop replay of recorded highway ticks through the slot planner (exp/box-planner).

Each recorded /behavior/highway tick carries the ego pose and speed and every observed body (hull, measured
velocity). For every tick the recorded car spent deciding (WAIT_GAP / ROUTE_FALLBACK, not yet changed), ask the
current HighwayPolicy: would it start the change now, and with what plan? Then check each "start now" against
what the same bodies actually did afterwards: along the plan's own speed profile, how close did the body we land
ahead of / behind actually come (by track id, while it stays visible)?

    python3 practice/sim/slot_replay.py practice/sim/runs/<stamp> [...] [--out file.json]
"""
import argparse, glob, json, math, os, re, sys, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'practice/sim'))
from cut_in_log import read_highway                                    # noqa: E402
from foscar_planning.highway_config import HighwayConfig                 # noqa: E402
from foscar_planning.highway_route import HighwayRoute                   # noqa: E402
from foscar_planning.highway_policy import HighwayPolicy                 # noqa: E402
from foscar_planning.highway_observation import ObservationSnapshot      # noqa: E402
from foscar_planning.highway_slot_planner import profile                 # noqa: E402
from foscar_planning.state import Context                                # noqa: E402

BRINGUP = ROOT/'src/foscar_bringup'


def load():
    cfg = HighwayConfig.load(ROOT/'src/foscar_planning/config/highway.yaml', control_config=BRINGUP/'config/low_speed.yaml')
    r = cfg.route
    road = HighwayRoute.from_files(BRINGUP/'maps/link_set.json', BRINGUP/'routes/global_path.txt', BRINGUP/'routes/checkpoints.txt',
                                   start_checkpoint=r['start_checkpoint'], hold_checkpoint=r['hold_checkpoint'],
                                   rejoin_checkpoint=r['rejoin_checkpoint'], end_checkpoint=r['end_checkpoint'], excluded_lanes=r['excluded_lanes'])
    return cfg, road


def snapshot(d):
    objs = {}
    for o in d.get('objects', []):
        vx, vy = o.get('velocity') or (0., 0.)
        objs[o['id']] = SimpleNamespace(id=o['id'], xy=np.asarray(o['xy'], dtype=float), speed_ready=bool(o['speed_ready']),
                                        velocity_bounds=(vx, vx, vy, vy), sample_count=5)
    return ObservationSnapshot(d['stamp']-max(0., d.get('scan_age_s') or 0.), True, 'OK', objs)


def check(road, cfg, L, k, plan, bodies):
    """Actual closest approach of the plan's rear / front body along the plan's profile (by id, while visible)."""
    d = L[k]; car = cfg.vehicle; back = car.length_m-car.rear_axle_to_front_m
    b = cfg.boxes
    ids = {x['id']: x for x in bodies if x['kind'] in ('measured', 'unmeasured')}
    rear = plan.rear if plan.rear in ids else None
    front = plan.front if plan.front in ids else None
    out = dict(rear=rear, front=front, rear_min=None, front_min=None)
    t0 = d['stamp']; ts = np.arange(0., 15., .1)
    x, _, _ = profile(d['ego_speed'], plan.goal, b.accel_mps2, cfg.stopping_decel_limit, ts)
    for e in L[k+1:]:
        dt = e['stamp']-t0
        if dt > plan.end_t+b.tail_s:break
        if dt < plan.cross_t*.6:continue
        se = d['global_s']+float(np.interp(dt, ts, x))
        for o in e.get('objects', []):
            if o['id'] not in (rear, front):continue
            st = road.project_many(np.asarray(o['xy'], dtype=float))
            if o['id'] == rear:
                g = se-back-float(st.max())
                out['rear_min'] = g if out['rear_min'] is None else min(out['rear_min'], g)
            else:
                g = float(st.min())-(se+car.rear_axle_to_front_m)
                out['front_min'] = g if out['front_min'] is None else min(out['front_min'], g)
    return out


def replay(run_dir, cfg, road):
    rows = []
    for log in sorted(glob.glob(os.path.join(run_dir, 'launch_*.log'))):
        m = re.search(r'(run_[0-9_]+)_0\.db3', open(log, errors='ignore').read())
        if not m:continue
        bag = os.path.expanduser('~/foscar_logs/bags/'+m.group(1))
        if not os.path.exists(bag):continue
        L = [d for d in read_highway(bag) if 'global_s' in d and d.get('ego_xy')]
        p = HighwayPolicy(road, cfg); lane = None
        for k, d in enumerate(L):
            if d.get('state') not in ('WAIT_GAP', 'ROUTE_FALLBACK') or d.get('current_lane') in (None, road.lanes[-1]):continue
            if d['current_lane'] != lane:p = HighwayPolicy(road, cfg);lane = d['current_lane']
            p.current_lane = lane;p.cp11 = bool(d.get('cp11_passed'));p.committed = None;p.previous = None
            ctx = Context(s=d['global_s'], x=d['ego_xy'][0], y=d['ego_xy'][1], yaw=d['ego_yaw'], v=d['ego_speed'], now=d['stamp'], mission_name='HIGHWAY')
            c0 = time.perf_counter()
            try:out = p.tick(ctx, snapshot(d))
            except Exception as ex:                                  # report, keep going
                rows.append(dict(run=os.path.basename(log), s=round(d['global_s'], 1), error=repr(ex)));continue
            ms = (time.perf_counter()-c0)*1e3
            row = dict(run_dir=os.path.basename(run_dir.rstrip('/')), run=os.path.basename(log)[7:-4], s=round(d['global_s'], 1),
                       lane=lane, kmh=round(d['ego_speed']*3.6), old=d.get('reason'), old_blocked=d.get('blocked_by') or [],
                       new=out.reason, plan=out.diagnostics.get('plan'), ms=round(ms, 2))
            if out.reason == 'CHANGE_STARTED':
                row['check'] = check(road, cfg, L, k, p.plan, out.diagnostics.get('bodies') or [])
            rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('runs', nargs='+'); ap.add_argument('--out')
    ap.add_argument('--set', action='append', default=[], help='boxes key=value override, e.g. rear_accel_mps2=1.0')
    a = ap.parse_args()
    cfg, road = load()
    if a.set:
        from dataclasses import replace
        cfg = replace(cfg, boxes=SimpleNamespace(**{**vars(cfg.boxes), **{k: float(v) for k, v in (x.split('=') for x in a.set)}}))
    rows = []
    for r in a.runs:rows += replay(r, cfg, road)
    ok = [r for r in rows if 'error' not in r]
    errs = [r for r in rows if 'error' in r]
    print(f'{len(ok)} deciding ticks replayed, {len(errs)} errors', errs[:3])
    ms = np.array([r['ms'] for r in ok]) if ok else np.zeros(1)
    print(f'time per tick: median {np.median(ms):.1f} ms, p95 {np.percentile(ms, 95):.1f}, max {ms.max():.1f}')
    rear_blocked = [r for r in ok if any(str(x).startswith('BOX_REAR') for x in r['old_blocked'])]
    print(f'old BOX_REAR refusals: {len(rear_blocked)} ticks; new starts the change on {sum(r["new"]=="CHANGE_STARTED" for r in rear_blocked)}')
    kinds = {}
    for r in ok:kinds[r['new']] = kinds.get(r['new'], 0)+1
    print('new decisions:', dict(sorted(kinds.items(), key=lambda kv: -kv[1])))
    starts = [r for r in ok if r['new'] == 'CHANGE_STARTED']
    modes = {}
    for r in starts:modes[r['plan']['mode']] = modes.get(r['plan']['mode'], 0)+1
    print('new change starts by mode:', modes)
    close = [r for r in starts if (r['check']['rear_min'] is not None and r['check']['rear_min'] < 1.) or
             (r['check']['front_min'] is not None and r['check']['front_min'] < 0.5)]
    seen = [r for r in starts if r['check']['rear_min'] is not None or r['check']['front_min'] is not None]
    print(f'starts checked against what the bodies did: {len(seen)} (rear gap < 1 m or front gap < 0.5 m: {len(close)})')
    for r in close[:15]:print('   ', r['run_dir'], r['run'], r['s'], f"{r['lane']}->", r['kmh'], 'km/h', r['plan']['mode'], r['plan']['goal_kmh'], r['check'])
    if a.out:json.dump(rows, open(a.out, 'w'), ensure_ascii=False);print('->', a.out)


if __name__ == '__main__':
    main()
