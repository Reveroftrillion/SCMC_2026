#!/usr/bin/env python3
"""PRACTICE ONLY: copy a saved MORAI scenario, moving only the ego start to each checkpoint.

Everything else (NPCs, pedestrians, obstacles, spawn points, lights) is kept. Scenario
coordinates share our map origin (mapInfo east/north offset 302595 / 4124145); yaw is the
ENU heading in degrees (checked: barricade -150.7 vs road -151.5). z is the nearest mapped
link point plus the source ego's height above its own link point. The ego starts in Park, stopped, on the route at the checkpoint station.

    python3 practice/scenarios/make_checkpoint_scenarios.py practice/CompetitionPractice3.json
    # one start at any station (highway-only runs skip the roundabout):
    python3 practice/scenarios/make_checkpoint_scenarios.py practice/CompetitionPractice3.json --at-s 980 --name Practice3_HW980
"""
import argparse
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/foscar_control'))
from foscar_control.core import Route                      # noqa: E402
from foscar_control.mission_config import read_checkpoints  # noqa: E402

LINKS = ROOT/'src/foscar_bringup/maps/link_set.json'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source')
    ap.add_argument('--prefix', default=None, help='file name prefix (default: source stem)')
    ap.add_argument('--back-m', type=float, default=0., help='start this far before each checkpoint')
    ap.add_argument('--out', default=str(Path(__file__).resolve().parent))
    ap.add_argument('--at-s', type=float, default=None, help='one scenario starting at this route station')
    ap.add_argument('--name', default=None, help='scenario name for --at-s')
    # Fixed NPCs 1 (20 km/h near the START/roundabout exit) and 35 (50 km/h toward the tollgate) crash with
    # the spawned traffic nearly every run and flood the collision channel (44 and 31 of 176 packets on
    # 2026-09-28). Practice scenarios leave them out (user 2026-09-28); '' keeps all.
    # 18 (50 km/h) crashed on a road crossing the highway near s1540 in consecutive batches (13:01, 13:13).
    ap.add_argument('--drop-vehicles', default='1,18,35', help='comma-separated vehicleList UNIQUEIDs to remove')
    a = ap.parse_args()
    src = json.loads(Path(a.source).read_text())
    if src['mapInfo']['eastOffset'] != 302595.0 or src['mapInfo']['northOffset'] != 4124145.0:
        sys.exit('scenario map origin differs from our map frame')
    route = Route(np.loadtxt(ROOT/'src/foscar_bringup/routes/global_path.txt'))
    pts = np.vstack([np.asarray(l['points'])[:, :3] for l in json.loads(LINKS.read_text())])
    tree = cKDTree(pts[:, :2])
    prefix = a.prefix or Path(a.source).stem
    # Vehicle origin height above the road, taken from the source ego (~0.34 m).
    ego = src['egoVehicle']['initPosition']['pos']
    lift = float(ego['z'])-float(pts[tree.query([float(ego['x']), float(ego['y'])])[1], 2])
    rows = []
    if a.at_s is not None:
        if not a.name:sys.exit('--at-s needs --name')
        starts = [(a.name, a.at_s)]
    else:
        starts = [(f'{prefix}_CP{number:02d}', route.project(*cp['xy'])[0]-a.back_m)
                  for number, cp in sorted(read_checkpoints(ROOT/'src/foscar_bringup/routes/checkpoints.txt').items())]
    for name, s in starts:
        s = max(0., s)
        x, y = route.at(s)
        heading = math.degrees(route.heading_at(s))
        z = float(pts[tree.query([x, y])[1], 2])+lift
        out = copy.deepcopy(src)
        drop = {int(v) for v in a.drop_vehicles.split(',') if v.strip()}
        out['vehicleList'] = [v for v in out['vehicleList'] if v['UNIQUEID'] not in drop]
        init = out['egoVehicle']['initPosition']
        init['pos'].update(x=float(x), y=float(y), z=z, _x=f'{x:.3f}', _y=f'{y:.3f}', _z=f'{z:.3f}')
        init['rot']['yaw'] = f'{heading:.3f}'
        init['rot']['roll'] = init['rot']['pitch'] = '0.000'
        init['gear'] = 1
        init['initLink'], init['initLinkRatio'] = '', 0
        out['egoVehicle']['currentVelocity'] = 0.
        init['vehicleSaveData']['velocity'] = 0.
        if len(name) > 30:
            sys.exit(f'{name}: longer than the 30-byte ScenarioLoad name field')
        Path(a.out, name+'.json').write_text(json.dumps(out, indent=4, ensure_ascii=False))
        rows.append((name, round(s, 1), round(x, 2), round(y, 2), round(z, 2), round(heading, 1)))
    for r in rows:
        print('%-26s s=%7.1f  x=%8.2f y=%8.2f z=%6.2f yaw=%7.1f' % r)


if __name__ == '__main__':
    main()
