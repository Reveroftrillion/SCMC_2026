#!/usr/bin/env python3
"""Build a directed MGeo road connector from the current spawn to the official route.

Only real from/to-node connections are used; no off-road shortcut or lane-change edge.
The official route is rotated at the merge point, preserving one complete lap.
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def clean(points):
    p = np.asarray(points, dtype=float)
    return p[np.r_[True, np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1) > 1e-6]]


def heading(vector):
    return math.atan2(vector[1], vector[0])


def difference(a, b):
    return abs((a-b+math.pi) % (2*math.pi)-math.pi)


def build(x, y, yaw_deg):
    records = json.loads((ROOT/'Readme/K-CITY맵데이터/link_set.json').read_text())
    links = {}
    adjacent = {}
    for record in records:
        p = clean(record['points'])
        if record['lazy_init'] or len(p) < 2:
            continue
        record = dict(record, points=p, length=float(np.linalg.norm(np.diff(p[:, :2],axis=0),axis=1).sum()),
                      entry=heading(p[1]-p[0]), exit=heading(p[-1]-p[-2]))
        links[record['idx']] = record
        adjacent.setdefault(record['from_node_idx'], []).append(record['idx'])
    candidates = []
    for key, link in links.items():
        p = link['points'][:, :2]
        delta = np.diff(p, axis=0)
        t = np.clip(np.sum((np.array([x,y])-p[:-1])*delta,axis=1)/np.sum(delta*delta,axis=1),0,1)
        distance = np.linalg.norm(p[:-1]+t[:,None]*delta-[x,y],axis=1)
        i = int(distance.argmin())
        if distance[i] <= 1.5 and difference(heading(delta[i]),math.radians(yaw_deg)) < math.radians(30):
            s = np.linalg.norm(delta[:i],axis=1).sum()+t[i]*np.linalg.norm(delta[i])
            candidates.append((float(distance[i]),key,float(s)))
    if not candidates:
        raise ValueError('Spawn is not on a forward-facing mapped lane within 1.5m')
    cross_track, start_id, start_s = min(candidates)
    official = clean(np.loadtxt(ROOT/'ros2_ws/src/foscar_bringup/routes/global_path.txt'))
    # The final point repeats the start. Use cyclic indices for the merge tangent.
    cycle = official[:-1]
    queue = [(links[start_id]['length']-start_s,start_id,(start_id,))]
    seen = set()
    while queue:
        distance, last_id, sequence = heapq.heappop(queue)
        if last_id in seen:
            continue
        seen.add(last_id)
        last = links[last_id]
        distances = np.linalg.norm(cycle[:,:2]-last['points'][-1,:2],axis=1)
        merge = int(distances.argmin())
        tangent = heading(cycle[(merge+1)%len(cycle)]-cycle[merge])
        if distances[merge] < 0.02 and difference(last['exit'],tangent) < math.radians(30):
            break
        for next_id in adjacent.get(last['to_node_idx'],[]):
            next_link = links[next_id]
            if np.linalg.norm(last['points'][-1,:2]-next_link['points'][0,:2]) > 0.02:
                continue
            if difference(last['exit'],next_link['entry']) > math.radians(45):
                continue
            heapq.heappush(queue,(distance+next_link['length'],next_id,sequence+(next_id,)))
    else:
        raise ValueError('No directed road connection to the official route')
    connector = clean(np.vstack([links[key]['points'] for key in sequence]))
    lap = np.vstack((cycle[merge:],cycle[:merge+1]))
    result = clean(np.vstack((connector,lap)))
    gaps = np.linalg.norm(np.diff(result[:,:2],axis=0),axis=1)
    if gaps.max() > 0.51:
        raise ValueError(f'Unexpected route gap: {gaps.max()}')
    metadata = dict(spawn=[x,y,yaw_deg], start_link=start_id, spawn_link_s_m=start_s,
                    spawn_cross_track_m=cross_track, connector_remaining_m=distance,
                    connector_links=list(sequence), official_merge_index=int(merge),
                    official_merge_xyz=cycle[merge].tolist(), points=len(result),
                    total_length_m=float(gaps.sum()), max_gap_m=float(gaps.max()),
                    note='Road geometry only; traffic lights and obstacles along connector are not handled.')
    destination = ROOT/'ros2_ws/src/foscar_bringup/routes'
    np.savetxt(destination/'spawn_path.txt',result,fmt='%.12f')
    (destination/'spawn_path.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x',type=float,default=-12.692728042602539)
    parser.add_argument('--y',type=float,default=-221.46871948242188)
    parser.add_argument('--yaw-deg',type=float,default=62.351287841796875)
    args = parser.parse_args()
    build(args.x,args.y,args.yaw_deg)
