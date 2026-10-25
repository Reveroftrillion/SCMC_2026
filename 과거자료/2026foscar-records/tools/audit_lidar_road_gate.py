#!/usr/bin/env python3
"""Recheck CLOSED recorded events against current road/path gates.

This is a geometry audit of already classified events, not raw-cloud replay.
Receipt-time nearest odometry can differ slightly from the original detector's
sensor-time interpolation. Never run this against an active SQLite recorder.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from analyze_pedestrian import messages, pose, rot
from foscar_lidar.obstacle_node import ObstacleNode, Track


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('output')
    args = parser.parse_args()
    records = list(messages(args.bag, {'/events','/localization/odometry','/localization/s','/map/route'}))
    odom = [(t,m) for t,n,m in records if n == '/localization/odometry']
    if not odom:
        parser.error('recording needs odometry')
    rclpy.init()
    node = ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),
                        Parameter('publish_markers',value=False)])
    tracks = {}; counts = Counter(); by_id = defaultdict(Counter)
    try:
        for t,name,msg in records:
            if name == '/map/route':
                node.on_route(msg)
            elif name == '/events':
                if msg.type not in ('OBSTACLE','PEDESTRIAN','VEHICLE'):
                    continue
                ot,o = min(odom,key=lambda row:abs(row[0]-t))
                if abs(ot-t)>.15 or node.route_xy is None:
                    counts['skipped_unsynchronized']+=1
                    continue
                xy,yaw = pose(o)
                point = rot(yaw) @ [msg.x,msg.y]+xy
                tr = tracks.setdefault(msg.value,Track(len(tracks),node.history_len))
                tr.history.append((t,*point))
                tr.x,tr.y,tr.size_x,tr.size_y = msg.x,msg.y,msg.size_x,msg.size_y
                tr.vx,tr.vy,tr.kind = msg.vx,msg.vy,msg.type
                node.ego = (*xy,yaw)
                idx = node.nearest_index(*xy)
                relevant,observed = node.output_gates(tr,idx,0.)
                label = 'driving' if relevant else 'observation_only' if observed else 'filtered'
                counts[label]+=1; by_id[msg.value][label]+=1
                if relevant:tr.published=True
    finally:
        node.destroy_node();rclpy.shutdown()
    result=dict(source=str(Path(args.bag).resolve()),scope='geometry audit; recorded classification retained',
                counts=dict(counts),tracks={k:dict(v) for k,v in by_id.items()})
    Path(args.output).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
