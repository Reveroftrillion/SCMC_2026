#!/usr/bin/env python3
"""Ordered raw-cloud -> production events -> YIELD probe, with no ROS transport.

Uses replay_lidar_sequence's extracted immutable input and ROS doubles. The
production event conversion and FSM are used, once per cloud on recorded poses.
No camera/signal policy, native executor timing or closed-loop vehicle is modeled.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import replay_lidar_sequence as sequence
from replay_lidar_ground import plain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--obstacle-source', type=Path,
                        help='preserved pre-change ctrl_obstacle_node.py for comparison')
    args = parser.parse_args()
    original_setup = sequence.setup_runtime
    cache, rows, classifications = {}, [], []
    observed = {}

    def setup(baseline):
        result = original_setup(baseline)
        from ctrl_lidar import ctrl_obstacle_node as detection
        if args.obstacle_source:
            name = 'ctrl_lidar.recorded_obstacle_baseline'
            module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader=None))
            exec(compile(args.obstacle_source.read_bytes(), str(args.obstacle_source), 'exec'), module.__dict__)
            module.perf_counter = lambda: detection.perf_counter()
            detection.ObstacleNode = module.ObstacleNode
        from ctrl_planning.ctrl_behavior_node import BehaviorNode
        from ctrl_planning.ctrl_state import Behavior
        from ctrl_planning.states.ctrl_yield import Yield
        from ctrl_control.ctrl_core import Route
        from ctrl_control.ctrl_mission_config import load_mission_config
        from ctrl_planning.ctrl_mission import MissionSupervisor
        import numpy as np
        import yaml

        port = sequence.PORT
        route = Route(np.asarray(json.loads((args.input/'metadata.json').read_text())['route']))
        config = yaml.safe_load((port/'src/ctrl_planning/config/behavior.yaml').read_text())['behavior_node']['ros__parameters']
        yielding = Yield(**{k[6:]: v for k, v in config.items() if k.startswith('yield.')})
        supervisor = MissionSupervisor(load_mission_config(port/'src/ctrl_planning/config/missions.yaml'))
        take_event = BehaviorNode.take_event
        original_cloud = detection.ObstacleNode.on_cloud
        original_classify = detection.ObstacleNode.classify
        original_init, original_tick = Behavior.__init__, Behavior.tick

        def classify(node, track):
            kind = original_classify(node, track)
            classifications.append(dict(t=detection.perf_counter(), id=track.id, kind=kind,
                history=list(track.history), ped=track.ped, confirmed=track.ped_confirmed))
            return kind

        def on_cloud(node, message):
            original_cloud(node, message)
            fp = json.loads(node.footprint_pub.values[-1].data)
            now = detection.perf_counter()
            proxy = SimpleNamespace(events=cache, evidence_after=-math.inf,
                footprint_stamp=fp['stamp'], footprints={o['id']: o['xy'] for o in fp['objects']},
                now=lambda: now, ego=lambda: node.ego,
                pose_at=node.ego_pose_at, route=route, s_offset=0.,
                ego_local_s=lambda x, y: route.project(x, y, node.ego_s)[0])
            for event in node.event_pub.values:
                take_event(proxy, event, path_relevant=True)
            for event in node.observation_pub.values:
                take_event(proxy, event, path_relevant=False)
            for key in list(cache):
                if now-cache[key].stamp > config['event_timeout_s']:
                    del cache[key]

        def init(behavior, states):
            original_init(behavior, list(states)+[yielding])

        def tick(behavior, ctx, **kwargs):
            ctx.events = [e for e in cache.values() if e.path_relevant]
            ctx.observations = [e for e in cache.values() if not e.path_relevant]
            supervisor.update(ctx)
            supervisor.apply_context(ctx)
            target = original_tick(behavior, ctx, **kwargs)
            supervisor.observe_pedestrian_yield(active=behavior.engaged['YIELD'],
                pedestrian=yielding.target_type == 'PEDESTRIAN',
                verified_clear=yielding.pedestrian_passage_confirmed)
            completed = supervisor.pedestrian_yield_completed
            if completed and 'first_completed' not in observed:
                observed['first_completed'] = dict(t=ctx.now, s=ctx.s, reason=yielding.reason)
            rows.append(dict(t=ctx.now, s=ctx.s, state=target.state, v_max=target.v_max,
                target_type=yielding.target_type, reason=yielding.reason,
                passage_confirmed=yielding.pedestrian_passage_confirmed,
                completed=completed, events=[dict(type=e.type, stamp=e.stamp, s=e.s, d=e.d,
                    id=e.value, relevant=e.path_relevant) for e in cache.values()]))
            return target

        detection.ObstacleNode.classify = classify
        detection.ObstacleNode.on_cloud = on_cloud
        Behavior.__init__, Behavior.tick = init, tick
        return result

    sequence.setup_runtime = setup
    sequence.replay(SimpleNamespace(input=args.input, output=args.output, baseline_head=False,
        behavior=True, focus_x=-62., focus_y=-10.4))
    source = args.obstacle_source or sequence.PORT/'src/ctrl_lidar/ctrl_lidar/ctrl_obstacle_node.py'
    summary = dict(frames=len(rows), states=dict(Counter(r['state'] for r in rows)),
        yield_target_types=dict(Counter(r['target_type'] for r in rows if r['state']=='YIELD')),
        obstacle_source=str(source), obstacle_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        input_manifest_sha256=hashlib.sha256((args.input/'manifest.json').read_bytes()).hexdigest(),
        **observed, limitations=['Recorded motion, tracking starts at window',
        'Production event conversion and YIELD, once per cloud; no native scheduling or signal overlay'])
    for name, value in [('pedestrian-ticks.json', rows), ('classifications.json', classifications),
                        ('pedestrian-summary.json', summary)]:
        with (args.output/name).open('x') as stream:
            json.dump(plain(value), stream, indent=2, allow_nan=False)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
