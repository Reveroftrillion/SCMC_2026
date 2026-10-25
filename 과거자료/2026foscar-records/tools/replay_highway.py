#!/usr/bin/env python3
"""Deterministic policy replay. No ROS publisher and no control transmitter."""
import argparse
import json
from pathlib import Path
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_route import HighwayRoute
from foscar_planning.highway_runtime import HighwayRuntime
from foscar_planning.state import Context,Target


def run_replay(rows,route,config):
    run=HighwayRuntime(config,route);out=[]
    for row in rows:
        e=row['ego'];stamp=float(row['stamp'])
        inside=route.start_s<=e['global_s']<route.end_s
        ctx=Context(s=e['global_s'],x=e['x'],y=e['y'],yaw=e['yaw'],v=e['v'],now=stamp,mission_name=config.mission if inside else '')
        run.accept(row['footprints'],stamp)
        run.apply(Target(),ctx)
        out.append(run.diagnostic or dict(stamp=stamp,reason='OUTSIDE_HIGHWAY',mode=config.mode))
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for field in ('input','config','link-file','route-file','checkpoint-file','output'):ap.add_argument('--'+field,required=True)
    ap.add_argument('--control-config',help='Use the same follower profile YAML as the live launch.')
    a=ap.parse_args();cfg=HighwayConfig.load(a.config,control_config=a.control_config)
    route=HighwayRoute.from_files(a.link_file,a.route_file,a.checkpoint_file,**cfg.route)
    rows=[json.loads(line) for line in Path(a.input).read_text().splitlines() if line.strip()]
    result=run_replay(rows,route,cfg)
    Path(a.output).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in result))
    print(f'{len(result)} decisions; no control transmitted')

if __name__=='__main__':main()
