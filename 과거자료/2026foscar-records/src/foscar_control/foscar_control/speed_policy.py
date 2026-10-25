"""Use the same configured highway checkpoints as the mission supervisor."""
import math
from pathlib import Path
from .mission_config import load_mission_config


def uncapped_highway_zones(filename):
    if not filename:return []
    config=load_mission_config(filename)
    zones=[]
    for zone in config['zones']:
        if zone['name']!='HIGHWAY' or zone['speed_kmh'] is not None:continue
        begin,end=float(zone.get('limit_start_s',zone['start_s'])),float(zone['end_s'])   # the checkpoint, not approach_s
        if not all(math.isfinite(v) for v in (begin,end)) or begin>=end:
            raise ValueError('invalid highway speed checkpoints')
        zones.append((begin,end))
    return zones
