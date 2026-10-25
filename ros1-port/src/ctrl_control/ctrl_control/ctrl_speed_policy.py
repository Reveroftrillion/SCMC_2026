"""Use the same configured highway checkpoints as the mission supervisor."""
import math
from pathlib import Path
from .ctrl_mission_config import load_mission_config


def uncapped_highway_zones(filename):
    if not filename:return []
    config=load_mission_config(filename)
    zones=[]
    for zone in config['zones']:
        if zone['name']!='HIGHWAY' or zone['speed_kmh'] is not None:continue
        start_s,end_s=float(zone.get('limit_start_s',zone['start_s'])),float(zone['end_s'])   # 제한 시작은 checkpoint 기준이며 approach_s를 사용하지 않는다
        if not all(math.isfinite(station) for station in (start_s,end_s)) or start_s>=end_s:
            raise ValueError('invalid highway speed checkpoints')
        zones.append((start_s,end_s))
    return zones
