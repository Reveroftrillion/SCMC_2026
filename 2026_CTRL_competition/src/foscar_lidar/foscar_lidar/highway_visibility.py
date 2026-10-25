"""Highway mission span and observation ROI for the obstacle node."""
import math
from pathlib import Path
import yaml
from foscar_control.mission_config import load_mission_config


class HighwaySensor:
    def __init__(self,start_s,end_s,front_m,rear_m,lateral_m):
        if not all(math.isfinite(v) for v in (start_s,end_s,front_m,rear_m,lateral_m)) or start_s>=end_s or min(front_m,rear_m,lateral_m)<=0:raise ValueError('invalid highway sensor bounds')
        self.start_s,self.end_s=start_s,end_s
        self.front_m,self.rear_m,self.lateral_m=front_m,rear_m,lateral_m

    @classmethod
    def from_files(cls,config_file,mission_file):
        if not config_file:return None
        data=yaml.safe_load(Path(config_file).read_text())
        if data['mode']=='off':return None
        mission=load_mission_config(mission_file)
        zone=next(z for z in mission['zones'] if z['name']==data['mission'])
        p=data['policy']
        # Observe the highway sensor_lead_in_m before its first checkpoint so lane-4 speeds are measured on
        # arrival: the entry decision waited ~0.3 s for them (TARGET_MOTION_PENDING, user 2026-09-28).
        # Only the lidar ROI moves; the mission, the policy and the speed-limit exemption still start at CP10.
        return cls(min(zone['start_s'],zone.get('limit_start_s',zone['start_s'])-p.get('sensor_lead_in_m',0.)),zone['end_s'],p['front_roi_m'],p['rear_roi_m'],p['lateral_roi_m'])

    def active(self,s):return s is not None and self.start_s<=s<self.end_s
