"""Resolve checkpoint-defined mission boundaries once from supplied geometry."""
from pathlib import Path
import math
import re
import numpy as np
import yaml
from .ctrl_core import Route


def read_checkpoints(filename):
    text=Path(filename).read_text()
    radius=re.search(r'반경\s*(\d+(?:\.\d+)?)\s*m',text)
    if radius is None or float(radius[1])<=0:raise ValueError('checkpoint radius missing')
    rows={}
    for name,body in re.findall(r'\[([^\]]+)\]([^\[]+)',text):
        if not name.isdigit():continue
        values=re.findall(r'-?\d+(?:\.\d+)?',body)
        if len(values)<3 or int(name) in rows:raise ValueError('invalid checkpoint position/id')
        rows[int(name)]={'xy':np.array([float(x) for x in values[:2]]),'radius':float(radius[1])}
    return rows


def geometry_path(value,base):
    if not isinstance(value,str) or not value:raise ValueError('geometry path required')
    if value.startswith('package://'):
        from ctrl_ros1.ctrl_packages import get_package_share_directory
        package,relative=value[len('package://'):].split('/',1)
        return Path(get_package_share_directory(package))/relative
    path=Path(value).expanduser()
    return path if path.is_absolute() else base/path


def load_mission_config(filename):
    path=Path(filename)
    data=yaml.safe_load(path.read_text())
    if not isinstance(data,dict):raise ValueError('invalid mission configuration')
    geometry=data.pop('geometry',None)
    route=checkpoints=None
    if geometry is not None:
        if not isinstance(geometry,dict) or set(geometry)!={'route','checkpoints'}:raise ValueError('invalid mission geometry')
        route=Route(np.loadtxt(geometry_path(geometry['route'],path.parent)))
        checkpoints=read_checkpoints(geometry_path(geometry['checkpoints'],path.parent))
    for row in data.get('zones',[]):
        for edge in ('start','end'):
            key=edge+'_checkpoint'
            if key not in row:continue
            if edge+'_s' in row or route is None:raise ValueError('ambiguous/missing checkpoint geometry')
            number=row.pop(key)
            if isinstance(number,bool) or not isinstance(number,int) or number not in checkpoints:raise ValueError('unknown checkpoint')
            cp=checkpoints[number];s,error,_=route.project(*cp['xy'])
            if not math.isfinite(s) or error>cp['radius']:raise ValueError('checkpoint outside reference route')
            row[edge+'_s']=float(s)
        if 'approach_s' in row:
            # The mission may take over before its first checkpoint (user 2026-09-28: the highway FSM from
            # the straight after the post-roundabout curve); the speed-limit exemption keeps the checkpoint.
            a=row.pop('approach_s')
            if isinstance(a,bool) or not isinstance(a,(int,float)) or not math.isfinite(a) or 'start_s' not in row or a>=row['start_s']:
                raise ValueError('approach_s must lie before the zone start')
            row['limit_start_s']=row['start_s'];row['start_s']=float(a)
    return data
