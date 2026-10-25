import json,math,cProfile,pstats,faulthandler,time
from pathlib import Path
import numpy as np,yaml
from foscar_control.core import Route
from foscar_planning.lane_prediction import LaneMap
from foscar_planning.state import Context
from foscar_planning.states.avoid import Avoid
j=json.load(open('practice/sim/obstacle_paths/stall_snapshot.json'));r=Route(np.loadtxt('src/foscar_bringup/routes/global_path.txt')[:,:2]);road=LaneMap.from_file('src/foscar_bringup/maps/link_set.json');p=next(iter(yaml.safe_load(Path('src/foscar_planning/config/behavior.yaml').read_text()).values()))['ros__parameters'];a=Avoid(**{k[6:]:v for k,v in p.items() if k.startswith('avoid.')});c=Context(s=j['s'],x=j['x'],y=j['y'],yaw=2*math.atan2(j['q'][2],j['q'][3]),v=j['v'],now=j['now'],route=r,road_map=road,mission_name='STATIC_OBSTACLE',occupancy_stamp=j['scan']['stamp'],occupancy_hulls={o['id']:o['xy'] for o in j['scan']['objects']},occupancy_sensor=(j['x'],j['y']))
faulthandler.dump_traceback_later(5,repeat=True);pr=cProfile.Profile();pr.enable();print('entry',a.check_entry(c),flush=True);print('boxes',a.boxes(),flush=True);t=a.compute_target(c);pr.disable();faulthandler.cancel_dump_traceback_later();print(t);pstats.Stats(pr).sort_stats('cumulative').print_stats(18)
