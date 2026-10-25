"""Lane-change admission around a time window [T0, T1] (bag-relative s): state, reason,
blocked_by, rear gap, gap strategy, admission profile, cap; add any 4th arg for object dumps.

    python3 highway_admission.py ~/foscar_logs/bags/run_<stamp> 60.3 60.9
"""
import sys,json
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
t0=None;lo,hi=float(sys.argv[2]),float(sys.argv[3])
while r.has_next():
    topic,data,t=r.read_next()
    if t0 is None:t0=t
    et=(t-t0)*1e-9
    if topic!='/behavior/highway' or not lo<=et<=hi:continue
    d=json.loads(deserialize_message(data,get_message(types[topic])).data)
    o={x['id'][-3:]:x for x in d['objects'] or []}
    rear={k:(x.get('velocity') or x.get('speed_mps') or x.get('vx')) for k,x in o.items() if k in ('143','152')}
    print('%.2f %s %s lane=%s tgt=%s v=%.1f blk=%s rear=%s strat=%s prof=%s cap=%s spd=%s'%(et,d['state'],d['reason'],d['current_lane'],d['target_link'],d['ego_speed'],d.get('blocked_by'),d.get('rear_gap_m'),d.get('gap_strategy'),d.get('admission_profile'),d.get('speed_cap_mps'),rear))
    if len(sys.argv)>4:print(json.dumps(o.get('152'))[:500]);print(json.dumps(d.get('scene'))[:800])
