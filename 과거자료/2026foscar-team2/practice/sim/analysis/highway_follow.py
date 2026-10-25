"""Highway lead-following trace (last N /behavior/highway messages, every STEP-th):
gap, lead speed, closing speed, comfort/hard caps.

    python3 highway_follow.py ~/foscar_logs/bags/run_<stamp> keys      # list status keys
    python3 highway_follow.py ~/foscar_logs/bags/run_<stamp> 200 5     # last 200 msgs, every 5th
"""
import sys,json
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
rows=[];t0=None;v=0;vmax=None
while r.has_next():
    topic,data,t=r.read_next()
    if t0 is None:t0=t
    et=(t-t0)*1e-9
    if topic=='/vehicle/status':
        m=deserialize_message(data,get_message(types[topic]));v=getattr(m,'speed',getattr(m,'velocity',0))
    elif topic=='/behavior/target':
        vmax=deserialize_message(data,get_message(types[topic])).v_max
    elif topic=='/behavior/highway':
        d=json.loads(deserialize_message(data,get_message(types[topic])).data)
        rows.append((et,v,vmax,d))
if len(sys.argv)>2 and sys.argv[2]=='keys':print(rows[-1][3].keys());print({k:rows[-1][3][k] for k in rows[-1][3] if 'follow' in k or 'cap' in k or 'speed' in k});sys.exit()
for et,v,vmax,d in rows[-int(sys.argv[2])::int(sys.argv[3])]:
    f=(d.get("following_objects") or [{}])[0];print("%.2f ego=%.1f cap=%.1f %s st=%s gap=%s lead=%s close=%s src=%s comf=%s hard=%s"%(et,d["ego_speed"],d["speed_cap_mps"] or -1,d["speed_reason"],d["state"],*(f.get(k) if not isinstance(f.get(k),float) else round(f.get(k),1) for k in ("gap_m","lead_speed_mps","closing_mps","motion_source","comfort_cap_mps","hard_cap_mps"))))
