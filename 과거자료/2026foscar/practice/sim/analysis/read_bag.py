"""Generic bag dump: TOPICS (comma list) between bag-relative T0 and T1 seconds.

    python3 read_bag.py ~/foscar_logs/bags/run_<stamp> /behavior/target,/events 40 60
"""
import sys,json
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
want=set(sys.argv[2].split(','))
t0=None
lo,hi=float(sys.argv[3]),float(sys.argv[4])
while r.has_next():
    topic,data,t=r.read_next()
    if t0 is None:t0=t
    et=(t-t0)*1e-9
    if topic not in want or not lo<=et<=hi:continue
    m=deserialize_message(data,get_message(types[topic]))
    if topic=='/behavior/target':
        print('%.2f TARGET state=%s v_max=%.2f stop_s=%.2f lat=%.2f entry_s=%.1f/%.1f ret=%.1f/%.1f ref=%s'%(et,m.state,m.v_max,m.stop_s,m.lat_offset,m.entry_s,m.entry_length_m,m.return_s,m.return_length_m,len(m.reference_x)))
    elif hasattr(m,'data'):print('%.2f %s %s'%(et,topic,m.data[:400]))
    elif topic=='/events':print('%.2f EV %s s=%.2f d=%.2f sx=%.2f sy=%.2f'%(et,m.type,m.s,getattr(m,'d',float('nan')),m.size_x,m.size_y))
    else:print('%.2f %s %s'%(et,topic,str(m)[:300]))
