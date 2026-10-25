"""Speed profile s1790-1990 every 5 m: speed, target v_max, limit source, state (practice only). python3 vprof.py N"""
import os,sys,glob,json,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for b in sorted(glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')))[-int(sys.argv[1]):]:
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    T={x.name:get_message(x.type) for x in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/control/telemetry','/localization/s','/behavior/target']))
    s=0;tg=None;nxt=1790;out=[]
    while r.has_next():
        tp,raw,t=r.read_next(); m=deserialize_message(raw,T[tp])
        if tp=='/localization/s': s=m.data; continue
        if tp=='/behavior/target': tg=m; continue
        if s>=nxt and tg is not None:
            d=json.loads(m.data)
            out.append('%d:%.0f/%.0f%s'%(nxt,d['speed_mps']*3.6,min(tg.v_max*3.6,99),'B' if (d.get('brake') or 0)>.05 else ''))
            nxt+=5
            if nxt>1990: break
    print(os.path.basename(b)); print('  '+' '.join(out))
