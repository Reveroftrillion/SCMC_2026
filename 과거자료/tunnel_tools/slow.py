"""Tunnel slowdowns: where v_max target drops below cruise, with state, telemetry reason. python3 slow.py N"""
import os,sys,glob,json,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for b in sorted(glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')))[-int(sys.argv[1]):]:
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    T={x.name:get_message(x.type) for x in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/control/telemetry','/localization/s','/behavior/target','/behavior/state']))
    s=0;st='';tg=None;rows=[];last=None
    while r.has_next():
        tp,raw,t=r.read_next(); m=deserialize_message(raw,T[tp])
        if tp=='/localization/s': s=m.data; continue
        if tp=='/behavior/state': st=m.data[:60]; continue
        if tp=='/behavior/target': tg=m; continue
        if not 1849<s<1990 or tg is None: continue
        d=json.loads(m.data)
        key=(round(tg.v_max*3.6),round(tg.stop_s) if tg.stop_s<1e6 else None,d.get('reason'),st.split('|')[0])
        if key!=last: rows.append((round(s,1),round(d['speed_mps']*3.6),round(d.get('brake') or 0,2))+key); last=key
    print('==',os.path.basename(b))
    for x in rows: print('  ',x)
