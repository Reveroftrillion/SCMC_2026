import os,sys,glob,json,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
for b in sorted(glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')))[-int(sys.argv[1]):]:
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={x.name:x.type for x in r.get_all_topics_and_types()}; Ms=get_message(types['/localization/s'])
    r.set_filter(rosbag2_py.StorageFilter(topics=['/control/telemetry','/localization/s']))
    s=0;prev=False;on=[]
    while r.has_next():
        tp,raw,t=r.read_next()
        if tp=='/localization/s': s=deserialize_message(raw,Ms).data; continue
        try: d=json.loads(deserialize_message(raw,String).data)
        except Exception: continue
        if not 1849<s<1990: continue
        bk=(d.get('brake') or 0)>.05
        if bk and not prev: on.append((round(s,1),d.get('reason')))
        prev=bk
    print(os.path.basename(b),'brake onsets in tunnel',len(on),on[:12])
