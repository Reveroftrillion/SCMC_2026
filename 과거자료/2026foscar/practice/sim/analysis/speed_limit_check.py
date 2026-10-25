"""Rule check: max speed outside the exempt zone s 1118.7-1741.7 (links A2256W000411 start - A2256W000153 end = CP10-CP13) (60 km/h limit, 15 s penalty).

    python3 speed_limit_check.py ~/foscar_logs/bags/run_<stamp>
"""
import sys
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
s=None;best=(0,0);over=[]
while r.has_next():
    topic,data,t=r.read_next()
    if topic=='/localization/s':s=deserialize_message(data,get_message(types[topic])).data
    elif topic=='/vehicle/status' and s is not None:
        m=deserialize_message(data,get_message(types[topic]))
        v=[getattr(m,k) for k in ('speed_mps','speed','velocity','vel_x') if hasattr(m,k)]
        if not v:print([f for f in dir(m) if not f.startswith('_')][:30]);break
        kmh=v[0]*3.6 if v[0]<60 else v[0]
        if not 1118.7<=s<1741.7:
            if kmh>best[0]:best=(kmh,s)
            if kmh>60:over.append((round(s),round(kmh,1)))
print('max outside zone %.1f km/h at s=%.0f'%best, 'over60 samples',len(over),over[:10])
