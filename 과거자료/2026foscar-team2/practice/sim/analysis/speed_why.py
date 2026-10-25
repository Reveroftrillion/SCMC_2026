"""Why is the car this slow? Samples a bag every DT s: station, speed, controller target,
behavior v_max/state, limit source, pedal, and the highway policy cap/reason.

    python3 speed_why.py ~/foscar_logs/bags/run_<stamp> 1.0 | sed 's/s= */s=/' | awk '{split($2,a,"="); if (a[2]>575 && a[2]<860) print}'
"""
import sys,json
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=SequentialReader();r.open(StorageOptions(uri=sys.argv[1],storage_id=''),ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
want={'/localization/s','/behavior/highway','/behavior/target','/control/telemetry'}
st={};last=-9;t0=None
while r.has_next():
    topic,data,t=r.read_next()
    if topic not in want:continue
    if t0 is None:t0=t
    m=deserialize_message(data,get_message(types[topic]))
    if topic=='/localization/s':st['s']=m.data
    elif topic=='/behavior/target':st['vmax']=m.v_max;st['bstate']=m.state
    elif topic=='/behavior/highway':
        d=json.loads(m.data);st['hw']=(d.get('state'),d.get('speed_reason'),d.get('speed_cap_mps'),d.get('gap_strategy'))
    else:
        try:st['tel']=json.loads(m.data)
        except Exception:st['tel']={}
    et=(t-t0)*1e-9
    if et-last>=float(sys.argv[2]) and 's' in st:
        last=et;tel=st.get('tel',{})
        f=lambda x:'%.1f'%(x*3.6) if isinstance(x,(int,float)) and x<1e6 else str(x)
        print('%5.1f s=%6.1f v=%s tgt=%s vmax=%s %s src=%s acc=%s brk=%s hw=%s'%(et,st['s'],f(tel.get('speed_mps')),f(tel.get('target_speed',tel.get('target_speed_mps'))),f(st.get('vmax')),st.get('bstate'),tel.get('limit_source'),tel.get('accel') and round(tel['accel'],2),tel.get('brake') and round(tel['brake'],2),st.get('hw') and (st['hw'][0],st['hw'][1],f(st['hw'][2]),st['hw'][3])))
