"""Smallest gap between the car body (odometry rear axle, -0.835..+3.8 m, half width 0.946) and each truth obstacle box
(objects_<i>.jsonl l/w/yaw), with speed and steering there (practice only). python3 body_gap.py RUN_DIR..."""
import os,sys,glob,json,math,bisect
import numpy as np,rosbag2_py
from datetime import datetime
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
REAR,FRONT,HW=.835,3.8,.946
OBJ={15:'car',16:'barricade',26:'cone26',27:'cone27',32:'cone32',33:'cone33'}
def bag_for(R,i):
    t0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['t'];best=None
    for b in glob.glob(os.path.expanduser('~/foscar_logs/bags/run_*')):
        try: tb=datetime.strptime(os.path.basename(b)[4:19],'%Y%m%d_%H%M%S').timestamp()
        except ValueError: continue
        if -5<=t0-tb<=300 and (best is None or t0-tb<best[0]): best=(t0-tb,b)
    return best[1]
def rect(cx,cy,yaw,l,w,n=9):
    c,s=math.cos(yaw),math.sin(yaw);pts=[]
    for a in np.linspace(-l/2,l/2,n):
        for b in (-w/2,w/2): pts.append((cx+c*a-s*b,cy+s*a+c*b))
    for b in np.linspace(-w/2,w/2,n):
        for a in (-l/2,l/2): pts.append((cx+c*a-s*b,cy+s*a+c*b))
    return np.array(pts)
def gap(px,py,yaw,P):
    c,s=math.cos(yaw),math.sin(yaw);d=P-[px,py]
    lx=c*d[:,0]+s*d[:,1];ly=-s*d[:,0]+c*d[:,1]
    ox=np.maximum(np.maximum(-REAR-lx,0),lx-FRONT);oy=np.maximum(np.maximum(-HW-ly,0),ly-HW)
    inside=(lx>=-REAR)&(lx<=FRONT)&(np.abs(ly)<=HW)
    g=np.where(inside,-np.minimum(np.minimum(lx+REAR,FRONT-lx),HW-np.abs(ly)),np.hypot(ox,oy))
    return float(g.min())
Mo=get_message('nav_msgs/msg/Odometry');Ms=get_message('std_msgs/msg/Float64')
for R in sys.argv[1:]:
  for i in range(20):
    if not os.path.exists(f'{R}/objects_{i}.jsonl'): continue
    o0=json.loads(open(f'{R}/objects_{i}.jsonl').readline())['objects']
    boxes={o['id']:rect(o['x'],o['y'],math.radians(o['yaw']),o['l'],o['w']) for o in o0 if o['id'] in OBJ}
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag_for(R,i),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/localization/odometry','/localization/s']))
    s=0;best={k:(9e9,0,0) for k in boxes}
    while r.has_next():
        tp,raw,t=r.read_next()
        if tp=='/localization/s': s=deserialize_message(raw,Ms).data;continue
        if not 1880<s<1975: continue
        m=deserialize_message(raw,Mo);q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        for k,P in boxes.items():
            g=gap(m.pose.pose.position.x,m.pose.pose.position.y,yaw,P)
            if g<best[k][0]: best[k]=(g,s,m.twist.twist.linear.x*3.6)
    print('%s/%d '%(os.path.basename(R),i)+' | '.join('%s %.2f m @s%.0f %.0fkm/h'%(OBJ[k],*best[k]) for k in sorted(boxes)))
