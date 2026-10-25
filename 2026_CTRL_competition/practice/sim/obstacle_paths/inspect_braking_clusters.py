"""Inspect a handful of braking scans, retaining material and height evidence."""
import os,json
os.environ['FOSCAR_LIDAR_DIAG']='1';os.environ['FOSCAR_LIDAR_DIAG_R']='45'
import numpy as np,rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_lidar.obstacle_node import ObstacleNode,stamp_to_sec
from foscar_control.core import Route
from foscar_planning.frenet import to_frenet
from pathlib import Path
rclpy.init();n=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('publish_markers',value=False)])
route=Route(np.loadtxt('src/foscar_bringup/routes/global_path.txt')[:,:2]);n.route_xy=route.points;n.route_s=route.s
r=SequentialReader();r.open(StorageOptions(uri='/home/foscar/foscar_logs/bags/run_20261006_174510_points',storage_id=''),ConverterOptions('',''));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};seen=set();out=[]
while r.has_next():
 topic,data,t=r.read_next();m=deserialize_message(data,types[topic])
 if topic.endswith('odometry'):n.on_odom(m);continue
 if not n.odom_buf:continue
 st=stamp_to_sec(m.header.stamp);n.ego=n.ego_pose_at(st)
 if n.ego is None:continue
 s=to_frenet(route,*n.ego[:2],300)[0]
 bucket=next((k for k in (293,296,300,305,309) if k<=s<k+1),None)
 if bucket is None or bucket in seen:continue
 seen.add(bucket);n.ego_s=s;n.on_mission(String(data=json.dumps(dict(mission='STATIC_OBSTACLE'))));n.detect(m,st)
 out.append(dict(s=s,t=st,clusters=n.ring_diag_cl))
 if len(seen)==5:break
Path('practice/sim/obstacle_paths/braking_clusters.json').write_text(json.dumps(out,indent=2))
for row in out:
 print('SCAN',round(row['s'],2))
 for c in row['clusters']:
  if c[6]=='kept':print(c)
n.destroy_node();rclpy.shutdown()
