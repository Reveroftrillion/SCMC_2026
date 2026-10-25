import os,json,sys,math,time
os.environ['FOSCAR_LIDAR_DIAG']='1';os.environ['FOSCAR_LIDAR_DIAG_R']='55'
import numpy as np,rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from foscar_lidar.obstacle_node import ObstacleNode,stamp_to_sec
sys.path.insert(0,'practice/sim')
from swerve_analysis import route,to_frenet
rclpy.init(); n=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('publish_markers',value=False)])
P,S=route();n.route_xy=P;n.route_s=S
r=SequentialReader();r.open(StorageOptions(uri='/home/foscar/foscar_logs/bags/run_20261006_161109_points',storage_id='sqlite3'),ConverterOptions('',''));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
f=open('/tmp/fragments_replay.jsonl','w');cnt=0
while r.has_next():
 topic,data,t=r.read_next();m=deserialize_message(data,types[topic])
 if topic.endswith('odometry'):n.on_odom(m);continue
 if not n.odom_buf:continue
 st=stamp_to_sec(m.header.stamp);n.ego=n.ego_pose_at(st)
 if n.ego is None:continue
 n.ego_s=to_frenet(P,S,*n.ego[:2])[0];n.on_mission(String(data=json.dumps(dict(mission='TUNNEL'))))
 ds=n.detect(m,st)
 f.write(json.dumps(dict(t=t/1e9,s=n.ego_s,ego=n.ego,cl=n.ring_diag_cl,dets=ds))+ '\n');cnt+=1
f.close();print('frames',cnt);n.destroy_node();rclpy.shutdown()
