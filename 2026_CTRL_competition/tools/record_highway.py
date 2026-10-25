#!/usr/bin/env python3
"""Passive JSONL recorder for highway evidence. Only subscribes; never drives."""
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,String
from foscar_control.core import quaternion_yaw


class Recorder(Node):
    def __init__(self,output):
        super().__init__('highway_passive_recorder')
        self.file=Path(output).open('x');self.ego=None;self.station=None
        self.create_subscription(Odometry,'/localization/odometry',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Float64,'/localization/s',lambda msg:setattr(self,'station',msg.data),10)
        self.create_subscription(String,'/perception/footprints',self.on_scan,10)
    def on_odom(self,msg):
        p=msg.pose.pose;q=p.orientation
        self.ego=dict(x=p.position.x,y=p.position.y,yaw=quaternion_yaw(q.w,q.x,q.y,q.z),v=msg.twist.twist.linear.x,
                      odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    def on_scan(self,msg):
        if self.ego is None or self.station is None:return
        try:data=json.loads(msg.data)
        except ValueError:return
        now=self.get_clock().now().nanoseconds/1e9
        self.file.write(json.dumps(dict(stamp=now,ego=dict(self.ego,global_s=self.station),footprints=data))+'\n');self.file.flush()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True)
    args=ap.parse_args();rclpy.init();node=Recorder(args.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.file.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
