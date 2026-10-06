#!/usr/bin/env python3
"""Existing planning displays, with real TF frames instead of map coordinate subtraction."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import copy
import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class VizPlanner:
    def __init__(self):
        self.dynamic = tf.TransformBroadcaster()
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.origin = None
        self.extrinsics = rospy.get_param('~lidar_xyz_rpy', [])
        self.lidar_frame = rospy.get_param('~lidar_frame', 'velodyne')
        if self.extrinsics and (len(self.extrinsics) != 6 or not all(math.isfinite(v) for v in self.extrinsics)):
            raise ValueError('lidar_xyz_rpy must be empty or [x,y,z,roll,pitch,yaw] in metres/radians')
        if not self.extrinsics:
            rospy.logwarn('No LiDAR extrinsics configured. Supply existing map->LiDAR TF or lidar_xyz_rpy; no guessed transform is published.')
        self.pubs = {}
        for topic in ('global_path', 'control_path', 'local_path'):
            self.pubs[topic] = rospy.Publisher('~viz_' + topic, Path, queue_size=1, latch=True)
            rospy.Subscriber('/' + topic, Path, self.path_callback, callback_args=topic, queue_size=1)
        self.pose_pub = rospy.Publisher('~viz_current_pose', Marker, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback, queue_size=1)

    def transform(self, child, translation, rotation):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = rospy.Time.now(), 'map', child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rotation
        return t

    def pose_callback(self, msg):
        if msg.header.frame_id != 'map' or msg.header.stamp == rospy.Time():
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in [p.x, p.y, q.x, q.y, q.z, q.w]):
            return
        if self.origin is None:
            self.origin = (p.x, p.y, 0.)
            transforms = [self.transform('planning_origin', self.origin, (0, 0, 0, 1))]
            if self.extrinsics:
                xyz, rpy = self.extrinsics[:3], self.extrinsics[3:]
                t = self.transform(self.lidar_frame, xyz, tf.transformations.quaternion_from_euler(*rpy))
                t.header.frame_id = 'planning_vehicle'
                transforms.append(t)
            self.static.sendTransform(transforms)
        # planning_vehicle is exactly the current_pose XY reference, not an assumed rear axle.
        self.dynamic.sendTransform((p.x, p.y, 0.), (q.x, q.y, q.z, q.w),
                                   msg.header.stamp, 'planning_vehicle', 'map')
        m = Marker()
        m.header = msg.header
        m.ns, m.id, m.type, m.action = 'current_pose', 0, Marker.ARROW, Marker.ADD
        m.pose = copy.deepcopy(msg.pose)
        m.pose.position.z = .2
        m.scale.x, m.scale.y, m.scale.z = 2., .35, .35
        m.color.r, m.color.g, m.color.a = 1., .8, 1.
        m.lifetime = rospy.Duration(.5)
        self.pose_pub.publish(m)

    def path_callback(self, msg, topic):
        path = copy.deepcopy(msg)
        # Path z contains controller curvature. Render it on the map plane only in RViz copies.
        for pose in path.poses:
            pose.pose.position.z = .05
        self.pubs[topic].publish(path)


if __name__ == '__main__':
    rospy.init_node('viz_planner')
    node = VizPlanner()
    rospy.spin()
