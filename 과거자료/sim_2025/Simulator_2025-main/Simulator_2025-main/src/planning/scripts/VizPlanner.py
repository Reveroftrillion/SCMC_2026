#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

from utils import make_pose_stamped

class VizPlanner:
    def __init__(self):
        self.viz_control_path_pub = rospy.Publisher('~viz_control_path', Path, queue_size=10)
        self.viz_global_path_pub = rospy.Publisher('~viz_global_path', Path, queue_size=10)
        self.viz_local_path_pub = rospy.Publisher('~viz_local_path', Path, queue_size=10)
        self.viz_current_pose_pub = rospy.Publisher('~viz_current_pose', Marker, queue_size=10)
        self.viz_completion_zone_pub = rospy.Publisher('~viz_completion_zone', Marker, queue_size=10)

        self.viz_offset_x = 0
        self.viz_offset_y = 0

        # Local path 목표 지점
        self.local_path_target = None

        rospy.Subscriber('/control_path', Path, self.control_path_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)
        rospy.Subscriber('/local_path', Path, self.local_path_callback)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback)
        rospy.Subscriber('/local_path_done', Bool, self.local_path_done_callback)

    def current_pose_callback(self, current_pose: PoseStamped):
        if self.viz_offset_x == 0:
            self.viz_offset_x = current_pose.pose.position.x
            self.viz_offset_y = current_pose.pose.position.y

        # Create arrow marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position (with offset)
        marker.pose.position.x = current_pose.pose.position.x - self.viz_offset_x
        marker.pose.position.y = current_pose.pose.position.y - self.viz_offset_y
        marker.pose.position.z = 0.0

        # Orientation (same as vehicle)
        marker.pose.orientation = current_pose.pose.orientation

        # Arrow size
        marker.scale.x = 2.0  # Length
        marker.scale.y = 0.3  # Width
        marker.scale.z = 0.3  # Height

        # Color (red arrow)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.viz_current_pose_pub.publish(marker)
        
    def global_path_callback(self, global_path: Path):
        if self.viz_offset_x == 0:
            return
        
        for pose in global_path.poses:
            pose.pose.position.x -= self.viz_offset_x
            pose.pose.position.y -= self.viz_offset_y
        
        self.viz_global_path_pub.publish(global_path)
        
    def control_path_callback(self, control_path: Path):
        if self.viz_offset_x == 0:
            return

        for pose in control_path.poses:
            pose.pose.position.x -= self.viz_offset_x
            pose.pose.position.y -= self.viz_offset_y

        self.viz_control_path_pub.publish(control_path)

    def local_path_callback(self, local_path: Path):
        if self.viz_offset_x == 0:
            return

        # Local path의 마지막 지점을 목표로 저장
        if len(local_path.poses) > 0:
            last_pose = local_path.poses[-1]
            self.local_path_target = (last_pose.pose.position.x, last_pose.pose.position.y)

            # Completion zone 시각화
            self.visualize_completion_zone()

        for pose in local_path.poses:
            pose.pose.position.x -= self.viz_offset_x
            pose.pose.position.y -= self.viz_offset_y

        self.viz_local_path_pub.publish(local_path)

    def local_path_done_callback(self, msg: Bool):
        """Local path 완료 시 completion zone 제거"""
        if msg.data:
            self.local_path_target = None
            # 빈 마커로 삭제
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "completion_zone"
            marker.id = 0
            marker.action = Marker.DELETE
            self.viz_completion_zone_pub.publish(marker)

    def visualize_completion_zone(self):
        """COMPLETION_DISTANCE 범위를 원으로 시각화"""
        if self.local_path_target is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "completion_zone"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # 목표 지점 위치 (offset 적용)
        marker.pose.position.x = self.local_path_target[0] - self.viz_offset_x
        marker.pose.position.y = self.local_path_target[1] - self.viz_offset_y
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        # 원 크기: COMPLETION_DISTANCE = 2.0m
        COMPLETION_DISTANCE = 2.0
        marker.scale.x = COMPLETION_DISTANCE * 2  # 지름
        marker.scale.y = COMPLETION_DISTANCE * 2
        marker.scale.z = 0.1  # 높이 (얇게)

        # 색상: 반투명 녹색
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # 반투명

        self.viz_completion_zone_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('viz_planner')
    viz_planner = VizPlanner()
    rospy.spin()
