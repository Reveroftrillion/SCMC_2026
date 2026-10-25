#!/usr/bin/env python3

import rospy
from simul_msgs.msg import ObjectInfo
from visualization_msgs.msg import Marker, MarkerArray

# ★★★ 중요 ★★★
# anom_depth_ros.py에서 이미 '차량 기준'으로 좌표를 변환했으므로,
# 이 마커의 기준 프레임(FRAME_ID)은 '차량 좌표계' 이름이어야 합니다.
# (보통 'base_link'를 많이 사용합니다)
FRAME_ID = "base_link" 

def info_callback(msg):
    marker_array = MarkerArray()
    
    # 메시지에 포함된 모든 객체에 대해 Marker를 생성합니다.
    for i in range(msg.objectCounts):
        marker = Marker()
        # 헤더 설정: 기준 좌표계와 현재 시간
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()
        
        # 네임스페이스와 ID (각 마커를 구별하기 위함)
        marker.ns = "obstacles"
        marker.id = i
        
        # 타입: 큐브(3D 박스)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 3D 중심 좌표 (ObjectInfo에서 그대로 가져옴 - 이미 차량 기준임)
        marker.pose.position.x = msg.centerX[i]
        marker.pose.position.y = msg.centerY[i]
        marker.pose.position.z = msg.centerZ[i]
        
        # 3D 방향 (회전 없음 - (0,0,0,1)은 회전 없음을 의미)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 3D 크기 (ObjectInfo에서 그대로 가져옴)
        marker.scale.x = msg.lengthX[i]
        marker.scale.y = msg.lengthY[i]
        marker.scale.z = msg.lengthZ[i]
        
        # 색상 (빨간색, 50% 투명도)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # 알파값 (투명도)
        
        # 마커 유지 시간 (0.2초 뒤 사라짐. 새 메시지가 계속 갱신해 줌)
        marker.lifetime = rospy.Duration(0.2) 

        marker_array.markers.append(marker)
        
    # 변환된 MarkerArray를 발행
    marker_pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node("marker_converter")
    
    # RViz로 보낼 MarkerArray 퍼블리셔
    marker_pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=1)
    
    # 기존 ObjectInfo 구독
    rospy.Subscriber("/object_info_static", ObjectInfo, info_callback)
    
    rospy.spin()