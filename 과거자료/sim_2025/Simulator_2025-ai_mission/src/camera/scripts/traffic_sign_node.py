#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from ultralytics import YOLO
import numpy as np
import rospkg

class YOLOv8Node:
    def __init__(self):
        # ROS 설정
        rospy.init_node("traffic_sign_node", anonymous=True)

        rospack = rospkg.RosPack()
        weights_path = rospack.get_path('camera') + '/models/traffic_sign.pt'

        # YOLOv8 모델 로드
        self.model = YOLO(weights_path)
        self.model.fuse()

        # CvBridge
        self.bridge = CvBridge()

        # Subscriber / Publisher
        self.traffic_sign_pub = rospy.Publisher("/traffic_sign_status", Int32, queue_size=1)
        self.bbox_image_pub = rospy.Publisher("/yolo/traffic_sign/image", Image, queue_size=1)  # 추가
        self.image_sub = rospy.Subscriber("/image_jpeg2/compressed", CompressedImage, self.image_callback)

        rospy.loginfo("YOLOv8 ROS Node Started!")

    def image_callback(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_img, conf=0.35, verbose=False)[0]

        bboxes = results.boxes
        names = results.names
        best_traffic_sign = "50"
        max_confidence = 0.0

        for i in range(len(bboxes)):
            class_index = int(bboxes.cls[i])
            class_name = names[class_index]
            confidence = float(bboxes.conf[i])
            x1, y1, x2, y2 = map(int, bboxes.xyxy[i])

            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}"
            cv2.putText(cv_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 신뢰도가 가장 높은 신호등 선택
            if confidence > max_confidence:
                max_confidence = confidence
                best_traffic_sign = label
            
        sign_img_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        sign_img_msg.header.frame_id = 'map'
        sign_img_msg.header.stamp = rospy.Time.now()
        self.bbox_image_pub.publish(sign_img_msg)

        best_traffic_sign_msg = Int32()
        best_traffic_sign_msg.data = int(best_traffic_sign[:2])
        self.traffic_sign_pub.publish(best_traffic_sign_msg)


if __name__ == "__main__":
    try:
        node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
