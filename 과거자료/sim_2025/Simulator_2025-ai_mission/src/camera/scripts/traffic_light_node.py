#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header, String
from ultralytics import YOLO
import numpy as np
import time
import rospkg

class YOLOv8Node:
    def __init__(self):
        # ROS 설정
        rospy.init_node("traffic_light_node", anonymous=True)

        rospack = rospkg.RosPack()
        weights_path = rospack.get_path('camera') + '/models/traffic_light.pt'

        # YOLOv8 모델 로드
        self.model = YOLO(weights_path)
        self.model.fuse()
        self.threshold_y = 0

        # CvBridge
        self.bridge = CvBridge()

        # Subscriber / Publisher
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.traffic_sign_pub = rospy.Publisher("/traffic_light_status", String, queue_size=1)
        self.bbox_image_pub = rospy.Publisher("/yolo/image/compressed", CompressedImage, queue_size=1)  # 추가

        # HSV 범위
        self.hsv_ranges = {
            'red1': (np.array([0, 100, 95]), np.array([10, 255, 255])),
            'red2': (np.array([160, 100, 95]), np.array([179, 255, 255])),
            'yellow': (np.array([20, 100, 95]), np.array([30, 255, 255])),
            'green': (np.array([40, 100, 95]), np.array([90, 255, 255]))
        }

        rospy.loginfo("YOLOv8 ROS Node Started!")

    def image_callback(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_img, verbose=False)[0]
        sign = "None"

        bboxes = results.boxes
        names = results.names
        
        if len(bboxes) > 0:
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
                    cropped_img = cv_img[y1:y2, x1:x2]
                    sign = self.check_traffic_sign(cropped_img)

        traffic_sign = String()
        traffic_sign.data = sign
        self.traffic_sign_pub.publish(traffic_sign)

        comp_img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img, dst_format="jpg")
        comp_img_msg.header.stamp = rospy.Time.now()
        comp_img_msg.header.frame_id = 'map'
        self.bbox_image_pub.publish(comp_img_msg)

    def check_traffic_sign(self, cropped_img):
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        
        detected_color = "None"
        for color, (lower, upper) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv_img, lower, upper)
            if cv2.countNonZero(mask) >= 10: 
                if "red" in color:
                    detected_color = "red"
                elif color == "yellow":
                    detected_color = "yellow"
                elif color == "green":
                    detected_color = "green"
                break

        return detected_color


if __name__ == "__main__":
    try:
        node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
