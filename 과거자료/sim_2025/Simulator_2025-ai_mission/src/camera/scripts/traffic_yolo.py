#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
from ultralytics import YOLO

CONFIDENCE = 0.4

import rospkg

rospack = rospkg.RosPack()
weights_path = rospack.get_path('camera') + '/models/1027_40epoch.pt'
model = YOLO(weights_path)
bridge = CvBridge()

# **(수정)** 처리된 이미지를 발행할 Publisher를 전역 변수로 선언
image_pub = None

last_bbox = []
hsv_ranges = {
    'red1': (np.array([0, 100, 95]), np.array([10, 255, 255])),
    'red2': (np.array([160, 100, 95]), np.array([179, 255, 255])),
    'yellow': (np.array([20, 100, 95]), np.array([30, 255, 255])),
    'green': (np.array([40, 100, 95]), np.array([90, 255, 255]))
}


def image_callback(msg):
    global last_bbox
    
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    results = model.predict(cv_image, conf=CONFIDENCE, verbose=False)
    results = results[0]
    bboxes = results.boxes
    names = results.names

    # 감지된 객체가 있을 때만 이미지에 그리고 발행하도록 수정
    if len(bboxes) > 0:
        # 가장 신뢰도 높은 신호등 찾기
        max_confidence = 0.0
        best_traffic_light = None

        for i in range(len(bboxes)):       
            class_index = int(bboxes.cls[i])
            class_name = names[class_index]
            confidence = float(bboxes.conf[i])
            x1, y1, x2, y2 = map(int, bboxes.xyxy[i])

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}"                                               
            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 신뢰도가 가장 높은 신호등 선택
            if confidence > max_confidence:
                max_confidence = confidence
                best_traffic_light = label
                last_bbox = [x1, y1, x2, y2]

        # 가장 신뢰도 높은 신호등 발행
        if best_traffic_light is not None:
            traffic_light_pub.publish(best_traffic_light)

    try:
        img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(img_msg)

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def check_traffic_sign(cropped_img):
    hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    
    detected_color = "None"
    for color, (lower, upper) in hsv_ranges.items():
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


def yolo_traffic_light_node():
    rospy.init_node('yolo_traffic_light_node', anonymous=True)

    # **(수정)** image_pub를 전역 변수로 사용
    global traffic_light_pub, image_pub

    rospy.Subscriber('/image_jpeg/compressed', CompressedImage, image_callback, queue_size=1)

    traffic_light_pub = rospy.Publisher('/traffic_light_status', String, queue_size=10)
    # **(추가)** 처리된 이미지를 발행할 Publisher 초기화
    image_pub = rospy.Publisher('/yolo_traffic_light/image_raw', Image, queue_size=1)


    rospy.spin()

if __name__ == '__main__':
    try:
        yolo_traffic_light_node()
    except rospy.ROSInterruptException:
        pass