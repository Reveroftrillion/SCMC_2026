#!/usr/bin/env python3
#-*- encoding: utf-8 -*-
import sys
import os

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from camera.cfg import LanenetParamsConfig
from slidewindow import SlideWindow

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

class LanenetDetection:
    def __init__(self):
        rospy.init_node('lanenet_detection_node')

        self.bridge = CvBridge()
        self.image = None
        self.slidewindow = SlideWindow()
        self.x_location = 320
        self.last_x_location = 320

        # Publishers
        self.lane_detection_pub = rospy.Publisher('/lanenet/lane_detection', Image, queue_size=1)
        self.angle_pub = rospy.Publisher('/lanenet/angle', Float32, queue_size=1)

        # Dynamic reconfigure 파라미터 초기값
        self.params = {
            # 흰색 HSV
            'white_H_min': 0, 'white_H_max': 180,
            'white_S_min': 0, 'white_S_max': 67,
            'white_V_min': 151, 'white_V_max': 255,
            # 노란색 HSV
            'yellow_H_min': 10, 'yellow_H_max': 35,
            'yellow_S_min': 108, 'yellow_S_max': 255,
            'yellow_V_min': 125, 'yellow_V_max': 255,
            # ROI
            'roi_y_start': 0, 'roi_y_end': 480,
            # Warping 소스 포인트
            'warp_src_x1': 100, 'warp_src_y1': 400,
            'warp_src_x2': 170, 'warp_src_y2': 330,
            'warp_src_x3': 470, 'warp_src_y3': 330,
            'warp_src_x4': 540, 'warp_src_y4': 400,
            # 이진화
            'binary_threshold': 20,
            # PID
            'pid_kp': 0.01, 'pid_ki': 0.001, 'pid_kd': 0.003,
        }

        # Dynamic reconfigure server
        self.srv = Server(LanenetParamsConfig, self.reconfigure_callback)

        print("[LaneNet] HSV-based lane detection initialized")

        rospy.Subscriber("/image_jpeg2/compressed", CompressedImage, self.image_callback)

        self.run()

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.image is None:
                rate.sleep()
                continue

            try:
                # PID 객체 생성
                pid = PID(self.params['pid_kp'], self.params['pid_ki'], self.params['pid_kd'])

                # 이미지 복사
                img_frame = self.image.copy()
                height, width = img_frame.shape[:2]

                # ROI 자르기
                roi_y_start = max(0, min(self.params['roi_y_start'], height - 1))
                roi_y_end = max(roi_y_start + 1, min(self.params['roi_y_end'], height))
                img_roi = img_frame[roi_y_start:roi_y_end, :]

                # HSV 변환
                img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

                # 흰색 마스크
                lower_white = np.array([self.params['white_H_min'],
                                       self.params['white_S_min'],
                                       self.params['white_V_min']])
                upper_white = np.array([self.params['white_H_max'],
                                       self.params['white_S_max'],
                                       self.params['white_V_max']])
                mask_white = cv2.inRange(img_hsv, lower_white, upper_white)

                # 노란색 마스크
                lower_yellow = np.array([self.params['yellow_H_min'],
                                        self.params['yellow_S_min'],
                                        self.params['yellow_V_min']])
                upper_yellow = np.array([self.params['yellow_H_max'],
                                        self.params['yellow_S_max'],
                                        self.params['yellow_V_max']])
                mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

                # 마스크 합치기
                mask_combined = cv2.bitwise_or(mask_yellow, mask_white)
                img_filtered = cv2.bitwise_and(img_roi, img_roi, mask=mask_combined)

                # Perspective Transform (Warping)
                roi_height, roi_width = img_filtered.shape[:2]

                src_points = np.float32([
                    [self.params['warp_src_x1'], self.params['warp_src_y1']],  # left bottom
                    [self.params['warp_src_x2'], self.params['warp_src_y2']],  # left top
                    [self.params['warp_src_x3'], self.params['warp_src_y3']],  # right top
                    [self.params['warp_src_x4'], self.params['warp_src_y4']]   # right bottom
                ])

                dst_points = np.float32([
                    [160, roi_height - 1],   # left bottom
                    [160, 0],                # left top
                    [480, 0],                # right top
                    [480, roi_height - 1]    # right bottom
                ])

                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                img_warped = cv2.warpPerspective(img_filtered, matrix, (640, roi_height))

                # 이진화
                grayed_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2GRAY)
                bin_img = np.zeros_like(grayed_img, dtype=np.uint8)
                threshold = self.params['binary_threshold']
                bin_img[grayed_img > threshold] = 1

                # Sliding Window
                out_img, self.x_location, _ = self.slidewindow.slidewindow(bin_img, True)

                if self.x_location is None:
                    self.x_location = self.last_x_location
                else:
                    self.last_x_location = self.x_location

                # 시각화
                bin_img_colored = cv2.cvtColor(bin_img * 255, cv2.COLOR_GRAY2BGR)
                img_blended = cv2.addWeighted(out_img, 1.0, bin_img_colored, 0.6, 0)

                # Publish lane detection image
                try:
                    lane_det_msg = self.bridge.cv2_to_imgmsg(img_blended, "bgr8")
                    self.lane_detection_pub.publish(lane_det_msg)
                except Exception as e:
                    rospy.logerr(f"Error publishing lane detection: {e}")

                # 조향각 계산 (부호 반대: 목표가 오른쪽이면 왼쪽으로 조향)
                pixel_error = 320 - self.x_location
                normalized_error = pixel_error / 8
                angle = pid.pid_control(normalized_error)
                self.angle_pub.publish(Float32(angle))

            except Exception as e:
                rospy.logerr(f"Error in processing: {e}")
                import traceback
                traceback.print_exc()

            rate.sleep()

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Reconfigure Request")
        self.params.update(config)
        return config

    def image_callback(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == "__main__":
    try:
        lanenet_detection_node = LanenetDetection()
    except rospy.ROSInterruptException:
        pass
