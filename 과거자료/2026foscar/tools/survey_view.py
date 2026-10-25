#!/usr/bin/env python3
"""Read-only RGB viewer for an already running survey (no UDP sockets/control)."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import time


def main():
    rclpy.init()
    node = Node('survey_rgb_view')
    frame = [None, 0.]

    def receive(msg):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                frame[:] = [img, time.monotonic()]
        except cv2.error as exc:
            node.get_logger().warning(str(exc))

    node.create_subscription(CompressedImage, '/sensors/camera/tele/compressed',
                             receive, qos_profile_sensor_data)
    title = 'Survey RGB - controls in survey terminal'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1280, 720)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            img = frame[0].copy() if frame[0] is not None else np.zeros((720, 1280, 3), np.uint8)
            age = time.monotonic()-frame[1]
            text = 'LIVE | Stop/save: ENTER in survey terminal' if age < 1. else 'NO RECENT CAMERA DATA'
            cv2.putText(img, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .65,
                        (0, 255, 0) if age < 1. else (0, 0, 255), 2)
            cv2.imshow(title, img)
            if cv2.waitKey(1) & 255 == ord('q') or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
