#!/usr/bin/env python3
"""카메라 뷰어 창 — RViz 없이 카메라와 신호 인식 결과를 눈으로 확인한다.

    ros2 run foscar_camera camera_view

기본으로 `yolo_signal_node` 의 디버그 영상(`/perception/traffic_light/debug/compressed`,
검출 박스·판정 색이 그려진 것)을 띄우고, 그게 안 오면 원본
`/sensors/camera/traffic/compressed` (카메라 7번) 을 그대로 보여준다.

키
    s   원본 프레임을 `~/foscar_logs/cam_<시각>.jpg` 로 저장 (현장점검 캡처용 — YOLO 학습 데이터로도 쓴다). 디버그 그림이 아닌 **원본**이 저장된다.
    q   종료
"""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

WINDOW = 'foscar_camera'


class CameraView(Node):

    def __init__(self):
        super().__init__('camera_view')
        p = self.declare_parameter
        self.raw_topic = p('image_topic', '/sensors/camera/traffic/compressed').value
        self.debug_topic = p('debug_topic', '/perception/traffic_light/debug/compressed').value
        self.save_dir = Path(p('save_dir', str(Path.home() / 'foscar_logs')).value).expanduser()
        self.scale = float(p('scale', 1.0).value)     # 화면이 작으면 0.5

        self.create_subscription(CompressedImage, self.raw_topic, self.on_raw, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.debug_topic, self.on_debug, qos_profile_sensor_data)
        self.raw_jpeg = None
        self.frame = None
        self.debug_seen = None           # 마지막 디버그 영상 수신 시각
        self.create_timer(1.0 / 30, self.refresh)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'{self.debug_topic} (없으면 {self.raw_topic}) | s=저장 q=종료')

    def on_raw(self, msg: CompressedImage) -> None:
        self.raw_jpeg = bytes(msg.data)
        now = self.get_clock().now()
        # 디버그 영상이 1초 넘게 안 오면 원본을 보여준다
        if self.debug_seen is None or (now - self.debug_seen).nanoseconds > 1e9:
            self.frame = self.decode(self.raw_jpeg)

    def on_debug(self, msg: CompressedImage) -> None:
        self.debug_seen = self.get_clock().now()
        self.frame = self.decode(bytes(msg.data))

    @staticmethod
    def decode(jpeg: bytes):
        return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

    def refresh(self) -> None:
        if self.frame is None:
            canvas = np.zeros((360, 640, 3), np.uint8)
            cv2.putText(canvas, 'no image yet', (200, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (200, 200, 200), 2)
            cv2.putText(canvas, self.raw_topic, (20, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (120, 120, 120), 1)
        else:
            canvas = self.frame
            if self.scale != 1.0:
                canvas = cv2.resize(canvas, None, fx=self.scale, fy=self.scale,
                                    interpolation=cv2.INTER_AREA)
        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            raise KeyboardInterrupt
        if key == ord('s'):
            self.save()

    def save(self) -> None:
        if self.raw_jpeg is None:
            self.get_logger().warn('저장할 원본 영상이 없습니다')
            return
        self.save_dir.mkdir(parents=True, exist_ok=True)
        path = self.save_dir / f'cam_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg'
        path.write_bytes(self.raw_jpeg)
        self.get_logger().info(f'저장: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
