#!/usr/bin/env python3
"""MORAI 카메라 UDP(JPEG 조각) 수신 + 라이다 점을 영상에 투영하는 공용 모듈 (ROS 불필요).

조각 구조: 'MOR' 3 | sec 4 | nsec 4 | index 4 | size 4 | JPEG 조각 | 꼬리 2 ('AI' 중간 / 'EI' 마지막)
`size` 만큼만 잘라 이어 붙인다 (마지막 조각은 뒤가 패딩).

카메라 모델: 2026_molit_comp_cam_set.json 전방 카메라 기준
    위치 (1.9, 0, 1.2) [차량 원점 기준]  pitch 2°  1280×720  FOV 90°  → fx = fy = 640
차량 원점은 지면에서 0.36 m 위이므로 base_link(지면) 기준 카메라 높이 = 1.56 m.
"""
import socket
import struct
import threading

import numpy as np

MAGIC = b'MOR'
HEADER = 19            # 'MOR' + sec + nsec + index + size


class CameraReceiver:
    def __init__(self, port=9202, bind_ip='0.0.0.0'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.settimeout(1.0)
        self.sock.bind((bind_ip, port))
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.frames = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf = bytearray()
        while self.running:
            try:
                d, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if d[:3] != MAGIC or len(d) < HEADER + 2:
                continue
            size = struct.unpack_from('<I', d, 15)[0]
            body = d[HEADER:HEADER + size]
            if not buf and body[:2] != b'\xff\xd8':
                continue
            buf += body
            if d[-2:] == b'EI':
                if buf[-2:] == b'\xff\xd9':
                    with self.lock:
                        self.latest_jpeg = bytes(buf)
                        self.frames += 1
                buf = bytearray()

    def get_jpeg(self):
        with self.lock:
            return self.latest_jpeg

    def get_image(self):
        """BGR ndarray (cv2). 없으면 None."""
        j = self.get_jpeg()
        if j is None:
            return None
        import cv2
        return cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


class PinholeCamera:
    """base_link(지면 z=0, 전방 +x, 좌 +y, 상 +z) 점 → 픽셀."""

    def __init__(self, x=1.9, y=0.0, z=1.2 + 0.36, pitch_deg=2.0, yaw_deg=0.0,
                 width=1280, height=720, fov_deg=90.0):
        self.t = np.array([x, y, z], np.float32)
        self.w, self.h = width, height
        self.fx = self.fy = (width / 2) / np.tan(np.radians(fov_deg / 2))
        self.cx, self.cy = width / 2, height / 2
        p, yw = np.radians(pitch_deg), np.radians(yaw_deg)
        # pitch 양수 = 아래를 봄 (MORAI 규약). 카메라 광축은 차량 +x.
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]], np.float32)
        Rz = np.array([[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]], np.float32)
        self.R = (Rz @ Ry).T          # 차량 → 카메라 정렬 좌표 (아직 x 전방)

    def project(self, pts):
        """(N,3) base_link → (N,2) 픽셀, 카메라 뒤·화면 밖은 NaN."""
        p = (pts[:, :3] - self.t) @ self.R.T
        X, Y, Z = p[:, 0], p[:, 1], p[:, 2]        # X 전방, Y 좌, Z 상
        u = self.cx - self.fx * Y / np.where(X > 0.1, X, np.nan)
        v = self.cy - self.fy * Z / np.where(X > 0.1, X, np.nan)
        out = np.stack([u, v], axis=1)
        ok = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        out[~ok] = np.nan
        return out
