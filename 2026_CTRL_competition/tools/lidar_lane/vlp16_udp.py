#!/usr/bin/env python3
"""MORAI VLP-16 UDP(9302) 를 ROS 없이 직접 받아 (x, y, z, intensity, ring) 로 푸는 공용 모듈.

foscar_lidar/lidar_bridge.py 와 같은 규약을 쓴다.
    x = R·cos(방위각), y = −R·sin(방위각), z = 거리·sin(수직각)  (라이다 원점 기준)
    방위각 0도 = 차량 전방, +y = 좌측, +z = 상방
반사도(intensity) 는 패킷의 1 byte 값(0~255) 그대로다.
"""
import socket
import threading
import time

import numpy as np

VERTICAL_DEG = np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15],
                        dtype=np.float32)
PACKET_SIZE = 1206
BLOCKS = 12
CHANNELS = 32
FIRING_US = 55.296
CHANNEL_US = 2.304


def _firing_fraction():
    seq, ch = np.divmod(np.arange(CHANNELS), 16)
    return ((FIRING_US * seq + CHANNEL_US * ch) / (2 * FIRING_US)).astype(np.float32)


_FRAC = _firing_fraction()
_VERT = np.deg2rad(np.tile(VERTICAL_DEG, 2)).astype(np.float32)
_COS_V, _SIN_V = np.cos(_VERT), np.sin(_VERT)
_RING = np.tile(np.arange(16, dtype=np.int16), 2)


def decode(packets, min_range=0.9, max_range=120.0):
    """한 바퀴 분량 패킷 리스트 → (N,5) float32 [x, y, z, intensity, ring]."""
    buf = np.frombuffer(b''.join(packets), dtype=np.uint8).reshape(len(packets), PACKET_SIZE)
    blocks = buf[:, :BLOCKS * 100].reshape(-1, BLOCKS, 100)
    az = (blocks[:, :, 2:4].copy().view(np.uint16).reshape(-1, BLOCKS).astype(np.float32) * 0.01)
    body = blocks[:, :, 4:].reshape(-1, BLOCKS, CHANNELS, 3)
    dist = (body[:, :, :, :2].copy().view(np.uint16).reshape(-1, BLOCKS, CHANNELS)
            .astype(np.float32) * 0.002)
    refl = body[:, :, :, 2].astype(np.float32)

    flat_az = az.reshape(-1)
    delta = np.diff(flat_az, append=flat_az[-1])
    delta[delta < 0] += 360.0
    if delta.size > 1:
        delta[-1] = delta[-2]
    azimuth = flat_az[:, None] + delta[:, None] * _FRAC[None, :]
    rad = np.deg2rad(np.mod(azimuth, 360.0)).astype(np.float32)

    d = dist.reshape(-1, CHANNELS)
    r = d * _COS_V[None, :]
    x = r * np.cos(rad)
    y = -r * np.sin(rad)
    z = d * _SIN_V[None, :]
    ring = np.broadcast_to(_RING[None, :], d.shape)

    keep = (d > min_range) & (d < max_range)
    return np.stack([x[keep], y[keep], z[keep], refl.reshape(-1, CHANNELS)[keep],
                     ring[keep].astype(np.float32)], axis=1).astype(np.float32)


class Vlp16Receiver:
    """백그라운드 스레드로 UDP 를 받아 한 바퀴씩 콜백 / 큐로 넘긴다."""

    def __init__(self, port=9302, bind_ip='0.0.0.0', callback=None):
        self.port = port
        self.callback = callback
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.settimeout(1.0)
        self.sock.bind((bind_ip, port))
        self.lock = threading.Lock()
        self.latest = None          # 최신 (N,5) 클라우드
        self.latest_time = None
        self.revolutions = 0
        self.stamps = []            # 최근 회전 시각 (Hz 계산용)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        packets, prev_az = [], None
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != PACKET_SIZE:
                continue
            az = int.from_bytes(data[2:4], 'little')
            if prev_az is not None and az < prev_az and packets:
                try:
                    cloud = decode(packets)
                except Exception:
                    cloud = None
                packets = []
                if cloud is not None:
                    now = time.monotonic()
                    with self.lock:
                        self.latest = cloud
                        self.latest_time = now
                        self.revolutions += 1
                        self.stamps.append(now)
                        self.stamps = [s for s in self.stamps if s > now - 3.0]
                    if self.callback:
                        self.callback(cloud)
            prev_az = az
            packets.append(data)
            if len(packets) > 400:
                packets = []

    def get(self):
        with self.lock:
            return self.latest, self.latest_time

    def hz(self):
        with self.lock:
            if len(self.stamps) < 2:
                return 0.0
            return (len(self.stamps) - 1) / max(self.stamps[-1] - self.stamps[0], 1e-3)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def to_base_link(cloud, lidar_x=2.0, lidar_y=0.0, lidar_z=2.11):
    """라이다 원점 → base_link(지면 z=0) 로 평행이동한 사본."""
    out = cloud.copy()
    out[:, 0] += lidar_x
    out[:, 1] += lidar_y
    out[:, 2] += lidar_z
    return out
