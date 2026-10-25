#!/usr/bin/env python3
"""MORAI VLP-16 UDP → `/sensors/lidar/points` (PointCloud2).

MORAI 는 자체 포맷이 아니라 **실제 Velodyne VLP-16 데이터 패킷을 그대로** 쏜다
(1206 byte, 블록 플래그 0xFFEE, factory byte 0x37/0x22 = Strongest/VLP-16).
그래서 규격 문서대로 파싱하면 된다. `ros-humble-velodyne` 을 설치하지 않아도 되도록
여기서 직접 푼다 — 대회 현장 PC 에 apt 설치가 막혀 있어도 돌아가야 하기 때문이다.

패킷 구조 (1206 byte)
    블록 12개 × 100 byte + 타임스탬프 4 byte + factory 2 byte
    블록  = 플래그 2 byte (0xFFEE) + 방위각 2 byte (0.01도 단위)
            + 32 × (거리 2 byte (2 mm 단위) + 반사도 1 byte)
    32 = 16채널 × 2회 발사. 두 번째 발사는 방위각이 살짝 더 돌아간 상태라 보간해 준다.

좌표 규약 (실측으로 확정)
    x = R·cos(방위각), y = −R·sin(방위각), z = 거리·sin(수직각)
    → 방위각 0도 = 차량 전방, +y = 좌측, +z = 상방 (ROS REP-103)
    K-City 정차 상태에서 지면 위 구조물 988개 중 910개가 +x 쪽에 몰리는 것으로 검증했다
    (신호등·기둥이 전부 차 앞에 있는 장면과 일치).

한 바퀴(방위각이 360도를 넘어 되감기는 시점)를 모아서 한 번에 발행한다.
"""

import socket
import threading

import numpy as np
import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_executors import ExternalShutdownException
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField

# VLP-16 채널별 수직각 (규격 고정값). 채널 순서대로 아래/위를 번갈아 쏜다.
VERTICAL_DEG = np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15],
                        dtype=np.float32)
PACKET_SIZE = 1206
BLOCKS = 12
CHANNELS = 32               # 16채널 × 2회 발사
FIRING_US = 55.296          # 한 발사 시퀀스 길이
CHANNEL_US = 2.304          # 채널 간 간격


def firing_fraction() -> np.ndarray:
    """블록 안에서 (발사순서, 채널) 이 차지하는 시간 비율 → 방위각 보간 계수."""
    seq, ch = np.divmod(np.arange(CHANNELS), 16)
    return ((FIRING_US * seq + CHANNEL_US * ch) / (2 * FIRING_US)).astype(np.float32)


class LidarBridge(Node):

    def __init__(self):
        super().__init__('lidar_bridge')

        p = self.declare_parameter
        self.bind_ip = p('bind_ip', '0.0.0.0').value
        self.port = p('port', 9302).value            # MORAI Destination Port
        self.topic = p('topic', '/sensors/lidar/points').value
        self.frame_id = p('frame_id', 'lidar').value
        self.min_range = p('min_range_m', 0.9).value
        self.max_range = p('max_range_m', 120.0).value
        self.rcvbuf = p('rcvbuf_bytes', 8 * 1024 * 1024).value

        # 미리 계산해 두는 상수 (프레임마다 다시 만들지 않는다)
        self.frac = firing_fraction()                                  # (32,)
        vert = np.deg2rad(np.tile(VERTICAL_DEG, 2)).astype(np.float32)  # (32,)
        self.cos_vert, self.sin_vert = np.cos(vert), np.sin(vert)

        self.pub = self.create_publisher(PointCloud2, self.topic, qos_profile_sensor_data)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 회전당 75패킷이 몰려 들어오므로 수신 버퍼를 넉넉히. 작으면 조용히 유실된다.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(self.rcvbuf))
        self.sock.settimeout(1.0)
        self.sock.bind((self.bind_ip, int(self.port)))

        self.revolutions = 0
        self.points = 0
        self.dropped = 0
        self.last_source = None
        self.warned_dual = False
        self.running = True
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.create_timer(3.0, self.report)
        self.get_logger().info(
            f'VLP-16 UDP :{self.port} → {self.topic} (frame={self.frame_id}) 수신 대기')

    # ------------------------------------------------------------------
    # 수신 스레드: 한 바퀴 모아서 발행
    # ------------------------------------------------------------------

    def receive(self) -> None:
        packets, prev_az = [], None
        while self.running:
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != PACKET_SIZE:
                self.dropped += 1
                continue
            self.last_source = source

            az = int.from_bytes(data[2:4], 'little')
            if prev_az is not None and az < prev_az and packets:
                # 방위각이 되감겼다 = 한 바퀴 완료
                try:
                    self.publish(packets)
                except Exception as exc:     # 한 프레임 실패로 노드가 죽으면 안 된다
                    self.get_logger().error(f'디코딩 실패: {exc}')
                packets = []
            prev_az = az
            packets.append(data)
            if len(packets) > 400:           # 되감기를 놓친 경우의 안전장치
                packets = []

    def publish(self, packets: list) -> None:
        stamp = self.get_clock().now().to_msg()
        buf = np.frombuffer(b''.join(packets), dtype=np.uint8).reshape(len(packets), PACKET_SIZE)

        if not self.warned_dual and buf[0, -2] == 0x39:
            self.warned_dual = True
            self.get_logger().warn('Dual return 모드입니다. 같은 방향이 두 번 들어와 점이 2배가 됩니다.')

        blocks = buf[:, :BLOCKS * 100].reshape(-1, BLOCKS, 100)
        # 방위각 (0.01도) — 블록 헤더 2byte. copy() 후 view 해야 정렬 문제가 없다.
        az = (blocks[:, :, 2:4].copy().view(np.uint16).reshape(-1, BLOCKS).astype(np.float32)
              * 0.01)
        body = blocks[:, :, 4:].reshape(-1, BLOCKS, CHANNELS, 3)
        dist = (body[:, :, :, :2].copy().view(np.uint16).reshape(-1, BLOCKS, CHANNELS)
                .astype(np.float32) * 0.002)
        refl = body[:, :, :, 2].astype(np.float32)

        # 블록 사이 방위각 증가분으로 채널별 방위각을 보간한다.
        # 이걸 안 하면 한 블록 안 32개 점이 모두 같은 각도로 찍혀 물체가 계단처럼 어긋난다.
        flat_az = az.reshape(-1)
        delta = np.diff(flat_az, append=flat_az[-1])
        delta[delta < 0] += 360.0                    # 0도 통과 구간
        if delta.size > 1:
            delta[-1] = delta[-2]                    # 마지막 블록은 직전 증가분으로 외삽
        azimuth = flat_az[:, None] + delta[:, None] * self.frac[None, :]
        rad = np.deg2rad(np.mod(azimuth, 360.0)).astype(np.float32)

        d = dist.reshape(-1, CHANNELS)
        r = d * self.cos_vert[None, :]
        x = r * np.cos(rad)
        y = -r * np.sin(rad)                         # +y = 좌측 (REP-103)
        z = d * self.sin_vert[None, :]

        keep = (d > self.min_range) & (d < self.max_range)
        cloud = np.stack([x[keep], y[keep], z[keep], refl.reshape(-1, CHANNELS)[keep]], axis=1)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = int(cloud.shape[0])
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = cloud.astype(np.float32).tobytes()
        self.pub.publish(msg)

        self.revolutions += 1
        self.points = int(cloud.shape[0])

    def report(self) -> None:
        if self.revolutions == 0:
            src = '아직 패킷 없음'
            self.get_logger().warn(
                f'{src} — MORAI 센서 Network Setting 의 Destination Port 가 '
                f'{self.port} 인지, Connect 되어 있는지 확인하세요.')
            return
        hz = self.revolutions / 3.0
        self.get_logger().info(
            f'{hz:.1f} Hz | 회전당 {self.points} 점 | 송신 {self.last_source[0]}:'
            f'{self.last_source[1]}' + (f' | 비정상 패킷 {self.dropped}' if self.dropped else ''))
        self.revolutions = 0

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
