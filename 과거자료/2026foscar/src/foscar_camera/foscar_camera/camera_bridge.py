#!/usr/bin/env python3
"""MORAI 카메라 7번 UDP → `/sensors/camera/traffic/compressed` (CompressedImage).

MORAI 는 카메라 한 장을 JPEG 로 압축해 UDP 로 쏜다. 1280×720 JPEG 는 보통 100~200 KB 라
UDP 한 번에 못 실어서 **여러 조각**으로 나눠 보내고, 마지막 조각에만 꼬리 `EI` 를 붙인다.
여기서는 조각을 순서대로 이어붙여 완성된 JPEG 를 그대로 `CompressedImage(format='jpeg')`
에 담아 발행한다 — **디코딩은 하지 않는다.** 받는 쪽(신호 인식 노드)이 필요할 때 풀면 되고,
브리지는 조각을 놓치지 않는 것만 신경 쓰면 된다.

조각 구조 (최대 65000 byte, 작년 `MORAI_UDP_NetworkModule/lib/define/Camera.py` 기준)
    'MOR' 3 byte | sec 4 | nsec 4 | index 4 | size 4 | JPEG 조각 (≤ 64979) | 꼬리 2 byte
    - 꼬리가 b'EI' 인 조각이 마지막 조각이다.
    - `size` 가 조각 길이보다 작으면 뒤는 패딩으로 보고 잘라낸다.
    - 부가 헤더 길이(16)는 MORAI 버전마다 달라진 적이 있어 `aux_header_bytes` 파라미터로 뺐다.
      옛 예제(`camera_udp.py`)는 index 4 byte 만 있어서 4 였다. 영상이 안 풀리면 이 값을 의심.

조각을 하나라도 놓치면 그 장은 JPEG 가 깨지므로 버린다. 시작이 `FF D8`(SOI), 끝이
`FF D9`(EOI) 인지로 확인한다. 20 Hz × 3 조각이라 수신 버퍼를 넉넉히 잡아야 유실이 없다.

카메라가 여러 대면 이 노드를 포트·토픽만 바꿔 여러 개 띄운다 (launch 의 `side_cameras`).

`header.stamp` 는 기본적으로 **수신 시각**(`stamp_source: recv`)이다. `morai` 로 바꾸면 헤더의
sec/nsec 를 그대로 쓴다 — 카메라 두 대를 **같은 시뮬 프레임끼리** 짝지어야 할 때만 필요하다
(조각 유실로 한쪽이 버려지면 수신 시각 기준으론 그 뒤가 한 프레임씩 밀려 짝지어진다).
기본값이 `recv` 라 주행 파이프라인은 무변경이다.
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

MAGIC = b'MOR'
TAIL_END = b'EI'
JPEG_SOI = b'\xff\xd8'
JPEG_EOI = b'\xff\xd9'


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')

        p = self.declare_parameter
        self.bind_ip = p('bind_ip', '0.0.0.0').value
        self.port = int(p('port', 9208).value)                 # MORAI Destination Port
        self.topic = p('topic', '/sensors/camera/traffic/compressed').value
        self.frame_id = p('frame_id', 'camera_7').value
        self.aux_header = int(p('aux_header_bytes', 16).value)  # sec+nsec+index+size
        self.stamp_source = str(p('stamp_source', 'recv').value)   # recv | morai
        if self.stamp_source not in ('recv', 'morai'):
            raise ValueError(f"stamp_source 는 recv|morai: {self.stamp_source!r}")
        self.rcvbuf = int(p('rcvbuf_bytes', 8 * 1024 * 1024).value)
        self.max_image_bytes = int(p('max_image_bytes', 4 * 1024 * 1024).value)

        self.pub = self.create_publisher(CompressedImage, self.topic, qos_profile_sensor_data)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
        self.sock.settimeout(1.0)
        self.sock.bind((self.bind_ip, self.port))

        self.frames = 0
        self.bytes = 0
        self.dropped = 0          # 깨진 JPEG (조각 유실)
        self.bad_packets = 0      # 헤더가 'MOR' 이 아닌 패킷
        self.last_source = None
        self.running = True
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.create_timer(3.0, self.report)
        self.get_logger().info(
            f'카메라 UDP :{self.port} → {self.topic} (frame={self.frame_id}, '
            f'stamp={self.stamp_source}) 수신 대기')

    # ------------------------------------------------------------------
    # 수신 스레드: 조각을 모아 한 장이 되면 발행
    # ------------------------------------------------------------------

    def receive(self) -> None:
        buf = bytearray()
        stamp = None                            # 이 장의 MORAI 스탬프 (sec, nsec)
        body_from = len(MAGIC) + self.aux_header
        while self.running:
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < body_from + 2 or data[:3] != MAGIC:
                self.bad_packets += 1
                continue
            self.last_source = source

            body, tail = data[body_from:-2], data[-2:]
            if self.aux_header >= 16:
                size = int.from_bytes(data[15:19], 'little', signed=True)
                if 0 < size < len(body):       # 고정 길이로 패딩된 조각
                    body = body[:size]

            # 새 장의 첫 조각인데 이전 장이 끝나지 않았다면 이전 장은 조각을 놓친 것
            if body[:2] == JPEG_SOI:
                if buf:
                    self.dropped += 1
                    buf.clear()
                stamp = self.read_stamp(data)
            buf += body

            if tail == TAIL_END:
                if buf[:2] == JPEG_SOI and buf[-2:] == JPEG_EOI:
                    try:
                        self.publish(bytes(buf), stamp)
                    except Exception as exc:       # 종료 중 발행 실패 등으로 스레드가 죽으면 안 된다
                        if self.running:
                            self.get_logger().error(f'발행 실패: {exc}')
                else:
                    self.dropped += 1
                buf.clear()
            elif len(buf) > self.max_image_bytes:  # 'EI' 를 계속 못 받는 경우의 안전장치
                self.dropped += 1
                buf.clear()

    def read_stamp(self, packet: bytes):
        """'MOR' 뒤 sec(3-6)/nsec(7-10). 옛 예제(aux_header 4)엔 없으니 None.

        nsec 가 1e9 를 넘으면 오프셋이 어긋난 것이므로 쓰지 않는다 — 이상한 스탬프로
        짝을 맞추느니 수신 시각으로 떨어지는 편이 낫다.
        """
        if self.aux_header < 16:
            return None
        sec = int.from_bytes(packet[3:7], 'little')
        nsec = int.from_bytes(packet[7:11], 'little')
        return (sec, nsec) if 0 <= nsec < 1_000_000_000 else None

    def publish(self, jpeg: bytes, stamp=None) -> None:
        msg = CompressedImage()
        if self.stamp_source == 'morai' and stamp is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg
        self.pub.publish(msg)
        self.frames += 1
        self.bytes += len(jpeg)

    def report(self) -> None:
        if self.frames == 0:
            hint = (f'헤더가 MOR 이 아닌 패킷 {self.bad_packets} 개 — 다른 센서가 이 포트를 쓰고 있습니다.'
                    if self.bad_packets else
                    f'MORAI 센서 Network Setting 의 Destination Port 가 {self.port} 인지, '
                    f'Connect 되어 있는지 확인하세요.')
            if self.dropped:
                hint = f'조각은 오는데 JPEG 가 안 맞춰집니다 (깨진 장 {self.dropped}). aux_header_bytes 를 의심하세요.'
            self.get_logger().warn(f'아직 영상 없음 — {hint}')
            self.bad_packets = self.dropped = 0
            return
        hz = self.frames / 3.0
        kb = self.bytes / self.frames / 1024
        extra = ''
        if self.dropped:
            extra += f' | 깨진 장 {self.dropped}'
        if self.bad_packets:
            extra += f' | 비정상 패킷 {self.bad_packets}'
        self.get_logger().info(
            f'{hz:.1f} Hz | 장당 {kb:.0f} KB | 송신 {self.last_source[0]}:{self.last_source[1]}' + extra)
        self.frames = self.bytes = self.dropped = self.bad_packets = 0

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
