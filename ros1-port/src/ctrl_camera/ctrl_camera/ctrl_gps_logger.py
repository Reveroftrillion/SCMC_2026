#!/usr/bin/env python3
"""GPS 지점 기록기 — 터미널에서 키를 누르면 그 순간의 GPS 위경도를 CSV 한 줄로 저장한다.

    ros2 run ctrl_camera gps_logger
    ros2 launch ctrl_camera ctrl_gps_record.launch.py        # 주행 스택과 같이 (보통 이쪽)

`/sensors/gps` (NavSatFix, sensor_bridge 발행) 의 마지막 유효 fix 를 들고 있다가 키가 들어오면
`~/foscar_logs/gps_<시각>.csv` 에 `time,lat,lon,alt,x,y` 로 덧붙인다. 파일은 첫 저장 때 만든다
(한 번도 안 누르면 빈 파일도 안 남는다).

x, y 는 map 절대좌표 [m] — `ctrl_gps.py` / `ctrl_localization.py` 와 같은 식이다:
WGS84(EPSG:4326) → UTM 52N(EPSG:32652) 로 투영한 뒤 map 원점(`origin_east`, `origin_north`,
기본값은 `low_speed.yaml` 의 localization 값) 을 뺀다. `routes/global_path.txt` 의 x y 와 같은 좌표계.

키
    스페이스 / Enter / 그 외 아무 키     현재 GPS 저장
    q                                 종료 (launch 안에서는 전체 종료)

`ros2 launch` 는 자식의 stdin 을 pty/pipe 로 바꾸므로 `sys.stdin` 으로는 터미널 키가 안 온다.
대신 자식도 같은 제어 터미널을 쓰기 때문에 **`/dev/tty` 를 직접 열어** 읽는다 — launch 를 띄운
터미널 창에 포커스를 두고 키를 누르면 된다. 읽는 동안 그 터미널은 cbreak 모드(에코 끔,
줄 버퍼링 끔)가 되고 종료 때 되돌린다. Ctrl-C 는 그대로 동작한다.
"""

import csv
import math
import os
import termios
import threading
import tty
from datetime import datetime
from pathlib import Path

from pyproj import Transformer
import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix

FIELDS = ('time', 'lat', 'lon', 'alt', 'x', 'y')


class GpsLogger(Node):

    def __init__(self):
        super().__init__('gps_logger')
        p = self.declare_parameter
        self.topic = p('gps_topic', '/sensors/gps').value
        self.save_dir = Path(p('save_dir', str(Path.home() / 'foscar_logs')).value).expanduser()
        self.timeout = float(p('sensor_timeout', 0.5).value)   # 이보다 오래된 fix 는 저장 거부
        # map 원점 (UTM 52N, m). ctrl_localization.py 기본값과 같다 — 바꾸면 경로 파일 좌표계와 어긋난다.
        self.origin = (float(p('origin_east', 302595.0).value), float(p('origin_north', 4124145.0).value))
        self.transformer = Transformer.from_crs(4326, 32652, always_xy=True)   # WGS84 → UTM 52N

        self.fix = None                      # 마지막 유효 NavSatFix
        self.fix_at = None                   # 그 수신 시각 (rclpy Time)
        self.path = self.save_dir / f'gps_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.count = 0
        self.lock = threading.Lock()
        self.create_subscription(NavSatFix, self.topic, self.on_gps, qos_profile_sensor_data)
        self.get_logger().info(f'{self.topic} → {self.path} | 아무 키=저장 q=종료 (launch 터미널에서)')

    def latlon_to_map(self, lat: float, lon: float):
        """WGS84 위경도를 map 원점 기준 (x, y)[m] 로 변환한다 (ctrl_gps.py 와 같은 식)."""
        east, north = self.transformer.transform(lon, lat)
        return east - self.origin[0], north - self.origin[1]

    def on_gps(self, msg: NavSatFix) -> None:
        # ctrl_bridge.py 는 fix 가 없거나 GPS 음영이면 status<0 / NaN 으로 보낸다 → 유효한 것만 기억
        if msg.status.status < 0 or not all(math.isfinite(v) for v in (msg.latitude, msg.longitude)):
            return
        with self.lock:
            self.fix, self.fix_at = msg, self.get_clock().now()

    def save(self) -> None:
        with self.lock:
            fix, at = self.fix, self.fix_at
        if fix is None:
            self.get_logger().warn('저장 안 함: 아직 유효한 GPS 가 없습니다')
            return
        age = (self.get_clock().now() - at).nanoseconds / 1e9
        if age > self.timeout:
            self.get_logger().warn(f'저장 안 함: 마지막 GPS 가 {age:.1f} s 전 (음영 구간?)')
            return
        self.save_dir.mkdir(parents=True, exist_ok=True)
        x, y = self.latlon_to_map(fix.latitude, fix.longitude)
        new = not self.path.exists()
        with self.path.open('a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(FIELDS)
            w.writerow([datetime.now().isoformat(timespec='milliseconds'),
                        f'{fix.latitude:.8f}', f'{fix.longitude:.8f}', f'{fix.altitude:.3f}',
                        f'{x:.3f}', f'{y:.3f}'])
        self.count += 1
        self.get_logger().info(f'저장 #{self.count}: {fix.latitude:.7f}, {fix.longitude:.7f} '
                               f'→ x={x:.2f} y={y:.2f} → {self.path.name}')


class TerminalKeys:
    """`/dev/tty` 를 cbreak 로 열고 한 글자씩 읽어 콜백에 넘긴다. 종료 때 터미널 설정을 복원한다."""

    def __init__(self, on_key, logger):
        self.on_key, self.logger = on_key, logger
        self.fd = self.saved = None
        self.thread = None

    def start(self) -> bool:
        try:
            self.fd = os.open('/dev/tty', os.O_RDONLY)
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        except OSError as e:
            self.logger.error(f'터미널을 열 수 없어 키 입력 없이 동작합니다: {e}')
            return False
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        return True

    def loop(self) -> None:
        while True:
            try:
                ch = os.read(self.fd, 1)
            except OSError:
                return
            if not ch:
                return
            self.on_key(ch.decode(errors='ignore'))

    def stop(self) -> None:
        if self.fd is not None and self.saved is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            except termios.error:
                pass
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    quit_event = threading.Event()

    def on_key(ch: str) -> None:
        if ch in ('q', 'Q', '\x03'):     # q 또는 Ctrl-C 바이트
            quit_event.set()
        else:
            node.save()

    keys = TerminalKeys(on_key, node.get_logger())
    keys.start()
    try:
        while rclpy.ok() and not quit_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        node.get_logger().info(f'종료: {node.count}개 저장' + (f' → {node.path}' if node.count else ''))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
