#!/usr/bin/env python3
"""race_hud_node — 랩·미션 현재기록과 신호인식 상태를 RViz 에 띄우고 최고기록을 저장한다.

    /behavior/mission (JSON), /localization/s, /behavior/signal_stop (JSON) → /hud/markers
    최고기록은 ~/foscar_logs/records.json 에 남고 (새 기록일 때만 덮어씀) 화면에는 안 띄운다:
    기록 게시 스크립트가 GitHub main README 로 올린다.
    표시는 자차(base_link) 옆 글자판 두 개: 기록판, 신호판.
"""
import json
import math

import ctrl_ros1 as rclpy
from rospy import Duration
from ctrl_ros1.ctrl_executors import ExternalShutdownException
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import qos_profile_sensor_data
from ctrl_ros1.ctrl_time import Time
from std_msgs.msg import ColorRGBA, Float64, String
from visualization_msgs.msg import MarkerArray
from ctrl_ros1.ctrl_messages import Marker

from ctrl_control.ctrl_mission_config import load_mission_config
from .ctrl_race_records import IGNORED, RaceRecords

SIGNAL_RGB = {'red': (1., .15, .1), 'yellow': (1., .8, .1), 'green': (.2, 1., .4),
              'left': (.2, 1., .4), 'green_left': (.2, 1., .4), 'red_left': (1., .5, .1)}
DECISION_RGB = {'STOP': (1., .15, .1), 'GO': (.2, 1., .4), 'UNKNOWN': (1., .65, .1), 'OFF': (.6, .6, .6)}


def fmt(seconds):
    if seconds is None or not math.isfinite(seconds):
        return '--'
    m, s = divmod(seconds, 60.)
    return f'{int(m)}:{s:05.2f}' if m >= 1 else f'{s:.2f}s'


class RaceHudNode(Node):
    def __init__(self):
        super().__init__('race_hud_node')
        p = self.declare_parameter
        self.records = RaceRecords(p('records_file', '~/foscar_logs/records.json').value,
                                   float(p('finish_s', 2155.33).value),   # full-lap end: old-path s2165, on the 2026-10-01 user path
                                   code=str(p('code_version', '').value))
        self.order = []
        config = p('mission_config', '').value
        if config:
            try:
                self.order = [z['name'] for z in load_mission_config(config)['zones']]
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().warning(f'mission order unavailable: {exc}')
        self.signal = None
        self.signal_at = -math.inf
        self.pub = self.create_publisher(MarkerArray, '/hud/markers', 1)
        self.create_subscription(String, '/behavior/mission', self.on_mission, 10)
        self.create_subscription(String, '/behavior/signal_stop', self.on_signal, 10)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
        self.create_timer(.2, self.publish)
        best = self.records.best_s('lap')
        self.get_logger().info(f'records {self.records.path} (lap best {fmt(best)})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_s(self, msg):
        before = self.records.best_s('lap')
        self.records.on_s(float(msg.data), self.now())
        if self.records.best_s('lap') != before:
            self.get_logger().info(f'NEW LAP BEST {fmt(self.records.best_s("lap"))}')

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        name = data.get('mission')
        if not isinstance(name, str):
            return
        old = self.records.mission
        self.records.on_mission(name, data.get('transition'), self.now())
        if old != name and old in self.records.last and old in self.records.new_best:
            self.get_logger().info(f'NEW {old} BEST {fmt(self.records.last[old])}')

    def on_signal(self, msg):
        try:
            self.signal = json.loads(msg.data)
            self.signal_at = self.now()
        except ValueError:
            pass

    def text(self, mid, text, rgb, x, y, size):
        m = Marker()
        m.header.frame_id = 'base_link'
        # Zero stamp = latest TF. With 'now' RViz dropped every frame whose base_link TF had not
        # arrived yet, and the boards flickered (user 2026-09-29).
        m.header.stamp = Time().to_msg()
        m.ns, m.id, m.type, m.action = 'race_hud', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 3.
        m.pose.orientation.w = 1.
        m.scale.z = float(size)
        m.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.)
        m.lifetime = Duration(secs=2)
        m.text = text
        return m

    def publish(self):
        now, r = self.now(), self.records
        lap = r.lap_elapsed(now)
        # Best records are not shown here any more: they go to the GitHub main README (user 2026-09-29).
        lines = [f'LAP {fmt(lap)}']
        current = r.mission if r.mission not in IGNORED else None
        names = self.order + [k for k in r.best['missions'] if k not in self.order]
        for name in names:
            if name == current:
                shown = f'> {name} {fmt(r.mission_elapsed(now))}'
            else:
                shown = f'  {name} {fmt(r.last.get(name))}'
            lines.append(shown)
        arr = MarkerArray()
        arr.markers.append(self.text(0, '\n'.join(lines), (.9, .95, 1.), -2., 14., 1.3))

        sig = self.signal if now - self.signal_at <= 1. else None
        if sig is None:
            text, rgb = 'SIGNAL  (no status)', (.6, .6, .6)
        else:
            decision = sig.get('decision') or ('ON' if sig.get('camera') else 'OFF')
            color = sig.get('color')
            age = sig.get('age_s')
            head = f'SIGNAL #{sig.get("intersection") or "-"}  CAM {"ON" if sig.get("camera") else "OFF"}'
            seen = f'{(color or "none").upper()}' + (f'  {age:.1f}s ago' if isinstance(age, (int, float)) else '')
            stop = sig.get('stop_rear_s')
            left = f'  line {stop - sig["rear_s"]:.0f} m' if isinstance(stop, (int, float)) and isinstance(sig.get('rear_s'), (int, float)) else ''
            text = f'{head}\n{seen}\n{decision}{left}'
            rgb = SIGNAL_RGB.get(color, DECISION_RGB.get(decision, (.6, .6, .6))) if sig.get('camera') else (.6, .6, .6)
        arr.markers.append(self.text(1, text, rgb, 6., -10., 1.6))
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = RaceHudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
