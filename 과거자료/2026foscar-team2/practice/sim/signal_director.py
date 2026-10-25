#!/usr/bin/env python3
"""Practice only: force every camera signal red, release it 5 s after the car stops at the line.

    python3 practice/sim/signal_director.py --out <run dir>/signals.jsonl

Uses the MORAI gRPC traffic-light API (forbidden in src/, allowed in practice/). Runs across a whole
loop_runner batch: a jump back in s starts a new lap. Per intersection it records where the car stopped,
whether it passed the line on red, and how long it took to move after the go colour.

    idle --(s >= start_s - arm_m)--> red --(stood still near the line for hold_s)--> go --(s > line + 10)--> done
    red: past the line without stopping = RAN_RED (go colour is then given at once)
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from morai_rpc import Morai

SIGNALS = {'intersection_01': 'C1256W000077', 'intersection_02': 'C1256W000091',
           'intersection_03': 'C1256W000018', 'intersection_04': 'C1256W000052'}
# 03 is a group of heads: set one alone and the head camera 7 sees keeps its own cycle (collection
# third_intersection.json same_state; 2026-09-29 matrix: the car saw green_left there and drove on).
SIBLINGS = {'intersection_03'}
RED, GREEN, LEFT = 1, 16, 32
ROOT = Path(__file__).resolve().parents[2]


class Director(Node):
    def __init__(self, args):
        super().__init__('signal_director')
        self.args = args
        self.api = Morai(args.grpc)
        lines = json.loads((ROOT / 'src/foscar_camera/config/yolo_stoplines.json').read_text())['lines']
        self.lines = sorted(lines, key=lambda x: x['stop_rear_s'])
        self.out = open(args.out, 'a')
        self.s = None
        self.v = 0.
        self.last_s = None
        self.lap = 0
        self.reset()
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_timer(.1, self.tick)
        self.last_send = 0.

    def reset(self):
        self.state = {x['id']: dict(phase='idle') for x in self.lines}

    def on_s(self, msg):
        s = float(msg.data)
        if self.last_s is not None and s < self.last_s - 50.:
            self.finish_lap()
        self.last_s = self.s = s

    def on_odom(self, msg):
        self.v = msg.twist.twist.linear.x

    def record(self, **row):
        row.update(lap=self.lap, wall=time.time())
        self.out.write(json.dumps(row) + '\n')
        self.out.flush()
        self.get_logger().info(json.dumps(row))

    def finish_lap(self):
        for key, st in self.state.items():
            if st['phase'] in ('red', 'go'):
                self.record(event='INCOMPLETE', intersection=key, phase=st['phase'])
        self.lap += 1
        self.reset()

    def go_colour(self, line):
        return LEFT if line['maneuver'] == 'left' else GREEN

    def send(self, key, colour):
        try:
            self.api.set_color(SIGNALS[key], colour, siblings=key in SIBLINGS)
        except Exception as exc:          # a missed impulse is re-sent on the next tick
            self.get_logger().warning(f'set {key} {colour}: {exc}')

    def tick(self):
        if self.s is None:
            return
        now = time.monotonic()
        resend = now - self.last_send >= .2
        for line in self.lines:
            key, st = line['id'], self.state[line['id']]
            stop = line['stop_rear_s']
            if st['phase'] == 'idle':
                if line['start_s'] - self.args.arm_m <= self.s < stop - 1.:
                    st.update(phase='red', red_at=now, red_s=self.s)
                    self.send(key, RED)
                    self.record(event='RED', intersection=key, s=self.s, v_kmh=self.v * 3.6)
            elif st['phase'] == 'red':
                if resend:
                    self.send(key, RED)
                near = abs(self.s - stop) <= self.args.near_m
                if abs(self.v) < .05 and near:
                    st.setdefault('stood_at', now)
                    st.setdefault('stood_s', self.s)
                    if now - st['stood_at'] >= self.args.hold_s:
                        st.update(phase='go', go_at=now)
                        self.send(key, self.go_colour(line))
                        self.record(event='GO', intersection=key, stop_s=st['stood_s'],
                                    stop_error_m=st['stood_s'] - stop, waited_s=now - st['red_at'])
                else:
                    st.pop('stood_at', None)
                    st.pop('stood_s', None)
                if st['phase'] == 'red' and self.s > stop + 1.:
                    self.record(event='RAN_RED', intersection=key, s=self.s, v_kmh=self.v * 3.6)
                    st.update(phase='go', go_at=now, ran=True)
                    self.send(key, self.go_colour(line))
            elif st['phase'] == 'go':
                if resend:
                    self.send(key, self.go_colour(line))
                if 'moved_at' not in st and self.v > .5:
                    st['moved_at'] = now
                if self.s > stop + 10.:
                    st['phase'] = 'done'
                    if not st.get('ran'):
                        self.record(event='DEPARTED', intersection=key,
                                    move_delay_s=st['moved_at'] - st['go_at'] if 'moved_at' in st else None,
                                    clear_line_s=now - st['go_at'])
        if resend:
            self.last_send = now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--grpc', default='192.168.0.11:7789')
    ap.add_argument('--arm-m', type=float, default=60., help='turn red this far before the camera window')
    ap.add_argument('--near-m', type=float, default=5., help='a stop within this of the line counts')
    ap.add_argument('--hold-s', type=float, default=5.)
    args = ap.parse_args()
    rclpy.init()
    node = Director(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish_lap()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
