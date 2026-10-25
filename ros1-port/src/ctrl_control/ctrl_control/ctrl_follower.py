from dataclasses import replace
import csv
import json
from datetime import datetime
import math
from pathlib import Path as FilePath
import time

import numpy as np
import ctrl_ros1 as rclpy
from ctrl_ros1.ctrl_executors import ExternalShutdownException
from ctrl_ros1.ctrl_node import Node
from ctrl_ros1.ctrl_qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from ctrl_interfaces.msg import ControlCommand, DrivingTarget, VehicleStatus

from .ctrl_core import Controller, Output, Route, Target, quaternion_yaw, select_route, wrap
from .ctrl_route_start import start_station_hint


from .ctrl_telemetry import control_telemetry
from .ctrl_reference import ReferenceLine
from .ctrl_speed_policy import uncapped_highway_zones


class Follower(Node):
    """Tracker: follows the route at the curvature-limited speed and obeys /behavior/target.

    MISSION HOOK — this node is the only thing the FSM / perception teams need to drive:
      publish ctrl_interfaces/DrivingTarget on /behavior/target at 20 Hz
        v_max      -> speed cap [m/s]        (0 = stop)
        stop_s     -> stop front wheels here (NaN = none)
        lat_offset -> drive left(+)/right(-) of the route [m]; blended in over an s-curve of
                      max(transition_min_m, v*transition_time_s) (return_time_s when going back to 0)
      /control/path shows the reference line actually being tracked (route + offset curve).
      `require_target: true` makes the tracker brake when no target arrives for target_timeout s.
      It is false until a behavior node exists, so the tracker cruises on its own.
    Everything else (steering, curvature profile, safety FAULTs, handshake) stays here on purpose."""
    def a_lat_zones(self):
        # Flat [begin_s, end_s, a_lat, ...] (ROS parameters cannot nest); zeros mean none.
        flat=[float(v) for v in self.p['a_lat_zones']]
        if len(flat)%3:raise ValueError('a_lat_zones needs begin, end, a_lat triples')
        return tuple(tuple(flat[i:i+3]) for i in range(0,len(flat),3) if flat[i+2]>0)

    def brake_steer_zones(self):
        # Flat [begin_s, end_s, ...] where braking lowers the path steering (zeros mean none)
        flat=[float(v) for v in self.p['brake_steer_zones']]
        if len(flat)%2:raise ValueError('brake_steer_zones needs begin, end pairs')
        return tuple(tuple(flat[i:i+2]) for i in range(0,len(flat),2) if flat[i+1]>flat[i])

    def emergency_brake_zones(self):
        # Flat [begin_s, end_s, ...] where the brake goes past 0.5 when far over the target, as on the highway's
        # references (zeros mean none)
        flat=[float(v) for v in self.p['emergency_brake_zones']]
        if len(flat)%2:raise ValueError('emergency_brake_zones needs begin, end pairs')
        return tuple(tuple(flat[i:i+2]) for i in range(0,len(flat),2) if flat[i+1]>flat[i])

    def boost_zones(self):
        # Flat [begin_s, end_s, ...]: stretches driven up to the profile at entry_accel_max (zeros mean none)
        flat=[float(v) for v in self.p['boost_zones']]
        if len(flat)%2:raise ValueError('boost_zones needs begin, end pairs')
        return tuple(tuple(flat[i:i+2]) for i in range(0,len(flat),2) if flat[i+1]>flat[i])

    def with_runout(self, route):
        # The finish only needs the car within 3 m of END; nothing says stop there. On the closed
        # competition loop (END == START) continue onto the first metres of the lap so the car
        # crosses END at speed and stops afterwards (2026-09-27: it braked ~100 m early).
        runout = float(self.p['finish_runout_m'])
        g = self.global_points[:, :2]
        if not self.p['continuous'] or runout <= 0 or np.hypot(*(g[-1]-g[0])) > 1.:
            return route
        head = Route(g)
        extra = head.points[(head.s > .5) & (head.s <= runout)]
        return Route(np.vstack([route.points, extra])) if len(extra) else route

    def __init__(self):
        super().__init__('path_follower')
        # route: 'auto' picks global/spawn from where the car is at launch; s is always global-route s.
        defaults = dict(path_file='', spawn_file='', route='auto', speed_kmh=5., speed_policy_file='', test_distance_m=20.,
                        lookahead_m=3., continuous=False, start_at_beginning=False,
                        require_target=False, target_timeout=0.5,
                        max_cross_track_m=1.5, sensor_timeout=1.5, a_lat_mps2=2.5, decel_mps2=1.5,
                        dr_speed_kmh=0., hold_brake=0.15, transition_time_s=1.5, return_time_s=2.0, transition_min_m=6.,
                        accel_max=0.4, entry_accel_max=0.6, a_lat_zones=[0.,0.,0.], boost_zones=[0.,0.], emergency_brake_zones=[0.,0.], finish_runout_m=0., finish_decel_mps2=1.0, highway_decel_mps2=0., exit_decel_mps2=0., exit_decel_m=0., brake_steer=0., brake_steer_zones=[0.,0.], accel_rise_per_s=3.0, lookahead_gain=1.0, lookahead_max_m=10., accel_mps2=1.0, reference_accel_mps2=0., reference_lookahead_max_m=0., steer_predict_s=0., reference_lateral_ki=-1., reference_steer_predict_s=-1., steer_tau=0.1,
                        steer_kick_deg=0., steer_kick_at_s=0., steer_kick_duration_s=0.3, lateral_ki=0.03,
                        sensor_mount_confirmed=False, log_dir=str(FilePath.home()/'foscar_logs'), shadow_config_file='')
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.uncapped_zones=uncapped_highway_zones(self.p['speed_policy_file'])
        self.global_points = np.loadtxt(self.p['path_file'])
        self.spawn_points = np.loadtxt(self.p['spawn_file']) if self.p['spawn_file'] else self.global_points
        self.route = self.controller = None      # built on the first pose (route:=auto needs it)
        if self.p['route'] != 'auto' and not self.p['start_at_beginning']:
            self.build_controller()
        self.odom = self.status = None
        self.arrivals = dict(odom=0., status=0.)
        self.previous_feedback = None
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, qos_profile_sensor_data)
        self.target, self.target_arrival = None, 0.
        self.target_stamp=-math.inf
        self.create_subscription(DrivingTarget, '/behavior/target', self.on_target, 1)
        # 'GPS' | 'DR' from localization. Optional DR cap; disabled by default.
        self.loc_mode = 'GPS'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/localization/mode', self.on_mode, latched)
        self.command_pub = self.create_publisher(ControlCommand, '/control/request', 1)
        self.state_pub = self.create_publisher(String, '/control/state', 1)
        self.telemetry_pub = self.create_publisher(String, '/control/telemetry', 1)
        self.target_pub = self.create_publisher(PointStamped, '/control/target', 1)
        self.path_pub = self.create_publisher(Path, '/control/path', 1)     # 지금 따라가는 기준선 (오프셋 곡선 포함)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(Path, '/map/route', qos)
        if self.route is not None:
            self.publish_route()
        directory = FilePath(self.p['log_dir']).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.log_path = directory/('follow_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.csv')
        self.log = self.log_path.open('w', newline='')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['elapsed_s','state','reason','x','y','yaw_rad','speed_mps','s_m','cross_track_m',
                              'target_speed_mps','accel','brake','steering_rad'])
        self.start = self.previous_tick = time.monotonic()
        self.last_report = 0.
        self.previous_state = ''
        self.handshake_start = None
        self.shadow_recorder = None
        self.shadow_failed = False
        self.shadow_request_stamp = None
        self.shadow_request = None
        self.shadow_request_error = ''
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'log={self.log_path}')

    def build_controller(self, x=None, y=None):
        if self.p['route'] == 'global':
            self.route, s_offset = Route(self.global_points), 0.
        elif self.p['route'] == 'spawn':
            self.route = Route(self.spawn_points)
            s_offset = float(self.route.s[-1]-Route(self.global_points).s[-1])
        else:
            self.route, s_offset = select_route(x, y, self.global_points, self.spawn_points)
        hint = start_station_hint(self.route, x, y, self.p['start_at_beginning'],
                                  FilePath(self.p['path_file']).with_name('checkpoints.txt'))
        self.route = self.with_runout(self.route)
        self.controller = Controller(self.route, self.p['speed_kmh'], self.p['test_distance_m'],
                                     self.p['lookahead_m'], self.p['max_cross_track_m'], self.p['continuous'],
                                     self.p['a_lat_mps2'], self.p['decel_mps2'], self.p['accel_max'],
                                     self.p['lookahead_gain'], self.p['lookahead_max_m'], self.p['accel_mps2'],
                                     self.p['steer_tau'], self.p['lateral_ki'], s_offset, self.p['hold_brake'],
                                     self.p['transition_time_s'], self.p['return_time_s'], self.p['transition_min_m'],self.p['entry_accel_max'],self.p['accel_rise_per_s'],
                                     uncapped_zones=self.uncapped_zones,a_lat_zones=self.a_lat_zones(),boost_zones=self.boost_zones(),
                                     finish_decel=self.p['finish_decel_mps2'],
                                     highway_decel=self.p['highway_decel_mps2'] or None,exit_decel=self.p['exit_decel_mps2'] or None,exit_decel_m=self.p['exit_decel_m'],
                                     brake_steer=self.p['brake_steer'],brake_steer_zones=self.brake_steer_zones(),
                                     emergency_brake_zones=self.emergency_brake_zones(),
                                     reference_a_acc=self.p['reference_accel_mps2'] or None,
                                     reference_lookahead_max=self.p['reference_lookahead_max_m'] or None,
                                     steer_predict_s=self.p['steer_predict_s'],
                                     reference_lateral_ki=None if self.p['reference_lateral_ki'] < 0 else self.p['reference_lateral_ki'],
                                     reference_steer_predict_s=None if self.p['reference_steer_predict_s'] < 0 else self.p['reference_steer_predict_s'])
        self.controller.previous = hint
        profile = self.controller.profile
        self.get_logger().info(f'route {len(self.route.points)} points / {self.route.s[-1]:.1f}m, global-s offset '
                               f'{s_offset:.1f}m; profile {profile.min()*3.6:.1f}-{profile.max()*3.6:.1f} km/h')

    def on_odom(self, msg):
        if self.fresh(msg):
            self.odom, self.arrivals['odom'] = msg, time.monotonic()

    def on_status(self, msg):
        if self.fresh(msg):
            self.status, self.arrivals['status'] = msg, time.monotonic()

    def on_target(self, msg):
        # v_max: finite or +inf (no cap); stop_s: finite or NaN (no stop); lat_offset: finite.
        preview_ok = (math.isfinite(msg.return_length_m) and msg.return_length_m >= 0
                      and (msg.return_length_m == 0 or math.isfinite(msg.return_s))
                      and math.isfinite(msg.entry_length_m) and msg.entry_length_m >= 0
                      and (msg.entry_length_m == 0 or math.isfinite(msg.entry_s)))
        chain_d, chain_s, chain_l = (list(getattr(msg, k, [])) for k in ('chain_offset', 'chain_start_s', 'chain_length_m'))
        chain_ok = (len(chain_d) == len(chain_s) == len(chain_l)
                    and all(math.isfinite(v) for v in chain_d + chain_s + chain_l) and all(v > 0 for v in chain_l))
        ok = (preview_ok and chain_ok and not math.isnan(msg.v_max) and msg.v_max >= 0 and not math.isinf(msg.stop_s)
              and math.isfinite(msg.lat_offset))
        stamp=msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9
        now=self.get_clock().now().nanoseconds*1e-9 if hasattr(self,'get_clock') else None
        if now is not None:
            if now<getattr(self,'target_clock',now):
                self.target_stamp=-math.inf
                self.target=replace(self.target,v_max=0.,state='CLOCK_RESET') if self.target is not None else None
            self.target_clock=now
        if stamp<=self.target_stamp or not self.fresh(msg):return
        reference=None
        release=getattr(msg,'reference_release',False)
        try:
            key=getattr(msg,'reference_id','')
            xs=list(getattr(msg,'reference_x',[]));ys=list(getattr(msg,'reference_y',[]));gs=list(getattr(msg,'reference_global_s',[]))
            if key or xs or ys or gs:
                if msg.header.frame_id!='map' or release or msg.lat_offset or msg.entry_length_m or msg.return_length_m:raise ValueError('reference mode conflict')
                old=self.target.reference if self.target is not None else None
                if old is not None and key==old.id:
                    if not (np.array_equal(old.global_s,gs) and np.array_equal(old.xy[:,0],xs) and np.array_equal(old.xy[:,1],ys)):raise ValueError('reference ID mutation')
                    reference=old
                else:reference=ReferenceLine(key,gs,np.c_[xs,ys])
        except (ValueError,TypeError,IndexError):ok=False
        if ok:
            self.target = Target(v_max=msg.v_max, stop_s=msg.stop_s, lat_offset=msg.lat_offset, state=msg.state,
                                 return_s=msg.return_s, return_length_m=msg.return_length_m,
                                 entry_s=msg.entry_s, entry_length_m=msg.entry_length_m,
                                 chain=tuple(zip(chain_d, chain_s, chain_l)),
                                 reference=reference,reference_release=release)
        else:
            self.target=replace(self.target,v_max=0.,state='INVALID_TARGET') if self.target is not None else Target(v_max=0.,state='INVALID_TARGET')
        self.target_stamp=stamp
        self.target_arrival = time.monotonic()

    def on_mode(self, msg):
        if msg.data != self.loc_mode:
            self.get_logger().warn(f'localization {self.loc_mode} → {msg.data}'
                                   + (f' (속도 상한 {self.p["dr_speed_kmh"]:.0f} km/h)' if msg.data == 'DR' and self.p['dr_speed_kmh'] > 0 else ''))
        self.loc_mode = msg.data

    def fresh(self, msg):
        age = (self.get_clock().now().nanoseconds-(msg.header.stamp.secs*10**9+msg.header.stamp.nsecs))/1e9
        return -0.05 <= age <= self.p['sensor_timeout']

    def publish_route(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.route.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            msg.poses.append(pose)
        self.route_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        dt = now-self.previous_tick
        self.previous_tick = now
        healthy = (self.odom is not None and self.status is not None
                   and all(now-t < self.p['sensor_timeout'] for t in self.arrivals.values()))
        x = y = yaw = speed = 0.
        mode = gear = 0
        if self.odom is not None:
            pose = self.odom.pose.pose
            x, y = pose.position.x, pose.position.y
            q = pose.orientation
            yaw = quaternion_yaw(q.w, q.x, q.y, q.z)
            speed = self.odom.twist.twist.linear.x
        if self.status is not None:
            mode, gear = self.status.ctrl_mode, self.status.gear
        if self.controller is None:
            if not healthy:
                output = Output('WAITING', 'waiting for a pose to pick the route')
            else:
                try:
                    self.build_controller(x, y)
                    self.publish_route()
                except ValueError as exc:
                    self.get_logger().error(f'{exc}: move the car onto the route or the respawn connector')
                    output = Output('FAULT', str(exc))
            if self.controller is None:
                self.publish_output(output, now, x, y, yaw, speed)
                return
        if healthy and abs(wrap(yaw-self.status.yaw_rad)) > math.radians(10):
            # Not a stop any more (user 2026-09-29: a stopped car cannot be revived in the competition):
            # drive on the localization yaw, as in dead reckoning.
            self.get_logger().warn('IMU/status yaw disagree >10deg: driving on the localization yaw',
                                   throttle_duration_sec=2.)
        if not self.p['sensor_mount_confirmed']:
            output = Output('WAITING', 'sensor mounting must be confirmed in config')
            if healthy:
                output.progress, output.error, _ = self.route.project(x, y)
        elif dt > 0.25 and self.controller.state == 'CRUISE':
            # Our own loop hiccup (camera launch, CPU spike): skip this tick instead of ending the run.
            # The bridge already brakes on its own if commands stop for command_timeout.
            self.get_logger().warn(f'control loop stalled {dt:.2f}s — skipping one tick')
            # Ordinary cruise coasts after a missed control interval; steering/brake stay held.
            c = self.controller
            if c.cruise_throttle_active:
                c.previous_accel = 0.
                c.cruise_previous_speed = None
            output = Output('CRUISE', f'control loop stalled {dt:.2f}s: holding steering and brake', c.previous_accel,
                            c.previous_brake, c.steering, progress=(c.previous or 0.)-c.s_offset)
            self.controller.since_ok += dt
        else:
            # A decision late by up to the dropout hold (1 s) is still the current one: without it a highway
            # reference read as missing and the car was stopped (user 2026-09-28).
            target = self.target if now-self.target_arrival <= self.p['target_timeout']+self.controller.dropout_hold_s else None
            if target is None and self.p['require_target'] and self.controller.state == 'CRUISE':
                # FSM went silent: hold the car (not a FAULT, resumes when targets return).
                target = Target(v_max=0., state='NO_TARGET')
            if self.loc_mode == 'DR' and self.p['dr_speed_kmh'] > 0:
                target = target or Target()
                target = replace(target,v_max=min(target.v_max,self.p['dr_speed_kmh']/3.6),state=target.state or 'DR')
            feedback = (self.arrivals['odom'], self.arrivals['status'])
            fresh_speed = self.previous_feedback is None or all(a > b for a, b in zip(feedback, self.previous_feedback))
            if fresh_speed:
                self.previous_feedback = feedback
            output = self.controller.step(x, y, yaw, speed, mode, gear, healthy, dt, target, fresh_speed=fresh_speed)
        if output.state == 'HANDSHAKE':
            if self.handshake_start is None:
                self.handshake_start = now
            elif now-self.handshake_start > 5:
                # Keep asking (user 2026-09-29: never give the run up).
                self.get_logger().warn('Auto/D handshake not answered for 5 s: still requesting',
                                       throttle_duration_sec=5.)
                self.handshake_start = now
        self.publish_output(output, now, x, y, yaw, speed)

    def publish_output(self, output, now, x, y, yaw, speed):
        msg = ControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.accel, msg.brake, msg.steering_rad, msg.state = output.accel, output.brake, output.steering, output.state
        # Test-only disturbance (steer_kick_deg 0 = off): once past steer_kick_at_s, add a steering step for
        # steer_kick_duration_s to what is sent, outside the controller, and see how the car recovers (user
        # 2026-10-01: without traffic nothing excites the high-speed sway).
        if self.p['steer_kick_deg'] and output.state == 'CRUISE':
            if getattr(self, 'kick_start', None) is None and output.progress >= self.p['steer_kick_at_s']:
                self.kick_start = now
                self.get_logger().warn(f'steering kick {self.p["steer_kick_deg"]:+.1f} deg at s={output.progress:.1f} '
                                       f'v={speed*3.6:.1f} km/h')
            if getattr(self, 'kick_start', None) is not None and now-self.kick_start < self.p['steer_kick_duration_s']:
                msg.steering_rad += math.radians(self.p['steer_kick_deg'])
        self.command_pub.publish(msg)
        summary = (f'{output.state} | {output.reason} | xy=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg '
                   f'v={speed*3.6:.2f}km/h s={output.progress:.2f}m error={output.error:.2f}m')
        state_msg = String()
        state_msg.data = summary
        self.state_pub.publish(state_msg)
        self.telemetry_pub.publish(String(data=json.dumps(control_telemetry(
            output, self.target if now-self.target_arrival<=self.p['target_timeout'] else None,
            self.get_clock().now().nanoseconds*1e-9, speed, self.loc_mode, self.p['dr_speed_kmh']))))
        target = PointStamped()
        target.header.stamp, target.header.frame_id = msg.header.stamp, 'map'
        target.point.x, target.point.y = output.target_x, output.target_y
        self.target_pub.publish(target)
        # 컨트롤러는 첫 pose 를 받아야 생긴다 — 그 전 틱(WAITING/HANDSHAKE)엔 기준선이 없다.
        if (self.controller is not None and output.state not in ('FAULT', 'FINISH')
                and self.path_pub.get_subscription_count() > 0):
            path = Path()
            path.header.stamp, path.header.frame_id = msg.header.stamp, 'map'
            # Shown ahead for 3 s of travel (15-60 m): the preview scales with speed (user 2026-09-29).
            horizon = min(60., max(15., 3.*abs(speed)))
            for px, py in self.controller.reference_path(output.progress+self.controller.s_offset, horizon=horizon):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(px), float(py)
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            self.path_pub.publish(path)
        self.writer.writerow([now-self.start, output.state, output.reason, x, y, yaw, speed,
                              output.progress, output.error, output.target_speed, output.accel, output.brake, output.steering])
        if now-self.last_report > 1 or output.state != self.previous_state:
            self.get_logger().info(summary)
            self.log.flush()
            self.last_report, self.previous_state = now, output.state

        if self.p.get('shadow_config_file'):
            self.record_shadow(output, msg, x, y, yaw, speed)

    def record_shadow(self, output, command, x, y, yaw, speed):
        if self.shadow_failed or self.controller is None or self.odom is None:
            return
        try:
            from .ctrl_shadow import (ShadowEngine, ShadowFrame, ShadowRecorder, ShadowSettings,
                                      VehiclePose, StationedPath, request_from_target)
            if self.shadow_recorder is None:
                settings = ShadowSettings.from_dict(json.loads(FilePath(self.p['shadow_config_file']).read_text(encoding='utf-8')))
                base = StationedPath.from_global_route('follower-route', 'map', self.route.points,
                                                       global_origin_m=-self.controller.s_offset)
                path = self.log_path.with_suffix('.shadow.jsonl')
                self.shadow_recorder = ShadowRecorder(ShadowEngine(base, settings), path)
                self.get_logger().info(f'shadow comparison log={path}')
            if self.shadow_recorder.error:
                raise RuntimeError(self.shadow_recorder.error)
            if self.target_stamp != self.shadow_request_stamp:
                self.shadow_request_stamp = self.target_stamp
                self.shadow_request_error = ''
                try:
                    self.shadow_request = request_from_target(self.target, self.target_stamp)
                except (TypeError, ValueError, AttributeError, OverflowError):
                    self.shadow_request = None
                    self.shadow_request_error = 'target_snapshot_error'
            error = self.shadow_request_error
            if self.target is not None and self.target.state in ('INVALID_TARGET', 'CLOCK_RESET'):
                error = self.target.state
            stamp = self.odom.header.stamp
            pose = VehiclePose(float(x), float(y), float(yaw), float(speed),
                               stamp.secs + stamp.nsecs * 1e-9, self.odom.header.frame_id)
            self.shadow_recorder.push(ShadowFrame(self.get_clock().now().nanoseconds * 1e-9,
                pose, float(output.progress), float(command.steering_rad), self.shadow_request,
                self.target_stamp, output.state == 'CRUISE', error))
        except Exception as exc:
            self.shadow_failed = True
            self.get_logger().error(f'shadow comparison disabled: {type(exc).__name__}: {exc}')

    def close(self):
        recorder = getattr(self, 'shadow_recorder', None)
        if recorder is not None and not recorder.close():
            self.get_logger().error('shadow log incomplete: ' + str(recorder.error))
        self.log.close()


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
