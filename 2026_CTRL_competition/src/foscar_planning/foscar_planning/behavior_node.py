#!/usr/bin/env python3
"""behavior_node — `/events` + 위치를 보고 `/behavior/target` 을 내는 FSM 코어 (개발계획서 §2).

    ros2 run foscar_planning behavior_node --ros-args -p states:=cruise,avoid,tunnel

`/behavior/target` 의 **유일한** 발행자다. tracker(path_follower) 는 이것 말고 아무것도 안 본다.
어떤 상태를 돌릴지는 `states` 파라미터가 정한다 — 목록에 없는 상태는 존재하지 않는 것과 같다.
(신호등 `stop_line` 은 카메라팀이 `states/stop_line.py` 를 넣고 목록에 이름을 추가하면 붙는다.)

입력
    /events                 foscar_interfaces/Event   인지 노드들. event_timeout_s 지나면 없어진 것으로 본다
    /localization/odometry  nav_msgs/Odometry        자차 x, y, yaw, 속도(twist.linear.x)
    /localization/s         std_msgs/Float64         global s (후륜축)
    /localization/mode      std_msgs/String          'GPS' | 'DR'
    /map/route              nav_msgs/Path (latched)  자차가 탄 경로 (local s). Frenet 변환에 씀
출력
    /behavior/target        foscar_interfaces/DrivingTarget  20 Hz
    /behavior/state         std_msgs/String   걸린 상태들 "AVOID|CRUISE" (priority 순)
    /behavior/markers       visualization_msgs/MarkerArray  후보 오프셋·장애물 박스·선택 경로 (map)

Event 좌표 처리
    frame_id 가 base_link 면 현재 자차 자세로 map 으로 옮긴 뒤 경로에 투영해 (s, d) 를 만든다.
    Event.s 가 있어도 여기서 다시 계산한다 — d 와 같은 투영에서 나와야 박스가 일관된다.
    global s = local s − s_offset. s_offset 은 /localization/s 와 자차 투영의 차이로 처음에 잰다.

시간은 노드 시계(`get_clock`) 초. 상태들은 ctx.now 만 본다.
"""

import importlib
import json
import numpy as np
import inspect
import threading
import math
import os
from dataclasses import replace
import hashlib
from pathlib import Path as FilePath
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from collections import deque
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from foscar_control.core import Route, quaternion_yaw
from foscar_interfaces.msg import DrivingTarget, Event
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, ColorRGBA, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from .frenet import base_to_map, s_offset_from, to_frenet, to_xy
from .avoid_choice import side_summary
from .tunnel_corridor import WallMemory
from .entry_gate import EntryGate
from .brake_shaper import JerkBrake
from .following_departure import STRATEGIES as DEPARTURE_STRATEGIES
from .mission import MissionSupervisor
from .mission_markers import bev_stop_markers, mission_markers, yield_status, checkpoint_markers, required_checkpoint_markers
from .prediction_markers import prediction_markers
from .circle_reference import CircularReference
from .circular_gap import CircularGap
from .ring_merge import RingMerge, ring_bodies
from .roundabout_boxes import FREE, RingLane
from .roundabout_planner import RoundaboutGeometry, RoundaboutPlanner
from .roundabout_markers import roundabout_markers
from .following_departure import FollowingDeparture
from .local_following import evaluate_following, begin_following, refresh_exit_prediction
from .prediction import GapPredictor
from .lane_prediction import LaneMap
from foscar_control.mission_config import geometry_path, load_mission_config, read_checkpoints
from .highway_config import HighwayConfig
from .highway_route import HighwayRoute
from .highway_runtime import HighwayRuntime
from .highway_markers import highway_markers
from .signal_stop import SignalStopPolicy
from .state import event_key, Behavior, BehaviorState, Context, EventInfo, Target


def load_state_class(name: str):
    """`foscar_planning.states.<name>` 안의 BehaviorState 서브클래스 하나를 찾는다.
    `yield` 는 파이썬 예약어라 파일이 `yield_.py` 다 — 이름 뒤에 `_` 를 붙인 모듈도 찾아본다."""
    try:
        module = importlib.import_module(f'foscar_planning.states.{name}')
    except ModuleNotFoundError:
        module = importlib.import_module(f'foscar_planning.states.{name}_')
    found = [c for _, c in inspect.getmembers(module, inspect.isclass)
             if issubclass(c, BehaviorState) and c is not BehaviorState and c.__module__ == module.__name__]
    if len(found) != 1:
        raise ImportError(f'states/{name}.py must define exactly one BehaviorState subclass, found {found}')
    return found[0]


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        p = self.declare_parameter
        # 런치에서 넘기기 쉽게 문자열 "cruise,avoid,tunnel" 로 받는다
        names = [n.strip() for n in str(p('states', 'cruise,avoid,yield,tunnel').value).split(',') if n.strip()]
        self.rate = p('rate_hz', 20.).value
        self.event_timeout = p('event_timeout_s', 0.5).value
        self.odom_timeout = p('odom_timeout_s', 1.0).value
        self.visual_details = p('visualization.details', False).value
        self.control_display = {}
        self.markers_on = p('markers', True).value
        self.marker_horizon = p('marker_horizon_m', 40.).value

        states = []
        for name in names:
            cls = load_state_class(name)
            params = {}
            for key, default in cls.defaults.items():
                params[key] = self.declare_parameter(f'{name}.{key}', default).value
            states.append(cls(**params))
        self.signal_stop = None
        self.signal_debug = self.create_publisher(String, "/behavior/signal_stop", 1)
        # YOLO runs only while this is true: a signal mission owns the car (missions.yaml signal).
        self.signal_camera_pub = self.create_publisher(Bool, '/behavior/signal_camera', 1)
        signal_file = p('traffic.config', '').value
        if bool(p('traffic.enabled', False).value):
            calibration = json.loads(FilePath(signal_file).read_text())
            path_file = p('traffic.path_file', '').value
            if hashlib.sha256(FilePath(path_file).read_bytes()).hexdigest() != calibration['route_sha256']:
                raise ValueError('traffic stop poses require the calibrated global route')
            self.signal_stop = SignalStopPolicy(calibration['lines'])
            self.create_subscription(String, '/perception/traffic_signal', self.on_signal, 10)
        self.behavior = Behavior(states)
        self.mission = None
        mission_enabled = p('mission.enabled', False).value
        mission_file = p('mission.config', '').value
        if mission_enabled:
            import yaml
            if not mission_file:
                raise ValueError('mission.enabled requires mission.config')
            self.mission = MissionSupervisor(load_mission_config(mission_file))
        self.evidence_after = -math.inf
        self.get_logger().info('states: ' + ', '.join(f'{s.name}({s.priority})' for s in self.behavior.states))

        self.odom: Optional[Odometry] = None
        self.pose_history = deque(maxlen=80)   # (stamp, x, y, yaw), ~4 s at 20 Hz
        self.odom_at = None
        self.s_global: Optional[float] = None
        self.loc_mode = 'GPS'
        self.route: Optional[Route] = None
        self.s_offset: Optional[float] = None
        self.events: Dict[tuple, EventInfo] = {}
        self.footprint_stamp = -math.inf
        self.footprints = {}
        self.last_state = ''
        lane_map = None
        lane_file = p('prediction.link_set', str(geometry_path('package://foscar_bringup/maps/link_set.json', None))).value
        if lane_file:
            try:
                lane_map = LaneMap.from_file(lane_file)
                self.get_logger().info(f'shadow prediction: {len(lane_map.links)} directed road links')
            except (OSError,ValueError,KeyError,TypeError) as exc:
                self.get_logger().warn(f'shadow lane map unavailable: {exc}; kinematic diagnostics only')
        self.road_map = lane_map
        control_config=p('prediction.control_config','').value
        highway_file=p('highway.config','').value
        highway_path=p('highway.path_file','').value
        highway_checkpoints=p('highway.checkpoint_file','').value
        self.required_checkpoints={}
        if highway_checkpoints:
            try:self.required_checkpoints=read_checkpoints(highway_checkpoints)
            except (OSError,ValueError) as exc:
                self.get_logger().warn(f'checkpoint display unavailable: {exc}')
        self.tunnel_walls = WallMemory()
        self.tunnel_corridor = None
        self.highway=None;self.highway_job=None;self.highway_from_s=-math.inf
        if highway_file:
            highway_config=HighwayConfig.load(highway_file,control_config=control_config)
            highway_route=None
            try:
                if highway_config.mode!='off':
                    highway_route=HighwayRoute.from_files(lane_file,highway_path,highway_checkpoints,**highway_config.route)
            except (OSError,ValueError,KeyError) as exc:
                self.get_logger().warn(f'highway map unavailable: {exc}')
            bounds=None
            if self.mission is not None and highway_config.mode!='off':
                zone=next((z for z in self.mission.zones if z.name==highway_config.mission),None)
                if zone is None:raise ValueError('highway mission missing')
                # The highway map starts at the checkpoint even when the mission takes over earlier (approach_s).
                bounds=(zone.limit_start_s if zone.limit_start_s is not None else zone.start_s,zone.end_s)
            # Built in the background (user 2026-10-09: ~1.5 s of start-up with the steering already live, the highway
            # is long after the start); joined before the highway's mission starts, at once when we start near it
            def build_highway(cfg=highway_config,route=highway_route,b=bounds):
                try:self.highway=HighwayRuntime(cfg,route,mission_bounds=b)
                except Exception as exc:self.get_logger().error(f'highway runtime failed: {exc!r}')
            self.highway_from_s=bounds[0] if bounds is not None else -math.inf
            self.highway_job=threading.Thread(target=build_highway,daemon=True);self.highway_job.start()
        self.highway_pub=self.create_publisher(String,'/behavior/highway',10)
        self.highway_marker_pub=self.create_publisher(MarkerArray,'/behavior/highway_markers',1) if self.markers_on else None
        self.roundabout_marker_pub=self.create_publisher(MarkerArray,'/behavior/roundabout_markers',1) if self.markers_on else None
        self.roundabout_marker_at=-math.inf

        self.prediction = GapPredictor(
            lane_map=lane_map,
            horizon=p('prediction.horizon_s', 7.).value,
            max_speed=p('prediction.max_speed_kmh', 20.).value / 3.6,
            min_length=p('prediction.vehicle_length_floor_m', 4.8).value,
            min_width=p('prediction.vehicle_width_floor_m', 2.).value)
        self.prediction.departure_profile_limits=tuple(p('prediction.'+name,value).value for name,value in (
            ('a_lat_mps2',2.5),('decel_mps2',1.5),('accel_mps2',1.0)))
        if control_config:
            import yaml
            with open(control_config,encoding='utf-8') as config_file:
                control=yaml.safe_load(config_file) or {}
            controller_params={}
            for name in ('/**','path_follower','/path_follower'):
                controller_params.update(control.get(name,{}).get('ros__parameters',{}))
            self.prediction.departure_profile_limits=tuple(float(controller_params.get(name,default))
                for name,default in zip(('a_lat_mps2','decel_mps2','accel_mps2'),self.prediction.departure_profile_limits))
        self.prediction.departure_cruise_speed=p('prediction.control_speed_kmh',
                                               self.prediction.max_speed*3.6).value/3.6
        self.entry_gate = EntryGate(**{key: p('roundabout.'+key, value).value
                                      for key, value in EntryGate.defaults.items()})
        self.prediction_result = None
        circle_ids=p('prediction.circle_links',
                     'A2256W000132,A2256W000133,A2256W000134,A2256W000135').value
        self.circle_reference=CircularReference.from_lanes(lane_map,
                                     [x.strip() for x in circle_ids.split(',') if x.strip()])
        # the ring's posted limit (map max_speed, km/h): the most an unmeasured ring car is taken to be doing (user
        # 2026-10-09: never an average speed)
        self.ring_limit_mps=math.inf
        try:
            rows={r['idx']:r for r in json.loads(FilePath(lane_file).expanduser().read_text())}
            self.ring_limit_mps=max(float(rows[k]['max_speed']) for k in circle_ids.split(',') if k.strip() in rows)/3.6
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.get_logger().warn(f'ring speed limit unavailable: {exc}')
        self.circular_gap = CircularGap(self.circle_reference,
            cooperative_rear=p('roundabout.cooperative_rear',True).value,
            rear_uncertainty_mps=p('roundabout.rear_uncertainty_mps',.175).value,
            rear_margin_m=p('roundabout.rear_margin_m',.3).value)
        self.circular_gap.local_entry=p('roundabout.local_entry',False).value
        self.entry_gate.ring_speed=self.prediction.max_speed
        self.circular_gap_on = p('roundabout.circular_gap', True).value
        # Entry as speed planning (ring_merge.py, user 2026-10-01). Built on the first route + s offset.
        self.merge_planner_on = p('roundabout.merge_planner', False).value
        self.ring_merge = None
        # Rebuilt roundabout (roundabout_boxes + roundabout_planner, branch roundabout-rebuild): replaces the entry gate
        # and the circular_gap admission. Built on the first route + s offset.
        self.box_planner_on = p('roundabout.box_planner', False).value
        # Measuring only (practice, no NPCs): drive the entry at a fixed approach speed (m/s; 0 = stop at the wait
        # station, hold 1 s, launch) to record our own entry motion. FOSCAR_RB_PROBE, unset = off.
        self.ring_probe = p('roundabout.probe_approach_mps', float(os.environ.get('FOSCAR_RB_PROBE', '-1'))).value
        self.ring_probe_held = None
        # Measuring only: approach at this speed (m/s) and drop v_max to 0 at s865 -- our stopping response.
        self.ring_stop_probe = float(os.environ.get('FOSCAR_RB_STOP', '-1'))
        # Measuring only: with FOSCAR_RB_PROBE=0, launch from the wait station on a v_max ramp of this accel (m/s2).
        self.ring_launch_probe = float(os.environ.get('FOSCAR_RB_LAUNCH_A', '-1'))
        # Data collection (user 2026-10-02): no entry judgement -- stand at the wait station, go at full pedal as soon as
        # our stretch of the ring holds no car, then keep the gap to the car ahead; the car behind is not looked at.
        self.ring_greedy = os.environ.get('FOSCAR_RB_GREEDY', '') == '1'
        # Watching the ring boxes / gaps in RViz: drive to the wait station and stand there, never enter.
        self.ring_hold = os.environ.get('FOSCAR_RB_HOLD', '') == '1'
        # The entry (tuck: go / no-go, the gap behind the nearest car, lead handoff, follow out) -- the competition
        # default since 2026-10-04 (user); FOSCAR_RB_TUCK=0/1 overrides, the greedy / hold practice modes turn it off.
        env_tuck = os.environ.get('FOSCAR_RB_TUCK', '')
        self.ring_tuck = (env_tuck == '1' if env_tuck in ('0', '1') else
                          bool(p('roundabout.tuck', True).value) and not (self.ring_greedy or self.ring_hold))
        # Practice runs: the ring spawner needs time to fill the ring after the scenario starts (user: wait 10 s).
        self.ring_warmup_s = float(os.environ.get('FOSCAR_RB_WARMUP_S', '0') or 0.)
        # Roundabout decisions start at the checkpoint; the ring is perceived from the mission start (user 2026-10-03)
        self.ring_judge_from_s = p('roundabout.judge_from_s', 810.).value
        # waiting for a lead, stop this much short of the lane (user 2026-10-02: a touch more room than the lane edge;
        # a small car passed close in front of our nose there)
        self.ROLL_WATCH_S = .6   # s the ring is watched before a go / no-go (p90 of 167 measured scan dropouts 0.59 s)
        self.RING_HOLD_BACK_M = .5
        self.HIGHWAY_JOIN_M = 300.   # the background highway build is waited for this far before its mission
        # the car behind the lead goes first by more than this: it is the lead (user 2026-10-04). 0 (was 0.25, user 2026-10-05):
        # a car first by 0.19-0.27 s hovered on the 0.25 line for 15 m, was never handed on, and we cut in front of it
        # and hit it (16:57 lap); no lap went first with a margin below 0 and made it.
        # 0.25 again (user 2026-10-09): at 0 the car behind, its speed read 20-47 % fast, was taken first and the gap
        # given up while it came after us (14:35 keep run)
        self.RING_HANDOFF_S = p('roundabout.handoff_s', .25).value
        # hold at the stop line when the car going first leaves no way in behind it (144263d); false = before it (A/B 2026-10-08)
        self.ring_cand_hold_on = p('roundabout.cand_hold', True).value
        # the car behind held for only once its nose will be this near the merge at no-return. 3.5 m (2026-10-05, 15-20 km/h
        # rings) -> 7 m (user 2026-10-09): coming in at 40 km/h no-return is now, and a 20 km/h car 6.5 m back, judged first
        # by 0.6 s four scans running, was let through and hit us
        self.RING_HOLD_NEAR_M = 7.
        self.RING_EXIT_ACCEL = 3.      # m/s2 the way out is driven at, judging when a car ahead is out of our way
        self.RING_EXIT_CLEAR_M = .5    # body to body kept to it on the way out
        self.ring_planner = None
        self.ring_plan = None;self.ring_plan_at = -math.inf
        self.ring_go_count = 0;self.ring_last_plan_seen = None
        if self.box_planner_on:
            self.entry_gate.enabled = False
            self.circular_gap_on = False
        self.entry_gate.circular_mode = self.circular_gap_on
        self.ring_brake = JerkBrake()
        self.ring_entered_at = None
        self.prediction_marker_pub=(self.create_publisher(MarkerArray, '/behavior/prediction_markers', 1)
                                    if self.markers_on else None)
        self.prediction_at = -math.inf
        self.prediction_pub = self.create_publisher(String, '/behavior/prediction', 10)
        self.clearance_pub = self.create_publisher(String, '/perception/clearance_queries', 1)

        self.create_subscription(String, '/control/telemetry', self.on_control_telemetry, 1)
        # Inputs on their own thread (main: MultiThreadedExecutor): the executor takes one message per subscription
        # per pass and each pass carried a 30-60 ms tick, while the lidar sends one message per object (1-4 a scan):
        # /events fell 0.4-0.6 s behind, a pedestrian walking in reached YIELD in a burst once near (2026-10-10).
        # self.events, pose_history and the pending scan are shared with the tick: event_lock.
        self.event_lock = threading.Lock()
        self.pending_footprints = None
        events_group = MutuallyExclusiveCallbackGroup()
        # Only the newest scan counts. Kept here and processed at the start of the tick (it updates the highway,
        # ring and tunnel wall state the tick uses): in the tick's own group the timer, always due, starved it and
        # the BEV guard read 'stale scan' (cap 0) 22 times a tunnel run -- a brake about every second (2026-10-10).
        self.create_subscription(String, '/perception/footprints', self.keep_footprints, 1, callback_group=events_group)
        self.create_subscription(Event, '/events', self.on_event, 50, callback_group=events_group)
        self.create_subscription(Event, '/perception/road_users', self.on_observation, 50, callback_group=events_group)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data,
                                 callback_group=events_group)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data,
                                 callback_group=events_group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/localization/mode', self.on_mode, latched)
        self.create_subscription(Path, '/map/route', self.on_route, latched)
        self.target_pub = self.create_publisher(DrivingTarget, '/behavior/target', 1)
        self.state_pub = self.create_publisher(String, '/behavior/state', 1)
        self.mission_pub = self.create_publisher(String, '/behavior/mission', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/behavior/markers', 1) if self.markers_on else None
        self.create_timer(1. / self.rate, self.tick)
        self.create_timer(2., self.report)

    # ------------------------------------------------------------------ 입력
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.odom, self.odom_at = msg, self.now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        o = msg.pose.pose
        q = o.orientation
        if stamp > 0. and (not self.pose_history or stamp > self.pose_history[-1][0]):
            with self.event_lock:
                self.pose_history.append((stamp, o.position.x, o.position.y, quaternion_yaw(q.w, q.x, q.y, q.z)))

    def pose_at(self, t: float):
        """Ego (x, y, yaw) interpolated at time t, or None outside the recent history.

        Lidar events are measured at scan time and arrive ~0.08 s later; projecting them
        with the latest pose shifted obstacles 0.6-1.2 m toward a swerving car's side
        and inflated the second tunnel barricade (2026-09-26).
        """
        h = self.pose_history
        if len(h) < 2 or not h[0][0] <= t <= h[-1][0] + .05:
            return None
        if t >= h[-1][0]:
            return h[-1][1:]
        i = next(k for k in range(1, len(h)) if h[k][0] >= t)
        (t0, x0, y0, a0), (t1, x1, y1, a1) = h[i-1], h[i]
        w = (t-t0)/max(t1-t0, 1e-9)
        return (x0+(x1-x0)*w, y0+(y1-y0)*w, a0+math.remainder(a1-a0, 2*math.pi)*w)

    def on_signal(self, msg: String):
        if self.signal_stop is None or self.s_global is None:
            return
        try:
            data = json.loads(msg.data)
            self.signal_stop.observe(data['id'], data['color'], self.s_global,
                                     float(data['stamp']), self.get_clock().now().nanoseconds / 1e9)
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('invalid camera signal message ignored')

    def on_s(self, msg: Float64):
        self.s_global = float(msg.data)   # signal_stop.current: at the tick (its state belongs to the tick's thread)

    def keep_footprints(self, msg: String):
        with self.event_lock:
            self.pending_footprints = msg

    def on_mode(self, msg: String):
        self.loc_mode = msg.data

    def on_route(self, msg: Path):
        if self.signal_stop is not None:
            self.signal_stop.reset()
        pts = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        try:
            self.route = Route(pts)
        except ValueError as e:
            self.get_logger().error(f'/map/route 를 못 씀: {e}')
            return
        self.s_offset = None
        if self.mission is not None:
            self.mission.reset()
            self.reset_mission_runtime(clear_observations=True)
        self.get_logger().info(f'경로 수신: {len(pts)}점 {self.route.s[-1]:.0f} m')

    def ego(self):
        """(x, y, yaw, v) 또는 None."""
        if self.odom is None or self.now() - self.odom_at > self.odom_timeout:
            return None
        o = self.odom.pose.pose
        q = o.orientation
        return (o.position.x, o.position.y, quaternion_yaw(q.w, q.x, q.y, q.z), self.odom.twist.twist.linear.x)

    def apply_ring_boxes(self, target, ctx, now):
        """Roundabout entry by roundabout_planner (module docstring there)."""
        gate=self.entry_gate
        if not gate.approach_s<=ctx.s<=gate.end_s or ctx.route is None or self.circle_reference is None:
            if self.ring_planner is not None and ctx.s<gate.approach_s-5.:
                self.ring_planner.committed=False;self.ring_planner.launch_mps2=math.inf;self.ring_planner.roll_off=False
            return target
        rp=self.ring_planner
        if rp is None or abs(getattr(rp,'s_offset',0.)-ctx.s_offset)>.5:
            # entry curve speeds at the path follower's roundabout lateral limit (low_speed.yaml a_lat_zones 6.0)
            g=RoundaboutGeometry(self.circle_reference,ctx.route,ctx.s_offset,gate.approach_s,gate.end_s,a_lat_mps2=6.)
            lane=RingLane(self.circle_reference,g.origin)
            try:
                table=json.loads((FilePath(get_package_share_directory('foscar_planning'))/'config'/'roundabout_motion.json').read_text())
            except (OSError,ValueError,PackageNotFoundError) as exc:
                self.get_logger().warn(f'roundabout motion table unavailable ({exc}): kinematic model');table=None
            rp=self.ring_planner=RoundaboutPlanner(g,lane,gate.stop_at_s if gate.stop_at_s>=0. else gate.entry_s,
                                                  gate.max_speed_kmh/3.6,motion_table=table)
            rp.s_offset=ctx.s_offset;rp.ring_limit_mps=self.ring_limit_mps
            # Matching the ring speed from afar (user 2026-10-02): ring cars at their typical speed (measured p50)
            rp.p.nominal=os.environ.get('FOSCAR_RB_NOMINAL','')=='1'
        v=max(0.,ctx.v)
        if self.ring_stop_probe>0.:
            self.prediction_result=dict(model='roundabout_boxes',decision='STOP_PROBE',evaluated_at=now)
            cap=0. if ctx.s>=865. else self.ring_stop_probe
            return replace(target,v_max=min(target.v_max,cap),state='ROUNDABOUT_WAIT')
        if self.ring_probe>=0.:
            return self.ring_probe_target(target,ctx,rp,v,now)
        if self.ring_greedy or self.ring_hold or self.ring_tuck:
            return self.ring_greedy_target(target,ctx,rp,v,now)
        if rp.committed:
            if ctx.s>=rp.g.s_out:
                self.prediction_result=dict(model='roundabout_boxes',decision='EXITED',evaluated_at=now)
                return target
            soft=getattr(rp,'launch_mps2',math.inf)<math.inf
            rp.launch_elapsed=now-rp.launch_t0 if soft else None
            if now-self.ring_plan_at>=.2 or getattr(self,'ring_commit_cap',None) is None:
                self.ring_commit_cap=rp.committed_speed(ctx.s,v);self.ring_plan_at=now
            cap,why=self.ring_commit_cap
            if cap is None or (cap<=0. and v<.3 and ctx.s<rp.g.s_in):
                rp.committed=False;self.ring_commit_cap=None;self.ring_plan=None;rp.launch_mps2=math.inf   # wait / plan the entry afresh
                return self.apply_ring_boxes(target,ctx,now)
            if soft and why.startswith('launch'):
                cap=max(.3,rp.launch_mps2*(now-rp.launch_t0))    # the soft launch as measured: the same v_max ramp
            elif soft:
                rp.launch_mps2=math.inf                          # it no longer fits: the fastest that does
            self.prediction_result=dict(model='roundabout_boxes',decision='ENTER',keep_gap_mps=round(cap,2),reason=why,
                                        evaluated_at=now,runs=[(round(r.lo,1),round(r.hi,1)) for r in rp.lane.runs])
            return replace(target,v_max=min(target.v_max,cap),state='ROUNDABOUT_ENTER')
        if not 0<=now-rp.lane.stamp<=.45:
            plan=None
        elif now-self.ring_plan_at>=.2 or self.ring_plan is None:
            plan=self.ring_plan=rp.plan(ctx.s,v);self.ring_plan_at=now
        else:plan=self.ring_plan
        if plan is None:
            cap,stop=rp.stop_speed(ctx.s),True
            self.prediction_result=dict(model='roundabout_boxes',decision='NO_SCAN',evaluated_at=now)
            state='ROUNDABOUT_WAIT'
        else:
            # Standing, a launch commits on two plans in a row (0.2 s apart): a car just coming into sight upstream
            # was seen only on the plan after the commit, which then gave it back -- 3 creeping false starts (11:12).
            if plan is not self.ring_last_plan_seen:
                self.ring_go_count=self.ring_go_count+1 if plan.commit else 0
                self.ring_last_plan_seen=plan
            standing=v<.3
            if plan.commit and (not standing or self.ring_go_count>=2) and target.v_max>0. and not math.isfinite(target.stop_s):
                rp.committed=True;self.ring_commit_cap=None
                rp.launch_mps2=plan.launch_mps2 if standing else math.inf;rp.launch_t0=now
                self.prediction_result=dict(model='roundabout_boxes',decision='GO',reason=plan.reason,evaluated_at=now,
                                            bodies=plan.bodies,blocked=plan.blocked)
                go=rp.keep_gap(ctx.s,v) if rp.launch_mps2==math.inf else .3
                return replace(target,v_max=min(target.v_max,go),state='ROUNDABOUT_ENTER')
            cap,stop=rp.speed(plan,ctx.s,v)
            state='ROUNDABOUT_APPROACH' if plan.decision=='APPROACH' else 'ROUNDABOUT_WAIT'
            self.prediction_result=dict(model='roundabout_boxes',decision=plan.decision,reason=plan.reason,
                                        approach_mps=plan.approach_mps,wait_s=plan.wait_s,evaluated_at=now,
                                        bodies=plan.bodies,blocked=plan.blocked,stop_s=rp.stop_s,
                                        runs=[(round(r.lo,1),round(r.hi,1),r.lo_speed,r.hi_speed) for r in rp.lane.runs])
        out=replace(target,v_max=min(target.v_max,cap),state=state)
        if stop and rp.stop_s-ctx.s<=2.:
            front=rp.stop_s+3.
            out=replace(out,stop_s=min(target.stop_s,front) if math.isfinite(target.stop_s) else front)
        return out

    def ring_greedy_target(self, target, ctx, rp, v, now):
        """Data collection mode (ring_greedy)."""
        g=rp.g
        if self.ring_hold:
            # stand where we are (the scenario starts us on the new stop line), only watch the ring
            self.prediction_result=dict(model='roundabout_boxes',decision='HOLD',evaluated_at=now)
            return replace(target,v_max=0.,state='ROUNDABOUT_WAIT')
        if (rp.committed and getattr(rp,'rolled',False) and ctx.s<g.s_in and 0<=now-rp.lane.stamp<=.45
                and v<=rp.follow_speed(max(0.,g.s_in-self.RING_HOLD_BACK_M-ctx.s),0.)):
            # Rolling in and still able to stop short of the lane: the go is reviewed every scan (a car can come in sight
            # after we chose: one hidden behind another hit us, 02:46 run 6). Two misses in a row: give it up, come to a
            # stop short of the lane where we are and look for the next chance there (user 2026-10-03). Past that point
            # the go stands.
            if rp.lane.stamp!=getattr(rp,'roll_stamp',None):
                rp.roll_stamp=rp.lane.stamp
                m_r,lead_r=rp.roll_in(ctx.s,v,rp.lane.runs,max(0.,now-rp.lane.stamp))
                fits=m_r is not None
                if fits and lead_r is not None and getattr(rp,'roll_empty',False):
                    # The go was for an empty way; a car has come onto it (we would only go in behind it now): that is
                    # the tuck's entry, not ours -- the go is withdrawn and the tuck takes over (user 2026-10-03: held
                    # from 65 m out we crawled in behind a 10 km/h car and the one after it ran into us, 22:08 runs)
                    rp.roll_handoff=True;rp.mark('handoff',lead_r)   # the tuck goes in behind it (user 2026-10-09)
                    self.get_logger().info('RB_ROLL_HANDOFF '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                        lead=[round(lead_r.lo,2),round(lead_r.hi,2)])))
                # Going on at our speed, the car behind is measured again every scan; it only counts against the go
                # when its speed is measured well (user 2026-10-03: truck speeds kept coming out wrong -- don't settle on
                # a speed too soon, keep measuring it right)
                # a miss only on a car measured: one not measured yet is no verdict (user 2026-10-09)
                blocker=getattr(rp,'roll_block_run',None)
                unsure=(not fits and (getattr(rp,'roll_why',None)=='unmeasured'
                                      or blocker is not None and not rp.measured(blocker)))
                if not fits:
                    self.get_logger().info(('RB_ROLL_UNSURE ' if unsure else 'RB_ROLL_MISS ')+json.dumps(dict(t=round(now,3),
                        s=round(ctx.s,2),v=round(v,2),why=getattr(rp,'roll_why',None),block=getattr(rp,'roll_block',None))))
                if not unsure:
                    rp.roll_miss=0 if fits else getattr(rp,'roll_miss',0)+1
                rp.roll_last_fits=fits
            # The go / no-go is settled before the new stop line (user 2026-10-09): at the last point the measured stop
            # still holds us at the line, a go not standing in the latest scan (missed or not yet sure) is a no-go, handed
            # to the tuck -- which may still take us in without stopping
            line=rp.line_s()
            deadline=(getattr(rp,'committed',False) and getattr(rp,'rolled',False) and ctx.s<line
                      and ctx.s+rp.stop_dist(v)>=line-.5 and not getattr(rp,'roll_last_fits',True))
            if deadline:
                rp.roll_miss=max(getattr(rp,'roll_miss',0),2)
            if getattr(rp,'roll_miss',0)>=2 or getattr(rp,'roll_handoff',False):
                # no way in ahead of the cars coming: handed to the tuck either way (user 2026-10-09), which takes it on
                # from where we are, rolling -- behind the car that would have hit us
                handoff=getattr(rp,'roll_handoff',False) or getattr(rp,'roll_miss',0)>=2
                blocker=getattr(rp,'roll_block_run',None)
                if blocker is not None and blocker.height>=0. and rp.measured(blocker):rp.mark('handoff',blocker)
                rp.committed=False;rp.rolled=False;rp.roll_miss=0;rp.roll_top=None;rp.roll_handoff=False;rp.roll_empty=False;rp.roll_off=True
                # stop where we are only when given up past the line; before it the line stands (a short flag set
                # at s 850 moved the stop 8 m on into the lane at the line and we stood in the ring, 03:13 run 3)
                # where we come to rest from here (measured stops), never past the lane: going on to the lane's edge
                # stood us where ring cars pass (878.0 / 878.6, 03:40 runs 1 and 4; hit there at 878.2, 03:13 run 3)
                rp.stopped_short=(min(g.s_in-self.RING_HOLD_BACK_M,ctx.s+rp.stop_dist(v)) if ctx.s>=rp.line_s()-1.
                                  else False)
                rp.tuck_lead=None;rp.tuck_view=None    # the car we were to follow is no target once we give the go up
                if handoff:rp.stopped_short=False    # handed to the tuck: it decides from here, coming on to the line
                self.get_logger().info('RB_ROLL_ABORT '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),handoff=handoff)))
        if rp.committed:
            if ctx.s>=g.s_out:
                return target
            top=getattr(rp,'roll_top',None)
            if getattr(rp,'rolled',False) and top is not None and ctx.s<g.s_merged:
                # rolled in at a lower top to come in behind the car ahead at its pace: drive that way in too -- judged
                # capped but driven at full pedal we came up on its tail and stopped at the lane (03:48 runs 2, 3)
                target=replace(target,v_max=min(target.v_max,top))
            # behind the car ahead from the launch on, not only once on the lane: brake to its speed in the gap
            e_hi=rp.front_arc(ctx.s);cap=rp.v_cap;waiting=False
            # its front past our nose: ahead (or beside: stop); its tail past where we turn off: it goes on round and
            # is no longer in our way (user 2026-10-02: from that moment speed up to the exit road's 60)
            # "Ahead of us" as seen from the line, or from where we are once past it: further back our nose stands in
            # far upstream (u -63 at s 811) and a car still coming up to the entry was taken for the one to follow --
            # we braked from 60 to a crawl behind it and the car after it hit us (20:54 settle 6; user 2026-10-03)
            ref=max(e_hi,rp.front_arc(rp.line_s()))
            ahead=[r for r in rp.lane.runs if r.hi>ref and r.lo<rp.exit_arc()]
            if getattr(rp,'rolled',False) and ctx.s<g.s_merged:
                # A go / no-go go is for an empty way: no car to follow on the way in (user 2026-10-09: it braked for the
                # car it had judged to come in behind us, slowed to 11 km/h and let that car in first). Whether the way
                # stays empty is its review's call; on the lane the ring's own following takes over.
                ahead=[]
            # the lead chosen at the launch stays the one we follow until it is past our exit or lost
            held=rp.held_lead(rp.lane.runs) if self.ring_tuck else None
            if held is not None and ctx.s<g.s_in and 0<=now-rp.lane.stamp<=.45 and rp.lane.stamp!=getattr(rp,'judge_stamp',None):
                # Who reaches the merge first, we or the car behind the lead (user 2026-10-04)? We: it is the car behind
                # us, never followed. It (by more than RING_HANDOFF_S, two scans in a row): it is the lead from here and
                # we go in behind it at once. Past where the measured stop (stop_dist) still holds us short of the
                # lane we go on behind the lead we have -- the turning point moves with our speed.
                rp.judge_stamp=rp.lane.stamp
                age=max(0.,now-rp.lane.stamp);cand=rp.rear_of(held,rp.lane.runs)
                # only while moving: standing past the hold-back point read as "no return" at 0 km/h and we pulled out
                # in front of a car going first (01:13 run)
                # No return once even the measured stop no longer holds us short of the lane (user 2026-10-04): at the
                # gentle 3 m/s2 it set in 25-40 m out at 46-49 km/h and a car going first was never handed on -- we cut
                # in in front of it and touched (03:38 run 3)
                no_return=v>=.5 and g.s_in-self.RING_HOLD_BACK_M-ctx.s<rp.stop_dist(v)
                margin=None
                # Judged only on a car measured well (its speed within SURE_SD -- a car first seen 1 m long reads the
                # tracker's start speed) and with nothing hidden between it and the lead (every box between seen free --
                # a car hidden there was skipped with its gap, 01:22 run 4); otherwise the lead is kept (user 2026-10-04)
                est=rp.estimate(cand) if cand is not None else None
                between=[b for b in rp.lane.boxes if cand is not None and b[0]>=cand.hi-1e-6 and b[1]<=held.lo+1e-6]
                seen=bool(between) and all(b[2]==FREE for b in between)
                if cand is not None and cand.coasted==0. and est is not None and est[1]<=rp.SURE_SD and seen:
                    path=rp.tuck_path(ctx.s,0.,held,rp.est_speed(held,rp.lead_speed(held)),rp.lane.runs,age,None,v0=v)
                    # no way in behind the lead as we drive is no verdict on the car behind it: kept (taken as "it goes
                    # first", every lead was handed on up to the window's end and we stood at the lane 14 s, 01:13 run)
                    if path is not None:
                        iv=g.interval(path[1]);tail=iv[0] if iv is not None else rp.front_arc(path[1])-rp.EGO_LEN
                        need=tail-rp.REAR_MIN_M-cand.hi;vc=max(rp.est_speed(cand,rp.rear_speed(cand)),.1)
                        margin=(need/vc if need>0. else 0.)-(age+path[0])
                rp.judge=dict(cand=None if cand is None else [round(cand.lo,1),round(cand.hi,1)],
                              margin=None if margin is None else (round(margin,2) if math.isfinite(margin) else -99.),
                              no_return=no_return)
                rp.judge_cand,rp.judge_margin=cand,margin
                first=margin is not None and margin<-self.RING_HANDOFF_S and not no_return
                rp.cand_first=getattr(rp,'cand_first',0)+1 if first else 0
                # handed on only to a car we have a way in behind. With none (too fast and close to fall in behind it) the
                # gap is no way in either (user 2026-10-05: neither ahead of it nor behind it, we went on and hit it,
                # 16:57 lap): held short of the lane until it goes first or we fit again; never past no-return
                # released only when we fit again or the car is handed on (below) -- not at no-return: braking lags the
                # hold's cap, no-return came on at 33 km/h 5 m out and released it into the car (17:39 run 4)
                if margin is not None and margin>=0.:rp.cand_hold=False
                # ...and only when that car will be close by the time we reach no-return (its nose within RING_HOLD_NEAR_M
                # of the merge reference): a car still 4-5.5 m back there let us in (16:48, 16:51, 17:48 laps, margins
                # -0.16..-0.32 s), the two that hit us were 2-3 m back (16:57, 17:46; user 2026-10-05: no gap the old
                # logic took may be given up)
                near=False
                if cand is not None:
                    t_nr=max(0.,g.s_in-self.RING_HOLD_BACK_M-rp.stop_dist(v)-ctx.s)/max(v,.5)
                    near=cand.hi+rp.est_speed(cand,rp.rear_speed(cand))*(age+t_nr)>-self.RING_HOLD_NEAR_M
                rp.judge_nopath=rp.cand_first>=2 and rp.tuck_path(ctx.s,0.,cand,rp.est_speed(cand,rp.lead_speed(cand)),rp.lane.runs,age,None,v0=v) is None
                if rp.judge_nopath:
                    rp.cand_first=0
                if self.ring_cand_hold_on and rp.cand_first==0 and first and near and getattr(rp,'judge_nopath',False):
                    if not getattr(rp,'cand_hold',False):
                        self.get_logger().info('RB_CAND_HOLD '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                            margin=rp.judge['margin'],cand=rp.judge['cand'],line=round(rp.line_s(),2),s_in=round(g.s_in,2),near=near)))
                    rp.cand_hold=True
                if rp.cand_first>=2:
                    rp.cand_first=0
                    self.get_logger().info('RB_LEAD_HANDOFF '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                        margin=rp.judge['margin'],old=[round(held.lo,2),round(held.hi,2)],new=[round(cand.lo,2),round(cand.hi,2)])))
                    rp.tuck_path(ctx.s,0.,cand,rp.lead_speed(cand),rp.lane.runs,age,None,v0=v)
                    rp.go_plan=list(getattr(rp,'path_samples',None) or []);rp.go_t=now   # the way in behind the new lead
                    rp.mark('lead',None);rp.mark('lead',cand);rp.mark('rear',rp.rear_of(cand,rp.lane.runs))
                    held=cand;rp.late_count=0;rp.late_hold=False;rp.fit_count=0;rp.judge_cand=None;rp.cand_hold=False
            if held is not None and held.lo<rp.exit_arc():
                # The lead, and anything seen between our nose and it: the nearest is followed -- a car in front of us is
                # never left out. Except, before the lane, the car behind the lead judged to reach the merge after us
                # (margin >= 0): it is the car behind us, not one to brake for (its nose past the line reference braked
                # us for it, 00:52-01:04 runs). Unjudged or going first, it is braked for as before (followed only the
                # lead, we pulled out in front of one, 01:13 run).
                behind_us=(getattr(rp,'judge_cand',None) if ctx.s<g.s_in and getattr(rp,'judge_stamp',None)==rp.lane.stamp
                           and getattr(rp,'judge_margin',None) is not None
                           and rp.judge_margin>=0. else None)
                if behind_us is not None:
                    over=[x for x in ahead if min(x.hi,behind_us.hi)-max(x.lo,behind_us.lo)>0.]
                    behind_us=over[0] if over else None
                ahead=[held]+[x for x in ahead if x is not held and x.lo<held.lo and x is not behind_us]
            elif held is not None:
                rp.tuck_lead=None;ahead=[]
            if ctx.s>=g.s_in and ahead:
                # Out of our way on the way out (user 2026-10-04): driven on at full pedal (RING_EXIT_ACCEL, the curve's
                # speed at most) until we leave the lane, our body stays RING_EXIT_CLEAR_M clear of its body (moving at
                # its speed) -> no longer followed. By the arc alone we followed it until s 897-899 while already turned
                # out 45-50 deg; bodies stay clear from s 892-893 (02:45 runs)
                age=max(0.,now-rp.lane.stamp);keep=[]
                for x in ahead:
                    vx_=rp.lead_speed(x);tm,sm,vm,clear=0.,ctx.s,v,True
                    while sm<g.s_out and tm<8.:
                        tm+=.1;vm=min(vm+self.RING_EXIT_ACCEL*.1,rp.curve_cap(sm));sm+=vm*.1
                        if rp.body_gap(rp.ego_box(sm),rp.ring_box(x.lo+vx_*(age+tm),x.hi+vx_*(age+tm)))<self.RING_EXIT_CLEAR_M:
                            clear=False;break
                    if not clear:keep.append(x)
                ahead=keep
            if self.ring_tuck:
                rp.tuck_view=dict(state='GO' if ctx.s<g.s_merged else 'FOLLOW',rear=None,tau=None,
                                  lead=None if not ahead else (min(ahead,key=lambda x:x.lo).lo,min(ahead,key=lambda x:x.lo).hi))
            if ahead:
                r=min(ahead,key=lambda x:x.lo);vl=rp.lead_speed(r)
                age=max(0.,now-rp.lane.stamp)            # moved on since the scan
                # followed from the launch on, as the prediction drives it: along the ring (our way in continued onto it
                # before the lane), braking room (measured stops) + the room we keep (standoff: 0.5 m in, 1 m on the ring)
                arc=r.lo+vl*age-e_hi
                if ctx.s>=g.s_in or arc>0.:
                    gap=arc-rp.gap_margin(ctx.s)-rp.standoff(ctx.s)   # braking room + the room we keep (planner)
                    cap=min(cap,max(.3 if gap>0. else 0.,rp.follow_speed(gap,vl)))
                elif getattr(rp,'go_t',None) is not None:
                    # Before the lane, the lead not yet past our stand-in nose: drive the launch the prediction
                    # planned for it. Held to a stop there instead, every fast launch came 0.8-1.5 s late; let go, we
                    # ran for the gap ahead of it (2026-10-02).
                    vp=rp.planned_speed(now-rp.go_t,ctx.s)
                    if vp is not None:
                        cap=min(cap,vp)
                # Before the lane the launch drives as planned for the lead held since the stop line; only real bodies
                # touching stop it there. (The stand-in gap stopped it until the lead passed our stand-in nose: right
                # gap, but 3 m late at 7 m/s on fast rings, 22:07 runs.)
                if ctx.s<g.s_in:
                    # held to the stand-in gap only if, from where we are and as fast as we go, the rest of the way in
                    # no longer fits behind this lead (it came slower than planned: we touched the lane beside it and
                    # hit it, 22:19); otherwise driven as planned
                    # Debounced: two scans in a row before holding, and once held, held until the lead is past our
                    # stand-in nose -- one noisy estimate dropped the cap to 0 for a tick and we jerked (22:22 runs)
                    behind=rp.lead_gap(ctx.s,r,vl*age)<=0.
                    if rp.lane.stamp!=getattr(rp,'late_stamp',None):
                        rp.late_stamp=rp.lane.stamp
                        late=rp.tuck_path(ctx.s,0.,r,vl,rp.lane.runs,age,None,True,v) is None
                        rp.late_count=getattr(rp,'late_count',0)+1 if late else 0
                        rp.fit_count=0 if late else getattr(rp,'fit_count',0)+1
                    # released as soon as the rest of the way in fits again (two scans), not only once the lead has
                    # passed our stand-in nose: we slowed from 4.8 to a stop and back over 2 s (22:47 run 5)
                    if not behind or getattr(rp,'fit_count',0)>=2:
                        rp.late_hold=False
                    elif getattr(rp,'late_count',0)>=2:
                        rp.late_hold=True
                    touching=rp.body_gap(rp.ego_box(ctx.s),rp.ring_box(r.lo+vl*age,r.hi+vl*age))<=0.
                    if behind and getattr(rp,'late_hold',False):
                        # Waiting for the lead, gently: no faster than lets us stop on the new stop line (measured stops),
                        # not a dead stop at once (user 2026-10-02: brake softer); released as soon as the lead is clear.
                        # Aimed at the lane's edge we rolled on to where trucks pass and stood there (user 2026-10-09)
                        cap=min(cap,rp.follow_speed(max(0.,rp.line_s()-ctx.s),0.))
                else:
                    touching=ctx.s<g.s_merged and rp.lead_gap(ctx.s,r,vl*age)<=0.
                if touching:
                    cap=0.                                # bodies about to touch while coming in
                if ctx.s<g.s_in and getattr(rp,'go_t',None) is not None and not rp.launch_clear(now-rp.go_t,ctx.s,r,age):
                    waiting='crossing'                    # another car will cross our way: hold short of the lane
            if ctx.s<g.s_in and getattr(rp,'cand_hold',False):
                # the car behind the lead goes first and we have no way in behind it: stop at the stop line, not at the
                # lane's edge where ring cars pass (stood at 878.8 there and was hit, 17:46 run 2) (user 2026-10-05)
                waiting='cand_hold'
            if waiting and ctx.s<g.s_in:
                # Where we wait, fixed as the wait starts (user 2026-10-09): on the new stop line whenever the measured stop
                # still gets us there -- the ring is read best from it -- else where that stop brings us, short of the
                # lane. Each wait used to aim at a point of its own (the lane's edge, a 0.3 m/s crawl, a point redone from
                # where we were every tick) and we crept on toward the ring with no go coming.
                if getattr(rp,'wait_at',None) is None:
                    line=rp.line_s();sd=rp.stop_dist(v)
                    rp.wait_at=line if ctx.s+sd<=line+.5 else min(g.s_in-self.RING_HOLD_BACK_M,ctx.s+sd)
                    self.get_logger().info('RB_WAIT '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                        why=waiting,at=round(rp.wait_at,2))))
                cap=min(cap,rp.follow_speed(max(0.,rp.wait_at-ctx.s),0.))
            else:
                rp.wait_at=None
            self.prediction_result=dict(model='roundabout_boxes',decision='GREEDY_ENTER',keep_gap_mps=round(cap,2),evaluated_at=now)
            return replace(target,v_max=min(target.v_max,cap),state='ROUNDABOUT_ENTER')
        if self.ring_tuck:
            # fresh scans, watched past the measured dropout limit (a GO 1.6 s after start saw no car yet, 16:13)
            fresh=0<=now-rp.lane.stamp<=.45 and rp.lane.watched_s()>=max(rp.lane.EMPTY_WATCH_S,self.ring_warmup_s)
            line=rp.line_s()
            short=getattr(rp,'stopped_short',False)
            if short:
                line=max(line,short)                 # gave up rolling in past the line: stop where we are, short of the lane
            if v>=.3 or (ctx.s<line-1. and not short):
                # Coming in: go on without stopping when our way in as measured fits -- every ring car either ahead of
                # us to follow or behind us with room to brake, nothing unseen able to reach us (user 2026-10-02);
                # otherwise come down to stop on the line and decide there.
                age=max(0.,now-rp.lane.stamp)
                # decisions start at the checkpoint, the ring perceived from the mission start on (user 2026-10-03)
                judging=ctx.s>=self.ring_judge_from_s
                # all the way in while moving, slow by the line too: keeping on at our speed, does the way in stay clear of
                # the car behind? (user 2026-10-03: left to the tuck near the line, we came to a stop with the way open)
                # Go / no-go is the quick call for coming in fast onto an empty way ahead of the cars upstream (user
                # 2026-10-03): it may not sit out the 2.27 s the tuck waits for a car lost from the scans (36 m at
                # 58 km/h; every go came at s 838-841). It watches the ring the measured p90 dropout (0.59 s); a car
                # that shows up later still ends the go at the review, before the point of no return.
                fresh_go=0<=now-rp.lane.stamp<=.45 and rp.lane.watched_s()>=max(self.ROLL_WATCH_S,self.ring_warmup_s)
                # once given up, go / no-go has no more say on this approach (it went GO -> miss -> GO again, user
                # 2026-10-09): the tuck takes it from there
                m,lead=(rp.roll_in(ctx.s,v,rp.lane.runs,age) if fresh_go and judging and v>=.3 and not getattr(rp,'roll_off',False)
                        else (None,None))
                if m is not None and lead is not None:
                    # Rolling in at speed is only for a way in with no car on it (user 2026-10-03): with a car to come in
                    # behind we stop on the line and the launch there (tuck) decides; go / no-go has no say at low speed
                    m,lead=None,None;rp.roll_why,rp.roll_block='way',None
                # go / no-go decides from the same start as the rest (user 2026-10-03: it must start earliest, it
                # came last); no waiting for the upstream ring to be within 38 m any more
                real=m is not None
                if m is None and fresh and v>=.3:
                    # what kept us from going (practice analysis: blocker run lo/hi, the speed taken, its estimate,
                    # when and where on our way in); pseudo cars for hidden stretches have no estimate
                    self.get_logger().info('RB_ROLL_NO '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                        why=getattr(rp,'roll_why',None),block=getattr(rp,'roll_block',None))))
                if m is not None and not real:
                    # a provisional go: shown, not driven (we keep able to stop on the line) until the ring we need is in
                    # sight (user 2026-10-03: gather early, decide once the data can be trusted)
                    if getattr(rp,'tentative',None) is None:
                        self.get_logger().info('RB_ROLL_TENTATIVE '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2))))
                    rp.tentative=now
                if real and target.v_max>0.:
                    # no plan to drive: rolling in, only the lead (follow cap) and the curve slow us
                    rp.committed=True;rp.go_t=now;rp.go_plan=[];rp.late_count=0;rp.late_hold=False;rp.fit_count=0
                    rp.rolled=True;rp.roll_miss=0;rp.stopped_short=False;rp.roll_empty=lead is None;rp.roll_handoff=False;rp.roll_last_fits=True
                    # a stand-in for a hidden stretch (height -1) is no car to hold and follow
                    rp.tuck_lead=None
                    if lead is not None and lead.height>=0.:rp.mark('lead',lead)   # marked: followed and its speed averaged (user 2026-10-04)
                    rp.tuck_view=dict(state='GO',lead=None if lead is None else (lead.lo,lead.hi),rear=None,tau=0.)
                    self.get_logger().info('RB_ROLL_GO '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                        lead=None if lead is None else [round(lead.lo,2),round(lead.hi,2)])))
                    return replace(target,v_max=target.v_max,state='ROUNDABOUT_ENTER')
                # watched as long as go / no-go (0.6 s): the 2.27 s held a tuck that fitted at s 820 until s 840 (22:24 runs)
                if fresh_go and judging and v>=.3 and target.v_max>0. and not (m is not None and real):
                    # Coming down toward the line is no must-stop (user 2026-10-03): going on in now at our speed, behind
                    # the car we would launch behind from the line, is checked every scan the same way (tuck from
                    # where we are, rolling) and taken when it fits
                    lt,tt,pt=rp.tuck(ctx.s,rp.lane.runs,now=now,v0=v,latest=0.)
                    # practice analysis: the rolling call too, not only the standing one
                    diag={k:(round(x,2) if isinstance(x,float) else x) for k,x in (getattr(rp,'tuck_diag',None) or {}).items()}
                    diag.update(t=round(now,3),s=round(ctx.s,2),v=round(v,2),rolling=True,tuck_why=getattr(rp,'tuck_why',None),
                                gap_why=getattr(rp,'gap_why',None))
                    self.get_logger().info('RB_TUCK_EVAL '+json.dumps(diag))
                    if lt is not None and tt is not None:
                        rp.committed=True;rp.go_t=now;rp.go_at=None;rp.go_lead=None;rp.late_count=0;rp.late_hold=False
                        rp.fit_count=0;rp.stopped_short=False;rp.rolled=False;rp.roll_top=None;rp.tentative=None
                        rp.go_plan=list(getattr(rp,'path_samples',None) or [])
                        rp.tuck_view=dict(getattr(rp,'tuck_view',None) or {},state='GO')
                        self.get_logger().info('RB_TUCK_GO '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                            rolling=True,tau=0.,pred=pt,lead=dict(lo=round(lt.lo,2),hi=round(lt.hi,2),
                            speed=None if lt.speed is None else [round(float(x),3) for x in lt.speed]))))
                        return replace(target,v_max=target.v_max,state='ROUNDABOUT_ENTER')
                self.prediction_result=dict(model='roundabout_boxes',decision='ROLL_WAIT',evaluated_at=now)
                if m is None or real:
                    rp.tentative=None
                # no target while coming in (an old one stood as an afterimage); a provisional go shows its lead
                rp.tuck_view=(dict(state='TENTATIVE',lead=None if lead is None else (lead.lo,lead.hi),rear=None,tau=None)
                              if m is not None and not real else None)
                out=replace(target,v_max=min(target.v_max,rp.follow_speed(max(0.,line-ctx.s),0.)),state='ROUNDABOUT_WAIT')
                if line-ctx.s<=2.:
                    out=replace(out,stop_s=min(target.stop_s,line+3.) if math.isfinite(target.stop_s) else line+3.)
                return out
            # On the line (low speed): the launch timed on the way we drive in (tuck) decides alone -- go / no-go is
            # for rolling in at speed onto an empty way only (user 2026-10-03)
            lead,tau,pred=rp.tuck(ctx.s,rp.lane.runs,now=now) if fresh and ctx.s>=self.ring_judge_from_s else (None,None,{})
            if fresh and ctx.s>=self.ring_judge_from_s and rp.lane.stamp!=getattr(self,'tuck_eval_stamp',None):
                self.tuck_eval_stamp=rp.lane.stamp      # practice analysis: each standing decision and why
                self.get_logger().info('RB_TUCK_EVAL '+json.dumps(dict(t=round(now,3),s=round(ctx.s,2),v=round(v,2),
                    **{k:(round(x,2) if isinstance(x,float) else x) for k,x in (getattr(rp,'tuck_diag',None) or {}).items()})))
            # The launch time, once planned for this lead, is kept: later estimates only bring it forward. At that time
            # we go unless going now would touch the lead or need harder than the hardest measured stop (no margins):
            # re-planned every scan it slipped back 0.1-0.3 s at a time and came 0.6 s late (21:20 run).
            same=lead is not None and getattr(rp,'go_lead',None) is not None and rp.held_lead(rp.lane.runs) is lead
            if lead is None or tau is None and not same:
                rp.go_at=None;rp.go_lead=None
            elif tau is not None:
                at=now+tau
                rp.go_at=at if not same or getattr(rp,'go_at',None) is None else min(rp.go_at,at)
                rp.go_lead=lead
            due=(getattr(rp,'go_at',None) is not None and now>=rp.go_at and lead is not None and (tau is None or tau>=.1)
                 and rp.tuck_path(ctx.s,0.,lead,rp.lead_speed(lead),rp.lane.runs,max(0.,now-rp.lane.stamp),None,True) is not None)
            if due:
                pred=dict(pred or {},kept_launch=True)
            if v<.3 and fresh and (tau is not None and tau<.1 or due) and target.v_max>0.:
                rp.committed=True;rp.go_t=now;rp.go_at=None;rp.go_lead=None;rp.late_count=0;rp.late_hold=False;rp.fit_count=0
                rp.stopped_short=False;rp.rolled=False;rp.roll_top=None
                rp.go_plan=list(getattr(rp,'path_samples',None) or [])   # the launch we predicted: drive it
                rp.tuck_view=dict(getattr(rp,'tuck_view',None) or {},state='GO')
                snap=dict(t=round(now,3),s=round(ctx.s,2),tau=tau,pred=pred,
                          lead=None if lead is None else dict(lo=round(lead.lo,2),hi=round(lead.hi,2),
                                speed=None if lead.speed is None else [round(float(x),3) for x in lead.speed]),
                          runs=[dict(lo=round(r.lo,2),hi=round(r.hi,2),speed=None if r.speed is None else [round(float(x),3) for x in r.speed],
                                     height=round(r.height,2),coasted=round(r.coasted,2)) for r in rp.lane.runs])
                self.get_logger().info('RB_TUCK_GO '+json.dumps(snap))
                return replace(target,v_max=target.v_max,state='ROUNDABOUT_ENTER')
            self.prediction_result=dict(model='roundabout_boxes',decision='TUCK_WAIT',launch_in=tau,evaluated_at=now)
            return replace(target,v_max=0.,state='ROUNDABOUT_WAIT')
        ivs=[iv for iv in (g.interval(x) for x in np.arange(g.s_in,g.s_merged+.01,.25)) if iv is not None]
        zone=(min(i[0] for i in ivs),max(i[1] for i in ivs)) if ivs else (0.,5.)
        runs=rp.lane.runs
        # Tuck in behind the car ahead: our full-pedal launch (measured) must stay behind its tail, at its estimated
        # speed, until we are merged; cars whose front has not reached our entry (behind us) are not looked at.
        launch=rp.motion(ctx.s,0.,0.,0.)
        busy=[]
        for r in runs:
            if r.hi<=zone[0]:
                continue
            if r.speed is None:
                busy.append(r);continue
            for tk,sk,_ in launch:
                iv=g.interval(sk)
                if iv is None:
                    continue
                if sk>g.s_merged:
                    break
                if r.lo+r.speed[0]*tk<iv[1]:
                    busy.append(r);break
        standing=v<.3 and ctx.s>=rp.stop_s-rp.p.stand_m
        fresh=0<=now-rp.lane.stamp<=.45
        if not self.ring_hold and standing and fresh and not busy and target.v_max>0. and not math.isfinite(target.stop_s):
            rp.committed=True
            snap=dict(t=round(now,3),s=round(ctx.s,2),zone=[round(zone[0],2),round(zone[1],2)],
                      runs=[dict(lo=round(r.lo,2),hi=round(r.hi,2),lo_hidden=r.lo_hidden,hi_hidden=r.hi_hidden,
                                 speed=None if r.speed is None else [round(float(x),3) for x in r.speed],coasted=round(r.coasted,2))
                            for r in runs])
            self.get_logger().info('RB_GREEDY_GO '+json.dumps(snap))
            return replace(target,v_max=target.v_max,state='ROUNDABOUT_ENTER')
        self.prediction_result=dict(model='roundabout_boxes',decision='GREEDY_WAIT',busy=len(busy),evaluated_at=now)
        out=replace(target,v_max=min(target.v_max,0. if standing else rp.stop_speed(ctx.s)),state='ROUNDABOUT_WAIT')
        if rp.stop_s-ctx.s<=2.:
            front=rp.stop_s+3.
            out=replace(out,stop_s=min(target.stop_s,front) if math.isfinite(target.stop_s) else front)
        return out

    def ring_probe_target(self, target, ctx, rp, v, now):
        """Fixed-approach entry for measuring our own motion (see ring_probe)."""
        a=self.ring_probe;limit=self.entry_gate.max_speed_kmh/3.6
        self.prediction_result=dict(model='roundabout_boxes',decision='PROBE',approach_mps=a,evaluated_at=now)
        if ctx.s>=rp.g.s_in or rp.committed:
            rp.committed=True
            if self.ring_launch_probe>0. and getattr(self,'ring_probe_launch',None) is not None:
                limit=min(limit,self.ring_launch_probe*(now-self.ring_probe_launch))
            return replace(target,v_max=min(target.v_max,max(limit,.3)),state='ROUNDABOUT_ENTER')
        if a>0.:
            cap=math.sqrt(a*a+2*rp.p.brake_mps2*max(0.,rp.stop_s-ctx.s)) if ctx.s<rp.stop_s else a
            return replace(target,v_max=min(target.v_max,cap),state='ROUNDABOUT_APPROACH')
        if v<.3 and ctx.s>=rp.stop_s-rp.p.stand_m:
            if self.ring_probe_held is None:self.ring_probe_held=now
            if now-self.ring_probe_held>=1.:
                rp.committed=True;self.ring_probe_launch=now
                return replace(target,v_max=min(target.v_max,limit),state='ROUNDABOUT_ENTER')
        cap=rp.stop_speed(ctx.s)
        out=replace(target,v_max=min(target.v_max,cap if not (v<.3 and ctx.s>=rp.stop_s-rp.p.stand_m) else 0.),state='ROUNDABOUT_WAIT')
        if rp.stop_s-ctx.s<=2.:out=replace(out,stop_s=rp.stop_s+3.)
        return out

    def plan_ring_merge(self, ctx, now):
        """Entry speed plan from the fresh ring tracks (ring_merge.RingMerge)."""
        merge=self.ring_merge
        if merge is None or abs(merge.s_offset-ctx.s_offset)>.5:
            merge=self.ring_merge=RingMerge(self.circle_reference,ctx.route,ctx.s_offset,
                                            self.entry_gate.approach_s,self.entry_gate.end_s)
            merge.s_offset=ctx.s_offset
            self.entry_gate.ring_merge=merge
        if merge.table is None:
            return dict(model='ring_merge',decision='UNKNOWN',reason='ring geometry unavailable',evaluated_at=now)
        gap=self.circular_gap
        if gap.local_entry and not 0<=now-gap.scan_stamp<=.45:
            return dict(model='ring_merge',decision='UNKNOWN',reason='latest lidar scan unavailable',evaluated_at=now)
        def extents(tr,age):
            lo,hi=gap.extents(tr,age)
            margin=.6+.35*age      # circular_gap's own padding: ring_merge adds its margins and growth
            return lo+margin,hi-margin
        bodies=merge.coast(ring_bodies(gap.tracks.values(),merge,now,extents=extents),now)
        stop_s=self.entry_gate.stop_at_s if self.entry_gate.stop_at_s>=0. else self.entry_gate.entry_s
        lidar=(ctx.x+merge.lidar_x_m*math.cos(ctx.yaw),ctx.y+merge.lidar_x_m*math.sin(ctx.yaw))
        result=merge.plan(ctx.s,max(0.,ctx.v),lidar,bodies,stop_s,self.entry_gate.max_speed_kmh/3.6,now)
        result['evidence_stamp']=gap.scan_stamp if gap.local_entry else now
        result['admission_model']='three_vehicle_entry'
        result['lane_tracks']=result['tracks']=len(bodies)
        return result

    def ego_local_s(self, x, y) -> Optional[float]:
        """자차의 local s. s_offset 을 아직 모르면 여기서 잰다 (경로 전체에서 한 번 찾는다)."""
        if self.route is None or self.s_global is None:
            return None
        if self.s_offset is None:
            self.s_offset = s_offset_from(self.route, x, y, self.s_global)
            self.get_logger().info(f'global-s offset {self.s_offset:.1f} m')
        return self.s_global + self.s_offset

    def on_control_telemetry(self, msg):
        try:
            data=json.loads(msg.data)
            if (isinstance(data,dict) and all(isinstance(data.get(k),(int,float))
                    and math.isfinite(data[k]) for k in ('stamp','target_speed_mps'))
                    and -.05<=self.now()-data['stamp']<=.5):
                self.control_display=data
        except (ValueError,TypeError):
            pass

    def cache_event_footprints(self, data, stamp):
        """Keep one scan, joining its immutable map outlines by lidar identity."""
        if (data.get('frame') != 'map' or not data.get('scan_valid', True)
                or stamp < self.footprint_stamp or not isinstance(data.get('objects'), list)):
            return
        hulls = {};heights = {};kinds = {};speeds = {};cautions = set()
        for obj in data['objects']:
            if not isinstance(obj, dict):
                continue
            identity, points = obj.get('id'), obj.get('xy')
            if (not isinstance(identity, str) or not identity.startswith('lidar:')
                    or not isinstance(points, list) or len(points) < 3):
                continue
            if not all(isinstance(p, list) and len(p) == 2
                       and all(type(v) in (int, float) and math.isfinite(v) for v in p)
                       for p in points):
                continue
            hull = tuple((float(x), float(y)) for x, y in points)
            area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(hull, hull[1:]+hull[:1]))
            if not math.isfinite(area) or abs(area) <= 1e-12:
                continue
            hulls[identity] = hull
            if type(obj.get('height')) in (int, float) and math.isfinite(obj['height']):
                heights[identity] = float(obj['height'])
            kinds[identity] = obj.get('kind')
            if obj.get('caution') is True:
                cautions.add(identity)
            if all(type(obj.get(k)) in (int, float) and math.isfinite(obj[k]) for k in ('vx', 'vy')):
                speeds[identity] = (float(obj['vx']), float(obj['vy']))   # map-frame velocity
        self.footprint_stamp, self.footprints = stamp, hulls
        self.footprint_heights = heights
        self.footprint_kinds = kinds
        self.footprint_speeds = speeds
        self.footprint_cautions = cautions
        self.footprint_cleared = tuple(q for q in data.get('cleared', []) if isinstance(q,dict))
        with self.event_lock:
            current = list(self.events.values())
        for info in current:
            if abs(info.stamp-stamp) <= 1e-6:
                info.footprint = hulls.get(info.value, ())

    def on_footprints(self,msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            stamp = data.get('stamp', -math.inf)
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not self.evidence_after <= stamp <= self.now() + .05):
                return
            self.cache_event_footprints(data, stamp)
            if self.highway is not None:self.highway.accept(data,self.now())
            walls = data.get('tunnel_walls')
            if (walls and self.route is not None and self.s_global is not None and self.s_offset is not None
                    and self.tunnel_walls.add(walls, self.route, self.s_global+self.s_offset, self.s_offset, to_frenet,
                                            self.road_map)
                    and (self.tunnel_corridor is None or self.tunnel_walls.due(self.now()))):
                tunnel = next((st for st in self.behavior.states if st.name == 'TUNNEL'), None)
                self.tunnel_corridor = self.tunnel_walls.corridor(
                    self.road_map, end_s=getattr(tunnel, 's_to', None))   # never extend past the tunnel
            if self.ring_planner is not None and getattr(self,'ego_xy',None) is not None and data.get('scan_valid',True):
                hulls={str(o['id']):o['xy'] for o in data.get('objects',[]) if isinstance(o,dict) and o.get('xy')}
                heights={str(o['id']):o.get('height') for o in data.get('objects',[]) if isinstance(o,dict) and o.get('xy')}
                if self.ring_planner.lane.update(stamp,hulls,self.ego_xy,heights) is not False and self.ring_tuck:
                    # practice: every run as measured this scan (ends, whether each end was measured, end speeds, the
                    # speed used) to rate the ring measurement offline
                    rr=[r for r in self.ring_planner.lane.runs if r.coasted==0.]
                    if rr:
                        f=lambda e:None if e is None else [round(float(e[0]),3),round(float(e[1]),3)]
                        self.get_logger().info('RB_RUNS '+json.dumps(dict(t=round(stamp,3),runs=[dict(lo=r.lo,hi=r.hi,
                            lc=r.lo_clear,hc=r.hi_clear,lh=r.lo_hidden,hh=r.hi_hidden,ls=f(r.lo_speed),hs=f(r.hi_speed),
                            sp=f(r.speed),h=round(r.height,2)) for r in rr])))
            previous_scan=self.circular_gap.scan_stamp
            self.circular_gap.on_footprints(data)
            if (self.circular_gap.local_entry and self.circular_gap.scan_stamp>previous_scan
                    and (self.mission is None or (self.mission.policy is not None and self.mission.policy.roundabout))):
                # Re-evaluate the new scan on the next control tick, rather
                # than holding a departed body's veto for the 5 Hz cache period.
                self.prediction_at=-math.inf
        except (ValueError,TypeError,KeyError):
            return

    def on_observation(self, msg: Event):
        self.on_event(msg, path_relevant=False)

    def on_event(self, msg: Event, path_relevant=True):
        with self.event_lock:
            self.take_event(msg, path_relevant)

    def take_event(self, msg: Event, path_relevant=True):
        t = self.now()
        if msg.value.startswith('lidar:'):
            observed = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if observed > 0.:
                t = observed
        if not self.evidence_after <= t <= self.now() + .05:
            return
        info = EventInfo(type=msg.type, stamp=t, path_relevant=path_relevant, s=msg.s, x=msg.x, y=msg.y, size_x=msg.size_x,
                         size_y=msg.size_y, vx=msg.vx, vy=msg.vy, value=msg.value, confidence=msg.confidence)
        if abs(t-self.footprint_stamp) <= 1e-6:
            info.footprint = self.footprints.get(info.value, ())
        ego = self.ego()
        if msg.header.frame_id == 'map':
            info.box_yaw = 0.
        if msg.header.frame_id == 'base_link' and ego is not None:
            scan = self.pose_at(t) if msg.value.startswith('lidar:') else None
            px, py, pyaw = scan if scan is not None else ego[:3]
            info.box_yaw = pyaw
            info.x, info.y = base_to_map(msg.x, msg.y, px, py, pyaw)
        if ego is not None and (msg.x or msg.y):
            near = self.ego_local_s(ego[0], ego[1])
            if near is not None:
                s_local, d = to_frenet(self.route, info.x, info.y, near)
                info.s, info.d = s_local - self.s_offset, d
        # Stable lidar identity replaces the previous position and type in one operation.
        key = event_key(info)
        previous = self.events.get(key)
        # Topics may arrive out of order; an older event must not revive a
        # road user already observed outside the driving corridor.
        if previous is None or info.stamp >= previous.stamp:
            self.events[key] = info

    def reset_mission_runtime(self, clear_observations=False):
        self.tunnel_walls.clear()
        self.tunnel_corridor = None
        if self.highway is not None:self.highway.reset()
        self.behavior.reset()
        self.entry_gate.reset()
        self.entry_gate.previous_s = None
        self.circular_gap.following_lead = None
        self.circular_gap.following_rear = None
        self.circular_gap.distance_history = []
        self.prediction_result = None
        self.prediction_at = -math.inf
        self.control_display = {}
        if clear_observations:
            self.evidence_after = self.now()
            with self.event_lock:
                self.events.clear()
            self.footprint_stamp = -math.inf
            self.footprints.clear()
            self.prediction.tracks.clear()
            self.prediction.retired.clear()
            old = self.circular_gap
            self.circular_gap = CircularGap(old.circle, horizon=old.horizon, step=old.step,
                cooperative_rear=old.cooperative_rear,
                rear_uncertainty_mps=old.rear_uncertainty_mps, rear_margin_m=old.rear_margin_m)
            self.circular_gap.following_lead = None
            self.circular_gap.following_rear = None
            self.circular_gap.distance_history = []
            self.circular_gap.local_entry = old.local_entry
            self.circular_gap.departure_accels = old.departure_accels
            self.s_offset = None

    # ------------------------------------------------------------------ 틱
    def tick(self):
        with self.event_lock:
            scan, self.pending_footprints = self.pending_footprints, None
        if scan is not None:
            self.on_footprints(scan)
        if self.signal_stop is not None and self.s_global is not None:
            self.signal_stop.current(self.s_global)
        ego = self.ego()
        if ego is None or self.s_global is None:
            return                                   # 위치를 모르면 아무 말도 안 한다 → tracker 가 세운다
        t = self.now()
        x, y, yaw, v = ego
        with self.event_lock:
            self.events = {k: e for k, e in self.events.items() if -.05 <= t - e.stamp <= self.event_timeout}
            current = list(self.events.values())
        ctx = Context(s=self.s_global, x=x, y=y, yaw=yaw, v=v, now=t, events=[e for e in current if e.path_relevant],
                      observations=[e for e in current if not e.path_relevant],
                      loc_mode=self.loc_mode, route=self.route, s_offset=self.s_offset or 0.,
                      road_map=self.road_map)
        if math.isfinite(self.footprint_stamp):
            scan_pose = self.pose_at(self.footprint_stamp)
            ctx.occupancy_stamp = self.footprint_stamp
            ctx.occupancy_hulls = self.footprints
            ctx.occupancy_heights = getattr(self,'footprint_heights',{})
            ctx.occupancy_kinds = getattr(self,'footprint_kinds',{})
            ctx.occupancy_speeds = getattr(self,'footprint_speeds',{})
            ctx.occupancy_cautions = getattr(self,'footprint_cautions',set())
            ctx.occupancy_cleared = getattr(self,'footprint_cleared',())
            ctx.occupancy_sensor = tuple(scan_pose[:2]) if scan_pose is not None else (x,y)
        roundabout_owned = True  # legacy standalone behavior
        allowed_states = None
        if self.mission is not None:
            if self.mission.update(ctx):
                self.reset_mission_runtime(self.mission.reset_observations)
                if self.mission.reset_observations:
                    ctx.events = []
                    ctx.observations = []
                self.get_logger().info(f'mission → {self.mission.policy.name}: {self.mission.reason}'
                                       + (f' ({self.mission.reset_detail})' if self.mission.reason == 'localization reset' else ''))
            yielding=next((s for s in self.behavior.states if s.name=='YIELD'),None)
            ctx.yield_left_key=(yielding.left_key if yielding is not None and not self.behavior.engaged['YIELD']
                                else None)
            self.mission.apply_context(ctx)
            allowed_states = self.mission.policy.allowed_states
            roundabout_owned = self.mission.policy.roundabout
            if self.mission.policy.name == 'LOCALIZATION_INVALID':
                self.publish(Target(state='LOCALIZATION_INVALID', v_max=0.), ctx)
                return
        signal_owned = self.signal_stop is not None and (self.mission is None or self.mission.policy.signal)
        if signal_owned:
            ctx.lateral_hold_s = self.signal_stop.lateral_hold_s(ctx.s)
        if all(math.isfinite(z) for z in (x, y, self.s_global)):
            self.ego_local_s(x, y)
            ctx.s_offset = self.s_offset or 0.
        if ctx.mission_name == 'TUNNEL' and self.tunnel_corridor is not None:
            ctx.road_map = self.tunnel_corridor   # measured walls; map where unseen
        ctx.vehicle_prediction_active = roundabout_owned and self.entry_gate.owns_vehicle_prediction(ctx)
        ctx.vehicle_following_active = (ctx.vehicle_prediction_active and self.circular_gap.local_entry
                                        and self.entry_gate.committed)
        if self.box_planner_on and roundabout_owned and self.entry_gate.approach_s<=ctx.s<=self.entry_gate.end_s:
            ctx.vehicle_prediction_active=True    # ring cars are the planner's; YIELD keeps only its braking corridor
            ctx.vehicle_following_active=False
            if self.ring_tuck or self.ring_greedy or self.ring_hold:
                # On the ring the car ahead is followed on its edge-filter speed (ring_greedy_target); YIELD's corridor
                # stopped us dead 4 m behind a lead pulling away at 5 m/s (run 15:21, s 887.5).
                names=[st.name for st in self.behavior.states]
                allowed_states=[n_ for n_ in (allowed_states if allowed_states is not None else names) if n_!='YIELD']
        if all(math.isfinite(z) for z in (x,y)):
            h=ctx.yaw;self.ego_xy=(x+2.*math.cos(h),y+2.*math.sin(h))   # lidar position for the ring boxes' shadows
        ctx.traffic_circle = self.circle_reference
        measured=self.prediction_result or {}
        if (ctx.vehicle_following_active
                and measured.get('following_model')=='measured_route_clearance'
                and measured.get('decision') in ('FOLLOW','WAIT')
                and 0<=t-measured.get('evaluated_at',-math.inf)<=.45):
            ctx.measured_clearance_ids=frozenset(p['id'] for p in measured.get('observed_corridor_polygons',[]))
            ctx.measured_clearance_stamp=measured.get('evidence_stamp',-math.inf)
        target = self.behavior.tick(ctx, allowed_states=allowed_states)
        if self.mission is not None:
            yielding=next((s for s in self.behavior.states if s.name=='YIELD'),None)
            if yielding is not None and self.mission.observe_pedestrian_yield(
                    active=self.behavior.engaged['YIELD'],
                    pedestrian=yielding.target_type=='PEDESTRIAN',
                    verified_clear=yielding.pedestrian_passage_confirmed):
                self.mission.apply_context(ctx)
                # Cruise owns only the mission cap in supervisor mode. Update
                # that contribution without ticking any state twice or removing
                # another state's stop / speed request.
                cruise=self.behavior.last_targets.get('CRUISE')
                if cruise is not None:cruise.v_max=ctx.mission_speed_mps
                target.v_max=min([ctx.mission_speed_mps]+[t.v_max for t in self.behavior.last_targets.values()])
        # Entering the ring we may tuck in close behind the lead; from 1 s after entering, reopen to the ordinary gap
        # over 3 s (user 2026-09-29, like the highway cut-in).
        if roundabout_owned and self.entry_gate.committed:
            if self.entry_gate.entered_ring and self.ring_entered_at is None:self.ring_entered_at=t
            since=0. if self.ring_entered_at is None else t-self.ring_entered_at
            ctx.ring_tuck=min(1.,max(0.,1.-(since-1.)/3.))
        else:self.ring_entered_at=None
        if ctx.mission_name in ('STATIC_OBSTACLE', 'TUNNEL'):
            avoid = next((st for st in self.behavior.states if st.name == 'AVOID'), None)
            if avoid is not None and avoid.bev is not None:
                self.clearance_pub.publish(String(data=json.dumps(avoid.bev.clearance_queries(t))))
        self.prediction.observe(ctx.events + ctx.observations, t)
        self.circular_gap.sensor_xy=(ctx.x,ctx.y)   # ring tracks' Kalman range term (RingTrack.kf_update)
        self.circular_gap.observe(self.prediction, t)
        if t - self.prediction_at >= .2:
            self.prediction_at = t
            use_circle = (roundabout_owned and self.circular_gap_on and
                          self.entry_gate.approach_s <= ctx.s <= self.entry_gate.end_s)
            if use_circle:
                waiting_s=(self.entry_gate.stop_at_s if self.entry_gate.stop_at_s>=0. else self.entry_gate.entry_s)
                self.clearance_pub.publish(String(data=json.dumps(self.circular_gap.clearance_queries(t,ctx,waiting_s))))
            if use_circle:
                if not self.entry_gate.committed:self.circular_gap.following_lead=None
                def departure(mode='decel'):
                    return FollowingDeparture(self.prediction,self.circle_reference,list(self.circular_gap.tracks.values()),
                                              follow_before_contact=self.circular_gap.local_entry,mode=mode)
                if self.circular_gap.local_entry and self.entry_gate.committed:
                    self.prediction_result=evaluate_following(self.circular_gap,ctx,departure(),entered_ring=self.entry_gate.entered_ring)
                elif self.merge_planner_on:
                    self.prediction_result=self.plan_ring_merge(ctx,t)
                else:
                    # Entry: speed up, keep, ease off -- in that order of preference, the first whose departure clears the
                    # gap (user 2026-09-29); easing off is the base case.
                    chosen=None;first=None
                    for mode in DEPARTURE_STRATEGIES:
                        result=self.circular_gap.evaluate(ctx,departure(mode))
                        if first is None:first=result
                        if result.get('decision')=='CANDIDATE_GAP':chosen=dict(result,ego_strategy=mode);break
                    # Neither clears (or the ring is not verified yet): keep easing off toward the entry.
                    self.prediction_result=chosen or dict(first,ego_strategy='decel')
            else:self.prediction_result=self.prediction.evaluate(ctx)
        if roundabout_owned and self.circular_gap.local_entry and self.entry_gate.committed:
            self.prediction_result=refresh_exit_prediction(
                self.prediction_result,ctx,self.circle_reference,self.entry_gate.entered_ring)
        was_committed=self.entry_gate.committed
        upstream_state=target.state
        upstream_stop=target.v_max<=0. or math.isfinite(target.stop_s)
        caps=dict(up=target.v_max)          # RB_CAPS (practice analysis): which cap holds our speed through the entry
        if roundabout_owned:
            target = self.entry_gate.apply(target, ctx, self.prediction_result)
            pr=self.prediction_result or {}
            caps.update(gate=target.v_max,gate_why=self.entry_gate.reason,gate_committed=self.entry_gate.committed,
                        old_follow=pr.get('following_speed_mps'),old_gap=pr.get('following_gap_m'),stop_s=target.stop_s)
            # Jerk-limited braking while entering and following in the ring (user 2026-09-29); an imminent conflict
            # (ROUNDABOUT_BRAKE) and the entry stop are not shaped.
            if target.state in ('ROUNDABOUT_ENTER','ROUNDABOUT_FOLLOW') and not math.isfinite(target.stop_s):
                # ...except with the ring lead inside emergency stopping range (latency + 8 m/s2 + 1 m): brake as hard
                # as the cap says. Shaped there, two entries closed to 0.4-0.6 m and the imminent-conflict brake
                # fired (07:45 runs 6 and 8).
                gap=(self.prediction_result or {}).get('following_gap_m')
                v=max(0.,ctx.v);near=gap is not None and gap<v*.25+v*v/16.+1.
                target=replace(target,v_max=self.ring_brake.apply(target.v_max,ctx.v,ctx.now,exempt=near))
            else:self.ring_brake.reset()
            caps['shaped']=target.v_max
        if self.box_planner_on and roundabout_owned:
            target=self.apply_ring_boxes(target,ctx,t)
            caps.update(judge=getattr(self.ring_planner,'judge',None) if self.ring_planner is not None else None,
                        box=target.v_max,box_state=target.state,
                        box_cap=(self.prediction_result or {}).get('keep_gap_mps'))
            if 810.<=ctx.s<=900. and t-getattr(self,'rb_caps_at',-math.inf)>=.1:
                self.rb_caps_at=t
                def r(x):return x if not isinstance(x,float) else (round(x,2) if math.isfinite(x) else None)
                self.get_logger().info('RB_CAPS '+json.dumps(dict(t=round(t,3),s=round(ctx.s,2),v=round(ctx.v,2),
                    **{k:r(x) for k,x in caps.items()})))
            # RViz (5 Hz): never let a drawing error touch the driving
            if self.roundabout_marker_pub is not None and t-self.roundabout_marker_at>=.2:
                self.roundabout_marker_at=t
                try:
                    inside=self.entry_gate.approach_s<=ctx.s<=self.entry_gate.end_s
                    self.roundabout_marker_pub.publish(roundabout_markers(self.ring_planner if inside else None,
                        self.prediction_result,(ctx.x,ctx.y),ctx.yaw,ctx.route,ctx.s_offset,self.get_clock().now().to_msg(),
                        boxes_only=self.ring_hold or self.ring_tuck))
                except Exception as exc:
                    self.get_logger().warn(f'roundabout markers: {exc}',throttle_duration_sec=5.)
        elif self.roundabout_marker_pub is not None and self.roundabout_marker_at>-math.inf:
            # Leaving the roundabout's ownership: clear once, the last drawing (lead outline included) stayed on in RViz
            self.roundabout_marker_at=-math.inf
            self.roundabout_marker_pub.publish(roundabout_markers(None,None,(ctx.x,ctx.y),ctx.yaw,ctx.route,ctx.s_offset,
                                                                  self.get_clock().now().to_msg()))
        if self.highway_job is not None and (not self.highway_job.is_alive() or ctx.s>=self.highway_from_s-self.HIGHWAY_JOIN_M):
            self.highway_job.join();self.highway_job=None
        if self.highway is not None:
            target=self.highway.apply(target,ctx)
            if self.highway.active:
                if 0.<=t-self.control_display.get('stamp',-math.inf)<=.5:
                    self.highway.diagnostic['control']=self.control_display
                self.highway_pub.publish(String(data=json.dumps(self.highway.diagnostic)))
            if self.highway_marker_pub is not None:
                self.highway_marker_pub.publish(highway_markers(self.highway,self.get_clock().now().to_msg()))
        if self.mission is not None:
            target = self.mission.limit(target)
        if self.circular_gap.local_entry and self.entry_gate.committed and not was_committed:
            begin_following(self.circular_gap,self.prediction_result)
        ctx.vehicle_following_active = bool(roundabout_owned and self.entry_gate.owns_vehicle_prediction(ctx)
                                            and self.circular_gap.local_entry and self.entry_gate.committed)
        if self.signal_stop is not None:
            line = self.signal_stop.current(ctx.s) if signal_owned else None
            self.signal_camera_pub.publish(Bool(data=line is not None))
            if signal_owned:
                target = self.signal_stop.apply(target, ctx.s, ctx.now)
            decision = ('OFF' if line is None else 'UNKNOWN' if self.signal_stop.color is None
                        else 'GO' if self.signal_stop.go(line) else 'STOP')
            self.signal_debug.publish(String(data=json.dumps(dict(
                camera=line is not None, decision=decision, mission=ctx.mission_name,
                intersection=self.signal_stop.key, color=self.signal_stop.color,
                maneuver=line['maneuver'] if line else None,
                age_s=ctx.now-self.signal_stop.last_stamp if math.isfinite(self.signal_stop.last_stamp) else None,
                last_seen=self.signal_stop.last_stamp if math.isfinite(self.signal_stop.last_stamp) else None,
                rear_s=ctx.s, stop_rear_s=line['stop_rear_s'] if line else None,
                stop_front_s=target.stop_s if math.isfinite(target.stop_s) else None))))
        self.publish(target, ctx)
        if t == self.prediction_at:
            diagnostic = dict(self.prediction_result,
                              mode='entry_control' if self.entry_gate.enabled and roundabout_owned else 'shadow',
                              gate_enabled=self.entry_gate.enabled and roundabout_owned,
                              mission=ctx.mission_name,
                              yield_status=yield_status(self.behavior,ctx),
                              upstream_state=upstream_state,upstream_stop=upstream_stop,
                              gate_reason=self.entry_gate.reason)
            self.prediction_pub.publish(String(data=json.dumps(diagnostic)))
            if self.prediction_marker_pub is not None:
                self.prediction_marker_pub.publish(prediction_markers(
                    self.prediction,ctx,diagnostic,target,self.get_clock().now().to_msg(),
                    self.entry_gate.reason,self.circle_reference,show_details=self.visual_details))

    def publish(self, target: Target, ctx: Context):
        msg = DrivingTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = target.state
        msg.v_max = float(target.v_max)
        msg.stop_s = float(target.stop_s)
        msg.lat_offset = float(target.lat_offset)
        msg.return_s = float(target.return_s)
        msg.return_length_m = float(target.return_length_m)
        msg.entry_s = float(target.entry_s)
        msg.entry_length_m = float(target.entry_length_m)
        if target.chain:
            msg.chain_offset = [float(d) for d, _, _ in target.chain]
            msg.chain_start_s = [float(st) for _, st, _ in target.chain]
            msg.chain_length_m = [float(ln) for _, _, ln in target.chain]
        if hasattr(msg,'reference_id'):
            msg.reference_release=target.reference_release
            if target.reference is not None:
                ref=target.reference
                msg.header.frame_id='map'
                msg.reference_id=ref.id
                msg.reference_global_s=ref.global_s.tolist()
                msg.reference_x=ref.xy[:,0].tolist();msg.reference_y=ref.xy[:,1].tolist()
        elif target.reference is not None:
            msg.v_max=0.  # An old generated interface must never silently omit a reference.
        self.target_pub.publish(msg)
        active = self.behavior.active_names()
        if target.state.startswith('ROUNDABOUT_'):
            active = [target.state] + active
        names = '|'.join(active)
        if self.mission is not None:
            mission = self.mission.policy
            guard = yield_status(self.behavior, ctx)
            reason = guard['reason'] if guard['kind']!='NONE' else self.entry_gate.reason if mission.roundabout else next(
                (getattr(s,'reason','') for s in self.behavior.states
                 if self.behavior.engaged[s.name] and getattr(s,'reason','')), '')
            self.mission_pub.publish(String(data=json.dumps(dict(
                mission=mission.name, substate=target.state, reason=reason,
                yield_status=guard,
                transition=self.mission.reason,
                speed_limit_kmh=None if math.isinf(self.mission.speed_mps) else self.mission.speed_mps*3.6,
                pedestrian_yield_completed=self.mission.pedestrian_yield_completed,
                allowed_states=sorted(mission.allowed_states)))))
        if names != self.last_state:
            self.get_logger().info(f'{self.last_state or "-"} → {names}  (s={ctx.s:.1f} v={ctx.v*3.6:.0f} km/h)')
            self.last_state = names
        self.state_pub.publish(String(data=names))
        if self.marker_pub is not None:
            self.marker_pub.publish(self.markers(target, ctx))

    def report(self):
        if self.odom is None:
            self.get_logger().warn('odometry 없음 — /behavior/target 발행 안 함')
        elif self.route is None:
            self.get_logger().warn('/map/route 없음 — Event 의 s,d 를 못 만든다 (path_follower 가 떠 있나?)')

    # ------------------------------------------------------------------ 시각화
    def markers(self, target: Target, ctx: Context) -> MarkerArray:
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if ctx.mission_name:
            arr.markers.extend(mission_markers(ctx,target,self.behavior,dict(self.prediction_result or {},display_max_age_s=self.entry_gate.max_age_s,display_gate_enabled=self.entry_gate.enabled),
                self.get_clock().now().to_msg(),control=self.control_display,
                gate_reason=self.entry_gate.reason).markers[1:])
        arr.markers.extend(required_checkpoint_markers(getattr(self,'required_checkpoints',{}),
            self.get_clock().now().to_msg()).markers)
        if self.route is None or self.s_offset is None:
            return arr
        stamp = self.get_clock().now().to_msg()
        if self.mission is not None:
            arr.markers.extend(checkpoint_markers(self.route,self.mission.zones,self.s_offset,stamp).markers)
        s0 = ctx.s + self.s_offset
        uid = [0]

        def line(points, rgba, width=0.08, ns='behavior'):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id = ns, uid[0]
            uid[0] += 1
            m.type, m.action = Marker.LINE_STRIP, Marker.ADD
            m.lifetime = Duration(sec=1)
            m.scale.x = width
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.pose.orientation.w = 1.
            m.points = [Point(x=float(px), y=float(py), z=0.1) for px, py in points]
            arr.markers.append(m)

        path_bases = {}
        show_paths = not ctx.mission_name or self.visual_details
        def offset_path(d, length):
            if length not in path_bases:
                stations = s0+np.linspace(0.,length,max(2,int(length/1.)))
                idx = np.clip(np.searchsorted(self.route.s,stations,side='right')-1,0,len(self.route.delta)-1)
                headings = np.arctan2(self.route.delta[idx,1],self.route.delta[idx,0])
                base = np.c_[np.interp(stations,self.route.s,self.route.points[:,0]),
                             np.interp(stations,self.route.s,self.route.points[:,1])]
                path_bases[length] = (base,np.c_[-np.sin(headings),np.cos(headings)])
            base,normal = path_bases[length]
            return base+d*normal

        # 후보 복도 + 장애물 박스 (AVOID 가 걸려 있을 때)
        avoid = next((s for s in self.behavior.states if s.name == 'AVOID' and self.behavior.engaged[s.name]), None)
        if avoid is not None:
            dbg = avoid.debug()
            plan = dbg.get('plan')
            if plan is not None and show_paths:
                for c in plan.candidates:
                    if c.d == plan.d and not plan.blocked:
                        continue
                    rgba = (0.9, 0.2, 0.2, 0.35) if not c.feasible else (0.6, 0.6, 0.6, 0.35)
                    line(offset_path(c.d, self.marker_horizon), rgba, 0.05, 'candidates')
            boxes = dbg.get('boxes', [])
            bev = getattr(avoid, 'bev', None) if ctx.mission_name in ('STATIC_OBSTACLE', 'TUNNEL') else None
            for b in ([] if bev is not None else boxes):
                corners = (list(b.vertices) + [b.vertices[0]] if b.vertices else
                           [(b.s_min, b.d_min), (b.s_max, b.d_min), (b.s_max, b.d_max), (b.s_min, b.d_max), (b.s_min, b.d_min)])
                line([to_xy(self.route, s + self.s_offset, d) for s, d in corners], (1., 0.5, 0., 0.9), 0.1, 'obstacles')
            if plan is not None and boxes:
                # Which side and why, beside the nearest box (always shown in AVOID).
                near = min(boxes, key=lambda b: b.s_min)
                s_mid = (near.s_min + near.s_max) / 2. + self.s_offset
                colours = {'CHOSEN': (0.1, 0.9, 0.2, 1.), 'FREE': (0.85, 0.85, 0.85, 1.), 'BLOCKED': (1., 0.2, 0.2, 1.)}
                sides = side_summary(plan, ctx.mission_name == 'TUNNEL', avoid.planner.w_left_bias)
                for name, sign in (('LEFT', 1), ('RIGHT', -1)):
                    row = sides[name]
                    rgba = colours[row['status']]
                    if row['d'] is not None:
                        span = [to_xy(self.route, s + self.s_offset, row['d'])
                                for s in (near.s_min - 3., near.s_max + 3.)]
                        line(span, rgba, 0.25 if row['status'] == 'CHOSEN' else 0.1, 'avoid_choice')
                    edge = max(b.d_max for b in boxes) if sign > 0 else -min(b.d_min for b in boxes)
                    # Outside both the box and this side's drawn offset line.
                    outer = max(edge, abs(row['d']) if row['d'] is not None else edge) + 1.8
                    x, y = to_xy(self.route, s_mid, sign * outer)
                    text = f'{name} {row["status"]}'
                    if row['status'] == 'CHOSEN':
                        text += f' {row["d"]:+.2f}m'
                    text += '\n' + (row['reason'] or f'free {row["free"][0]:.2f}-{row["free"][1]:.2f}m')
                    m = Marker()
                    m.header.frame_id, m.header.stamp = 'map', stamp
                    m.ns, m.id = 'avoid_choice_text', uid[0]
                    uid[0] += 1
                    m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                    m.lifetime = Duration(sec=1)
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 1.5
                    m.pose.orientation.w = 1.
                    m.scale.z = 0.7
                    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
                    m.text = text
                    arr.markers.append(m)
        # Occupancy is sensor evidence, including when avoidance is inactive.
        observed_avoid = next((s for s in self.behavior.states if s.name == 'AVOID'), None)
        bev = (getattr(observed_avoid, 'bev', None)
               if ctx.mission_name in ('STATIC_OBSTACLE', 'TUNNEL') else None)
        if bev is not None:
            arr.markers.extend(bev_stop_markers(ctx,target,bev.diagnostic,stamp))
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id = 'bev_occupied', uid[0]
            uid[0] += 1
            m.type, m.action = Marker.CUBE_LIST, Marker.ADD
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = bev.field.resolution
            m.scale.z = .12
            m.color = ColorRGBA(r=1.,g=.5,b=0.,a=.75)
            m.lifetime = Duration(sec=1)
            m.points = [Point(x=float(x),y=float(y),z=.2) for x,y in bev.field.occupied_cells]
            arr.markers.append(m)
        # Measured tunnel wall faces that bound the avoidance offset (cyan).
        if ctx.mission_name == 'TUNNEL' and self.tunnel_corridor is not None:
            for right, left in self.tunnel_corridor.lines():
                line(right.tolist(), (0.1, 0.9, 0.9, 0.9), 0.15, 'tunnel_walls')
                line(left.tolist(), (0.1, 0.9, 0.9, 0.9), 0.15, 'tunnel_walls')
        # 지금 tracker 에게 시킨 오프셋 (lat_offset 요청, 계단) — 실제 따라가는 곡선은 tracker 가
        # /control/path 로 낸다 (RViz 'Tracked Line', 초록 굵은 선). 요청은 가늘고 흐리게.
        rgba = (0.1, 0.9, 0.2, 0.45) if not math.isfinite(target.stop_s) else (0.95, 0.8, 0.1, 0.6)
        if show_paths:
            line(offset_path(target.lat_offset, self.marker_horizon), rgba, 0.06, 'chosen')
        # 정지 지점
        if math.isfinite(target.stop_s):
            sx, sy = to_xy(self.route, target.stop_s + self.s_offset, 0.)
            # Yellow when it is only where our sight ends (the guard's 'unobserved' stop, user 2026-10-11: red looked
            # like a real stop behind the barricade and the cones); red for a stop for something there
            diag = (getattr(getattr(next((st for st in self.behavior.states if st.name == 'AVOID'), None), 'bev', None),
                            'diagnostic', None) or {})
            sight = diag.get('reason') == 'unobserved' and abs(diag.get('stop_s', math.nan) - target.stop_s) < 1e-6
            line([to_xy(self.route, target.stop_s + self.s_offset, dd) for dd in (-1.75, 1.75)],
                 (1., 0.8, 0.1, 0.9) if sight else (1., 0.1, 0.1, 1.), 0.2, 'stop')
        if ctx.mission_name:
            if not self.visual_details:
                hidden={'chosen','candidates'}
                if ctx.mission_name not in ('STATIC_OBSTACLE','TUNNEL'):hidden.add('obstacles')
                arr.markers=[m for m in arr.markers if m.ns not in hidden]
            return arr
        # 상태판
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', stamp
        m.ns, m.id = 'text', uid[0]
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = ctx.x, ctx.y, 3.
        m.pose.orientation.w = 1.
        m.scale.z = 0.8
        m.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        text = f'{ctx.mission_name + " / " if ctx.mission_name else ""}{target.state}  s={ctx.s:.0f}  v={ctx.v*3.6:.0f}km/h\n'
        text += f'v_max={"-" if math.isinf(target.v_max) else f"{target.v_max*3.6:.0f}km/h"}  offset={target.lat_offset:+.2f}'
        if math.isfinite(target.stop_s):
            text += f'  stop_s={target.stop_s:.1f}'
        if avoid is not None and avoid.plan is not None:
            text += '\nBLOCKED' if avoid.plan.blocked else ''
        m.text = text
        arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = BehaviorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
