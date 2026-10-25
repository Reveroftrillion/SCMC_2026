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
import inspect
import math
from dataclasses import replace
import hashlib
from pathlib import Path as FilePath
from collections import deque
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
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
from .mission_markers import mission_markers, yield_status, checkpoint_markers, required_checkpoint_markers
from .prediction_markers import prediction_markers
from .circle_reference import CircularReference
from .circular_gap import CircularGap
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
        self.highway=None
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
            self.highway=HighwayRuntime(highway_config,highway_route,mission_bounds=bounds)
        self.highway_pub=self.create_publisher(String,'/behavior/highway',10)
        self.highway_marker_pub=self.create_publisher(MarkerArray,'/behavior/highway_markers',1) if self.markers_on else None

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
        self.circular_gap = CircularGap(self.circle_reference,
            cooperative_rear=p('roundabout.cooperative_rear',True).value,
            rear_uncertainty_mps=p('roundabout.rear_uncertainty_mps',.175).value,
            rear_margin_m=p('roundabout.rear_margin_m',.3).value)
        self.circular_gap.local_entry=p('roundabout.local_entry',False).value
        self.entry_gate.ring_speed=self.prediction.max_speed
        self.circular_gap_on = p('roundabout.circular_gap', True).value
        self.entry_gate.circular_mode = self.circular_gap_on
        self.ring_brake = JerkBrake()
        self.ring_entered_at = None
        self.prediction_marker_pub=(self.create_publisher(MarkerArray, '/behavior/prediction_markers', 1)
                                    if self.markers_on else None)
        self.prediction_at = -math.inf
        self.prediction_pub = self.create_publisher(String, '/behavior/prediction', 10)
        self.clearance_pub = self.create_publisher(String, '/perception/clearance_queries', 1)

        self.create_subscription(String, '/control/telemetry', self.on_control_telemetry, 1)
        self.create_subscription(String, '/perception/footprints', self.on_footprints, 10)
        self.create_subscription(Event, '/events', self.on_event, 10)
        self.create_subscription(Event, '/perception/road_users', self.on_observation, 10)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Float64, '/localization/s', self.on_s, qos_profile_sensor_data)
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
        self.s_global = float(msg.data)
        if self.signal_stop is not None:
            self.signal_stop.current(self.s_global)

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

    def on_footprints(self,msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            stamp = data.get('stamp', -math.inf)
            if not self.evidence_after <= stamp <= self.now() + .05:
                return
            if self.highway is not None:self.highway.accept(data,self.now())
            walls = data.get('tunnel_walls')
            if (walls and self.route is not None and self.s_global is not None and self.s_offset is not None
                    and self.tunnel_walls.add(walls, self.route, self.s_global+self.s_offset, self.s_offset, to_frenet,
                                            self.road_map)
                    and (self.tunnel_corridor is None or self.tunnel_walls.due(self.now()))):
                tunnel = next((st for st in self.behavior.states if st.name == 'TUNNEL'), None)
                self.tunnel_corridor = self.tunnel_walls.corridor(
                    self.road_map, end_s=getattr(tunnel, 's_to', None))   # never extend past the tunnel
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
        t = self.now()
        if msg.value.startswith('lidar:'):
            observed = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if observed > 0.:
                t = observed
        if not self.evidence_after <= t <= self.now() + .05:
            return
        info = EventInfo(type=msg.type, stamp=t, path_relevant=path_relevant, s=msg.s, x=msg.x, y=msg.y, size_x=msg.size_x,
                         size_y=msg.size_y, vx=msg.vx, vy=msg.vy, value=msg.value, confidence=msg.confidence)
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
            self.events.clear()
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
        ego = self.ego()
        if ego is None or self.s_global is None:
            return                                   # 위치를 모르면 아무 말도 안 한다 → tracker 가 세운다
        t = self.now()
        x, y, yaw, v = ego
        self.events = {k: e for k, e in self.events.items() if -.05 <= t - e.stamp <= self.event_timeout}
        ctx = Context(s=self.s_global, x=x, y=y, yaw=yaw, v=v, now=t, events=[e for e in self.events.values() if e.path_relevant],
                      observations=[e for e in self.events.values() if not e.path_relevant],
                      loc_mode=self.loc_mode, route=self.route, s_offset=self.s_offset or 0.,
                      road_map=self.road_map)
        roundabout_owned = True  # legacy standalone behavior
        allowed_states = None
        if self.mission is not None:
            if self.mission.update(ctx):
                self.reset_mission_runtime(self.mission.reset_observations)
                if self.mission.reset_observations:
                    ctx.events = []
                    ctx.observations = []
                self.get_logger().info(f'mission → {self.mission.policy.name}: {self.mission.reason}')
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
        self.prediction.observe(ctx.events + ctx.observations, t)
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
        if roundabout_owned:
            target = self.entry_gate.apply(target, ctx, self.prediction_result)
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

        def offset_path(d, length):
            n = max(2, int(length / 1.))
            return [to_xy(self.route, s0 + i * length / (n - 1), d) for i in range(n)]

        # 후보 복도 + 장애물 박스 (AVOID 가 걸려 있을 때)
        avoid = next((s for s in self.behavior.states if s.name == 'AVOID' and self.behavior.engaged[s.name]), None)
        if avoid is not None:
            dbg = avoid.debug()
            plan = dbg.get('plan')
            if plan is not None:
                for c in plan.candidates:
                    if c.d == plan.d and not plan.blocked:
                        continue
                    rgba = (0.9, 0.2, 0.2, 0.35) if not c.feasible else (0.6, 0.6, 0.6, 0.35)
                    line(offset_path(c.d, self.marker_horizon), rgba, 0.05, 'candidates')
            boxes = dbg.get('boxes', [])
            for b in boxes:
                corners = [(b.s_min, b.d_min), (b.s_max, b.d_min), (b.s_max, b.d_max), (b.s_min, b.d_max), (b.s_min, b.d_min)]
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
                    m.ns, m.id = 'avoid_choice', uid[0]
                    uid[0] += 1
                    m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                    m.lifetime = Duration(sec=1)
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 1.5
                    m.pose.orientation.w = 1.
                    m.scale.z = 0.7
                    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
                    m.text = text
                    arr.markers.append(m)
        # Measured tunnel wall faces that bound the avoidance offset (cyan).
        if ctx.mission_name == 'TUNNEL' and self.tunnel_corridor is not None:
            for right, left in self.tunnel_corridor.lines():
                line(right.tolist(), (0.1, 0.9, 0.9, 0.9), 0.15, 'tunnel_walls')
                line(left.tolist(), (0.1, 0.9, 0.9, 0.9), 0.15, 'tunnel_walls')
        # 지금 tracker 에게 시킨 오프셋 (lat_offset 요청, 계단) — 실제 따라가는 곡선은 tracker 가
        # /control/path 로 낸다 (RViz 'Tracked Line', 초록 굵은 선). 요청은 가늘고 흐리게.
        rgba = (0.1, 0.9, 0.2, 0.45) if not math.isfinite(target.stop_s) else (0.95, 0.8, 0.1, 0.6)
        line(offset_path(target.lat_offset, self.marker_horizon), rgba, 0.06, 'chosen')
        # 정지 지점
        if math.isfinite(target.stop_s):
            sx, sy = to_xy(self.route, target.stop_s + self.s_offset, 0.)
            line([to_xy(self.route, target.stop_s + self.s_offset, dd) for dd in (-1.75, 1.75)], (1., 0.1, 0.1, 1.), 0.2, 'stop')
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
