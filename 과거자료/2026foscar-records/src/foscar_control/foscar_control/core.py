"""ROS-independent wire parsing, route geometry and low speed controller."""
import math
import struct
from dataclasses import dataclass

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def ctrl_packet(accel, brake, steering, *, mode=2, gear=4):
    if not all(math.isfinite(v) for v in (accel, brake, steering)):
        raise ValueError('nonfinite command')
    body = struct.pack('<bbbfffff', mode, gear, 1, 0., 0.,
                       max(0., min(1., accel)), max(0., min(1., brake)),
                       max(-1., min(1., steering)))
    return b'#MoraiCtrlCmd$' + struct.pack('<iiii', 23, 0, 0, 0) + body + b'\r\n'


def handoff_packet(status):
    """Stop, confirm Park in Auto, then release the latched brake in Manual.

    Manual mode ignores a simultaneous gear change in the simulator, so Park
    feedback is required before sending the Manual command. None means stale.
    """
    if status is None or not math.isfinite(status['speed']) or abs(status['speed']) >= .1:
        return ctrl_packet(0., 1., 0.)
    if status['gear'] != 1:
        return ctrl_packet(0., 1., 0., gear=1)
    return ctrl_packet(0., 0., 0., mode=1, gear=1)


def parse_status(raw):
    if len(raw) != 181 or raw[:11] != b'#MoraiInfo$' or raw[-2:] != b'\r\n':
        raise ValueError('not Competition Vehicle Status (181 bytes)')
    if struct.unpack_from('<i', raw, 11)[0] != 152:
        raise ValueError('status payload length')
    values = struct.unpack_from('<iibbfiff' + 'f' * 22 + '38s', raw, 27)
    if not all(math.isfinite(v) for v in values[:-1]):
        raise ValueError('nonfinite status')
    return dict(speed=values[20]/3.6, yaw=math.radians(values[19]),
                mode=values[2], gear=values[3], steer_deg=values[29],
                link=values[30].rstrip(b'\0').decode('ascii'))


def parse_imu(raw):
    if len(raw) != 115 or raw[:9] != b'#IMUData$' or raw[-2:] != b'\r\n':
        raise ValueError('IMU framing')
    if struct.unpack_from('<i', raw, 9)[0] != 88:
        raise ValueError('IMU payload length')
    values = struct.unpack_from('<ii10d', raw, 25)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite IMU')
    q = np.array(values[2:6])  # w,x,y,z
    norm = np.linalg.norm(q)
    if not 0.9 < norm < 1.1:
        raise ValueError('invalid quaternion norm')
    return q/norm, values[6:9], values[9:12], values[:2]


def quaternion_yaw(w, x, y, z):
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def nmea_coordinate(value, direction):
    value = float(value)
    degrees = int(value/100)
    minutes = value-degrees*100
    if direction not in ('N', 'S', 'E', 'W') or not 0 <= minutes < 60:
        raise ValueError('invalid NMEA coordinate')
    result = (degrees+minutes/60)*(-1 if direction in ('S', 'W') else 1)
    if not math.isfinite(result):
        raise ValueError('nonfinite coordinate')
    return result


def parse_gps(raw):
    fixes = []
    for line in raw.decode('ascii').strip('\0\r\n ').splitlines():
        body, check = line.strip().rsplit('*', 1)
        checksum = 0
        for char in body[1:]:
            checksum ^= ord(char)
        if not body.startswith('$') or checksum != int(check[:2], 16):
            raise ValueError('NMEA checksum')
        f = body.split(',')
        if f[0][-3:] == 'GGA':
            valid = int(f[6] or 0) > 0
            fix = dict(valid=valid, utc=f[1], kind='GGA')
            if valid:
                fix.update(lat=nmea_coordinate(f[2], f[3]), lon=nmea_coordinate(f[4], f[5]),
                           alt=float(f[9] or 0))
        elif f[0][-3:] == 'RMC':
            valid = f[2] == 'A'
            fix = dict(valid=valid, utc=f[1], kind='RMC')
            if valid:
                fix.update(lat=nmea_coordinate(f[3], f[4]), lon=nmea_coordinate(f[5], f[6]), alt=0.)
        else:
            continue
        if valid and (not -90 <= fix['lat'] <= 90 or not -180 <= fix['lon'] <= 180):
            raise ValueError('coordinate out of range')
        if valid and fix['lat'] == 0 and fix['lon'] == 0:
            # GPS shadow (tunnel s=1881-1998): the simulator keeps the fix flag but zeroes lat/lon.
            fix['valid'] = False
        fixes.append(fix)
    return fixes


class SteeringMap:
    """Provisional monotone calibration; +0.5 point was measured at 17 km/h."""
    def __init__(self, angles_deg, commands):
        self.angles = np.radians(angles_deg)
        self.commands = np.array(commands, dtype=float)
        if (len(self.angles) < 2 or len(self.angles) != len(self.commands)
                or not np.all(np.isfinite(self.angles)) or not np.all(np.isfinite(self.commands))
                or self.angles[0] != 0 or self.commands[0] != 0
                or np.any(np.diff(self.angles) <= 0) or np.any(np.diff(self.commands) <= 0)
                or self.commands[-1] > 1):
            raise ValueError('invalid steering calibration')

    def command(self, radians):
        return math.copysign(float(np.interp(abs(radians), self.angles, self.commands)), radians)


class Route:
    def __init__(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2 or not np.all(np.isfinite(points)):
            raise ValueError('invalid route')
        keep = np.r_[True, np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) > 1e-6]
        self.points = points[keep, :2]
        self.delta = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if len(self.lengths) == 0 or self.lengths.max() > 5:
            raise ValueError('empty route or route gap > 5m')
        self.s = np.r_[0., np.cumsum(self.lengths)]
        # +-4 m: polyline jitter no longer reads as tight bends (+-1.5 m rated the R 50-75 m
        # arcs of CP7-8 / CP15-end as R 33-45 m). +-5 m and wider started diluting a real
        # 12 m long R 9 m corner on the spawn route (tracking error 0.29 -> 0.36+ m). 2026-09-26.
        self.curvature = self._curvature(window=4., dilate=4.)

    def _curvature(self, window, dilate):
        """|d(heading)/ds| over +-window metres (suppresses 0.5 m sampling noise), then the max over
        +-dilate metres so corner entry/exit are never rated straighter than the corner itself."""
        heading = np.unwrap(np.arctan2(self.delta[:, 1], self.delta[:, 0]))
        heading = np.r_[heading, heading[-1]]
        lo = np.searchsorted(self.s, self.s-window)
        hi = np.minimum(np.searchsorted(self.s, self.s+window), len(self.s)-1)
        span = self.s[hi]-self.s[lo]
        kappa = np.where(span > 0, np.abs(heading[hi]-heading[lo])/np.maximum(span, 1e-9), 0.)
        lo = np.searchsorted(self.s, self.s-dilate)
        hi = np.minimum(np.searchsorted(self.s, self.s+dilate), len(self.s)-1)
        # Windowed max without a Python loop (routes are built many times per planning tick).
        width = int((hi-lo).max())+1 if len(lo) else 1
        index = lo[:, None]+np.arange(width)[None, :]
        padded = np.r_[kappa, -np.inf]
        index = np.where(index <= hi[:, None], index, len(kappa))
        return padded[index].max(axis=1)

    def speed_profile(self, cruise, a_lat, a_dec, a_acc=1.0, min_radius=4.3, curvature=None):
        """Per-point speed cap: lateral acceleration limit, a backward pass so the car brakes at
        a_dec before each corner, and a forward pass so it accelerates out at no more than a_acc
        (full throttle while still steering pushed cross-track past 0.4 m at 30 km/h).
        Radii below the steering limit are clamped."""
        if not np.all(np.asarray(cruise)>0) or np.any(np.asarray(a_lat) <= 0) or np.any(np.asarray(a_dec) <= 0) or a_acc <= 0:
            raise ValueError('speed profile limits must be positive')
        kappa = np.minimum(self.curvature if curvature is None else curvature, 1./min_radius)
        profile = np.minimum(cruise, np.sqrt(a_lat/np.maximum(kappa, 1e-9)))
        dec = np.broadcast_to(np.asarray(a_dec, dtype=float), profile.shape)   # per point (highway_decel) or one value
        for i in range(len(profile)-2, -1, -1):
            profile[i] = min(profile[i], math.sqrt(profile[i+1]**2+2*dec[i]*self.lengths[i]))
        for i in range(1, len(profile)):
            profile[i] = min(profile[i], math.sqrt(profile[i-1]**2+2*a_acc*self.lengths[i-1]))
        return profile

    def speed_at(self, s, profile):
        return float(np.interp(float(np.clip(s, 0, self.s[-1])), self.s, profile))

    def project(self, x, y, previous=None):
        lo, hi = 0, len(self.delta)
        if previous is not None:
            lo = max(0, int(np.searchsorted(self.s, previous-3))-1)
            hi = min(hi, int(np.searchsorted(self.s, previous+8))+1)
        idx = np.arange(lo, hi)
        vectors = self.delta[idx]
        t = np.clip(np.sum((np.array([x, y])-self.points[idx])*vectors, axis=1)/self.lengths[idx]**2, 0, 1)
        projected = self.points[idx]+t[:, None]*vectors
        distance = np.linalg.norm(projected-np.array([x, y]), axis=1)
        best = int(np.argmin(distance))
        i = idx[best]
        return (float(self.s[i]+t[best]*self.lengths[i]), float(distance[best]),
                math.atan2(self.delta[i, 1], self.delta[i, 0]))

    def at(self, s):
        s = float(np.clip(s, 0, self.s[-1]))
        return float(np.interp(s, self.s, self.points[:, 0])), float(np.interp(s, self.s, self.points[:, 1]))

    def heading_at(self, s):
        i = min(max(int(np.searchsorted(self.s, float(s), side='right'))-1, 0), len(self.delta)-1)
        return math.atan2(self.delta[i, 1], self.delta[i, 0])


def select_route(x, y, global_points, spawn_points, tolerance=5.):
    """Pick the route the car is actually on. Returns (Route, s_offset) where s_offset is what to
    subtract from that route's s to get GLOBAL route s (mission positions such as stop lines are
    always given in global s). On the global lap the offset is 0; on the spawn connector it is the
    connector length, so reported s is negative until the car joins the lap."""
    lap = Route(global_points)
    if lap.project(x, y)[1] <= tolerance:
        return lap, 0.
    spawn = Route(spawn_points)
    if spawn.project(x, y)[1] <= tolerance:
        return spawn, float(spawn.s[-1]-lap.s[-1])
    raise ValueError(f'car is {min(lap.project(x, y)[1], spawn.project(x, y)[1]):.0f} m from both routes')


FRONT_AXLE = 3.0   # m ahead of the rear axle (route s is measured at the rear axle)


class PoseEstimator:
    """GPS + IMU yaw + 속도 → (x, y). GPS 가 끊겨도 위치를 계속 낸다 (ROS 없음, localization 이 쓴다).

    매 틱 update(dt, yaw, speed, fix) 를 부른다. fix 는 이번 틱에 새로 들어온 유효 GPS 위치 (x, y) 또는 None.
      GPS  : 새 fix 가 오면 그 위치. fix 사이(30 Hz 대 50 Hz 틱)는 속도×yaw 로 이어 붙여 계단이 없다.
      DR   : fix 가 gps_timeout 초 없으면 마지막 위치에서 speed×(cos yaw, sin yaw) 를 적분 (추측항법).
             터널 118 m (s=1881~1998) 이 존재 이유. IMU 는 절대 yaw 를 주고 무노이즈라 드리프트가 거의 없다.
      복귀 : GPS 가 돌아오면 DR 오차만큼을 blend_s 초(오차가 크면 blend_rate m/s 로 더 길게) 에 걸쳐
             지운다. 한 번에 점프하면 tracker 가 'position jump' 로 보거나 핸들을 홱 꺾는다.
    """
    def __init__(self, gps_timeout=0.3, blend_s=2.0, blend_rate=2.0):
        if gps_timeout <= 0 or blend_s <= 0 or blend_rate <= 0:
            raise ValueError('gps_timeout, blend_s, blend_rate must be positive')
        self.gps_timeout, self.blend_s, self.blend_rate = gps_timeout, blend_s, blend_rate
        self.x = self.y = None
        self.mode = 'INIT'          # 'INIT' (아직 fix 없음) | 'GPS' | 'DR'
        self.gps_age = math.inf
        self.correction = (0., 0.)  # 복귀 직후 DR 오차 (DR 위치 - GPS 위치)
        self.blend_elapsed = self.blend_total = 0.
        self.dr_distance = 0.       # 이번(또는 마지막) DR 구간에서 적분한 거리 [m]
        self.last_error = 0.        # 마지막 복귀 때 측정된 DR 오차 [m]
        self.event = ''             # 'FIRST' | 'DR' | 'REACQUIRED' — 바뀐 틱에만 채워짐 (로그용)

    def update(self, dt, yaw, speed, fix=None):
        self.event = ''
        if self.x is not None:
            self.x += speed*math.cos(yaw)*dt
            self.y += speed*math.sin(yaw)*dt
            if self.mode == 'DR':
                self.dr_distance += abs(speed)*dt
        self.gps_age += dt
        self.blend_elapsed += dt
        if fix is not None:
            self.gps_age = 0.
            gx, gy = fix
            if self.x is None:
                self.x, self.y, self.mode = gx, gy, 'GPS'
                self.event = 'FIRST'
                return self.mode
            if self.mode == 'DR':
                self.correction = (self.x-gx, self.y-gy)
                self.last_error = math.hypot(*self.correction)
                self.blend_elapsed = 0.
                self.blend_total = max(self.blend_s, self.last_error/self.blend_rate)
                self.mode, self.event = 'GPS', 'REACQUIRED'
            weight = max(0., 1.-self.blend_elapsed/self.blend_total) if self.blend_total > 0 else 0.
            self.x, self.y = gx+self.correction[0]*weight, gy+self.correction[1]*weight
        elif self.mode == 'GPS' and self.gps_age > self.gps_timeout:
            self.mode, self.event = 'DR', 'DR'
            self.dr_distance = 0.
        return self.mode


@dataclass
class Target:
    """ROS-free mirror of foscar_interfaces/DrivingTarget. This is THE hook for mission logic:
    an FSM (or a test) hands one of these to Controller.step() every tick.
      v_max      speed cap [m/s] applied on top of the curvature profile; 0 stops the car
      stop_s     stop with the FRONT wheels at this route distance [m]; NaN = no stop
      lat_offset drive this far left (+) of the route [m]; the tracker blends it in over an s-curve
      state      name of the FSM state, logged only
    Absent target (None) == Target() == plain cruise."""
    v_max: float = math.inf
    stop_s: float = math.nan
    lat_offset: float = 0.
    state: str = ''
    reference: object = None
    reference_release: bool = False
    return_s: float = math.nan
    return_length_m: float = 0.  # <=0: no scheduled return (legacy target)
    entry_s: float = math.nan
    entry_length_m: float = 0.  # <=0: start the transition at the current position
    chain: tuple = ()           # ((d, global start s, length), ...): every move at once (tunnel slalom)



@dataclass
class Output:
    state: str
    reason: str
    accel: float = 0.
    brake: float = 1.
    steering: float = 0.
    progress: float = 0.
    error: float = 0.
    target_speed: float = 0.
    target_x: float = 0.
    target_y: float = 0.


class Controller:
    EMERGENCY_FROM_MPS = 6.25   # speed above the target where brake 0.5 saturates; brake 1.0 at +5 m/s beyond
    def __init__(self, route, speed_kmh=5., distance=20., lookahead=3., max_error=1.5, continuous=False,
                 a_lat=2.5, a_dec=1.5, accel_max=0.4, lookahead_gain=1.0, lookahead_max=10., a_acc=1.0,
                 steer_tau=0.1, lateral_ki=0.03, s_offset=0., hold_brake=0.15,
                 transition_time_s=1.5, return_time_s=2.0, transition_min_m=6.,entry_accel_max=.6, accel_rise_per_s=3.,
                 uncapped_zones=(), profile_global_s=None, a_lat_zones=(), finish_decel=1.0, dropout_hold_s=1.0, boost_zones=(),
                 reference_a_acc=None, reference_lookahead_max=None, steer_predict_s=0.,
                 reference_lateral_ki=None, reference_steer_predict_s=None, fast_kmh=None, fast=None,
                 emergency_brake=False, highway_decel=None, exit_decel=None, exit_decel_m=0., brake_steer=0., brake_steer_zones=()):
        # 60 km/h is the legal limit outside the highway exception zone; above it is untested (2026-09-15).
        if not 0 < speed_kmh <= 100 or not 0 < distance <= 150 or lookahead <= 0 or max_error <= 0:
            raise ValueError('limits: speed (0,100] km/h, distance (0,150] m')
        if not 0 < accel_max <= 1 or lookahead_gain < 0 or lookahead_max < lookahead:
            raise ValueError('invalid accel_max or lookahead gain')
        self.active_reference = None
        self.reference_controller = None
        self.reference_options = dict(speed_kmh=speed_kmh,distance=distance,lookahead=lookahead,
            max_error=max_error,continuous=True,a_lat=a_lat,a_dec=a_dec,accel_max=accel_max,
            lookahead_gain=lookahead_gain,lookahead_max=reference_lookahead_max or lookahead_max,a_acc=reference_a_acc or a_acc,
            steer_tau=steer_tau,lateral_ki=lateral_ki,hold_brake=hold_brake,
            entry_accel_max=entry_accel_max,accel_rise_per_s=accel_rise_per_s,a_lat_zones=tuple(a_lat_zones),
            steer_predict_s=steer_predict_s,emergency_brake=True,brake_steer=brake_steer,brake_steer_zones=tuple(brake_steer_zones),
            fast=None if reference_lateral_ki is None and reference_steer_predict_s is None else
                 (lateral_ki if reference_lateral_ki is None else reference_lateral_ki,
                  steer_predict_s if reference_steer_predict_s is None else reference_steer_predict_s,
                  tuple(fast_kmh or (80., 95.))))
        # Steer from where the car will be when the command takes effect (sim steering lags ~0.3 s): at 110 km/h a
        # fixed 14 m preview with that lag kept a ~2 s sway of +-0.2 m / +-0.5 deg steering (2026-10-01, no NPCs).
        self.steer_predict_s = steer_predict_s
        # (lateral_ki, steer_predict_s, (from_kmh, to_kmh)): steering values blended in with speed. Highway references
        # only: ki 0 + 0.1 s killed the ~2 s sway above 90 km/h but tracked the 42-65 km/h lane changes worse
        # (max 0.34-0.71 m on change curves vs 0.27-0.48, 2026-10-01), so below from_kmh the usual values stay.
        self.fast = fast
        # Brake past 0.5 only on the highway's references (user 2026-10-01): elsewhere an AVOID cap stepping 59 -> 26
        # km/h braked 0.8 at the static obstacles (s283), where 0.5 had always done.
        self.emergency_brake = emergency_brake
        self.route, self.speed = route, speed_kmh/3.6
        self.a_lat_recover = a_lat      # route lateral limit (measured sweep), also bounds the way back
        self.jump_time = 0.             # seconds the pose has been jumping (see step)
        self.distance, self.lookahead, self.max_error = distance, lookahead, max_error
        self.continuous = continuous
        # 2026-09-15 measured: accel 0.3 -> 1.3 m/s2, brake 0.2 -> 3.1 m/s2, brake 1.0 -> 9 m/s2.
        self.accel_max, self.lookahead_gain, self.lookahead_max = accel_max, lookahead_gain, lookahead_max
        if not 0<entry_accel_max<=1.:raise ValueError('invalid entry acceleration limit')
        self.entry_accel_max=entry_accel_max
        self.uncapped_zones=tuple(uncapped_zones)
        # global stations where the car speeds up at entry_accel_max like the roundabout entry (user 2026-10-04: out of
        # the roundabout the P pedal took 25 -> 35 km/h over 13 m at 1.3-1.7 m/s2 with 60 allowed)
        self.boost_zones=tuple(boost_zones)
        stations=route.s-s_offset if profile_global_s is None else np.asarray(profile_global_s,dtype=float)
        if stations.shape!=route.s.shape or not np.isfinite(stations).all() or np.any(np.diff(stations)<=0):
            raise ValueError('invalid profile global stations')
        cruise=np.full(len(route.s),self.speed)
        # Highway stations: rate curvature over +-10 m. The CP12 link joint is an 11 deg
        # polyline kink that +-4 m still read as a ~37 km/h bend (2026-09-27); there are
        # no short tight corners there, unlike the urban R 9 m corner that needs +-4 m.
        kappa=route.curvature.copy()
        wide=None
        for begin,end in self.uncapped_zones:
            if not all(math.isfinite(v) for v in (begin,end)) or begin>=end:
                raise ValueError('invalid uncapped speed interval')
            zone=(stations>=begin)&(stations<end)
            cruise[zone]=math.inf
            if wide is None:wide=route._curvature(window=10., dilate=10.)
            kappa[zone]=wide[zone]
            # Be under the ordinary limit when the zone ends: braking lagged a 2.5 m/s2
            # ramp by ~2 km/h and crossed CP13 at 62 km/h (2026-09-27).
            cruise[(stations>=end-15.)&(stations<end)]=self.speed-3/3.6
            entry=int(np.searchsorted(stations,begin))
            if 0<entry<len(stations):cruise[entry]=self.speed
            # Reach the ordinary limit before a boundary between sampled points.
            # This also keeps interpolated speeds at the exact checkpoint legal.
            boundary=int(np.searchsorted(stations,end))
            if 0<boundary<len(stations):cruise[boundary-1]=min(cruise[boundary-1],self.speed)
        # Per-station lateral limit: stretches without missions may corner harder (e.g. CP15 to
        # the finish, R 40-57 m arcs held to ~40 km/h at 2.5 m/s2; 2026-09-27). Not a speed cap.
        lateral=np.full(len(route.s),float(a_lat))
        for begin,end,value in a_lat_zones:
            if not all(math.isfinite(v) for v in (begin,end,value)) or begin>=end or value<=0:
                raise ValueError('invalid a_lat zone')
            lateral[(stations>=begin)&(stations<end)]=value
        # Braking inside an uncapped (highway) zone at highway_decel, into its bends and its end (user 2026-10-04: comfort
        # is not scored, hold the top speed and brake late; at 2.5 m/s2 the ramp to the ordinary cap began ~s1600).
        decel=np.full(len(route.s),float(a_dec))
        if highway_decel:
            for begin,end in self.uncapped_zones:decel[(stations>=begin)&(stations<end)]=highway_decel
        # ...except its last exit_decel_m (CP12 -> CP13), braked at exit_decel (user 2026-10-05): at 6 m/s2 the car sped up to
        # 107 km/h past CP12 and braked 0.3-0.4 on the bending rejoin; the yaw swayed ~3x as much as at 4 m/s2.
        if exit_decel:
            for begin,end in self.uncapped_zones:decel[(stations>=max(begin,end-exit_decel_m))&(stations<end)]=exit_decel
        self.decel=decel
        # Less steering correction while braking (user 2026-10-05): braking 0.27 from 101 km/h (CP12 -> CP13) the steering
        # and the yaw swung together at ~1.3 s and stopped as the brake came off -- the load on the front wheels turns the
        # car more for the same steering. The correction beyond the path's own bend is divided by (1 + brake_steer x brake) on brake_steer_zones
        # (global s). Not a yaw-rate feedback: that one came ~0.45 s late on a 1.3 s sway and fed it.
        self.brake_steer=float(brake_steer);self.brake_steer_zones=tuple(tuple(z) for z in brake_steer_zones)
        self.global_stations=stations
        # No fixed highway cap: curvature still limits speed. The backward
        # pass anticipates the return to the ordinary cap at the zone exit.
        self.profile = route.speed_profile(cruise, lateral, decel, a_acc, curvature=kappa)
        # Future limits without acceleration history from bends on an unused path.
        self.braking_profile = route.speed_profile(cruise, lateral, decel, math.inf, curvature=kappa)
        if not math.isfinite(accel_rise_per_s) or accel_rise_per_s <= 0:
            raise ValueError('invalid throttle rise rate')
        self.accel_rise_per_s = accel_rise_per_s
        self.previous_accel = 0.
        # First-order steering filter: raw pure pursuit flipped sign ~9x/s on GPS/yaw noise.
        # Keep tau small: the simulator's steering actuator already lags ~0.3 s and tau=0.3 diverged at 24 km/h.
        self.steer_tau = steer_tau
        self.steering = 0.
        # Slow integral on signed cross-track: pure pursuit alone left a 0.3 m offset on straights
        # at 40 km/h (10 m lookahead, ~-0.5 deg steering bias). Bounded to +-1.5 deg.
        self.lateral_ki, self.lateral_integral = lateral_ki, 0.
        # Route s minus s_offset == global s (see select_route). Target.stop_s and Output.progress are global.
        self.s_offset = s_offset
        # lat_offset 은 시간 램프가 아니라 **s 에 대한 곡선**으로 적용한다: 요청이 바뀐 지점 s0 에서
        # 길이 L 동안 smoothstep 으로 d_from → d_to. 시간 램프(m/s)는 기준선이 차와 무관하게 움직여
        # 속도가 빠르면 복귀 거리가 길어지고(20 km/h 11 m, 40 km/h 22 m), 빠르게 올리면 기준선이
        # 차보다 먼저 가서 횡오차 FAULT 가 났다 (2026-09-20). 곡선은 RViz 에 그대로 보이고
        # 주시점이 곡선 위를 따라가므로 조향이 자연스럽다. L = max(transition_min_m, v·T),
        # T 는 진입(transition_time_s)과 복귀(return_time_s, 0 쪽으로 줄 때) 를 따로 둔다.
        # foscar_planning avoid 는 같은 값으로 선회 시작 지점을 역산한다 (behavior.yaml 과 같게).
        self.transition_time, self.return_time, self.transition_min = transition_time_s, return_time_s, transition_min_m
        self.transition_coefficients = None
        self.return_request = None
        self.return_coefficients = None
        self.scheduled_return = None  # (local start s, bypass offset, length)
        self.offset_target = 0.     # 마지막으로 요청받은 lat_offset
        self.entry_applied = None   # (entry_s, entry_length_m) 마지막으로 적용한 진입 곡선 요청
        self.transition = None      # (s0, d_from, d_to, L) — None 이면 offset_target 을 그대로
        self.chain = None           # tunnel slalom: [(start, d_from, d_to, L, coefficients)] or None
        self.chain_request = None
        self.chain_final = 0.
        self.offset = 0.            # 현재 s 에서 곡선이 주는 오프셋 (FAULT 밴드·적분항용)
        self.a_stop = 1.0           # m/s^2 decel ramp toward a stop_s / route end
        # Signal stop lines: keep the speed and brake late and hard (comfort not scored, user 2026-10-06). The 1.0 ramp
        # crept the last 20 km/h for 3.3 s into the s309.9 line.
        self.a_signal = 4.0
        self.a_finish = finish_decel  # route-end ramp only (runout past the finish line)
        # Sensor dropout is NOT a fault: hold the wheel and brake gently until data returns (HOLD).
        # A latched FAULT ended a run on a 0.5 s simulator hitch (2026-09-19); the sim only gets
        # slower at the venue (judge/recorder load), and stopping is never the right answer to lost data.
        self.hold_brake = hold_brake
        # A short data gap preserves steering/brake. Ordinary cruise coasts so delayed
        # throttle cannot cross its speed limit; other missions preserve their last command.
        self.dropout_hold_s = dropout_hold_s
        self.previous_brake = 0.
        self.since_ok = 0.          # seconds since the last accepted pose (scales the jump check)
        self.previous = None
        self.stop_s = None
        self.state = 'WAITING'
        self.reason = 'waiting for sensors'
        self.integral = 0.
        self.cruise_previous_speed = None
        self.cruise_acceleration = 0.
        self.cruise_throttle_active = False
        self.previous_target = None  # last target speed, for the rate it is falling at
        self.target_fall = 0.        # m/s2 the target speed is falling at (filtered)

    def fail(self, reason):
        self.state, self.reason = 'FAULT', reason
        return Output(self.state, reason)

    @staticmethod
    def curve_coefficients(d0, d1, slope, second, length):
        c0, c1, c2 = d0, slope*length, .5*second*length**2
        gap = d1-c0
        return (c0, c1, c2, 10*gap-6*c1-3*c2, -15*gap+8*c1+3*c2, 6*gap-3*c1-c2)

    @staticmethod
    def curve_state(coefficients, u, length):
        d = sum(a*u**i for i, a in enumerate(coefficients))
        slope = sum(i*coefficients[i]*u**(i-1) for i in range(1, 6))/length
        second = sum(i*(i-1)*coefficients[i]*u**(i-2) for i in range(2, 6))/length**2
        return d, slope, second

    def offset_state(self, s):
        """공간 곡선의 d, d/ds, d²/ds². 재계획 때 세 경계값을 모두 이어 준다."""
        if self.chain is not None:
            return self.chain_state(s)
        if self.scheduled_return is not None and s >= self.scheduled_return[0]:
            start, _, length = self.scheduled_return
            if s >= start+length:
                return 0., 0., 0.
            return self.curve_state(self.return_coefficients, (s-start)/length, length)
        if self.transition is None:
            return self.offset_target, 0., 0.
        s0, d0, d1, length = self.transition
        if s >= s0 + length:
            return d1, 0., 0.
        c = self.transition_coefficients
        if c is None:
            c = self.curve_coefficients(d0, d1, 0., 0., length)
        return self.curve_state(c, max(0., (s-s0)/length), length)

    def offset_at(self, s):
        return self.offset_state(s)[0]

    def chain_offset(self, target, s):
        """Tunnel slalom: every move [(d, start global s, length)] as one offset profile, planned ahead
        (user 2026-09-29: the next curve appeared only once the previous one was done). A new chain starts
        from the offset, slope and curvature now; later moves start from the previous target at rest."""
        chain = tuple((float(np.clip(d, -6., 6.)), float(st)+self.s_offset, float(ln)) for d, st, ln in target.chain)
        if chain == self.chain_request:
            return
        prev = self.offset_state(s)
        segments, first = [], True
        for d, st, ln in chain:
            if st + ln <= s:                    # already driven: its target is where we are
                prev = (d, 0., 0.) if not first else prev
                continue
            start = max(s, st) if first else st
            length = max(.5, st + ln - start)
            segments.append((start, prev[0], d, length, self.curve_coefficients(prev[0], d, prev[1], prev[2], length)))
            prev, first = (d, 0., 0.), False
        self.chain, self.chain_request = segments, chain
        self.chain_final = chain[-1][0] if chain else 0.
        self.offset_target = self.chain_final
        self.transition = self.scheduled_return = self.return_request = None
        self.transition_coefficients = self.return_coefficients = None

    def chain_state(self, s):
        if not self.chain:
            return self.chain_final, 0., 0.
        for start, d0, d1, length, c in self.chain:
            if s < start:
                return d0, 0., 0.
            if s < start + length:
                return self.curve_state(c, (s-start)/length, length)
        return self.chain[-1][2], 0., 0.

    def legacy_offset(self, target, s, speed):
        """lat_offset + one entry curve + one scheduled return (every mission but the tunnel slalom)."""
        wanted = float(np.clip(target.lat_offset, -6., 6.))
        requested_return = None
        if (target.return_length_m > 0 and math.isfinite(target.return_s)
                and math.isfinite(target.return_length_m) and abs(wanted) > 1e-6):
            requested_return = (target.return_s+self.s_offset, wanted, target.return_length_m)
        return_changed = (self.scheduled_return is not None and requested_return != self.return_request
                          and (s >= self.scheduled_return[0]
                               or (requested_return is not None and s >= requested_return[0])))
        entry = (target.entry_s, target.entry_length_m)
        new_entry = (target.entry_length_m > 0 and math.isfinite(target.entry_length_m)
                     and math.isfinite(target.entry_s) and entry != self.entry_applied)
        # A new curve (new entry) is a new manoeuvre even mid-return: the tunnel slalom re-planned to
        # hold its side for a barricade seen during the return, and the return ran into it (2026-09-29).
        continuing_return = (return_changed and requested_return is not None and wanted == self.offset_target
                             and not new_entry)
        if continuing_return:
            # 뒤끝 추정이 갱신되어도 복귀 중인 차를 다시 bypass offset으로 보내지 않는다.
            # 현재 접선/곡률에서 갱신된 복귀 끝(오프셋 0)까지 이어 간다.
            d0, slope, second = self.offset_state(s)
            length = max(self.transition_min, requested_return[0]+requested_return[2]-s)
            self.return_coefficients = self.curve_coefficients(d0, 0., slope, second, length)
            self.scheduled_return = (s, d0, length)
        elif wanted != self.offset_target or return_changed or new_entry:
            returning = abs(wanted) < abs(self.offset) and wanted*self.offset >= 0
            length = max(self.transition_min, abs(speed)*(self.return_time if returning else self.transition_time))
            d0, slope, second = self.offset_state(s)
            # 오프셋이 커져도 이미 달리던 곡선의 접선/곡률을 0으로 되돌리지 않는다.
            # 진행 중 같은 방향 보정은 원래 완료 위치를 유지한다.
            if (self.transition is not None and s < self.transition[0]+self.transition[3]
                    and wanted*self.offset_target > 0 and abs(wanted) >= abs(self.offset_target)):
                length = max(self.transition_min, self.transition[0]+self.transition[3]-s)
            start = s
            if (self.transition is not None and s < self.transition[0]
                    and wanted*self.offset_target > 0):
                start, length = self.transition[0], self.transition[3]
            entry = (target.entry_s, target.entry_length_m)
            valid_entry = (target.entry_length_m > 0 and math.isfinite(target.entry_length_m)
                           and math.isfinite(target.entry_s))
            if (self.offset_target == 0. and abs(d0)+abs(slope)+abs(second) < 1e-9 and valid_entry):
                # Announce the entry while its start is still ahead of the vehicle.
                # The lookahead point then moves into the curve without a lateral jump.
                start = max(s, target.entry_s+self.s_offset)
                length = target.entry_length_m
                self.entry_applied = entry
            elif valid_entry and entry != self.entry_applied:
                # A new manoeuvre planned from the current offset (e.g. tunnel slalom):
                # its planner sized the curve to the room before the obstacle and capped
                # speed to match, so use that length instead of speed x transition_time.
                start = max(s, target.entry_s+self.s_offset)
                length = max(self.transition_min, target.entry_length_m)
                self.entry_applied = entry
            self.transition_coefficients = self.curve_coefficients(d0, wanted, slope, second, length)
            self.transition = (start, d0, wanted, length)
            if abs(d0-wanted) < 1e-9 and abs(slope)+abs(second) < 1e-9:
                self.transition = None
                self.transition_coefficients = None
            self.offset_target = wanted
        if not continuing_return and requested_return != self.return_request:
            if requested_return is not None:
                # 진입과 복귀가 겹치면 기준선이 뛰지 않도록 진입 끝 뒤로 보낸다.
                entry_end = self.transition[0]+self.transition[3] if self.transition is not None else s
                self.scheduled_return = (max(requested_return[0], entry_end), wanted, requested_return[2])
                self.return_coefficients = self.curve_coefficients(wanted, 0., 0., 0., requested_return[2])
            else:
                self.scheduled_return = None
                self.return_coefficients = None
        self.return_request = requested_return


    def reference_path(self, s, horizon=30., step=1.):
        """지금 따라가는 기준선 (route + offset 곡선) 의 (x, y) 점들 — RViz 표시용."""
        if self.active_reference is not None:
            r=self.active_reference;q=r.q_at_global(s-self.s_offset)
            return [r.at_q(qi) for qi in np.arange(q,min(q+horizon,r.q[-1])+step/2,step)]
        pts = []
        for k in range(int(horizon/step)+1):
            si = min(s+k*step, self.route.s[-1])
            px, py = self.route.at(si)
            d = self.offset_at(si)
            if d:
                h = self.route.heading_at(si)
                px, py = px-math.sin(h)*d, py+math.cos(h)*d
            pts.append((px, py))
            if si >= self.route.s[-1]:
                break
        return pts

    TARGET_FALL_TAU_S = .3

    def step(self, x, y, yaw, speed, mode, gear, healthy, dt=0.05, target=None, *, fresh_speed=True):
        """One 20 Hz control tick. `target` is the mission hook (see Target); None means cruise."""
        # Keep the parent estimate current even while a local reference owns steering.
        # Restart after missing/invalid feedback rather than differentiating across a gap.
        if healthy and fresh_speed and mode == 2 and gear == 4 and 0. < dt <= .2 and all(math.isfinite(v) for v in (x, y, yaw, speed)):
            if self.cruise_previous_speed is not None:
                measured = (speed-self.cruise_previous_speed)/dt
                self.cruise_acceleration += dt/(.15+dt)*(measured-self.cruise_acceleration)
            self.cruise_previous_speed = speed
        else:
            self.cruise_previous_speed = None
            # The actuator may still be accelerating during a brief data gap.
            # Preserve its last trend; do not differentiate across missing samples.
            if mode != 2 or gear != 4:
                self.cruise_acceleration = 0.
        # Early returns (hold, handshake, fault, stop) all send zero throttle.
        if (target is not None and target.reference is not None) or self.active_reference is not None:
            self.cruise_throttle_active = False
            if self.state in ('FAULT','FINISH'):
                return Output(self.state,self.reason,progress=(self.previous or 0.)-self.s_offset)
            if not all(math.isfinite(v) for v in (x,y,yaw,speed)):
                return Output('HOLD','invalid pose or speed: holding steering, gentle brake',0.,self.hold_brake,
                              self.steering,progress=(self.previous or 0.)-self.s_offset)
            from .reference_control import step_reference
            return step_reference(self,x,y,yaw,speed,mode,gear,healthy,dt,target or Target())
        previous_accel = self.previous_accel
        self.previous_accel = 0.
        target = target or Target()
        if self.state in ('FAULT', 'FINISH'):
            return Output(self.state, self.reason, progress=(self.previous or 0.)-self.s_offset)
        self.since_ok += dt
        if not healthy:
            if self.state != 'CRUISE':
                return Output(self.state, 'waiting for fresh GPS/IMU/status', progress=(self.previous or 0.)-self.s_offset)
            if self.since_ok <= self.dropout_hold_s+1e-9:
                self.previous_accel = 0. if self.cruise_throttle_active else previous_accel
                return Output('CRUISE', f'data gap {self.since_ok:.1f}s: holding steering and brake',
                              self.previous_accel, self.previous_brake, self.steering, progress=(self.previous or 0.)-self.s_offset)
            return Output('HOLD', f'sensor timeout {self.since_ok:.1f}s: holding steering, gentle brake',
                          0., self.hold_brake, self.steering, progress=(self.previous or 0.)-self.s_offset)
        # No latched FAULT while driving (user 2026-09-29: in the competition a stopped car cannot be
        # revived, the run is over). Each former FAULT now degrades and recovers by itself:
        #   invalid pose     -> HOLD (steering held, gentle brake) until a valid pose returns
        #   off the path     -> pure pursuit steers back from where the car is, speed capped for the way back
        #   heading > 60 deg -> the same, with the heading slow-down applied even mid-manoeuvre
        #   overspeed        -> braking only (the speed control already brakes to the profile)
        #   position jump    -> carry on the last command; a jump lasting past the dropout hold is accepted
        #   Auto/D lost      -> back to the handshake (brake, request Auto/D again), then drive on
        if not all(math.isfinite(v) for v in (x, y, yaw, speed)):
            return Output('HOLD', 'invalid pose or speed: holding steering, gentle brake', 0., self.hold_brake,
                          self.steering, progress=(self.previous or 0.)-self.s_offset)
        s, error, heading = self.route.project(x, y, self.previous)
        px, py = self.route.at(s)
        signed_error = -math.sin(heading)*(x-px)+math.cos(heading)*(y-py)   # +left of route
        # Safety checks stay here regardless of what the FSM asks for (last line of defence).
        # 오프셋 이동 중에는 차가 기준선을 못 따라와도 차선 중심과 오프셋 선 **사이**에 있으면
        # 정상이다: 그 밴드 밖으로 나간 거리만 횡오차로 본다 (2026-09-20 s=338 FAULT).
        self.offset = self.offset_at(s)
        lo, hi = min(0., self.offset), max(0., self.offset)
        band_error = max(lo-signed_error, signed_error-hi, 0.)
        recover_cap, recover_why = math.inf, ''
        if band_error > self.max_error:
            if self.previous is None and error > 5:
                return Output('WAITING', f'car is {error:.0f}m from the route at launch: use route:=spawn from the '
                              'respawn position or route:=global from the official START', error=error)
            # Way back: band_error over one lookahead, curvature ~ 2 e / L^2, within the route's lateral limit.
            look = min(max(self.lookahead, self.lookahead_gain*abs(speed)), self.lookahead_max)
            recover_cap = math.sqrt(self.a_lat_recover*look*look/(2.*band_error))
            recover_why = f'RECOVER cross track {band_error:.2f}m: steering back at <= {recover_cap*3.6:.0f} km/h'
        force_heading = abs(wrap(heading-yaw)) > math.radians(60)
        if force_heading:
            recover_why = recover_why or f'RECOVER heading off by {math.degrees(abs(wrap(heading-yaw))):.0f}deg'
        # Odometry steps per GPS fix (~1.3 m at 40 km/h), so scale the glitch check with speed —
        # and with how long we were blind (after a HOLD the car has legitimately moved on).
        if self.previous is not None and abs(s-self.previous) > max(2., abs(speed)*max(0.5, self.since_ok)):
            self.jump_time += dt
            if self.jump_time <= self.dropout_hold_s+1e-9:
                self.previous_accel = 0. if self.cruise_throttle_active else previous_accel
                return Output('CRUISE', f'position jump {s-self.previous:+.1f}m: holding steering and brake',
                              self.previous_accel, self.previous_brake, self.steering,
                              progress=self.previous-self.s_offset, error=error)
            # still jumped after the hold: the new position is the real one
        self.jump_time = 0.
        self.previous = s
        self.since_ok = 0.
        if self.stop_s is None:
            # Closed route endpoint must not be mistaken for lap completion at launch.
            self.stop_s = self.route.s[-1] if self.continuous else min(s+self.distance, self.route.s[-1])
        if self.state == 'WAITING':
            self.state = 'HANDSHAKE'
        if mode != 2 or gear != 4:
            if self.state == 'CRUISE':
                self.state = 'HANDSHAKE'
                return Output('HANDSHAKE', 'Auto/D feedback lost: braking and requesting Auto/D again',
                              progress=s-self.s_offset, error=error)
            return Output('HANDSHAKE', 'request Auto/D with brake held', progress=s-self.s_offset, error=error)
        self.state = 'CRUISE'
        # --- mission hook 1/3: stop_s. Front wheels at stop_s == rear axle (our s) at stop_s-FRONT_AXLE.
        # A passed non-signal target is ignored. An active red must keep braking after an overshoot.
        hold_s = self.stop_s
        mission_stop = target.stop_s+self.s_offset-FRONT_AXLE if math.isfinite(target.stop_s) else math.inf
        signal_stop = 'SIGNAL_STOP' in (target.state or '') and math.isfinite(target.stop_s)
        if mission_stop < hold_s and (mission_stop > s-0.5 or signal_stop):
            hold_s = mission_stop
        remaining = hold_s-s
        if remaining <= 0.3 and abs(speed) < 0.12:
            if hold_s < self.stop_s:
                return Output('CRUISE', f'holding at stop_s={target.stop_s:.1f}', progress=s-self.s_offset,
                              error=error, target_x=x, target_y=y)
            self.state = 'FINISH'
            self.reason = ('route end reached; brake held' if self.continuous
                           else 'test distance reached; brake held')
            return Output('FINISH', self.reason, progress=s-self.s_offset, error=error)
        # --- mission hook 2/3: lat_offset → s 곡선. 요청이 바뀌면 지금 s 에서 새 전이를 시작한다.
        # --- mission hook 2/3: the lateral offset curve. A chain (tunnel slalom) is every move at once;
        # otherwise one offset with an optional entry curve and scheduled return.
        if target.chain:
            self.chain_offset(target, s)
        else:
            if self.chain is not None:
                # Back to the single-offset mode from wherever the chain left the car.
                self.offset_target = self.offset_state(s)[0]
                self.chain = self.chain_request = None
                self.transition = self.scheduled_return = None
            self.legacy_offset(target, s, speed)
        lookahead = min(max(self.lookahead, self.lookahead_gain*speed), self.lookahead_max)
        # 정지 위치는 속도/제동에만 사용한다. 조향 목표까지 정지 위치로 자르면
        # 남은 거리가 0에 가까워질 때 작은 횡오차도 최대 조향으로 증폭된다.
        ex, ey, eyaw, es = x, y, yaw, s
        ki, predict = self.lateral_ki, self.steer_predict_s
        if self.fast is not None:
            lo_, hi_ = self.fast[2];w_ = float(np.clip((speed*3.6-lo_)/max(hi_-lo_, 1e-6), 0., 1.))
            ki, predict = ki+(self.fast[0]-ki)*w_, predict+(self.fast[1]-predict)*w_
        if predict > 0:
            t_ = predict;rate_ = speed*math.tan(self.steering)/FRONT_AXLE
            eyaw = yaw+rate_*t_;mid = yaw+rate_*t_/2
            ex, ey, es = x+speed*t_*math.cos(mid), y+speed*t_*math.sin(mid), s+speed*t_
        s_ahead = min(es+lookahead, self.route.s[-1])
        tx, ty = self.route.at(s_ahead)
        offset_ahead = self.offset_at(s_ahead)      # 주시점은 곡선 위 → 전이가 조향에 미리 반영된다
        if offset_ahead:
            h = self.route.heading_at(s_ahead)
            tx, ty = tx-math.sin(h)*offset_ahead, ty+math.cos(h)*offset_ahead
        dx, dy = tx-ex, ty-ey
        lateral = -math.sin(eyaw)*dx+math.cos(eyaw)*dy
        curvature = 2*lateral/max(dx*dx+dy*dy, 0.25)
        transitioning = self.transition is not None and s < self.transition[0]+self.transition[3]
        if self.scheduled_return is not None:
            rs, _, rl = self.scheduled_return
            transitioning = transitioning or (s+lookahead >= rs and s < rs+rl)
        if self.chain:
            transitioning = transitioning or any(st <= s+lookahead and s < st+ln for st, _, _, ln, _ in self.chain)
        if not transitioning:
            self.lateral_integral = float(np.clip(self.lateral_integral-ki*(signed_error-self.offset)*dt,
                                                  -math.radians(1.5), math.radians(1.5)))
        path_steer = math.atan(3.*curvature)
        if self.brake_steer>0 and self.previous_brake>0:
            gs_ = float(np.interp(s, self.route.s, self.global_stations))
            if any(b_ <= gs_ < e_ for b_, e_ in self.brake_steer_zones):
                # Only the correction is lowered, not the steering the path's own bend needs over the preview
                # (user 2026-10-05: lowering it all ran the car 0.5 m wide braking into the CP13 bend at ~60 km/h).
                end_ = min(s+lookahead, self.route.s[-1])
                bend = math.atan(3.*wrap(self.route.heading_at(end_)-self.route.heading_at(s))/max(end_-s, 1.))
                path_steer = bend+(path_steer-bend)/(1.+self.brake_steer*self.previous_brake)
        raw = float(np.clip(path_steer+self.lateral_integral,
                            -math.radians(34.9), math.radians(34.9)))
        self.steering += (raw-self.steering)*dt/(self.steer_tau+dt)
        steering = self.steering
        # PP의 미리보기 때문에 회피 곡선의 접선과 실제 yaw도 일치하지 않는다.
        # 의도한 횡기동을 방향 이탈로 감속시키면 회피/복귀 때 속도가 출렁인다.
        # 횡기동 중에는 이 경험적 보정만 생략한다. 도로 곡률·정지·미션 상한은 그대로다.
        maneuvering = transitioning or abs(self.offset_target) > 1e-6 or abs(self.offset) > 1e-6
        heading_error = 0. if (maneuvering and not force_heading) else wrap(heading-yaw)
        # --- mission hook 3/3: v_max caps the curvature profile; the stop ramp caps both.
        cruise = min(self.route.speed_at(s, self.profile), max(0., target.v_max), recover_cap)
        # The route end (past the finish line) may be braked for harder than a stop line.
        stop_decel = (self.a_finish if hold_s >= self.stop_s
                      else self.a_signal if 'SIGNAL_STOP' in (target.state or '') else self.a_stop)
        target_speed = min(cruise/math.cosh(math.pi*heading_error),
                           math.sqrt(2*stop_decel*max(0., remaining-0.25)))
        if remaining <= 0.3:
            return Output('CRUISE', 'stopping', progress=s-self.s_offset, error=error, target_x=tx, target_y=ty)
        # Use the current geometric/mission/stop cap immediately. Smoothing a
        # remembered speed target makes a released stop keep braking a moving car.
        entry_acceleration = (target.state == 'ROUNDABOUT_ENTER'
                              or any(b <= s-self.s_offset < e for b, e in self.boost_zones))
        speed_error = target_speed-speed
        self.integral = float(np.clip(self.integral+speed_error*dt, -0.3, 0.3))
        effort = 0.2*speed_error+0.1*self.integral
        ordinary_cruise = (target.state in ('', 'CRUISE', 'TUNNEL') and not maneuvering and not recover_why
                           and not self.emergency_brake and not math.isfinite(target.stop_s)
                           and target.v_max >= self.speed and self.speed <= 60./3.6
                           and not any(b <= s-self.s_offset < e for b, e in self.uncapped_zones))
        self.cruise_throttle_active = ordinary_cruise
        if ordinary_cruise:
            # 2026-10-06 measured throttle response: gain 3.47 m/s2 per pedal, delay ~0.125 s.
            # P=0.2 took 4.1 s from 52 to 58.5 km/h. P=0.4 with 0.35 s positive-acceleration
            # preview took 2.25 s and stayed below 60 in the tested gain/delay envelope.
            # Braking still uses actual speed; a falling target cannot gain throttle.
            effort = .4*(speed_error-.35*max(0., self.cruise_acceleration))+.1*self.integral
        # Coast on small overspeed (measured 0.32 m/s2 coast decel); brake only beyond 0.4 m/s.
        # Feed-forward from the profile slope so corner entry does not wait for a speed error
        # (measured: brake 0.2 -> 3.1 m/s2, coast alone -> 0.32 m/s2).
        horizon = max(speed, 1.)*0.5
        ahead = min(self.route.speed_at(s+horizon, self.profile), target_speed,
                    math.sqrt(2*stop_decel*max(0., remaining-horizon-0.25)))   # the stop ramp's own slope, fed forward
        decel_needed = max(0., (speed*speed-ahead*ahead)/(2*horizon)-0.32) if speed > ahead else 0.
        # ...and from how fast the behavior cap (v_max) is falling (the highway follow cap closing on a lead): the
        # P-only brake settled ~7 km/h above a cap falling at 2.2 m/s2 (0.08 x 1.9 m/s = brake 0.15) and the car ended
        # 1.5 m inside the target gap (2026-09-30). Only v_max: the heading term and the profile jitter with pose
        # noise (60 km/h straight offset 0.104 -> 0.137 m when they fed it); the profile has its own feed-forward.
        # Filtered over TARGET_FALL_TAU_S so a step is not a spike.
        cap = max(0., target.v_max) if math.isfinite(target.v_max) else None
        if cap is not None and self.previous_target is not None and dt > 0:
            # A step (a new plan's cap) is the P term's job: at most the decel the feed-forward can express, so one
            # step adds ~0.02 instead of 0.3 on top of it (brake 0.5 on a 44 -> 29 km/h YIELD cap, 2026-09-30).
            fall = min((self.previous_target-cap)/dt, 0.3*15.+0.32)
            self.target_fall += (fall-self.target_fall)*dt/(self.TARGET_FALL_TAU_S+dt)
        else:self.target_fall = 0.
        self.previous_target = cap
        # Added to the feedback, not max'ed with it: max(ff 0.15, P 0.13) was the same brake as before and the lag
        # stayed (2026-09-30 run 030549). Only while the cap binds (at or below our speed).
        target_ff = (float(np.clip((self.target_fall-0.32)/15., 0, 0.3))
                     if cap is not None and speed > cap-0.4 and cap <= cruise+1e-6 else 0.)
        brake_ff = float(np.clip(decel_needed/15., 0, 0.3))
        accel = float(np.clip(effort, 0, self.accel_max)) if speed_error >= -0.4 and brake_ff == 0 and target_ff == 0 else 0.
        if entry_acceleration and speed_error>.15 and target.v_max>speed+.4 and brake_ff==0 and target_ff==0:
            accel=max(accel,self.entry_accel_max)
        brake = max(brake_ff, float(np.clip(-speed_error*0.08, 0, 0.5)) if speed_error < -0.4 else 0.)
        brake = min(0.5, brake+target_ff) if target_ff > 0 else brake
        if signal_stop and speed > .12:
            # A late red can need more than the profile feed-forward's 0.3 pedal.
            # Reserve the measured ~0.15 s response distance and a 0.15 m feed-forward
            # margin. The profile keeps its 0.25 m margin. A 0.25 m extra margin
            # stopped early enough to need another creep; 0.15 m kept all 288
            # tested brake-response conditions ahead of the line and within 0.5 m.
            braking_distance = max(.1, remaining-.15-max(0., speed)*.15)
            required_decel = speed*speed/(2.*braking_distance)
            if required_decel > stop_decel:
                brake = max(brake, min(1., required_decel/15.))
                accel = 0.
        # Emergency: far above the cap (a slow lead known late at speed), past where the P term saturates at 0.5 the
        # brake keeps rising to 1.0 (~9 m/s2 vs ~5 at 0.5, measured 2026-09-15). At 106-110 km/h a 48 km/h lead 19 m
        # ahead needed ~10 m/s2 and was hit at brake 0.5 (2026-10-01 laps). Ordinary braking (<= 6.25 m/s over) unchanged.
        over = -speed_error-self.EMERGENCY_FROM_MPS
        if over > 0 and self.emergency_brake:brake = max(brake, min(1., .5+over*.1))
        if target_speed <= 1e-6:
            # A stop cap must not leak positive PI integral into throttle at rest.
            accel = 0.
            brake = max(brake, self.hold_brake)
            self.integral = 0.
        # A cached status can remain healthy while its publisher has gone quiet.
        # Ordinary cruise must not increase throttle from that repeated speed.
        if ordinary_cruise and not fresh_speed:
            accel = 0.
        # Smooth throttle increases only; reductions and braking remain immediate.
        # Bound elapsed time so a delayed tick cannot jump straight to full throttle.
        accel = min(accel, previous_accel+self.accel_rise_per_s*max(0., min(dt, .1)))
        self.previous_accel = accel
        self.previous_brake = brake
        return Output('CRUISE', recover_why or target.state or 'tracking', accel, brake, steering, s-self.s_offset, error,
                      target_speed, tx, ty)
