import json
import math
from pathlib import Path
import struct
import unittest

import numpy as np
from pyproj import Transformer

from foscar_control.core import (Controller, PoseEstimator, Route, SteeringMap, Target, ctrl_packet, parse_gps,
                                parse_imu, parse_status, select_route)


def nmea(body):
    checksum = 0
    for c in body:
        checksum ^= ord(c)
    return f'${body}*{checksum:02X}\r\n'.encode()


class ShutdownHandoffTest(unittest.TestCase):
    def command(self, status):
        from foscar_control import core
        return struct.unpack_from('<bbbfffff', core.handoff_packet(status), 30)

    def test_moving_or_missing_feedback_keeps_braking(self):
        for status in (None, dict(speed=2., gear=4, mode=2)):
            cmd = self.command(status)
            self.assertEqual(cmd[:3], (2, 4, 1))
            self.assertEqual(cmd[-3:], (0., 1., 0.))

    def test_stopped_vehicle_requests_park_before_manual(self):
        cmd = self.command(dict(speed=0., gear=4, mode=2))
        self.assertEqual(cmd[:3], (2, 1, 1))
        self.assertEqual(cmd[-3:], (0., 1., 0.))

    def test_confirmed_park_releases_brake_in_manual(self):
        cmd = self.command(dict(speed=0., gear=1, mode=2))
        self.assertEqual(cmd[:3], (1, 1, 1))
        self.assertEqual(cmd[-3:], (0., 0., 0.))


class CoreTest(unittest.TestCase):
    def test_captured_packets(self):
        samples = json.loads(Path(__file__).with_name('sensor_samples.json').read_text())
        status = parse_status(bytes.fromhex(samples['9099']))
        self.assertEqual(status['gear'], 4)
        self.assertAlmostEqual(math.degrees(status['yaw']), 62.038929, places=4)
        self.assertEqual(status['link'], 'A2256W000172')
        q, _, _, _ = parse_imu(bytes.fromhex(samples['9104']))
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.)
        self.assertTrue(parse_gps(bytes.fromhex(samples['9102']))[0]['valid'])
        packet = ctrl_packet(0.1, 0., -0.2)
        self.assertEqual(len(packet), 55)
        self.assertEqual(struct.unpack_from('<i', packet, 14)[0], 23)
        self.assertEqual(struct.unpack_from('<bbb', packet, 30), (2, 4, 1))

    def test_bad_and_invalid_packets(self):
        for bad in [b'', b'#MoraiInfo$'+bytes(218), bytes(181)]:
            with self.assertRaises(ValueError):
                parse_status(bad)
        with self.assertRaises(ValueError):
            parse_imu(bytes(115))
        with self.assertRaises(ValueError):
            parse_gps(b'$GPRMC,x*00\r\n')
        self.assertFalse(parse_gps(nmea('GPRMC,123,V,,,,,,,,,,,'))[0]['valid'])
        self.assertFalse(parse_gps(nmea('GPGGA,123,,,,,0,0,,,,,,,'))[0]['valid'])
        # Tunnel (GPS shadow): simulator keeps status 'A' / fix 1 but zeroes the coordinates.
        self.assertFalse(parse_gps(nmea('GPRMC,215244.9,A,0000.0000,N,00000.0000,E,000.0,056.8,150926,000.0,E,A,'))[0]['valid'])
        self.assertFalse(parse_gps(nmea('GPGGA,215244.9,0000.0000,N,00000.0000,E,1,09,1.0,0.0,M,0.0,M,,'))[0]['valid'])

    def test_coordinate_transform(self):
        fix = parse_gps(nmea('GPRMC,123,A,3714.4567,N,12646.4599,E,0,0,120926,,,A,'))[0]
        x, y = Transformer.from_crs(4326,32652,always_xy=True).transform(fix['lon'], fix['lat'])
        self.assertAlmostEqual(x-302595., -13.0126, places=3)
        self.assertAlmostEqual(y-4124145., -222.16, delta=0.2)

    def test_steering_sign_and_limits(self):
        lookup = SteeringMap([0,15.2,34.9], [0,0.5,1])
        self.assertAlmostEqual(lookup.command(math.radians(15.2)), .5)
        self.assertEqual(lookup.command(-10), -1)
        self.assertEqual(lookup.command(0), 0)

    def test_tracking_stopping_and_dropout(self):
        route = Route(np.column_stack((np.arange(101), np.zeros(101))))
        controller = Controller(route)
        self.assertEqual(controller.step(0,0,0,0,1,4,True).state, 'HANDSHAKE')
        moving = controller.step(0,0,0,0,2,4,True)
        self.assertGreater(moving.accel, 0)
        self.assertEqual(moving.steering, 0)
        # Ordinary cruise coasts during a brief sensor dropout while holding steering/brake;
        # past the 1 s hold it is a HOLD, never a latched FAULT: it resumes.
        gap = controller.step(0,0,0,0,2,4,False)
        self.assertEqual((gap.state, gap.accel, gap.brake, gap.steering), ('CRUISE', 0., 0., moving.steering))
        for _ in range(20):
            hold = controller.step(0,0,0,0,2,4,False)
        self.assertEqual((hold.state, hold.accel, hold.brake), ('HOLD',0,controller.hold_brake))
        self.assertEqual(hold.steering, moving.steering)
        self.assertEqual(controller.step(0,0,0,0,2,4,True).state,'CRUISE')
        controller = Controller(route, distance=5)
        for x in range(6):
            output = controller.step(x,0,0,0,2,4,True)
        self.assertEqual(output.state,'FINISH')
        self.assertEqual(output.brake,1)

    def test_wrong_heading_off_route_and_position_jump(self):
        route = Route(np.column_stack((np.arange(101), np.zeros(101))))
        # No latched FAULT while driving (user 2026-09-29): wrong heading / off the route recover by
        # steering back slowly, a position jump carries on the last command, then is accepted.
        out = Controller(route).step(0,0,math.pi,0,2,4,True)
        self.assertEqual(out.state,'CRUISE'); self.assertIn('RECOVER heading', out.reason)
        out = Controller(route).step(0,2,0,0,2,4,True)
        self.assertEqual(out.state,'CRUISE'); self.assertIn('RECOVER cross track', out.reason)
        self.assertLess(out.steering, 0)                       # back toward the route (to the right)
        self.assertIn('route:=spawn', Controller(route).step(0,58,0,0,2,4,True).reason)
        c = Controller(route)
        c.step(0,0,0,0,2,4,True)
        out = c.step(7,0,0,0,2,4,True)
        self.assertEqual(out.state,'CRUISE'); self.assertIn('position jump', out.reason)
        for _ in range(int(c.dropout_hold_s/0.05)+1):
            out = c.step(7,0,0,0,2,4,True)
        self.assertNotIn('position jump', out.reason)           # held past the dropout hold: accepted
        # at 40 km/h a 2.5 m step between GPS fixes is normal; 7 m is still a jump
        c = Controller(route, speed_kmh=40, continuous=True)
        c.step(0,0,0,11,2,4,True)
        self.assertEqual(c.step(2.5,0,0,11,2,4,True).state,'CRUISE')
        self.assertIn('position jump', c.step(9.5,0,0,11,2,4,True).reason)
        left = Controller(route).step(0,-0.5,0,0,2,4,True)
        self.assertGreater(left.steering,0)

    def test_hold_then_resume_tolerates_the_distance_driven_blind(self):
        # 40 km/h, odom gone for 1.5 s (30 ticks) -> the car moved ~17 m; that is not a 'position jump'.
        route = Route(np.column_stack((np.arange(201), np.zeros(201))))
        c = Controller(route, speed_kmh=40, continuous=True)
        c.step(0,0,0,11,2,4,True)
        c.step(1,0,0,11,2,4,True)
        for k in range(30):
            self.assertEqual(c.step(1,0,0,11,2,4,False).state, 'CRUISE' if k<20 else 'HOLD')
        self.assertEqual(c.step(18,0,0,11,2,4,True).state, 'CRUISE')
        # ...but the same 17 m step with no blind time in between is still a jump.
        c = Controller(route, speed_kmh=40, continuous=True)
        c.step(0,0,0,11,2,4,True)
        self.assertIn('position jump', c.step(17,0,0,11,2,4,True).reason)

    def test_pose_estimator_dead_reckons_through_a_tunnel_and_reacquires_without_a_jump(self):
        # 6 m/s straight for 2 s on GPS (30 Hz), 20 s blackout (120 m, the real tunnel is 118 m),
        # then GPS returns. Noise-free sensors -> DR error must be tiny and no per-tick jump on return.
        est = PoseEstimator(gps_timeout=0.3, blend_s=2.0)
        dt, v = 0.02, 6.
        truth = [0., 0.]
        max_step = 0.
        modes = []
        for k in range(int(30/dt)):
            t = k*dt
            truth[0] += v*dt
            gps_on = t < 2 or t > 22
            fix = tuple(truth) if gps_on and k % 2 == 0 else None    # ~25 Hz fixes
            before = (est.x, est.y)
            modes.append(est.update(dt, 0., v, fix))
            if before[0] is not None:
                max_step = max(max_step, math.hypot(est.x-before[0], est.y-before[1]))
        self.assertEqual(modes[0], 'GPS')
        self.assertEqual(modes[int(2.5/dt)], 'DR')
        self.assertEqual(modes[-1], 'GPS')
        self.assertLess(abs(est.x-truth[0]), 0.05)
        self.assertGreater(est.dr_distance, 115)
        self.assertLess(est.last_error, 0.2)
        self.assertLess(max_step, v*dt*1.5)          # never more than ~1.5 ticks of motion

    def test_pose_estimator_blends_a_real_dr_error_at_the_given_rate(self):
        # Speed reads 5% high during a 10 s blackout at 10 m/s -> 5 m along-track error at reacquire.
        est = PoseEstimator(gps_timeout=0.3, blend_s=2.0, blend_rate=2.0)
        dt = 0.02
        est.update(dt, 0., 10., (0., 0.))
        x_true = 0.
        for k in range(int(10/dt)):
            x_true += 10.*dt
            est.update(dt, 0., 10.5, None)
        self.assertEqual(est.mode, 'DR')
        self.assertAlmostEqual(est.x-x_true, 5., delta=0.2)
        steps = []
        for k in range(int(4/dt)):
            x_true += 10.*dt
            before = est.x
            est.update(dt, 0., 10., (x_true, 0.) if k % 2 == 0 else None)
            steps.append(est.x-before)
        self.assertEqual(est.mode, 'GPS')
        self.assertAlmostEqual(est.last_error, 5., delta=0.2)
        self.assertAlmostEqual(est.blend_total, 2.5, delta=0.1)   # 5 m / 2 m/s > blend_s
        self.assertLess(abs(est.x-x_true), 0.05)                   # converged
        self.assertTrue(all(st > 0 for st in steps))               # never moved backwards
        self.assertLess(max(steps), 10.*dt*1.2)                    # no jump: >= 8 m/s forward, <= 12 m/s

    def test_utm_grid_convergence_at_kcity_matches_the_measured_imu_yaw_bias(self):
        # 20 drive logs: motion heading - IMU yaw = -1.40 +- 0.05 deg. MORAI IMU yaw is true-north,
        # our map is UTM52 grid-north; localization adds this angle to the IMU yaw.
        from pyproj import Proj
        lon, lat = Proj('epsg:32652')(302595., 4124145., inverse=True)
        gamma = Proj('epsg:32652').get_factors(lon, lat).meridian_convergence
        self.assertAlmostEqual(gamma, -1.35, delta=0.05)

    def test_pose_estimator_curve(self):
        # Circle of radius 20 m at 5 m/s with no GPS at all after the first fix: DR must follow the arc.
        est = PoseEstimator()
        dt, v, r = 0.02, 5., 20.
        est.update(dt, 0., v, (0., 0.))
        yaw = 0.
        for _ in range(int(2*math.pi*r/v/dt)):
            yaw += v/r*dt
            est.update(dt, yaw, v, None)
        self.assertEqual(est.mode, 'DR')
        self.assertLess(math.hypot(est.x, est.y), 0.5)             # back at the start after one lap

    def test_closed_route_start_does_not_finish(self):
        route = Route([[0,0],[1,0],[1,1],[0,1],[0,0]])
        s, _, _ = route.project(0,0)
        self.assertEqual(s, 0)

    def test_continuous_passes_test_limit_and_stops_at_route_end(self):
        route = Route(np.column_stack((np.arange(251), np.zeros(251))))
        c = Controller(route, distance=20, continuous=True)
        for x in range(250):
            out = c.step(x,0,0,0,2,4,True)
            self.assertEqual(out.state,'CRUISE',f'x={x}: {out.reason}')
        out = c.step(250,0,0,0,2,4,True)
        self.assertEqual((out.state,out.accel,out.brake),('FINISH',0,1))
        self.assertIn('route end',out.reason)
        # continuous mode keeps the safety stops; a dropout carries on, then is only a HOLD
        c = Controller(route, continuous=True)
        c.step(0,0,0,0,2,4,True)
        self.assertEqual(c.step(0,0,0,0,2,4,False).state,'CRUISE')
        for _ in range(20):
            out = c.step(0,0,0,0,2,4,False)
        self.assertEqual(out.state,'HOLD')
        self.assertIn('RECOVER', Controller(route, continuous=True).step(0,3,0,0,2,4,True).reason)

    def test_curved_route_bicycle_simulation(self):
        theta = np.linspace(0,1.5,301)
        route = Route(np.column_stack((20*np.sin(theta),20*(1-np.cos(theta)))))
        c = Controller(route, distance=20)
        x,y,yaw,speed = 0.,-0.3,0.,0.
        max_error = 0.
        for _ in range(1400):
            out = c.step(x,y,yaw,speed,2,4,True,0.05)
            self.assertNotEqual(out.state,'FAULT',out.reason)
            max_error = max(max_error,out.error)
            speed = max(0,speed+(out.accel*4-out.brake*5-0.15)*0.05)
            yaw += speed/3*math.tan(out.steering)*0.05
            x += speed*math.cos(yaw)*0.05
            y += speed*math.sin(yaw)*0.05
            if out.state == 'FINISH':
                break
        self.assertEqual(out.state,'FINISH')
        self.assertLess(max_error,0.5)

    def test_speed_profile_straight_arc_and_decel_ramp(self):
        straight = Route(np.column_stack((np.arange(0,200,0.5), np.zeros(400))))
        profile = straight.speed_profile(30/3.6, a_lat=2.5, a_dec=1.5)
        self.assertTrue(np.allclose(profile, 30/3.6))
        self.assertAlmostEqual(straight.speed_at(50, profile), 30/3.6)
        theta = np.linspace(0, math.pi/2, 200)
        arc = Route(np.column_stack((np.r_[np.arange(0,100,0.5), 100+20*np.sin(theta)],
                                     np.r_[np.zeros(200), 20*(1-np.cos(theta))])))
        profile = arc.speed_profile(30/3.6, a_lat=2.5, a_dec=1.5)
        # arc of R=20 limits to sqrt(2.5*20)=7.07 m/s; route entry at s=100 must already be slow
        self.assertAlmostEqual(profile.min(), math.sqrt(2.5*20), delta=0.4)
        self.assertLess(arc.speed_at(100, profile), math.sqrt(2.5*20)+0.5)
        self.assertAlmostEqual(arc.speed_at(10, profile), 30/3.6)
        ramp = profile[:200]
        self.assertTrue(np.all(np.diff(ramp) <= 1e-9))
        # decel ramp obeys v^2 = v_end^2 + 2 a d
        d = 100-arc.s[:200]
        self.assertTrue(np.all(ramp**2 <= math.sqrt(2.5*20)**2+2*1.5*d+0.5))
        # exit ramp: after the arc (s>=100+10*pi) speed rises no faster than a_acc allows
        exit_s = 100+10*math.pi
        after = arc.s >= exit_s
        gain = profile[after]**2-math.sqrt(2.5*20)**2
        self.assertTrue(np.all(gain <= 2*1.0*(arc.s[after]-exit_s)+0.5))
        self.assertLess(arc.speed_at(exit_s+2, profile), math.sqrt(2.5*20)+1.0)
        # hairpin tighter than steering limit is clamped to 4.3 m radius, not zero speed
        pin = Route([[0,0],[1,0],[2,0],[2.2,0.4],[2,0.8],[1,0.8],[0,0.8]])
        self.assertGreater(pin.speed_profile(10, 2.5, 1.5, 1.0).min(), math.sqrt(2.5*4.3)-0.1)

    def test_controller_uses_profile_and_cruise_overspeed(self):
        route = Route(np.column_stack((np.arange(0,200,0.5), np.zeros(400))))
        c = Controller(route, speed_kmh=30, continuous=True)
        out = c.step(0,0,0,0,2,4,True)
        self.assertEqual(out.state, 'CRUISE')
        self.assertAlmostEqual(out.target_speed, 30/3.6, delta=0.05)
        self.assertLessEqual(out.accel, 0.4)
        # driving a bit above the local profile target is fine; +5 km/h over it brakes (no stop, 2026-09-29)
        self.assertEqual(c.step(1,0,0,30/3.6+0.4,2,4,True).state, 'CRUISE')
        out = c.step(2,0,0,35.5/3.6,2,4,True)
        self.assertEqual(out.state, 'CRUISE'); self.assertEqual(out.accel, 0); self.assertGreater(out.brake, 0)
        with self.assertRaises(ValueError):
            Controller(route, speed_kmh=101)

    def test_lookahead_grows_with_speed(self):
        route = Route(np.column_stack((np.arange(0,200,0.5), np.zeros(400))))
        c = Controller(route, speed_kmh=30, continuous=True)
        slow = c.step(0,0,0,0,2,4,True)
        fast = c.step(0.5,0,0,8,2,4,True)
        self.assertAlmostEqual(slow.target_x, 3, delta=0.01)
        self.assertAlmostEqual(fast.target_x, 0.5+1.0*8, delta=0.01)

    def test_fast_curved_route_bicycle_simulation(self):
        theta = np.linspace(0, math.pi/2, 200)
        # straight, R=30 quarter arc, straight again heading +y
        route = Route(np.column_stack((np.r_[np.arange(0,100,0.5), 100+30*np.sin(theta), np.full(119, 130.)],
                                       np.r_[np.zeros(200), 30*(1-np.cos(theta)), 30+np.arange(0.5,60,0.5)])))
        c = Controller(route, speed_kmh=30, continuous=True)
        x,y,yaw,speed,delta = 0.,0.,0.,0.,0.
        max_error, max_speed_in_arc = 0., 0.
        for _ in range(3000):
            out = c.step(x,y,yaw,speed,2,4,True,0.05)
            self.assertNotEqual(out.state,'FAULT',out.reason)
            max_error = max(max_error,out.error)
            if 100 < out.progress < 100+30*math.pi/2:
                max_speed_in_arc = max(max_speed_in_arc, speed)
            speed = max(0,speed+(out.accel*4-out.brake*15-0.3)*0.05)
            delta += (out.steering-delta)*0.05/0.35   # measured ~0.3 s steering actuator lag
            yaw += speed/3*math.tan(delta)*0.05
            x += speed*math.cos(yaw)*0.05
            y += speed*math.sin(yaw)*0.05
            if out.state == 'FINISH':
                break
        self.assertEqual(out.state,'FINISH')
        self.assertLess(max_error,0.4)
        self.assertLess(max_speed_in_arc, math.sqrt(2.5*30)+0.7)

    def test_noisy_straight_does_not_chatter(self):
        # 5 cm position / 0.5 deg yaw noise (GPS+IMU) must not flip the steering sign every tick
        # nor toggle throttle/brake; the vehicle must still track within 0.15 m.
        route = Route(np.column_stack((np.arange(0,300,0.5), np.zeros(600))))
        c = Controller(route, speed_kmh=10, continuous=True, lookahead=4)
        rng = np.random.default_rng(1)
        x,y,yaw,speed,delta = 0.,0.,0.,0.,0.
        steer, brake_on, errors = [], [], []
        for _ in range(1400):
            out = c.step(x+rng.normal(0,0.05), y+rng.normal(0,0.05), yaw+rng.normal(0,math.radians(0.5)),
                         speed, 2, 4, True, 0.05)
            self.assertNotEqual(out.state,'FAULT',out.reason)
            if out.progress > 40:
                steer.append(out.steering); brake_on.append(out.brake > 0); errors.append(abs(y))
            speed = max(0,speed+(out.accel*4-out.brake*15-0.3)*0.05)
            delta += (out.steering-delta)*0.05/0.35
            yaw += speed/3*math.tan(delta)*0.05
            x += speed*math.cos(yaw)*0.05
            y += speed*math.sin(yaw)*0.05
        step = np.degrees(np.abs(np.diff(steer))).mean()   # unfiltered: ~2.4 deg/tick
        toggles = np.sum(np.array(brake_on[1:]) != np.array(brake_on[:-1]))/(len(steer)*0.05)
        self.assertLess(step, 0.4, f'steering moves {step:.2f} deg/tick')
        self.assertLess(np.degrees(np.std(steer)), 1.0)
        self.assertLess(toggles, 0.5, f'brake toggles {toggles:.2f}/s')
        self.assertLess(max(errors), 0.15)

    def test_noisy_real_route_with_actuator_lag_stays_stable(self):
        # Regression: steer_tau=0.3 on top of the ~0.3 s simulator actuator lag diverged at 24 km/h
        # (cross-track 1.55 m FAULT at s=77 on 2026-09-15). Full spawn route, noise + actuator lag
        # + a 0.8 deg steering bias (real runs showed ~-0.5 deg, leaving 0.3 m offset on straights).
        route = Route(np.loadtxt(Path(__file__).parents[2]/'foscar_bringup/routes/spawn_path.txt'))
        for kmh in (30, 60):
            x, y = route.points[0]
            yaw = math.atan2(*(route.points[1]-route.points[0])[::-1])
            c = Controller(route, speed_kmh=kmh, continuous=True)
            rng = np.random.default_rng(2)
            speed = delta = 0.
            errors, progress = [], []
            for _ in range(20000):
                out = c.step(x+rng.normal(0,0.05), y+rng.normal(0,0.05), yaw+rng.normal(0,math.radians(0.5)),
                             speed, 2, 4, True, 0.05)
                self.assertNotEqual(out.state,'FAULT',f'{kmh} km/h: {out.reason}')
                if out.state == 'FINISH':
                    break
                errors.append(out.error)
                progress.append(out.progress)
                speed = max(0,speed+(out.accel*4-out.brake*15-0.3)*0.05)
                delta += (out.steering+math.radians(-0.8)-delta)*0.05/0.35
                yaw += speed/3*math.tan(delta)*0.05
                x += speed*math.cos(yaw)*0.05
                y += speed*math.sin(yaw)*0.05
            self.assertEqual(out.state,'FINISH', f'{kmh} km/h')
            self.assertLess(max(errors), 0.4 if kmh <= 40 else 0.45, f'{kmh} km/h')  # 60 km/h: stability margin, not spec
            straight = np.array(errors)[(np.array(progress) > 1250) & (np.array(progress) < 1550)]
            self.assertLess(straight.mean(), 0.11, f'{kmh} km/h straight offset')  # 0.104 at 60 after the +-4 m curvature window
            self.assertLess(len(errors)*0.05, 7*60)

    def test_decel_feedforward_brakes_before_speed_error_builds(self):
        # entering a corner: at the target speed (zero error) the controller must already brake
        theta = np.linspace(0, math.pi/2, 200)
        arc = Route(np.column_stack((np.r_[np.arange(0,100,0.5), 100+20*np.sin(theta)],
                                     np.r_[np.zeros(200), 20*(1-np.cos(theta))])))
        c = Controller(arc, speed_kmh=40, continuous=True)
        c.step(0,0,0,0,2,4,True)
        target = arc.speed_at(80, c.profile)
        self.assertLess(target, 40/3.6)
        c.previous = 79.8     # driven there (a 0 -> 80 m teleport is a position jump, it only passed as a FAULT)
        out = c.step(80,0,0,target,2,4,True)
        self.assertEqual(out.accel, 0)
        self.assertGreater(out.brake, 0.05)
        # entering a corner far too fast brakes harder, it does not end the run (2026-09-29)
        out = c.step(81,0,0,target+6/3.6,2,4,True)
        self.assertEqual(out.state, 'CRUISE'); self.assertEqual(out.accel, 0); self.assertGreater(out.brake, 0.05)

    def drive(self, c, route, target_fn, ticks=4000, start=(0., 0., 0.)):
        """Bicycle-model loop returning per-tick (s, speed, out). target_fn(tick) -> Target or None."""
        x, y, yaw = start
        speed = delta = 0.
        log = []
        for i in range(ticks):
            out = c.step(x, y, yaw, speed, 2, 4, True, 0.05, target_fn(i))
            self.assertNotEqual(out.state, 'FAULT', out.reason)
            log.append((out.progress, speed, out))
            if out.state == 'FINISH':
                break
            speed = max(0, speed+(out.accel*4-out.brake*15-0.3)*0.05)
            delta += (out.steering-delta)*0.05/0.35
            yaw += speed/3*math.tan(delta)*0.05
            x += speed*math.cos(yaw)*0.05
            y += speed*math.sin(yaw)*0.05
        return log

    def test_a_falling_v_max_is_followed_without_the_p_only_lag(self):
        # 2026-09-30: the highway follow cap fell at 2.2 m/s2; the P-only brake (0.08 per m/s) settled ~1.9 m/s above
        # it and the car ended 1.5 m inside the target gap. The target's own fall now feeds the brake forward.
        route = Route(np.column_stack((np.arange(0,1000,0.5), np.zeros(2000))))
        c = Controller(route, speed_kmh=60, continuous=True)
        cap = lambda i: 16. if i < 500 else max(8., 16.-2.2*(i-500)*0.05)
        log = self.drive(c, route, lambda i: Target(v_max=cap(i)), ticks=700)
        lag = [v-cap(i) for i, (s, v, _) in enumerate(log) if 520 <= i < 570]    # mid-ramp
        self.assertLess(max(lag), 0.8)

    def test_target_v_max_caps_speed(self):
        route = Route(np.column_stack((np.arange(0,400,0.5), np.zeros(800))))
        c = Controller(route, speed_kmh=30, continuous=True)
        log = self.drive(c, route, lambda i: Target(v_max=3.), ticks=1200)
        speeds = [v for s, v, _ in log if s > 60]
        self.assertLess(max(speeds), 3.3)
        self.assertGreater(np.mean(speeds), 2.5)
        # cap lifted -> back to profile
        c = Controller(route, speed_kmh=30, continuous=True)
        log = self.drive(c, route, lambda i: Target(v_max=3.) if i < 600 else Target(), ticks=1400)
        # Peak after the lift: the last tick may already be in the route-end stop ramp.
        self.assertGreater(max(v for _, v, _ in log[600:]), 6.)

    def test_target_stop_s_stops_front_wheels_there_and_resumes(self):
        route = Route(np.column_stack((np.arange(0,400,0.5), np.zeros(800))))
        c = Controller(route, speed_kmh=30, continuous=True)
        # red light: stop line at s=150 -> rear axle (our s) must settle at 147 +-0.5
        log = self.drive(c, route, lambda i: Target(stop_s=150.) if i < 1500 else Target(), ticks=2600)
        held = [(s, v, o) for s, v, o in log[:1500] if v < 0.05 and s > 100]
        self.assertGreater(len(held), 40, 'never stopped')
        self.assertAlmostEqual(held[-1][0], 147., delta=0.5)
        self.assertEqual(held[-1][2].state, 'CRUISE')
        self.assertIn('stop_s', held[-1][2].reason)
        self.assertEqual(held[-1][2].brake, 1.)
        # green: resumes and passes the line
        self.assertGreater(log[-1][0], 200.)
        # stop request behind us is ignored (already passed the line)
        c = Controller(route, speed_kmh=30, continuous=True)
        log = self.drive(c, route, lambda i: Target(stop_s=10.), ticks=400, start=(20., 0., 0.))
        self.assertGreater(log[-1][1], 5.)

    def test_stop_line_does_not_amplify_small_lateral_errors(self):
        # 정지 0.5 m 전: 5 cm 오차 때문에 조향이 포화되면 저속에서 좌우로 요동친다.
        # 정지선은 종방향 제동 목표이며, 조향 주시 거리를 줄이면 안 된다.
        route = Route(np.column_stack((np.arange(0, 100, 0.1), np.zeros(1000))))
        for error in (-0.05, 0.05):
            with self.subTest(error=error):
                c = Controller(route, speed_kmh=20, continuous=True)
                for _ in range(20):
                    out = c.step(9.5, error, 0., 0.6, 2, 4, True,
                                 target=Target(stop_s=13.))
                self.assertEqual(out.state, 'CRUISE')
                self.assertLess(abs(math.degrees(out.steering)), 5.)
                self.assertLess(out.steering * error, 0.)
                self.assertLess(out.target_speed, 0.8)

    def test_target_lat_offset_shifts_tracking_smoothly(self):
        route = Route(np.column_stack((np.arange(0,400,0.5), np.zeros(800))))
        c = Controller(route, speed_kmh=20, continuous=True)
        ys = []
        x, y, yaw, speed, delta = 0., 0., 0., 0., 0.
        for i in range(2400):
            out = c.step(x, y, yaw, speed, 2, 4, True, 0.05, Target(lat_offset=1.0 if i > 400 else 0.))
            self.assertNotEqual(out.state, 'FAULT', out.reason)
            ys.append((out.progress, y, out.error))
            speed = max(0, speed+(out.accel*4-out.brake*15-0.3)*0.05)
            delta += (out.steering-delta)*0.05/0.35
            yaw += speed/3*math.tan(delta)*0.05
            x += speed*math.cos(yaw)*0.05
            y += speed*math.sin(yaw)*0.05
        late = [y for s, y, e in ys if s > 250]
        self.assertAlmostEqual(np.mean(late), 1.0, delta=0.15)
        # offset ramps at <= 0.5 m/s: 1 m takes >= 2 s (40 ticks); check no instantaneous jump in target
        # cross-track error reported is still relative to the route (for FAULT checks), offset is intentional
        self.assertLess(max(e for s, y, e in ys), 1.5)

    def test_entry_preview_does_not_jump_to_the_middle_of_the_curve(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=20., continuous=True, lookahead=4.)
        t = Target(lat_offset=-1.75)
        t.entry_s, t.entry_length_m = 20., 8.33
        out = c.step(14.5, 0., 0., 5.5, 2, 4, True, .05, t)
        self.assertAlmostEqual(out.target_y, 0., places=7)
        out = c.step(14.775, 0., 0., 5.5, 2, 4, True, .05, t)
        self.assertLess(out.target_y, 0.)
        self.assertLess(abs(out.target_y), .01)
        self.assertAlmostEqual(c.offset_at(28.33), -1.75)

    def test_wider_box_before_entry_keeps_the_preview_start(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=20., continuous=True, lookahead=4.)
        t = Target(lat_offset=-1.75, entry_s=20., entry_length_m=8.33)
        c.step(12., 0., 0., 5.5, 2, 4, True, .05, t)
        t.lat_offset = -2.
        out = c.step(12.275, 0., 0., 5.5, 2, 4, True, .05, t)
        self.assertAlmostEqual(out.target_y, 0., places=7)
        self.assertAlmostEqual(c.offset_at(20.), 0., places=7)
        self.assertAlmostEqual(c.offset_at(28.33), -2., places=7)

    def test_delayed_box_rear_keeps_returning_instead_of_reentering_bypass(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=50., continuous=True)
        t = Target(lat_offset=-2., return_s=30., return_length_m=20.)
        c.step(0., 0., 0., 10., 2, 4, True, .05, t)
        for x in range(1, 34):
            c.step(x, c.offset_at(x), 0., 10., 2, 4, True, .05, t)
        before = c.offset_state(33.)
        c.step(33., before[0], 0., 10., 2, 4, True, .05,
               Target(lat_offset=-2., return_s=30.8, return_length_m=20.))
        for a, b in zip(before, c.offset_state(33.)):
            self.assertAlmostEqual(a, b)
        self.assertGreater(c.offset_at(40.), -1.7)
        self.assertAlmostEqual(c.offset_at(50.8), 0.)
        offsets = [c.offset_at(x) for x in np.linspace(33., 51., 181)]
        self.assertTrue(all(b >= a-1e-9 for a, b in zip(offsets, offsets[1:])))

    def test_return_moved_behind_current_position_does_not_jump(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        t = Target(lat_offset=2., return_s=30., return_length_m=16.)
        c.step(0., 0., 0., 8., 2, 4, True, .05, t)
        for x in range(1, 29):
            c.step(x, c.offset_at(x), 0., 8., 2, 4, True, .05, t)
        before = c.offset_state(28.)
        c.step(28., 2., 0., 8., 2, 4, True, .05,
               Target(lat_offset=2., return_s=25., return_length_m=16.))
        for a, b in zip(before, c.offset_state(28.)):
            self.assertAlmostEqual(a, b)

    def test_cancelled_return_with_same_offset_keeps_current_curve_continuous(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        t = Target(lat_offset=2., return_s=30., return_length_m=16.)
        c.step(0., 0., 0., 8., 2, 4, True, .05, t)
        for x in range(1, 39):
            c.step(x, c.offset_at(x), 0., 8., 2, 4, True, .05, t)
        before = c.offset_state(38.)
        # 새 장애물로 복귀 취소: 같은 bypass offset을 유지하되 현재 경로에서 이어야 한다.
        c.step(38., before[0], 0., 8., 2, 4, True, .05, Target(lat_offset=2.))
        after = c.offset_state(38.)
        for a, b in zip(before, after):
            self.assertAlmostEqual(a, b)

    def test_return_curve_is_visible_before_rear_clearance(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True, lookahead=8.)
        t = Target(lat_offset=2.)
        t.return_s, t.return_length_m = 30., 16.
        c.step(0., 0., 0., 8., 2, 4, True, .05, t)
        for x in range(1, 28):
            c.step(x, c.offset_at(x), 0., 8., 2, 4, True, .05, t)
        out = c.step(28., 2., 0., 8., 2, 4, True, .05, t)
        self.assertLess(out.target_y, 2.)  # 뒤범퍼가 빠지기 전에 앞쪽 복귀 곡선이 보인다.
        self.assertAlmostEqual(c.offset_at(30.), 2.)  # 실제 기준선은 clearance 전까지 유지
        self.assertAlmostEqual(c.offset_at(38.), 1.)
        self.assertAlmostEqual(c.offset_at(46.), 0.)
        self.assertLess(out.steering, 0.)

    def test_scheduled_return_uses_local_route_s_offset(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True, s_offset=20.)
        t = Target(lat_offset=2.)
        t.return_s, t.return_length_m = 30., 16.
        c.step(20., 0., 0., 8., 2, 4, True, .05, t)
        self.assertAlmostEqual(c.offset_at(50.), 2.)
        self.assertAlmostEqual(c.offset_at(58.), 1.)

    def test_intentional_avoidance_yaw_does_not_request_deceleration(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        c.step(0., 0., 0., 8., 2, 4, True, .05, Target(lat_offset=1.75))
        # PP previews the curve; actual yaw and reference tangent need not coincide.
        out = c.step(.4, 0., .2, 8., 2, 4, True, .05, Target(lat_offset=1.75))
        self.assertAlmostEqual(out.target_speed, 30./3.6)
        normal = Controller(route, speed_kmh=30., continuous=True)
        out = normal.step(0., 0., .2, 8., 2, 4, True, .05, Target())
        self.assertLess(out.target_speed, 30./3.6)  # 일반 차선 방향 이탈 보정은 유지

    def test_released_speed_cap_ramps_acceleration(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=50., continuous=True, a_acc=1.)
        capped = c.step(0., 0., 0., 5., 2, 4, True, .05, Target(v_max=5.))
        released = c.step(.25, 0., 0., 5., 2, 4, True, .05, Target())
        self.assertAlmostEqual(released.target_speed, 50./3.6)
        self.assertLessEqual(released.accel-capped.accel, .15+1e-9)
        stopped = c.step(.5, 0., 0., 5., 2, 4, True, .05, Target(v_max=0.))
        self.assertEqual(stopped.target_speed, 0.)  # 정지/감속은 램프로 지연하지 않는다.

    def test_transition_tracking_lag_does_not_wind_up_lateral_trim(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        for x in (0., .5, 1., 1.5, 2., 2.5, 3., 3.5, 4.):
            c.step(x, 0., 0., 8., 2, 4, True, .05, Target(lat_offset=2.))
        self.assertEqual(c.lateral_integral, 0.)

    def test_retarget_keeps_curve_tangent_and_curvature(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        c.step(0., 0., 0., 6., 2, 4, True, target=Target(lat_offset=1.75))
        h, at = 1e-3, 5.
        before = [c.offset_at(at+k*h) for k in (-1, 0, 1)]
        slope = (before[2]-before[0])/(2*h)
        curvature = (before[2]-2*before[1]+before[0])/h**2
        # Drive through intermediate poses to obey the position-jump guard.
        for x in (1., 2., 3., 4.):
            c.step(x, c.offset_at(x), 0., 6., 2, 4, True, target=Target(lat_offset=1.75))
        c.step(at, before[1], math.atan(slope), 6., 2, 4, True, target=Target(lat_offset=2.))
        after = [c.offset_at(at+k*h) for k in (0, 1, 2)]
        self.assertAlmostEqual(after[0], before[1], places=6)
        self.assertAlmostEqual((after[1]-after[0])/h, slope, delta=1e-3)
        self.assertAlmostEqual((after[2]-2*after[1]+after[0])/h**2, curvature, delta=1e-3)

    def test_following_avoid_curve_does_not_trigger_heading_braking(self):
        route = Route(np.c_[np.arange(0., 200., .5), np.zeros(400)])
        c = Controller(route, speed_kmh=30., continuous=True)
        c.step(0., 0., 0., 8., 2, 4, True, target=Target(lat_offset=2.))
        for x in range(1, 9):
            h = .001
            slope = (c.offset_at(x+h)-c.offset_at(x-h))/(2*h)
            out = c.step(x, c.offset_at(x), math.atan(slope), 8., 2, 4, True,
                         target=Target(lat_offset=2.))
        self.assertAlmostEqual(out.target_speed, 30./3.6, delta=.01)

    def test_offset_transition_is_a_curve_in_s_and_band_error_tolerates_the_lag(self):
        """lat_offset 은 요청 지점부터 L = max(min, v·T) 동안 smoothstep 곡선으로 붙는다 (시간 램프 아님).
        복귀는 return_time_s 로 더 짧고, 그동안 차가 차선 중심과 오프셋 선 사이에 있으면 FAULT 가 아니다."""
        route = Route(np.column_stack((np.arange(0,400,0.5), np.zeros(800))))
        c = Controller(route, speed_kmh=20, continuous=True, transition_time_s=2.0, return_time_s=1.5, transition_min_m=6.)
        c.step(0, 0, 0, 6., 2, 4, True, 0.05, Target(lat_offset=0.))
        c.step(0, 0, 0, 6., 2, 4, True, 0.05, Target(lat_offset=2.))
        s0, d0, d1, L = c.transition
        self.assertEqual((d0, d1, L), (0., 2., 12.))                    # 6 m/s × 2 s
        self.assertAlmostEqual(c.offset_at(s0), 0.); self.assertAlmostEqual(c.offset_at(s0+6.), 1.)
        self.assertAlmostEqual(c.offset_at(s0+12.), 2.); self.assertAlmostEqual(c.offset_at(s0+3.), 2.*(6*.25**5-15*.25**4+10*.25**3))
        self.assertEqual(len(c.reference_path(s0)), 31)
        # 곡선 끝 뒤에서 복귀 요청: 지금 s 의 곡선값(2.0)에서 0 으로, 6 m/s × 1.5 s = 9 m
        c = Controller(route, speed_kmh=20, continuous=True, transition_time_s=2.0, return_time_s=1.5, transition_min_m=6.)
        c.offset_target = 2.                                            # 이미 2 m 옆에서 달리는 중
        self.assertEqual(c.step(20, 2., 0, 6., 2, 4, True, 0.05, Target(lat_offset=2.)).state, 'CRUISE')
        out = c.step(20, 0.5, 0, 6., 2, 4, True, 0.05, Target(lat_offset=0.))
        self.assertEqual(out.state, 'CRUISE', out.reason)              # 밴드 안(0~2) 이면 정상
        s0, d0, d1, L = c.transition
        self.assertEqual((s0, d0, d1, L), (20., 2., 0., 9.))
        self.assertAlmostEqual(c.offset_at(24.5), 1.)
        # 밴드 밖 1.5 m 이상이면 복구 주행(RECOVER, 감속하며 복귀): 오프셋 반대쪽으로 1.6, 오프셋 너머로 1.6
        self.assertIn('RECOVER', c.step(20, -1.6, 0, 6., 2, 4, True, 0.05, Target(lat_offset=0.)).reason)
        c = Controller(route, speed_kmh=20, continuous=True); c.offset_target = 2.
        self.assertIn('RECOVER', c.step(0, 3.6, 0, 6., 2, 4, True, 0.05, Target(lat_offset=2.)).reason)
        # 저속 하한: 1 m/s 면 6 m
        c = Controller(route, speed_kmh=20, continuous=True)
        c.step(0, 0, 0, 1., 2, 4, True, 0.05, Target(lat_offset=0.))
        c.step(0, 0, 0, 1., 2, 4, True, 0.05, Target(lat_offset=1.))
        self.assertEqual(c.transition[3], 6.)

    def test_select_route_auto_and_global_s_offset(self):
        lap = np.column_stack((np.arange(0, 100, 0.5), np.zeros(200)))          # global s 0..99.5
        theta = np.linspace(0, math.pi/2, 32)[:-1]
        connector = np.vstack((np.column_stack((np.full(20, -10.), np.arange(-20, -10, 0.5))),
                               np.column_stack((-10*np.cos(theta), -10+10*np.sin(theta)))))  # 10 m + quarter arc
        spawn = np.vstack((connector, lap))
        # on the lap -> global route, no offset
        route, offset = select_route(50, 0.2, lap, spawn)
        self.assertEqual(offset, 0.)
        self.assertAlmostEqual(route.s[-1], 99.5)
        # on the connector -> spawn route, offset = connector length so reported s stays in global terms
        route, offset = select_route(-10, -15, lap, spawn)
        self.assertAlmostEqual(offset, Route(spawn).s[-1]-99.5, delta=0.01)
        self.assertGreater(offset, 20.)
        # nowhere near either -> ValueError with a hint
        with self.assertRaises(ValueError):
            select_route(500, 500, lap, spawn)
        # controller on the spawn route reports global s (negative on the connector) and takes stop_s in global s
        c = Controller(route, speed_kmh=20, continuous=True, s_offset=offset)
        out = c.step(-10, -15, math.pi/2, 0, 2, 4, True)
        self.assertAlmostEqual(out.progress, 5.-offset, delta=0.01)
        log = self.drive(c, route, lambda i: Target(stop_s=50.), ticks=1500, start=(-10., -15., math.pi/2))
        held = [(s, v) for s, v, o in log if v < 0.05 and s > 20]
        self.assertGreater(len(held), 20)
        self.assertAlmostEqual(held[-1][0], 47., delta=0.5)


if __name__ == '__main__':
    unittest.main()


def test_roundabout_entry_accelerates_promptly_but_obeys_front_stop():
    route=Route(np.c_[np.arange(100.),np.zeros(100)])
    entry=Controller(route,speed_kmh=20.,continuous=True)
    normal=Controller(route,speed_kmh=20.,continuous=True)
    for controller in (entry,normal):
        controller.step(0,0,0,0,2,4,True,target=Target(v_max=0.))
    for _ in range(4):
        a=entry.step(0,0,0,0,2,4,True,target=Target(v_max=5.,state='ROUNDABOUT_ENTER'))
        b=normal.step(0,0,0,0,2,4,True,target=Target(v_max=5.,state='CRUISE'))
    assert a.accel>=.55 and b.accel<=.4
    assert a.target_speed==b.target_speed==5.
    stopped=entry.step(0,0,0,2.,2,4,True,target=Target(v_max=0.,state='ROUNDABOUT_ENTER'))
    assert stopped.accel==0 and stopped.brake>0


def test_stop_release_while_moving_does_not_keep_braking():
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,speed_kmh=50.,continuous=True)
    stopped=c.step(0,0,0,5.,2,4,True,target=Target(v_max=0.))
    assert stopped.brake>0 and stopped.accel==0
    released=c.step(.25,0,0,5.,2,4,True,target=Target())
    assert released.target_speed>5.
    assert released.brake==0 and 0<released.accel<=.15+1e-9


def test_throttle_rises_promptly_and_resets_on_sensor_hold():
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,speed_kmh=50.,continuous=True)
    outputs=[c.step(0,0,0,0,2,4,True) for _ in range(4)]
    assert 0<outputs[0].accel<=.15+1e-9
    assert outputs[-1].accel==.4
    kept=c.step(0,0,0,0,2,4,False)
    assert kept.accel==0.                     # cruise cannot keep accelerating without fresh feedback
    for _ in range(20):
        held=c.step(0,0,0,0,2,4,False)
    assert held.accel==0                      # past the 1 s hold: HOLD resets the throttle
    resumed=c.step(0,0,0,0,2,4,True)
    assert 0<resumed.accel<=.15+1e-9
    stopped=c.step(0,0,0,0,2,4,True,target=Target(v_max=0.))
    assert stopped.accel==0


def test_mapped_tunnel_offset_is_not_clipped_to_old_three_metres():
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,speed_kmh=22.,continuous=True)
    c.step(0,0,0,4,2,4,True,target=Target(lat_offset=3.25,entry_s=5,entry_length_m=12))
    assert c.offset_target==3.25


def test_new_entry_from_an_existing_offset_uses_the_planned_length():
    # Tunnel slalom: already at -1.5, a new manoeuvre to +4.75 sized to 25 m of room.
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,speed_kmh=60.,continuous=True)
    first=Target(lat_offset=-1.5,entry_s=2,entry_length_m=12)
    for x in range(31):   # drive along the first curve without position jumps
        c.step(float(x),c.offset_at(float(x)),0,8,2,4,True,target=first)
    assert abs(c.offset_at(30.)+1.5)<1e-6
    c.step(31.,-1.5,0,8,2,4,True,target=Target(lat_offset=4.75,entry_s=31,entry_length_m=25))
    assert c.transition[0]==31 and c.transition[3]==25       # not 8 m/s x 1.5 s = 12 m
    assert abs(c.offset_at(31+25)-4.75)<1e-6 and c.offset_at(31+12)<4.


def test_same_entry_request_keeps_the_existing_widening_rule():
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,speed_kmh=60.,continuous=True)
    c.step(0,0,0,8,2,4,True,target=Target(lat_offset=3.,entry_s=2,entry_length_m=12))
    end=c.transition[0]+c.transition[3]
    c.step(5,c.offset_at(5.),0,8,2,4,True,target=Target(lat_offset=3.5,entry_s=2,entry_length_m=12))
    assert abs(c.transition[0]+c.transition[3]-end)<1e-6        # completion point kept


def test_a_lat_zone_raises_curve_speed_only_inside_it():
    # CP15-to-finish arcs (R 40-57 m) may corner harder than the rest of the route (2026-09-27).
    theta = np.linspace(0, math.pi, 400)
    pts = np.column_stack((np.r_[np.arange(0, 100, .5), 100+50*np.sin(theta), np.arange(100, 0, -.5)],
                           np.r_[np.zeros(200), 50*(1-np.cos(theta)), np.full(200, 100.)]))
    route = Route(pts)
    base = Controller(route, speed_kmh=60, continuous=True, a_lat=2.5)
    zoned = Controller(route, speed_kmh=60, continuous=True, a_lat=2.5, a_lat_zones=((100., 257., 4.5),))
    mid = route.speed_at(178., zoned.profile)
    assert abs(route.speed_at(178., base.profile) - math.sqrt(2.5*50)) < .5
    assert abs(mid - math.sqrt(4.5*50)) < .6
    assert route.speed_at(20., zoned.profile) == route.speed_at(20., base.profile)   # outside: unchanged


def test_route_end_uses_finish_decel_but_stop_lines_keep_theirs():
    # The route end lies past the finish line (runout), so it may be braked for hard; a stop
    # line keeps the gentle a_stop ramp (2026-09-27).
    route = Route(np.column_stack((np.arange(0, 200, .5), np.zeros(400))))
    hard = Controller(route, speed_kmh=60, continuous=True, finish_decel=4.)
    soft = Controller(route, speed_kmh=60, continuous=True)
    def target_at(c, s, t=Target()):
        c.stop_s = None; c.previous = None; c.state = 'CRUISE'
        return c.step(s, 0., 0., 16., 2, 4, True, .05, t).target_speed
    assert target_at(hard, 160.) > target_at(soft, 160.) + 3.
    line = Target(stop_s=100.)
    assert abs(target_at(hard, 60., line) - target_at(soft, 60., line)) < 1e-9


def test_highway_references_may_use_their_own_acceleration_ramp():
    # 2026-09-28: passing reached 2.4 m/s2 of 3.4 available; highway references ramp at reference_accel_mps2.
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    assert Controller(route,a_acc=4.,reference_a_acc=6.).reference_options['a_acc']==6.
    assert Controller(route,a_acc=4.).reference_options['a_acc']==4.


def test_highway_references_may_use_their_own_steering_integral_and_lag_compensation():
    # 2026-10-01 kick test at ~114 km/h: ki 0 + 0.1 s prediction damped the sway, but tracked 42-65 km/h lane changes
    # worse: blended in from 80 to 95 km/h on highway references only; other sections keep theirs.
    route=Route(np.c_[np.arange(200.),np.zeros(200)])
    c=Controller(route,lateral_ki=.03,reference_lateral_ki=0.,reference_steer_predict_s=.1)
    assert c.lateral_ki==.03 and c.steer_predict_s==0. and c.fast is None
    assert c.reference_options['fast']==(0.,.1,(80.,95.))
    inner=Controller(route,**c.reference_options)
    assert inner.lateral_ki==.03 and inner.steer_predict_s==0. and inner.fast==(0.,.1,(80.,95.))
    assert Controller(route,lateral_ki=.03).reference_options['fast'] is None


def test_steering_from_the_predicted_pose_turns_less_toward_a_line_it_is_already_closing_on():
    # 0.5 m left of a straight line, already steering back toward it: the pose 0.1 s ahead is closer, so less steer.
    route=Route(np.c_[np.arange(300.),np.zeros(300)])
    def steer(predict):
        c=Controller(route,speed_kmh=60.,lookahead=7.,lookahead_max=14.,steer_predict_s=predict,steer_tau=1e-3)
        c.steering=math.radians(-1.)
        out=None
        for _ in range(3):out=c.step(50.,.5,0.,25.,2,4,True,.05,Target(v_max=25.))
        return out.steering
    assert steer(.1)>steer(0.)


def test_far_above_the_cap_the_brake_goes_past_half_up_to_full():
    # 2026-10-01: a slow lead known late at 110 km/h needed ~10 m/s2; the brake stopped at 0.5 (~5 m/s2).
    route=Route(np.c_[np.arange(400.),np.zeros(400)])
    def brake(v,cap,highway=True):
        c=Controller(route,speed_kmh=100.,distance=150.,lookahead=7.,lookahead_max=14.,a_acc=50.,a_dec=50.,emergency_brake=highway)
        out=None
        for _ in range(3):out=c.step(60.,0.,0.,v,2,4,True,.05,Target(v_max=cap))
        assert abs(out.target_speed-cap)<.01
        return out.brake
    assert brake(20.,16.)<=.5            # 4 m/s over: ordinary braking as before
    assert brake(30.,12.)==1.            # 18 m/s over: full brake
    assert .5<brake(24.,15.)<1.          # 9 m/s over: in between
    assert brake(30.,12.,highway=False)==.5   # off the highway (the main controller): 0.5 as before
    assert Controller(route,lateral_ki=.03).reference_options['emergency_brake'] is True
