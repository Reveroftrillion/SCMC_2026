"""Closed-loop synthetic diagnostic, not a ROS/MORAI or road-safety validation.

Uses the installed route, mission ranges and production decision functions.
Only this harness assumes calibrated frames, verified boundaries/ROIs and a
uniform 6 m road half-width. It never writes or changes the real config.
Run: python competition_minimal/tests/continuous_sim.py
"""
import copy
import importlib.util
import math
from pathlib import Path
import struct
import sys
import threading
from types import ModuleType
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from control_core import Driver
from localization_core import CourseProgress, Estimator
from traffic_core import TrafficDecision
from udp_protocol import Command, ControlGate, StatusSequence, command_packet, parse_status_packet


def lidar_node(path, zones, return_s):
    def mod(name, **values):
        obj = ModuleType(name)
        for k, v in values.items(): setattr(obj, k, v)
        return obj
    dummy = type('Dummy', (), {})
    modules = {'rospy': mod('rospy', loginfo_throttle=lambda *a: None),
               'geometry_msgs': mod('geometry_msgs'),
               'geometry_msgs.msg': mod('geometry_msgs.msg', PoseStamped=dummy),
               'sensor_msgs': mod('sensor_msgs', point_cloud2=mod('point_cloud2')),
               'sensor_msgs.msg': mod('sensor_msgs.msg', PointCloud2=dummy),
               'std_msgs': mod('std_msgs'),
               'std_msgs.msg': mod('std_msgs.msg', Bool=dummy, Float64=dummy)}
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('simulation_lidar', ROOT/'scripts/lidar_missions.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    n = module.LidarSafety.__new__(module.LidarSafety)
    n.path = np.asarray(path); n.cum = np.r_[0., np.cumsum(np.linalg.norm(np.diff(n.path, axis=0), axis=1))]
    n.total = n.cum[-1]; n.zones = zones; n.allow_avoidance = True
    n.road_half_width = 6.; n.highway_return_s = return_s
    n.offset = 3.2; n.lookahead = 5.; n.plan = n.plan_mode = n.ped_lock = None
    n.lateral = 0.; n.lock = threading.Lock()
    return n


def status_packet(speed, yaw, now, gear=4, mode=2):
    values = [0]*30
    sec = int(now); nsec = int((now-sec)*1e9)
    values[:4] = [sec, nsec, mode, gear]
    values[19], values[20] = math.degrees(yaw), speed*3.6
    body = struct.pack('<iibbfiff' + 'f'*22 + '38s', *values, b'link')
    return b'#MoraiInfo$' + struct.pack('<iiii', 152, 0, 0, 0) + body + b'\r\n'


def ground_samples(driver, progress, pose):
    # Idealized, dense ground returns; no assertion about real LiDAR visibility.
    ss = np.arange(max(0., progress-22), min(driver.s[-1], progress+65), 1.)
    ds = np.arange(-6.5, 6.51, .75)
    world = []
    for s in ss:
        x, y, yaw = driver.point(s)
        for d in ds: world.append((x-math.sin(yaw)*d, y+math.cos(yaw)*d))
    delta = np.asarray(world) - np.asarray(pose[:2])
    c, sn = math.cos(pose[2]), math.sin(pose[2])
    body = np.column_stack((c*delta[:, 0]+sn*delta[:, 1], -sn*delta[:, 0]+c*delta[:, 1]))
    # Match ground_and_clusters' raw x ROI (-25,40), translated by the
    # configured +2 m LiDAR x offset; never fabricate farther returns.
    return body[(body[:,0] > -23.) & (body[:,0] < 42.) & (np.abs(body[:,1]) < 20.)]


def obstacle(driver, ego, position, side, pose, ident, size, height=1.5):
    x, y, direction = driver.point(position)
    x -= math.sin(direction)*side; y += math.cos(direction)*side
    dx, dy = x-pose[0], y-pose[1]
    c, sn = math.cos(pose[2]), math.sin(pose[2])
    bx, by = c*dx+sn*dy, -sn*dx+c*dy
    radius = size/2
    return dict(s=position, d=side, hs=radius, hd=radius, bx=bx, by=by,
                radius=radius, wx=x, wy=y, vx=0., vy=0., observed=1, id=ident,
                size=size, height=height, clear=0)


def simulate(max_steps=16000, dt=.1, late_pedestrian=False):
    config = yaml.safe_load((ROOT/'config/missions.yaml').read_text(encoding='utf-8'))
    path = np.loadtxt(ROOT/'paths/global_path.txt', usecols=(0, 1))
    driver = Driver(path); course = CourseProgress(path); est = Estimator(); gate = ControlGate()
    sequence = StatusSequence()
    # Only in simulation, assume the provided candidate crossing and ground
    # bounds have been independently verified. Never alter real mission YAML.
    intersections = copy.deepcopy(config['intersections'])
    for item in intersections: item['verified'] = True
    traffic = TrafficDecision(intersections, True)
    lidar = lidar_node(path, config['zones'], config['highway_return_s'])
    x, y, yaw = driver.point(0.); speed = 0.; now = 1.
    est.fix(x, y, now)
    seen = set(); events = []; starts = {}; previous_mode = None
    ped = None; dr_distance = None; signal_red_stop = False; signal_green_restart = False
    max_error = 0.; reason = 'step limit'
    gate_brake = False
    gear, mode = 4, 2
    finish_speed = finish_error = stopped_error = None
    last_progress, stagnant_time = 0., 0.
    for step in range(max_steps):
        now += dt
        packet = status_packet(speed, yaw, now, gear, mode)
        status, stamp, kind = parse_status_packet(packet)
        sequence.accept(stamp); gate.observe(status, now)
        in_tunnel = config['zones']['tunnel'][0] <= (course.progress or 0) < config['zones']['tunnel'][0]+45
        if not est.advance(now, status.speed, yaw):
            reason = 'localization rejected advance'; break
        if not in_tunnel:
            if not est.fix(x, y, now): reason = 'localization rejected GPS'; break
        if not course.update(est.x, est.y):
            reason = 'course projection failed'; break
        s = course.progress; max_error = max(max_error, course.error)
        if in_tunnel:
            if dr_distance is None: dr_distance = s
            if s > dr_distance+5: seen.add('DR')
        elif dr_distance is not None: seen.add('GPS_RESTORED')
        if step == 0 and s < 1 and not driver.finished: seen.add('START')
        for name, (a, b) in config['zones'].items():
            if a <= s <= b and name not in seen:
                seen.add(name); events.append((round(now, 1), round(s, 1), name))
        idx = traffic.current(s)
        if idx is not None:
            key = 'signal%d' % (idx+1)
            if key not in starts: starts[key] = now; seen.add(key); events.append((round(now,1),round(s,1),key))
            label = ('4red' if idx == 0 and now-starts[key] < 18 else
                     '4greenleft' if intersections[idx]['direction'] == 'left' else '4green')
            traffic.observe(s, [label], now)
        tstop, tlimit, treason = traffic.decision(s, now)
        if idx == 0 and treason.startswith('STOP') and speed < .25: signal_red_stop = True
        if idx == 0 and signal_red_stop and treason.startswith(('GO', 'COMMITTED')) and speed > .5: signal_green_restart = True
        # Objects are fixed in world/path coordinates and appear as the car approaches.
        # Pedestrian and roundabout conflicts clear through scripted observations.
        objects = []
        static_s = 330.601
        if static_s-34 < s < static_s+8:
            objects.append(obstacle(driver,s,static_s,0.,(x,y,yaw),10,0.7))
        if (442 if late_pedestrian else 423) < s < 465:
            if 'ped' not in starts: starts['ped'] = now
            # Observed crossing at 1.2 m/s, then stays visibly outside lane.
            side = min(4.5, max(0., now-starts['ped']-5.)*1.2)
            p = obstacle(driver,s,457.,side,(x,y,yaw),20,.7)
            if ped is not None: p['clear'] = ped.get('clear', 0)
            ped = p; objects.append(p)
        if 815 < s < 840:
            if 'round' not in starts: starts['round'] = now
            # Vehicle starts moving out of the entry corridor after 4 seconds.
            side = 4. + max(0., now-starts['round']-4.)*4.
            if side < 18.:
                objects.append(obstacle(driver,s,830.,side,(x,y,yaw),30,2.))
        if 1180 < s < 1240:
            if 'lead' not in starts: starts['lead'] = now
            # A stopped NPC starts moving after a short delay. The event log
            # makes it explicit if passing required the NPC to move away.
            delta = max(0., now-starts['lead']-15.) * 4.
            if delta > 0 and 'LEAD_MOVES' not in seen:
                seen.add('LEAD_MOVES'); events.append((round(now,1),round(s,1),'lead_moves'))
            objects.append(obstacle(driver,s,1215.+delta,0.,(x,y,yaw),40,2.))
        tunnel_obj = config['zones']['tunnel'][0]+63
        if tunnel_obj-34 < s < tunnel_obj+8:
            objects.append(obstacle(driver,s,tunnel_obj,0.,(x,y,yaw),50,0.7))
        # Physical footprint check against synthetic boxes; the planner's
        # predicted clearance alone does not establish actual tracking safety.
        if any(-1.8-o['radius'] <= o['bx'] <= 3.5+o['radius'] and
               abs(o['by']) <= 1.196+o['radius'] for o in objects):
            reason = 'synthetic vehicle footprint intersects object at s=%.2f' % s
            break
        pose = (est.x, est.y, yaw)
        if step % 5 == 0:
            ground = ground_samples(driver,s,pose) if objects or lidar.plan else None
            lidar.decide(objects, s, objects, ground, pose)
        if 815 < s < 840 and lidar.stop: seen.add('ROUND_YIELD')
        if 'ROUND_YIELD' in seen and 815 < s < 840 and not lidar.stop and speed > .5: seen.add('ROUND_RELEASE')
        if s >= config['highway_return_s']: seen.add('CP12')
        if s >= config['zones']['highway'][1]: seen.add('CP13')
        # These are the only checkpoint coordinates supplied for independent
        # radius checks; passing their route indices alone is insufficient.
        for checkpoint, point in (('CP10_3M', (62.0739, 256.4949)),
                                  ('CP12_3M', (75.4142, -217.8551)),
                                  ('CP13_3M', (70.2141, -365.7485))):
            if math.hypot(x-point[0], y-point[1]) <= 3.0:
                seen.add(checkpoint)
        if lidar.ped_lock is not None: seen.add('YIELD')
        if 'YIELD' in seen and lidar.ped_lock is None and ped is not None: seen.add('PED_CLEAR')
        if lidar.ped_lock is not None and speed < .2: seen.add('PED_STOP')
        if 'PED_STOP' in seen and 'PED_CLEAR' in seen and speed > .5: seen.add('PED_RESTART')
        if lidar.plan_mode != previous_mode:
            if lidar.plan_mode: events.append((round(now,1),round(s,1),'plan_'+lidar.plan_mode)); seen.add('plan_'+lidar.plan_mode)
            elif previous_mode: events.append((round(now,1),round(s,1),'return_'+previous_mode)); seen.add('return_'+previous_mode)
            previous_mode = lidar.plan_mode
        limit = min(tlimit, lidar.speed_limit, 15/3.6 if in_tunnel else math.inf)
        cmd = driver.update(est.x,est.y,yaw,s,status.speed,lidar.lateral,limit,tstop or lidar.stop,dt)
        if lidar.plan and abs(lidar.lateral) > .5 and abs(cmd[2]) > .02:
            seen.add('OFFSET_STEERING')
        output = gate.output(cmd,now,now,driver.finished)
        if output is None:
            if gate.complete and driver.finished:
                seen.add('MANUAL_P'); reason = 'finished_and_handed_over'
            else:
                reason = 'control gate disabled'
            break
        wire = Command.from_buffer_copy(command_packet(*output))
        # Simple kinematic bicycle with idealized acceleration/braking.
        acceleration = 2.5*wire.accel - 4.0*wire.brake - .03*speed
        speed = max(0., speed + acceleration*dt)
        yaw += speed/3.0*math.tan(wire.steer*math.radians(40.))*dt
        yaw = math.atan2(math.sin(yaw),math.cos(yaw))
        x += speed*math.cos(yaw)*dt; y += speed*math.sin(yaw)*dt
        # Synthetic MORAI acknowledgment; never assume the real simulator does so.
        if speed < .05:
            gear, mode = wire.gear, wire.mode
        if driver.finished:
            if finish_speed is None:
                finish_speed = speed; finish_error = math.hypot(x-path[-1,0], y-path[-1,1])
                events.append((round(now,1),round(s,1),'finish speed=%.2f' % speed))
            gate_brake = gate_brake or wire.brake == 1.
            seen.add('FINISH')
            if speed < .05: stopped_error = math.hypot(x-path[-1,0], y-path[-1,1])
        if s > last_progress + .5:
            last_progress, stagnant_time = s, now
        elif now-stagnant_time > 70:
            from lidar_core import candidate_clear, trajectory_observed
            clear = candidate_clear(objects,*lidar.plan,from_s=s) if lidar.plan else None
            observed = trajectory_observed(ground,lidar.path,lidar.cum,pose,s,lidar.plan) if lidar.plan and ground is not None else None
            reason = ('no progress for 70 seconds; traffic=%s lidar_stop=%s plan=%s candidate_clear=%s observed=%s objects=%s' %
                      (treason,lidar.stop,lidar.plan_mode,clear,observed,
                       [(round(o['bx'],1),round(o['by'],1),round(o['s']-s,1)) for o in objects]))
            break
        if step % 1000 == 0 and step:
            events.append((round(now,1),round(s,1),'telemetry speed=%.2f traffic=%s lidar=%s limit=%.2f' %
                           (speed,treason,lidar.stop,limit)))
    result = {'reason':reason,'seconds':round(now,1),'progress':round(course.progress or 0.,2),
              'length':round(driver.s[-1],2),'speed':round(speed,2),'max_error':round(max_error,2),
              'red_stop':signal_red_stop,'green_restart':signal_green_restart,'finish_brake':gate_brake,
              'finish_speed':finish_speed, 'finish_position_error':finish_error,
              'stopped_position_error':stopped_error,
              'seen':sorted(seen),'events':events}
    return result


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--late-pedestrian', action='store_true',
                        help='stress case: person appears only about 15 m ahead; may collide')
    args = parser.parse_args()
    result = simulate(late_pedestrian=args.late_pedestrian)
    print(json.dumps(result, indent=2))
    required = {'START','signal1','signal2','signal3','signal4','static','pedestrian','roundabout',
                'ROUND_YIELD','ROUND_RELEASE','highway','CP12','CP13','CP10_3M','CP12_3M','CP13_3M','tunnel','DR','GPS_RESTORED',
                'plan_static','return_static','YIELD','PED_STOP','PED_CLEAR','PED_RESTART','plan_highway','return_highway',
                'plan_tunnel','return_tunnel','OFFSET_STEERING','FINISH','MANUAL_P'}
    missing = sorted(required-set(result['seen']))
    if (missing or not (result['red_stop'] and result['green_restart'] and result['finish_brake']) or
            result['stopped_position_error'] is None or result['stopped_position_error'] > 3.):
        print('FAILED: missing events:', ', '.join(missing))
        sys.exit(1)
    print('PASS: synthetic continuous mission loop')
