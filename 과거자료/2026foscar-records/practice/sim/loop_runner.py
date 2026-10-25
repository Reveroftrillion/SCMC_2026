#!/usr/bin/env python3
"""PRACTICE ONLY: repeat one mission from a saved scenario and judge each run.

Per run: ScenarioLoad (with network connections) -> run_competition.sh (drives!)
-> watch ROS topics + collision (9092) + ObjectInfo (7505) -> stop the stack
-> one JSON line in practice/sim/runs/<stamp>/results.jsonl.

Uses channels forbidden in competition (collision, ObjectInfo, scenario load).
Lives in practice/ only; the driving code never imports it.

    python3 practice/sim/loop_runner.py SenarioLoadTest --runs 3 --end-s 2010
"""
import argparse
import json
import math
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
RECORDS = Path.home()/'foscar_logs'/'records.json'   # best records, written by race_hud_node while driving
LAPS = Path.home()/'foscar_logs'/'laps.json'         # every full lap (runner), shown in the README
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from sim_scenario import packet as scenario_packet, sim_ip   # noqa: E402
from record_collision import decode_packet           # noqa: E402

import rclpy                                          # noqa: E402
from rclpy.node import Node                           # noqa: E402
from rclpy.qos import qos_profile_sensor_data         # noqa: E402
from std_msgs.msg import Float64, String              # noqa: E402
from nav_msgs.msg import Odometry                     # noqa: E402

SIM_IP = sim_ip()   # src/foscar_bringup/config/low_speed.yaml
# Full EgoVehicleStatus (practice network: sim 9106 -> 9111). Ground truth only.
TRUTH_FMT = '<iibbfiff'+'f'*22+'38s'+'f'*12


def truth(raw):
    if len(raw) != 229 or raw[:11] != b'#MoraiInfo$':
        return None
    v = struct.unpack_from(TRUTH_FMT, raw, 27)
    return dict(x=v[14], y=v[15], yaw=math.radians(v[19]), vx=v[20], vy=v[21], ax=v[26], ay=v[27],
                lat_force=max(abs(f) for f in v[31:35]), slip=max(abs(f) for f in v[35:39]))
STACK_PORTS = (9099, 9102, 9104, 9302)


def udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port))
    s.setblocking(False)
    return s


def objects(raw):
    out = {}
    rec = '<hh'+'f'*16+'38s'
    n = struct.calcsize(rec)
    for i in range(20):
        v = struct.unpack_from(rec, raw, 38+i*n)
        if v[0] or v[1]:
            # the position is the rear axle end: the body centre lies l/2 - rear overhang ahead along the heading
            out[v[0]] = dict(type=v[1], xy=(round(v[2], 2), round(v[3], 2)), yaw=math.radians(v[5]), l=v[6], w=v[7],
                             ahead=v[6]/2-v[11])
    return out


def npc_centre(o):
    """Body centre of a listed / logged object: its position is l/2 - rear overhang behind it (1.48 m on the ring
    cars, 2026-10-02; logs before then carry no rear overhang: that value)."""
    yaw = o['yaw'] if 'ahead' in o else math.radians(o.get('yaw', 0.))
    ahead = o['ahead'] if 'ahead' in o else (o['l']/2-o['ro'] if 'ro' in o else 1.48)
    x, y = o['xy'] if 'xy' in o else (o['x'], o['y'])
    return x+ahead*math.cos(yaw), y+ahead*math.sin(yaw), yaw


def box(x, y, yaw, l, w):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+u*c-v*s, y+u*s+v*c) for u, v in ((l/2, w/2), (l/2, -w/2), (-l/2, -w/2), (-l/2, w/2))]


def box_gap(A, B):
    """Separation of two convex quads (0 when they overlap)."""
    best = 0.
    for P in (A, B):
        for i in range(4):
            ax, ay = P[(i+1) % 4][1]-P[i][1], P[i][0]-P[(i+1) % 4][0]; n = math.hypot(ax, ay)
            if n < 1e-9:
                continue
            pa = [(p[0]*ax+p[1]*ay)/n for p in A]; pb = [(p[0]*ax+p[1]*ay)/n for p in B]
            best = max(best, min(pb)-max(pa), min(pa)-max(pb))
    return best


EGO_CENTRE, EGO_L, EGO_W = 1.4825-.28, 4.635, 1.892    # truth ego pose: rear axle +0.28 m


def object_rows(raw):
    """Every listed object in full (id, type, pose, size, speed) for objects_<n>.jsonl: which detections were
    real cars (user 2026-10-01: a red box ahead of a truck in a lane that looked empty). Fields per the ObjectInfo
    record: pos xyz, heading, size xyz, overhang, wheelbase, rear overhang, velocity xyz, acceleration xyz."""
    out = []
    rec = '<hh'+'f'*16+'38s'
    n = struct.calcsize(rec)
    for i in range(20):
        v = struct.unpack_from(rec, raw, 38+i*n)
        if v[0] or v[1]:
            out.append(dict(id=v[0], type=v[1], x=round(v[2], 2), y=round(v[3], 2), z=round(v[4], 2), yaw=round(v[5], 2),
                            l=round(v[6], 2), w=round(v[7], 2), h=round(v[8], 2), v=round(math.hypot(v[12], v[13]), 2),
                            ro=round(v[11], 3)))
    return out


def stack_pids():
    out = subprocess.run(['ps', '-eo', 'pid,cmd'], capture_output=True, text=True).stdout
    keys = ('ros2 launch foscar_bringup competition', 'install/foscar_', 'ros2 bag record', 'static_transform_publisher --x')
    # rviz.sh (bev_node + rviz2) stays up across runs on purpose.
    return [int(l.split()[0]) for l in out.splitlines()[1:]
            if any(k in l for k in keys) and 'bev_node' not in l]


def ports_busy():
    out = subprocess.run(['ss', '-lun'], capture_output=True, text=True).stdout
    return [p for p in STACK_PORTS if f':{p} ' in out]


def stop_stack(proc):
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        for _ in range(12):            # bridge hands off to Manual/P on SIGINT
            if proc.poll() is not None:
                break
            time.sleep(1)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        pids = stack_pids()
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(3)


class Watch(Node):
    def __init__(self):
        super().__init__('practice_loop_watch')
        self.s = math.nan
        self.telemetry = {}
        self.behavior = ''
        self.pose = None
        self.mode = ''
        self.create_subscription(Float64, '/localization/s', lambda m: setattr(self, 's', m.data), qos_profile_sensor_data)
        self.create_subscription(String, '/control/telemetry', self.on_tel, 10)
        self.create_subscription(String, '/behavior/state', lambda m: setattr(self, 'behavior', m.data), 10)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/localization/mode', lambda m: setattr(self, 'mode', m.data), 10)

    def on_odom(self, m):
        o = m.pose.pose
        q = o.orientation
        self.pose = (o.position.x, o.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def on_tel(self, msg):
        try:
            self.telemetry = json.loads(msg.data)
        except ValueError:
            pass


def pose_error(errs):
    """Our pose minus ground truth. The GPS-section median is the fixed reference-point
    offset (rear axle vs sim origin); DR drift is measured relative to it."""
    import statistics
    if not errs['GPS']:
        return None
    lon0 = statistics.median(e[0] for e in errs['GPS'])
    lat0 = statistics.median(e[1] for e in errs['GPS'])
    out = dict(gps_offset_lon=round(lon0, 3), gps_offset_lat=round(lat0, 3), gps_samples=len(errs['GPS']),
               dr_samples=len(errs['DR']))
    if errs['DR']:
        out.update(dr_max_lon=round(max(abs(e[0]-lon0) for e in errs['DR']), 3),
                   dr_max_lat=round(max(abs(e[1]-lat0) for e in errs['DR']), 3),
                   dr_end_lat=round(errs['DR'][-1][1]-lat0, 3))
    return out


def code_version():
    git = lambda *a: subprocess.run(['git', '-C', str(HERE), *a], capture_output=True, text=True).stdout.strip()
    commit = git('rev-parse', '--short', 'HEAD')
    return f"{git('rev-parse', '--abbrev-ref', 'HEAD')}@{commit}{'+dirty' if git('status', '--porcelain', '--untracked-files=no') else ''}"


def keep_records(rec, before, no_npc=False):
    """Records only from a run that passed: a run stopped, aborted or driven on a frozen simulator
    (dead reckoning kept moving the car) set a 7.2 s box mission and a 12.35 s pedestrian mission
    (2026-09-29). Every full lap from the start line to the end is logged with its time."""
    if no_npc:
        rec = dict(rec, result='NO_NPC')          # an empty road sets no record (unfair on the highway)
    if rec.get('result') != 'PASS':
        now = RECORDS.read_text() if RECORDS.is_file() else None
        if now != before:
            if before is None:
                RECORDS.unlink()
            else:
                RECORDS.write_text(before)
            print('records from a run that did not pass: discarded', flush=True)
        return
    if rec.get('scenario') == 'Practice3_START' and (rec.get('s_end') or 0) >= 2164. and rec.get('drive_s'):
        try:
            laps = json.loads(LAPS.read_text()) if LAPS.is_file() else []
        except ValueError:
            laps = []
        laps.append({'s': rec['drive_s'], 'at': rec['started'], 'code': code_version(),
                     'collisions': len(rec.get('collisions') or [])})
        LAPS.write_text(json.dumps(laps, indent=1, ensure_ascii=False))


def one_run(args, index, out_dir, watch):
    rec = dict(run=index, scenario=args.scenario, started=datetime.now().isoformat(timespec='seconds'))
    # 1) reset the world
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        # --no-npc: clear the world (delete_all) and load it without NPC vehicles. npc=False alone only keeps the
        # NPCs already on the map (controlled tests of our own driving, user 2026-09-29).
        s.sendto(scenario_packet(args.scenario, delete_all=args.no_npc, network=True, npc=not args.no_npc), (SIM_IP, 9095))
    time.sleep(args.settle)
    # 2) drive
    log = open(out_dir/f'launch_{index}.log', 'w')
    proc = subprocess.Popen(['bash', str(ROOT/'run_competition.sh'), 'rviz:='+('true' if args.rviz else 'false'),
                             'bag_points:='+('true' if args.record_points else 'false'), *args.launch_arg],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
    col, obj, tru = udp(9092), udp(7505), udp(9111)
    errs = {'GPS': [], 'DR': []}          # (lon, lat) pose error vs ground truth, vehicle frame
    peak = dict(ay=0., slip=0., lat_force=0., vy=0.)
    peak_at = {}                      # key -> (s, km/h) where the peak happened
    ay_by_10m = {}                    # 10 m station bin -> (|ay|, km/h): where lateral load builds
    known, hits, ghosts, raw_slots = {}, [], [], []
    latched = set()   # obstacle ids whose collision flag was already on while far from us
    truth_file, truth_logged = open(out_dir/f'truth_{index}.jsonl', 'w'), [0.]
    objects_file, objects_logged = open(out_dir/f'objects_{index}.jsonl', 'w'), [0.]
    origin, pushed, jitter = {}, {}, {}   # static obstacle id -> first listed xy / displacement by us / by itself
    t0 = time.monotonic()
    started = moving_since = None
    s_start = None
    history, min_v, max_brake = deque(), math.inf, 0.
    drops = []
    stops, stopped = [], False        # full stops after departure (e.g. an AVOID stop line)
    splits = {}                       # boundary station -> time since departure
    recent, last_tel = [], None
    true_v = true_xy = None; true_yaw = 0.   # ground-truth speed / position (9111): vetoes telemetry glitches and far collision packets
    result = None
    try:
        while result is None:
            rclpy.spin_once(watch, timeout_sec=.05)
            for sock in select.select([col, obj, tru], [], [], 0)[0]:
                for _ in range(64):
                    try:
                        raw, _ = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    if sock is tru:
                        g = truth(raw)
                        if g:
                            true_v = math.hypot(g['vx'], g['vy'])/3.6   # truth velocity is km/h
                            true_xy = (g['x'], g['y']); true_yaw = g['yaw']
                            # True pose with wall time, to line up with the bag (practice/sim/box_error.py)
                            if time.time() - truth_logged[0] >= .05:
                                truth_logged[0] = time.time()
                                truth_file.write(json.dumps(dict(t=round(truth_logged[0], 3), x=round(g['x'], 3),
                                                                 y=round(g['y'], 3), yaw=round(g['yaw'], 5))) + '\n')
                        if g and started is not None:
                            v_kmh = round(math.hypot(g['vx'], g['vy']), 1)   # truth velocity is km/h
                            for k in peak:
                                if abs(g[k]) > peak[k]:
                                    peak[k] = abs(g[k])
                                    peak_at[k] = (round(watch.s, 1), v_kmh)
                            if math.isfinite(watch.s):
                                b = int(watch.s//10*10)
                                if abs(g['ay']) > ay_by_10m.get(b, (0.,))[0]:
                                    ay_by_10m[b] = (round(abs(g['ay']), 2), v_kmh)
                            if watch.pose is not None and watch.mode in errs:
                                dx, dy = watch.pose[0]-g['x'], watch.pose[1]-g['y']
                                c, sn = math.cos(g['yaw']), math.sin(g['yaw'])
                                errs[watch.mode].append((c*dx+sn*dy, -sn*dx+c*dy))
                    elif sock is obj and raw[:14] == b'#MoraiObjInfo$':
                        seen = objects(raw)
                        known.update(seen)
                        # Wall time, to line up with the bag like truth_<n>.jsonl.
                        if time.time() - objects_logged[0] >= .1:
                            objects_logged[0] = time.time()
                            objects_file.write(json.dumps(dict(t=round(objects_logged[0], 3), objects=object_rows(raw))) + '\n')
                        # Static obstacles (type 2) do not move by themselves: one displaced from where it
                        # was first listed was pushed by us. Collision packets missed most barricade hits
                        # (user 2026-09-29: "it drives on pushing the obstacles").
                        for oid, o in seen.items():
                            if o['type'] != 2:
                                continue
                            first = origin.setdefault(oid, o['xy'])
                            moved = math.hypot(o['xy'][0]-first[0], o['xy'][1]-first[1])
                            if moved > .15 and oid not in pushed and oid not in jitter and started is not None:
                                # Only while our car is next to it (true pose, centre distance): a barricade
                                # spawned badly shivers by itself; the last tunnel one moved 0.2 m with the
                                # car 3.8 m away and untouched (user 2026-09-29). Real hits were 2.7-2.8 m.
                                gap = math.hypot(o['xy'][0]-true_xy[0], o['xy'][1]-true_xy[1]) if true_xy else None
                                row = dict(t=round(time.monotonic()-t0, 2), s=watch.s, id=oid, xy=first,
                                           moved=round(moved, 2), dist=gap and round(gap, 2))
                                if gap is None or gap < 3.2:
                                    pushed[oid] = row
                                else:
                                    jitter[oid] = row
                    elif sock is col:
                        try:
                            slots = decode_packet(raw)['slots']
                            # A latched flag clears once its id drops out of a packet.
                            latched &= {sl['object_id'] for sl in slots}
                            for slot in slots:
                                # Every non-empty slot, raw (user 2026-09-29: every tunnel run hit a barricade
                                # and none was recorded: type or listing filters dropped them).
                                if started is not None and slot['object_id'] != 0 and len(raw_slots) < 200:
                                    raw_slots.append(dict(t=round(time.monotonic()-t0, 2), s=watch.s, id=slot['object_id'],
                                                          type=slot['object_type'],
                                                          pos=[round(x, 2) for x in slot['position']]))
                                # Before the ego starts, collision packets are the previous run's
                                # crash or the respawn (2 of 3 runs "collided" at s858, 2026-09-26).
                                # ObjectInfo carries at most 20 bodies: with 21 in the tunnel scenario the last barricade
                                # was never listed and a hit on it was dropped silently (user 2026-09-29). A static
                                # obstacle / pedestrian cannot be hit by anyone else, so an unlisted one is ours.
                                # An unlisted flag already on in the first 3 s (we have hardly moved) is latched too: the
                                # CP08 start reported both tunnel boxes "hit" 1.8 s in, 1000 m away (2026-09-30).
                                if (started is not None and slot['object_id'] != 0 and slot['object_id'] not in known
                                        and slot['object_type'] in (0, 2) and time.monotonic()-t0-started < 3.):
                                    latched.add(slot['object_id'])
                                if (started is not None and slot['object_id'] != 0 and slot['object_id'] not in known and slot['object_type'] in (0, 2)
                                        and slot['object_id'] not in latched
                                        and not any(h['id'] == slot['object_id'] and h.get('unlisted') for h in hits[-1:])):
                                    hits.append(dict(t=round(time.monotonic()-t0, 2), s=watch.s, id=slot['object_id'],
                                                     type=slot['object_type'], dist=None, unlisted=True))
                                if started is not None and slot['object_id'] in known and slot['object_type'] in (0, 1, 2):   # 0 pedestrian (assumed), 1 vehicle, 2 obstacle
                                    ko = known[slot['object_id']]
                                    ox, oy = ko['xy']
                                    gap = math.hypot(ox-true_xy[0], oy-true_xy[1]) if true_xy else None
                                    # Body to body for vehicles (both at their real body centres): centre to centre a
                                    # nose-to-tail touch is 4-5 m and fell between "ours" (< 4 m) and "far" (> 5 m),
                                    # dropped silently -- three roundabout hits, 2026-10-02 21:02
                                    bgap = None
                                    if true_xy and slot['object_type'] == 1:
                                        cx, cy, oyaw = npc_centre(ko)
                                        bgap = box_gap(box(true_xy[0]+EGO_CENTRE*math.cos(true_yaw), true_xy[1]+EGO_CENTRE*math.sin(true_yaw),
                                                           true_yaw, EGO_L, EGO_W), box(cx, cy, oyaw, ko['l'], ko['w']))
                                    row = dict(t=round(time.monotonic()-t0, 2), s=watch.s, id=slot['object_id'],
                                               type=slot['object_type'], dist=gap and round(gap, 1),
                                               body_gap=None if bgap is None else round(bgap, 2))
                                    # Only a vehicle within 4 m of us is our collision (user 2026-09-28: the NPCs that
                                    # crashed around the map were removed; crashes elsewhere are not tracked). Real
                                    # hits were 2.7-2.8 m centre to centre; wrecks we passed at 4.7-5.5 m were not.
                                    # A static obstacle or pedestrian is hit by nobody else: always ours. The centre
                                    # gate dropped a hit on the edge of a 3 m wide barricade (full lap s1918, 2026-09-29).
                                    # ...unless it is beyond any body we could touch (a real edge hit is <= ~4.5 m centre
                                    # to centre): every CP14 start reported the first tunnel box "hit" 62.5 m away 1.8 s
                                    # in, and the packet kept coming until we were 9.8 m from it (2026-09-30).
                                    far = gap is not None and gap > 5.
                                    # The flag of a box that spawned touching something stays on all run (on from 2 s,
                                    # 130-150 m away, and still on as we passed it 4.8 m off, 2026-09-30 full laps):
                                    # a flag already on while far is not ours until it has gone off once.
                                    if far and slot['object_type'] in (0, 2):
                                        latched.add(slot['object_id'])
                                    if slot['object_id'] in latched and slot['object_type'] in (0, 2):
                                        if len(ghosts) < 20:
                                            ghosts.append(dict(row, latched=True))
                                    elif (slot['object_type'] in (0, 2) and not far) or (bgap is not None and bgap < .5) \
                                            or (bgap is None and gap is not None and gap < 4.0):
                                        if not any(h['id'] == slot['object_id'] for h in hits[-1:]):
                                            hits.append(row)
                                    elif far and len(ghosts) < 20:
                                        ghosts.append(row)
                        except ValueError:
                            pass
            now = time.monotonic()-t0
            tel = watch.telemetry
            raw_v = tel.get('speed_mps')
            if raw_v is not None and tel is not last_tel:
                recent.append(raw_v)
                del recent[:-3]
            last_tel = tel
            # Median of the last 3 samples: telemetry once read 0.0 for a single sample
            # at 19 km/h and ended a clean run as a "collision" (2026-09-26).
            v = sorted(recent)[len(recent)//2] if recent else None
            state = tel.get('state', '')
            if started is None and state == 'CRUISE' and v is not None and v > .5:
                started, s_start = now, watch.s
            if v is not None:
                if started is not None and watch.s > args.watch_from:
                    min_v = min(min_v, v)
                    max_brake = max(max_brake, tel.get('brake', 0.) or 0.)
                # Unbraked loss of >2.5 m/s within 0.5 s: an impact, not our deceleration
                # (the recorded barricade hit went 43 -> 5 km/h in ~1 s with brake 0).
                history.append((now, v, tel.get('brake') or 0.))
                while history and now-history[0][0] > 1.:
                    history.popleft()
                recent_hist = [h for h in history if now-h[0] <= .5]
                # The sim status speed itself glitched to 0 at 42 km/h (2026-09-26, s1987);
                # an impact must also show in the ground-truth speed. Brakes count over 1 s: the car
                # keeps slowing after a brake 1.0 pulse eases to 0.15 (pedestrian yield, 2026-09-27).
                if (started is not None and recent_hist[0][1]-v > 2.5 and max(h[2] for h in history) < .2
                        and now-recent_hist[0][0] > .3 and (true_v is None or recent_hist[0][1]-true_v > 2.5)):
                    drops.append(dict(t=round(now, 2), s=round(watch.s, 1), from_kmh=round(recent_hist[0][1]*3.6, 1),
                                      to_kmh=round(v*3.6, 1)))
                # crawling (< 2 km/h) counts as standing: a car creeping in front of a barricade kept the
                # batch waiting for the full timeout (2026-09-29)
                moving_since = now if v > .55 else moving_since
                if started is not None:
                    for b in args.splits:
                        if b not in splits and math.isfinite(watch.s) and watch.s >= b:
                            splits[b] = round(now-started, 2)
                if started is not None and now-started > 2.:
                    if v < .3 and (true_v is None or true_v < .3) and not stopped:
                        stops.append(round(watch.s, 1))
                    stopped = v < .3
            # Collision packets and speed drops are only recorded (collisions / speed_drops) for later analysis,
            # not a reason to stop: a crash shows as the car standing still > --stop-s (user 2026-09-28).
            if math.isfinite(watch.s) and watch.s >= args.end_s:
                result = 'PASS'
            elif state == 'FAULT':
                result = 'FAULT:'+str(tel.get('reason', ''))[:60]
            elif proc.poll() is not None:
                # A required node died and the launch shut down: say so now instead of waiting for the
                # timeout (behavior_node crashed and the batch sat for 2 minutes, 2026-09-29).
                result = 'STACK_EXITED'
            elif started is not None and moving_since is not None and now-moving_since > args.stop_s:
                result = f'STOPPED:{watch.behavior}'   # stood still > --stop-s: treated as a crash / deadlock
            elif started is None and now > args.start_timeout:
                result = 'NO_START:'+state
            elif now > args.timeout:
                result = 'TIMEOUT'
    finally:
        truth_file.close()
        objects_file.close()
        col.close()
        obj.close()
        tru.close()
        stop_stack(proc)
        log.close()
    if pushed and result == 'PASS':
        result = 'PUSHED:' + ','.join(str(p['id']) for p in pushed.values())
    # Contacts are judged by the simulator's collision packets (user 2026-10-02). The truth-box check below is kept
    # for information only: the NPC truth point is ~1.5 m behind the body centre (lidar hulls seen whole, front end
    # +1.34 / rear end +1.61 m p50, t8/t9 runs), so its boxes reported 7 contacts that never happened.
    min_gap, contacts = body_contacts(out_dir, index)
    if hits and (result == 'PASS' or str(result).startswith('PUSHED')):
        result = 'COLLISION:' + ','.join(sorted({str(h.get('id')) for h in hits}))
    rec.update(result=result, s_start=s_start, s_end=round(watch.s, 1) if math.isfinite(watch.s) else None,
               drive_s=round(time.monotonic()-t0-(started or 0.), 1) if started else None,
               min_kmh=round(min_v*3.6, 1) if math.isfinite(min_v) else None, max_brake=round(max_brake, 2),
               splits=splits, full_stops=stops, collisions=hits[:5], pushed=list(pushed.values()), moved_alone=list(jitter.values()), collision_slots=[dict(r) for i, r in enumerate(raw_slots) if i == 0 or (r["id"], r["type"]) != (raw_slots[i-1]["id"], raw_slots[i-1]["type"])][:20], far_collision_packets=ghosts[:5], speed_drops=drops[:5],
               truth_peaks={k: round(v, 2) for k, v in peak.items()}, truth_peaks_at=peak_at,
               ay_bins={b: a for b, (a, v) in sorted(ay_by_10m.items())},
               ay_top=sorted(([b, a, v] for b, (a, v) in ay_by_10m.items()), key=lambda r: -r[1])[:10], pose_error=pose_error(errs),
               obstacles={k: v for k, v in known.items() if v['type'] == 2},
               min_body_gap_m=min_gap, body_contacts=contacts)
    return rec


def body_contacts(out_dir, index, ego_center_m=1.4825-.28, ego_l=4.635, ego_w=1.892):
    """Ego vs NPC body overlap from the logged truth (oriented boxes, both at their body centres). For information:
    contacts are judged by the collision packets. Truth ego pose: rear axle +0.28 m.
    Returns (min gap m, [contacts]) -- a contact per NPC id at its first overlapping sample."""
    def corners(x, y, yaw, l, w):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x+u*c-v*s, y+u*s+v*c) for u in (-l/2, l/2) for v in (-w/2, w/2)]
    def inside(p, P):
        cx, cy = sum(q[0] for q in P)/4, sum(q[1] for q in P)/4
        for a, b in ((P[2], P[0]), (P[1], P[0])):
            ax, ay = a[0]-b[0], a[1]-b[1];n = math.hypot(ax, ay)
            if abs((p[0]-cx)*ax+(p[1]-cy)*ay)/n > n/2+1e-6:return False
        return True
    def gap(A, B):
        if any(inside(p, B) for p in A) or any(inside(p, A) for p in B):return 0.
        return min(math.hypot(p[0]-q[0], p[1]-q[1]) for p in A for q in B)
    try:
        truth = [json.loads(l) for l in open(out_dir/f'truth_{index}.jsonl')]
        frames = [json.loads(l) for l in open(out_dir/f'objects_{index}.jsonl')]
    except (OSError, ValueError):
        return None, []
    if not truth:return None, []
    tt = [r['t'] for r in truth];best = math.inf;hits = {}
    import bisect
    for fr in frames:
        p = truth[min(bisect.bisect_left(tt, fr['t']), len(truth)-1)];h = p['yaw']
        E = corners(p['x']+ego_center_m*math.cos(h), p['y']+ego_center_m*math.sin(h), h, ego_l, ego_w)
        for o in fr['objects']:
            if o.get('type') != 1 or math.hypot(o['x']-p['x'], o['y']-p['y']) > 15.:continue
            cx, cy, oyaw = npc_centre(o)                         # the logged position is the rear axle end
            g = gap(E, corners(cx, cy, oyaw, o['l'], o['w']))
            best = min(best, g)
            if g <= 0. and o['id'] not in hits:
                hits[o['id']] = dict(t=round(fr['t']-truth[0]['t'], 2), id=o['id'], kmh=round(o['v']), x=round(p['x'], 1), y=round(p['y'], 1))
    return (round(best, 2) if math.isfinite(best) else None), list(hits.values())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario')
    ap.add_argument('--runs', type=int, default=1)
    ap.add_argument('--end-s', type=float, default=2010.)
    ap.add_argument('--watch-from', type=float, default=1870., help='speed stats from this station')
    ap.add_argument('--settle', type=float, default=4.)
    ap.add_argument('--stop-s', type=float, default=8., help='stationary this long after starting = STOPPED')
    ap.add_argument('--start-timeout', type=float, default=30.)
    ap.add_argument('--timeout', type=float, default=150.)
    ap.add_argument('--rviz', action='store_true')
    ap.add_argument('--launch-arg', action='append', default=[], help='extra run_competition.sh arg, e.g. camera:=true')
    ap.add_argument('--record-points', action='store_true', help='also bag raw lidar points (bags/run_<stamp>_points)')
    ap.add_argument('--no-npc', action='store_true', help='load the scenario without its NPC vehicles')
    ap.add_argument('--splits', type=float, nargs='*',
                    default=[275., 390., 405., 495., 860., 920., 1118.7, 1741.7, 1875., 2000.],
                    help='stations whose passing time is recorded (mission boundaries)')
    args = ap.parse_args()
    busy = ports_busy()
    if busy or stack_pids():
        sys.exit(f'stack already running / ports busy {busy}: stop it first')
    out_dir = HERE/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir.mkdir(parents=True)
    rclpy.init()
    watch = Watch()
    try:
        for i in range(args.runs):
            before = RECORDS.read_text() if RECORDS.is_file() else None
            rec = one_run(args, i, out_dir, watch)
            keep_records(rec, before, args.no_npc)
            with open(out_dir/'results.jsonl', 'a') as f:
                f.write(json.dumps(rec)+'\n')
            print(json.dumps(rec, ensure_ascii=False), flush=True)
            # New best records -> GitHub main README (user 2026-09-29); never fails the batch.
            try:
                done = subprocess.run([sys.executable, str(HERE/'records_readme.py')], capture_output=True, text=True, timeout=60)
                print(done.stdout.strip() or done.stderr.strip()[-200:], flush=True)
            except (OSError, subprocess.SubprocessError) as exc:
                print('records README:', exc, flush=True)
            # Every lane change with its box picture and outcome -> pass_records.jsonl (user 2026-09-30: learn from
            # each cut-in which box states worked); never fails the batch.
            try:
                done = subprocess.run([sys.executable, str(HERE/'pass_log.py'), str(out_dir), '--run', str(i)],
                                      capture_output=True, text=True, timeout=180)
                print((done.stdout.strip() or done.stderr.strip()[-200:]).splitlines()[-1:] or '', flush=True)
            except (OSError, subprocess.SubprocessError, IndexError) as exc:
                print('pass log:', exc, flush=True)
            watch.s, watch.telemetry, watch.pose, watch.mode = math.nan, {}, None, ''
    finally:
        watch.destroy_node()
        rclpy.shutdown()
    print('results:', out_dir/'results.jsonl')


if __name__ == '__main__':
    main()
