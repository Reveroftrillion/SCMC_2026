#!/usr/bin/env python3
"""LiDAR based conservative stop, speed and optional path offset requests."""
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Float64

from lidar_core import (ground_and_clusters, project, profile, choose_avoidance,
                        candidate_clear, update_pedestrian_latch, follow_speed,
                        retain_occupancy, trajectory_observed, choose_highway_change)


class LidarSafety:
    def __init__(self):
        self.lock = threading.Lock()
        self.path = np.loadtxt(rospy.get_param('~path_file'), usecols=(0, 1))
        if len(self.path) < 2:
            raise ValueError('path must contain at least two points')
        lengths = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.cum = np.r_[0.0, np.cumsum(lengths)]
        self.total = self.cum[-1]
        self.closed = np.linalg.norm(self.path[-1] - self.path[0]) < 1.0
        self.zones = rospy.get_param('~zones', {})
        self.frames = rospy.get_param('~lidar_frames', ['velodyne', 'lidar'])
        self.sensor = tuple(rospy.get_param('~lidar_xyz', [2.0, 0.0, 2.11]))
        self.allow_avoidance = bool(rospy.get_param('~allow_avoidance', False))
        self.offset = float(rospy.get_param('~avoidance_offset', 3.2))
        # A configured offset is not evidence that the adjacent space is road.
        self.road_half_width = float(rospy.get_param('~verified_road_half_width', 0.0))
        self.highway_return_s = float(rospy.get_param('~highway_return_s', 0.0))
        self.lookahead = float(rospy.get_param('~offset_lookahead', 5.0))
        self.timeout = float(rospy.get_param('~timeout', 0.8))
        self.pose = None
        self.pose_time = self.progress_time = self.cloud_time = 0.0
        self.progress = None
        self.plan = None
        self.plan_mode = None
        self.stop = True
        self.speed_limit = 0.0
        self.lateral = 0.0
        self.ped_lock = None
        self.tracks = []
        self.object_memory = []
        self.next_id = 1
        self.pub_stop = rospy.Publisher('/competition/lidar_stop', Bool, queue_size=1)
        self.pub_speed = rospy.Publisher('/competition/lidar_speed_limit', Float64, queue_size=1)
        self.pub_offset = rospy.Publisher('/competition/lateral_offset', Float64, queue_size=1)
        rospy.Subscriber('/competition/pose', PoseStamped, self.on_pose, queue_size=1)
        rospy.Subscriber('/competition/progress', Float64, self.on_progress, queue_size=1)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.on_cloud, queue_size=1, buff_size=2**24)
        rospy.Timer(rospy.Duration(0.1), self.publish)

    def in_zone(self, name, s):
        zone = self.zones.get(name)
        return bool(zone and len(zone) == 2 and float(zone[0]) <= s <= float(zone[1]))

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 0.5:
            return
        x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        with self.lock:
            self.pose = (p.x, p.y, yaw)
            self.pose_time = time.monotonic()

    def on_progress(self, msg):
        if not math.isfinite(msg.data) or msg.data < 0 or msg.data > self.total + 10:
            return
        with self.lock:
            self.progress = msg.data
            self.progress_time = time.monotonic()

    def associate(self, boxes, now):
        # World-coordinate matching removes the apparent movement due to ego motion.
        old = self.tracks
        used = set()
        new = []
        for box in boxes:
            best, distance = None, 2.0
            for i, track in enumerate(old):
                if i in used or now - track['time'] > 1.0:
                    continue
                d = math.hypot(box['wx'] - track['wx'], box['wy'] - track['wy'])
                if d < distance:
                    best, distance = i, d
            if best is None:
                box.update(id=self.next_id, vx=0.0, vy=0.0, observed=1, clear=0)
                self.next_id += 1
            else:
                used.add(best)
                prev = old[best]
                dt = max(0.05, now - prev['time'])
                box.update(id=prev['id'], vx=(box['wx']-prev['wx'])/dt,
                           vy=(box['wy']-prev['wy'])/dt,
                           observed=prev['observed']+1, clear=prev.get('clear', 0))
            box['time'] = now
            new.append(box)
        # Keep identities through brief occlusion; a missing pedestrian must
        # never count as an observed clear frame.
        new.extend(t for i, t in enumerate(old) if i not in used and now-t['time'] <= 1.0)
        self.tracks = new

    def remember_objects(self, boxes, now, pose):
        """Keep recently observed occupancy for safety checks across dropouts."""
        self.object_memory, occupancy = retain_occupancy(self.object_memory, boxes, now, pose)
        return occupancy

    def on_cloud(self, msg):
        now = time.monotonic()
        if msg.header.stamp.to_sec() > 0 and rospy.Time.now().to_sec() > 0:
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if age < -0.5 or age > self.timeout:
                rospy.logwarn_throttle(5, 'LiDAR message timestamp is stale; stopped')
                self.invalidate()
                return
        with self.lock:
            pose, progress = self.pose, self.progress
            if (pose is None or progress is None or now-self.pose_time > self.timeout or
                    now-self.progress_time > self.timeout):
                self.stop, self.cloud_time = True, 0.0
                return
        if msg.header.frame_id.lstrip('/') not in self.frames:
            rospy.logerr_throttle(5, 'LiDAR frame %s is not configured; stopped', msg.header.frame_id)
            self.invalidate()
            return
        try:
            raw = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
            observation = ground_and_clusters(raw, self.sensor, with_ground=True)
            if observation is None:
                rospy.logwarn_throttle(5, 'LiDAR ground support invalid or cloud empty; stopped')
                self.invalidate()
                return
            clusters, ground = observation
            if len(clusters) > 1000:
                rospy.logwarn_throttle(5, 'Too many LiDAR obstacles to process safely; stopped')
                self.invalidate()
                return
            x, y, yaw = pose
            c, sn = math.cos(yaw), math.sin(yaw)
            boxes, world = [], []
            for lo, hi, count in clusters:
                # Transform all corners; projected half extents account for object size.
                corners = np.array([[xx, yy] for xx in (lo[0], hi[0]) for yy in (lo[1], hi[1])])
                wx = x + c*corners[:, 0] - sn*corners[:, 1]
                wy = y + sn*corners[:, 0] + c*corners[:, 1]
                cx, cy = float(np.mean(wx)), float(np.mean(wy))
                world.append([cx, cy])
                boxes.append(dict(wx=cx, wy=cy, bx=(lo[0]+hi[0])/2,
                                  by=(lo[1]+hi[1])/2, height=hi[2]-lo[2],
                                  size=max(hi[0]-lo[0], hi[1]-lo[1]),
                                  radius=math.hypot(hi[0]-lo[0], hi[1]-lo[1])/2 + 0.15,
                                  count=count))
            projected = project(self.path, self.cum, world) if world else []
            for box, (s, d) in zip(boxes, projected):
                if self.closed:
                    s += round((progress - s)/self.total)*self.total
                box.update(s=s, d=d, hs=box['radius'], hd=box['radius'])
            if time.monotonic() - now > self.timeout:
                rospy.logwarn_throttle(5, 'LiDAR processing took too long; stopped')
                self.invalidate()
                return
            with self.lock:
                self.associate(boxes, now)
                occupancy = self.remember_objects(boxes, now, pose)
                self.decide(boxes, progress, occupancy, ground, pose)
                self.cloud_time = time.monotonic()
        except Exception as exc:
            self.invalidate()
            rospy.logerr_throttle(5, 'LiDAR processing failed; stopped: %s', str(exc))

    def invalidate(self):
        with self.lock:
            self.stop, self.cloud_time = True, 0.0

    def decide(self, boxes, s, occupancy=None, ground=None, pose=None):
        occupancy = boxes if occupancy is None else occupancy
        stop, speed = False, 16.6667
        pedestrian = self.in_zone('pedestrian', s)
        ped_zone = self.zones.get('pedestrian')
        if ped_zone and len(ped_zone) == 2 and float(ped_zone[0]) - 40.0 <= s <= float(ped_zone[1]):
            # Slow before entering the crossing area, leaving distance to brake.
            speed = min(speed, 20.0 / 3.6)
        roundabout = self.in_zone('roundabout', s)
        highway = self.in_zone('highway', s)
        avoid_zone = self.in_zone('static', s) or self.in_zone('tunnel', s)
        forward = [b for b in boxes if -2.0 < b['s'] - s < 35.0]

        pedestrian_hazards = []
        if pedestrian:
            # Limit approach speed before a person becomes visible; otherwise a
            # late detection can be physically too close for full braking.
            for b in forward:
                # Small returns can be partial views of a person; only height and
                # width cap rule out obvious large structures.
                predicted_d = b['d']
                if b['observed'] > 1:
                    predicted_d = project(self.path, self.cum,
                                          [[b['wx'] + 2*b['vx'], b['wy'] + 2*b['vy']]])[0][1]
                if (b['size'] < 2.5 and abs(b['d']) < 5.0 and
                        (abs(b['d']) < 2.5 or abs(predicted_d) < 2.5)):
                    pedestrian_hazards.append(b['id'])
        if self.ped_lock is not None:
            self.ped_lock = update_pedestrian_latch(self.ped_lock, boxes, s)
        if self.ped_lock is None and pedestrian_hazards:
            self.ped_lock = pedestrian_hazards[0]
        if self.ped_lock is not None:
            stop = True

        if roundabout:
            speed = min(speed, 2.5)
            # Without validated yield geometry, any nearby return is conflicting.
            if any(-3 < b['bx'] < 22 and abs(b['by']) < 12 for b in occupancy):
                stop = True

        corridor = [b for b in occupancy if -2 < b['s'] - s < 35 and abs(b['d']) < 1.2 + b['hd']]
        closest = min((b['s']-s-b['hs'] for b in corridor), default=100.0)
        local_corridor = [b for b in occupancy if b['bx'] > 0 and
                          abs(b['by']) < 1.2 + b['radius']]
        local_front = min((b['bx'] - b['radius'] for b in local_corridor), default=100.0)
        # At parallel lanes and junctions, nearest-path projection can be
        # ambiguous. Use the local view as an additional conservative guard.
        if not self.plan:
            closest = min(closest, local_front)
        elif highway:
            # Once a validated lane-change plan is active, a vehicle left
            # behind in the original lane is not necessarily in our path.
            # The actual body-frame corridor still triggers emergency braking.
            closest = local_front
        if highway:
            # Before two observations, assume the lead object is stationary.
            lead = (min(local_corridor, key=lambda b: b['bx']-b['radius'], default=None)
                    if self.plan else min(corridor, key=lambda b: b['s']-s-b['hs'], default=None))
            lead_speed = 0.0
            if lead and lead['observed'] > 1 and not lead.get('ghost'):
                i = min(len(self.path)-2, max(0, int(np.searchsorted(self.cum, lead['s'] % self.total) - 1)))
                tangent = self.path[i+1] - self.path[i]
                norm = np.linalg.norm(tangent)
                if norm > 1e-6:
                    lead_speed = (lead['vx']*tangent[0] + lead['vy']*tangent[1])/norm
            speed = min(speed, follow_speed(closest, lead_speed))
            if closest < 8.0:
                stop = True
        elif closest < 30:
            speed = min(speed, max(0.0, (closest - 6.0) / 2.0))

        # Avoidance requires validated drivable lateral width supplied by operator.
        # Retain a plan on conflicts: stop on its current trajectory until clear.
        if self.plan and s > self.plan[3] + self.plan[4]:
            self.plan = None
            self.plan_mode = None
        if self.plan and not candidate_clear(occupancy, self.plan[0], self.plan[1], self.plan[2], self.plan[3], self.plan[4], from_s=s):
            stop = True
        if self.plan and self.plan_mode in ('static', 'tunnel', 'highway'):
            # Keep the existing trajectory but brake if the remaining swept
            # area is not currently observed. Missing points are not free space.
            if ground is None or pose is None or not trajectory_observed(ground, self.path, self.cum, pose, s, self.plan):
                stop = True
        if self.plan is None and avoid_zone and self.allow_avoidance and corridor and self.road_half_width > 0:
            tunnel = self.in_zone('tunnel', s)
            for offset in (self.offset, -self.offset):
                candidate = choose_avoidance(occupancy, s, self.offset, road_half_width=self.road_half_width,
                                             side_offsets=(offset,))
                if (candidate and ground is not None and pose is not None and
                        trajectory_observed(ground, self.path, self.cum, pose, s, candidate)):
                    self.plan, self.plan_mode = candidate, ('tunnel' if tunnel else 'static')
                    break
        if (self.plan is None and highway and self.allow_avoidance and corridor and
                self.road_half_width > 0 and self.highway_return_s > s and closest < 30.0):
            # No lane change unless the whole manoeuvre, including a 15 m rear
            # target-lane observation, is confirmed. Return before CP12.
            for offset in (self.offset, -self.offset):
                candidate = choose_highway_change(occupancy, s, self.highway_return_s - 15.0,
                                                  side_offset=offset, road_half_width=self.road_half_width)
                if (candidate and ground is not None and pose is not None and
                        trajectory_observed(ground, self.path, self.cum, pose, s, candidate, rear_check=15.0)):
                    self.plan, self.plan_mode = candidate, 'highway'
                    break
        if self.plan:
            # Lookahead offset matches the controller's lookahead target.
            self.lateral = profile(s + self.lookahead, *self.plan)
            speed = min(speed if not corridor else 3.0, 3.0)
        else:
            self.lateral = 0.0
            if closest < 8.0:
                stop = True
        # Local-frame emergency protection also covers ambiguous path projection.
        if any(0 < b['bx'] < 5 and abs(b['by']) < 1.2 + b['radius'] for b in occupancy):
            stop = True
        self.stop, self.speed_limit = stop, (0.0 if stop else max(0.0, speed))
        rospy.loginfo_throttle(2., 'LiDAR s=%.1f objects=%d static=%s pedestrian=%s roundabout=%s highway=%s tunnel=%s plan=%s offset=%.2f stop=%s limit=%.1fm/s',
                               s, len(boxes), self.in_zone('static', s), pedestrian, roundabout,
                               highway, self.in_zone('tunnel', s), bool(self.plan), self.lateral, stop, self.speed_limit)

    def publish(self, _event):
        now = time.monotonic()
        with self.lock:
            stale = (now-self.cloud_time > self.timeout or now-self.pose_time > self.timeout or
                     now-self.progress_time > self.timeout)
            stop = self.stop or stale
            speed = 0.0 if stop else self.speed_limit
            # Do not switch a retained path back to the centre during a dropout;
            # stop on that path until fresh sensing can re-evaluate it.
            offset = self.lateral
        self.pub_stop.publish(Bool(data=stop))
        self.pub_speed.publish(Float64(data=speed))
        self.pub_offset.publish(Float64(data=offset))


if __name__ == '__main__':
    rospy.init_node('lidar_missions')
    LidarSafety()
    rospy.spin()
