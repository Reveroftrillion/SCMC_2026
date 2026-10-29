#!/usr/bin/env python3
"""Report configured drive gates without ROS or a MORAI connection.

Ordered entries are potential gates along the route, not a claim that the
vehicle can reach each entry. Conditional hazards may occur anywhere.
"""
import math
from pathlib import Path
import sys

import yaml
from control_core import Driver, valid_zones
from traffic_core import TrafficDecision


def diagnose(root, args=None):
    args = args or {}
    def path(name, default):
        return Path(args.get(name, str(root / default)))
    def flag(name, default=False):
        value = args.get(name, default)
        return value if isinstance(value, bool) else str(value).lower() in ('true', '1', 'yes')
    with path('mission_config', 'config/missions.yaml').open(encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    with path('sensor_config', 'config/vehicle_sensor.yaml').open(encoding='utf-8') as f:
        sensor = yaml.safe_load(f)
    with path('path_file', 'paths/global_path.txt').open(encoding='utf-8-sig') as f:
        points = [tuple(map(float, line.split()[:2])) for line in f
                  if line.strip() and not line.lstrip().startswith('#')]
    driver = Driver(points)
    length = driver.s[-1]
    zones_ok = bool(cfg.get('mission_zones_verified', False)) and valid_zones(cfg.get('zones'), length)
    if cfg.get('unverified_sections_required', True) and not cfg.get('unverified_sections'):
        zones_ok = False  # Mirrors controller.py, including its empty-list policy.
    traffic = TrafficDecision(cfg.get('intersections', []), cfg.get('positions_verified', False))
    traffic_ok = traffic.verified and len(traffic.items) == 4
    lines = [
        'Path length %.3f m. Entries are ordered potential holds, not a successful drive prediction.' % length,
        'START / throughout: drive=%s; enable_control=%s; status_layout_confirmed=%s (transmit needs all three; drive supplies the enable_control default).' % (
            flag('drive'), flag('enable_control', flag('drive')), flag('status_layout_confirmed')),
        'START / throughout: sensor_reference_verified=%s; mission_zones_valid=%s; traffic_configuration_valid=%s.' % (
            bool(sensor.get('sensor_reference_verified', False)), zones_ok, traffic_ok),
        'START / throughout: valid 181-byte competition status from configured MORAI IP, increasing source timestamp and live Auto/D required; 229-byte Ego cannot arm.',
        'START / throughout: initial valid GPS near first 20 m of path, fresh IMU and speed, valid localization; fresh LiDAR, traffic, controller inputs and command required.',
        'ANYWHERE: invalid/stale input, invalid pose or speed limit, localization failure, DR timeout, excessive deviation, unsafe offset, command/status timeout or mode change can stop or inhibit transmission.',
        'TRAFFIC: detector/load/image/ROI failures or stale observations yield UNKNOWN; missed unconfirmed crossing holds stop after the active window.',
        'ANYWHERE: LiDAR stale/invalid timestamp or frame, missing ground support, processing timeout, obstacle or pedestrian hazard requests braking. A mode change may stop transmission instead of sending brake.',
        'Avoidance: allow_avoidance=%s; verified_road_half_width=%s; a collision-free observed path, valid sensor frame and clearance are also required; these require live/road-boundary evidence.' % (
            cfg.get('allow_avoidance', False), cfg.get('verified_road_half_width', sensor.get('verified_road_half_width', 0.0))),
    ]
    events = []
    for n, item in enumerate(cfg.get('intersections', []), 1):
        start, stop = float(item['start_s']), float(item['stop_s'])
        if item.get('verified', True) is not True:
            events.append((start - 5, 'traffic %d unverified: unconditional stop from %.3f m' % (n, start-5)))
        events.append((start, 'traffic %d (%s): RED/YELLOW/UNKNOWN limits speed; stop target rear s=%.3f m; two fresh allowed green frames permit restart' % (
            n, item['direction'], stop-traffic.front_offset-traffic.buffer)))
    for section in cfg.get('unverified_sections', []):
        start = float(section['start_s'])
        events.append((start-5, 'unverified section: unconditional stop target %.3f m; %s' % (start-5, section.get('reason'))))
    for name in ('static', 'pedestrian', 'roundabout', 'highway', 'tunnel'):
        begin, end = map(float, cfg['zones'][name])
        detail = {
            'static': 'obstruction without a validated clear bypass can stop',
            'pedestrian': 'candidate crossing latches stop until observed clear; missing observation is not clear',
            'roundabout': 'any nearby occupancy can force yield; empty observed region permits progress',
            'highway': 'lead vehicle safety distance can stop; lane changes need enabled and validated clear space',
            'tunnel': 'DR is limited to 90 s, 15 km/h; obstacle, poor observation or unavailable bypass can stop',
        }[name]
        events.append((begin, '%s zone until %.3f m: %s' % (name, end, detail)))
    events.append((length-1.5, 'FINISH: within final 1.5 m and within 2.5 m of endpoint latches brake; bridge requests P then manual only after stationary confirmation'))
    lines.append('--- Ordered path gates / conditional holds ---')
    lines.extend('s=%8.3f  %s' % event for event in sorted(events, key=lambda e: e[0]))
    lines.append('Evidence: static start equals traffic 2 start by user confirmation; remaining unverified sections are retained. Live sensor, lane boundaries, ROI and handover are not verified offline.')
    return lines


def main():
    args = dict(value.split(':=', 1) for value in sys.argv[1:] if ':=' in value)
    try:
        print('\n'.join(diagnose(Path(__file__).resolve().parents[1], args)))
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise SystemExit('Gate diagnostic failed: %s' % exc)


if __name__ == '__main__':
    main()
