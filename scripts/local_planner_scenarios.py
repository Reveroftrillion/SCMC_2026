"""Deterministic offline fixtures; expected states are harness decisions, not ROS output.

Calls the production core, obstacle memory and return timer. No vehicle dynamics,
TF, controller or ROS adapter is simulated. Fixture dimensions are synthetic only.
"""
from collections import Counter
import copy
from dataclasses import dataclass, field
import math
import numpy as np
from local_planning_core import FrenetLocalPlanner
from local_planning_state import ObstacleMemory, return_progress


@dataclass
class Frame:
    time: float = 10.
    x: float = 0.
    y: object = 0.
    yaw: float = 0.
    obstacles: tuple = ()
    expected_state: str = 'NORMAL'
    expected_stop: bool = False
    side: str = ''


@dataclass
class Scenario:
    name: str
    points: np.ndarray
    frames: list
    updates: dict = field(default_factory=dict)


def scenarios():
    straight = np.column_stack((np.arange(0., 200.5, .5), np.zeros(401)))
    def curve(radius):
        s = np.arange(0., 80.5, .5)
        return np.column_stack((radius*np.sin(s/radius), radius*(1-np.cos(s/radius))))
    duplicate = np.insert(straight, [4, 8, 20], straight[[4, 8, 20]], axis=0)
    centre = ((12., 0., .7),)
    return [
        Scenario('straight_no_obstacle', straight, [Frame()]),
        Scenario('center_obstacle', straight, [Frame(obstacles=centre, expected_state='STATIC_OBSTACLE', side='nonzero')]),
        Scenario('left_obstacle', straight, [Frame(obstacles=((12.,1.,.7),), expected_state='STATIC_OBSTACLE', side='right_or_center')]),
        Scenario('right_obstacle', straight, [Frame(obstacles=((12.,-1.,.7),), expected_state='STATIC_OBSTACLE', side='left_or_center')]),
        Scenario('multiple_obstacles', straight, [Frame(obstacles=((12.,0.,.7),(18.,1.,.5)), expected_state='STATIC_OBSTACLE')]),
        Scenario('all_blocked', straight, [Frame(obstacles=((12.,0.,6.),), expected_state='STATIC_OBSTACLE', expected_stop=True)]),
        Scenario('narrow_corridor', straight, [Frame(obstacles=centre, expected_state='STATIC_OBSTACLE', expected_stop=True)],
                 {'road_half_width': 1.4}),
        Scenario('curved_road', curve(100.), [Frame()]),
        Scenario('duplicate_waypoint', duplicate, [Frame()]),
        Scenario('sharp_curvature', curve(1.5), [Frame(expected_state='HOLD', expected_stop=True)]),
        Scenario('obstacle_disappears', straight, [
            Frame(obstacles=centre, expected_state='STATIC_OBSTACLE'),
            Frame(time=10.2, expected_state='STATIC_OBSTACLE'),  # memory still occupied
            Frame(time=12.2, expected_state='RETURN_TO_GLOBAL'),
            Frame(time=13.1, expected_state='NORMAL')]),
        Scenario('return_to_global', straight, [
            Frame(obstacles=centre, expected_state='STATIC_OBSTACLE'),
            Frame(time=13., x=19., y='previous_offset', expected_state='RETURN_TO_GLOBAL'),
            Frame(time=13.1, x=20., expected_state='RETURN_TO_GLOBAL'),
            Frame(time=14., x=20., expected_state='NORMAL')]),
    ]


RESULT_FIELDS = ['scenario','frame','time_s','result','state','expected_state','stop','expected_stop',
                 'valid_candidates','selected_offset','selected_cost','min_clearance_m',
                 'selected_peak_curvature','selected_peak_steering_rate_per_m','minimum_candidate_clearance_m',
                 'memory_obstacles','reference_removed_points','rejections','reason']


def evaluate_scenario(scenario, config, check_expectations=True, apply_updates=True):
    p = copy.deepcopy(config)
    if apply_updates:
        p.update(scenario.updates)
    try:
        core = FrenetLocalPlanner(p)
        memory = ObstacleMemory(p['obstacle_memory_s'], p['obstacle_merge_distance'])
    except (ValueError, TypeError, KeyError) as exc:
        return [dict(scenario=scenario.name, frame=0, result='CONFIG_ERROR', state='UNAVAILABLE',
                     stop=True, valid_candidates=0, reason=str(exc))]
    rows = []
    active, clear_since = False, None
    for number, frame in enumerate(scenario.frames):
        row = dict(scenario=scenario.name, frame=number, time_s=frame.time,
                   expected_state=frame.expected_state, expected_stop=frame.expected_stop,
                   valid_candidates=0, selected_offset='', selected_cost='', min_clearance_m='',
                   selected_peak_curvature='', selected_peak_steering_rate_per_m='',
                   minimum_candidate_clearance_m='', rejections='', reason='')
        selected, candidates = None, []
        try:
            y = core.previous.offset if frame.y == 'previous_offset' and core.previous is not None else frame.y
            pose = (frame.x, float(y), frame.yaw)
            index = int(np.argmin(np.linalg.norm(scenario.points-[pose[0],pose[1]], axis=1)))
            reference = core.reference(scenario.points, index)
            obstacles = memory.observe(frame.obstacles, frame.time, frame.time)
            row['memory_obstacles'] = len(obstacles)
            selected, candidates, (s0, d, heading) = core.plan(reference, pose, obstacles)
            row['valid_candidates'] = sum(not c.rejection for c in candidates)
            row['rejections'] = '; '.join('%s=%d' % item for item in sorted(Counter(c.rejection for c in candidates if c.rejection).items()))
            if selected is None:
                state, stop = 'STATIC_OBSTACLE', True
                active, clear_since = True, None
            else:
                # Mirror the adapter's centre preference only for reporting state.
                if active:
                    centre = next((c for c in candidates if c.offset == 0 and not c.rejection), None)
                    if centre is not None:
                        selected = centre
                core.previous = selected
                returning = selected.offset == 0 and (active or abs(d) > p['return_d_tolerance'] or
                                                      abs(heading) > p['return_heading_tolerance'])
                stop = False
                if returning:
                    complete, clear_since = return_progress(d, heading, frame.time, clear_since, p)
                    state, active = ('NORMAL', False) if complete else ('RETURN_TO_GLOBAL', True)
                    if complete:
                        core.reset()
                        clear_since = None
                elif selected.offset == 0 and not active:
                    state = 'NORMAL'
                    clear_since = None
                else:
                    state, active, clear_since = 'STATIC_OBSTACLE', True, None
                row.update(selected_offset=selected.offset, selected_cost=selected.cost,
                           min_clearance_m=selected.clearance,
                           selected_peak_curvature=float(np.max(np.abs(selected.curvature))))
                # Use the same reference-distance coordinate as production candidate rejection.
                length = min(p['horizon'], (len(reference.rx)-1)*reference.interval-s0)
                distance = np.linspace(0., length, len(selected.xy))
                rate = np.gradient(np.arctan(p['wheelbase']*selected.curvature), distance)
                row['selected_peak_steering_rate_per_m'] = float(np.max(np.abs(rate)))
            if candidates:
                row['minimum_candidate_clearance_m'] = min(c.clearance for c in candidates)
        except (ValueError, TypeError, KeyError, IndexError, np.linalg.LinAlgError) as exc:
            state, stop, clear_since = 'HOLD', True, None
            row['reason'] = str(exc)
        row.update(state=state, stop=stop, reference_removed_points=core.reference_diagnostic.get('removed_points', ''))
        ok = state == frame.expected_state and stop == frame.expected_stop
        if selected is not None and not stop:
            ok = ok and selected.clearance > 0 and math.isfinite(selected.cost)
            if frame.side == 'nonzero': ok = ok and selected.offset != 0
            elif frame.side == 'right_or_center': ok = ok and selected.offset <= 0
            elif frame.side == 'left_or_center': ok = ok and selected.offset >= 0
        row['result'] = ('PASS' if ok else 'FAIL') if check_expectations else 'OBSERVED'
        if check_expectations and not ok:
            row['reason'] = (row['reason'] + '; ' if row['reason'] else '') + 'expected %s stop=%s side=%s' % (
                frame.expected_state, frame.expected_stop, frame.side)
        rows.append(row)
    return rows


def display(rows):
    print('scenario/frame | result | state | stop | valid | offset | cost | clearance | reason')
    for r in rows:
        print('%s/%s | %s | %s | %s | %s | %s | %s | %s | %s' % (
            r['scenario'], r['frame'], r['result'], r['state'], r['stop'], r['valid_candidates'],
            r.get('selected_offset',''), r.get('selected_cost',''), r.get('min_clearance_m',''), r.get('reason','')))
