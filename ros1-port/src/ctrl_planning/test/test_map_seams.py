from pathlib import Path

import numpy as np
import pytest

from ctrl_planning.ctrl_lane_prediction import LaneMap


def lanes(shared=True, adjacent=True, reverse=False, shortened=False):
    a = dict(idx='a', from_node_idx='a0', to_node_idx='a1',
             points=[[0., 0.], [10., 0.]], width_start=3.5, width_end=3.5,
             lane_mark_left=['boundary'], can_move_left_lane=False)
    b = dict(idx='b', from_node_idx='b0', to_node_idx='b1',
             points=[[2. if shortened else 0., 3.85], [8. if shortened else 10., 3.85]],
             width_start=3.5, width_end=3.5, lane_mark_right=['boundary' if shared else 'median'])
    if adjacent:
        a['left_lane_change_dst_link_idx'] = 'b'
        b['right_lane_change_dst_link_idx'] = 'a'
    if reverse:
        b['points'].reverse()
    return [a, b]


def test_shared_internal_lane_boundary_fills_survey_gap_without_expanding_curbs():
    road = LaneMap(lanes())
    assert road.contains_road([[5., 1.925], [5., -1.91], [5., 5.76]]).tolist() == [True, False, False]


@pytest.mark.parametrize('kwargs', [dict(shared=False), dict(adjacent=False), dict(reverse=True)])
def test_close_parallel_links_require_shared_boundary_adjacency_and_direction(kwargs):
    assert not LaneMap(lanes(**kwargs)).contains_road([[5., 1.925]])[0]


def test_seam_strip_stops_at_overlapping_link_ends():
    road = LaneMap(lanes(shortened=True))
    assert road.contains_road([[1., 1.925], [2.1, 1.925], [7.9, 1.925], [9., 1.925]]).tolist() == [False, True, True, False]


def test_surveyed_shared_boundary_points_are_pavement():
    map_file = Path(__file__).resolve().parents[2] / 'ctrl_bringup/maps/link_set.json'
    road = LaneMap.from_file(map_file)
    assert np.all(road.contains_road([[-65.47780463, -146.55084947], [-65.4673665, -147.45413507]]))


def test_intersection_internal_holes_do_not_expand_outer_road():
    road=LaneMap.from_file(Path(__file__).parents[2]/'ctrl_bringup/maps/link_set.json')
    pts=[[-57.67,-144.93],[-56.16,-144.43],[-55.1,-136.5],[-57.7,-154.1]]
    assert road.contains_road(pts).tolist()==[False]*4
    assert road.contains_intersection_road(pts).tolist()==[True,True,True,False]


def test_unmarked_enclosed_area_is_not_assumed_intersection_pavement():
    road=LaneMap([dict(idx=str(i),from_node_idx=str(i),to_node_idx=str(i+1),
                      points=[a,b],width_start=1.)
                  for i,(a,b) in enumerate([([0,0],[10,0]),([10,0],[10,10]),
                                             ([10,10],[0,10]),([0,10],[0,0])])])
    assert not road.contains_intersection_road([[5,5]])[0]


def test_intersection_entry_can_delay_right_shift_until_road_opens():
    import time
    from ctrl_control.ctrl_core import Route
    from ctrl_planning.ctrl_state import Context
    from ctrl_planning.states.ctrl_avoid import Avoid, Remembered
    from ctrl_planning.ctrl_avoid_planner import polygon_box
    from ctrl_planning.ctrl_mapped_avoidance import paved_moves
    bringup=Path(__file__).parents[2]/'ctrl_bringup'
    route=Route(np.loadtxt(bringup/'routes/global_path.txt')[:,:2])
    road=LaneMap.from_file(bringup/'maps/link_set.json')
    x,y=route.at(309.7)
    c=Context(s=309.7,x=x,y=y,yaw=route.heading_at(309.7),v=0.,now=1.,
              route=route,road_map=road,mission_name='STATIC_OBSTACLE')
    a=Avoid(margin_m=.5)
    a.memory=[Remembered(polygon_box([(319.18,1.23),(327.59,-1.45),(328.55,1.55),(320.13,4.23)]),1.)]
    started=time.perf_counter();t=a.obstacle_target(c)
    print('intersection candidate seconds',time.perf_counter()-started)
    assert a.slalom.blocked_at is None and not a.slalom.return_blocked
    assert t.lat_offset<0. and t.entry_s>c.s
    assert paved_moves(c,a.slalom_origin,t.chain,c.s,max(st+ln for _,st,ln in t.chain),
                       a.front_m,a.rear_overhang_m,a.static_planner.half)
