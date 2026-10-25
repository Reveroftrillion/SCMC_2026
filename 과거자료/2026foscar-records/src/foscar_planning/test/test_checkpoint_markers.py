from pathlib import Path
import numpy as np
import yaml
from foscar_control.mission_config import load_mission_config
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from foscar_control.core import Route
from foscar_planning.mission import MissionSupervisor
from foscar_planning.mission_markers import checkpoint_markers


def test_checkpoints_use_fsm_stations_and_merge_shared_boundary():
    cfg=load_mission_config(Path(__file__).parents[1]/'config/missions.yaml')
    zones=MissionSupervisor(cfg).zones
    route=Route(np.c_[np.arange(2301.),np.zeros(2301)])
    markers=checkpoint_markers(route,zones,100.,Time()).markers
    lines=[m for m in markers if m.type==Marker.LINE_STRIP]
    labels=[m for m in markers if m.type==Marker.TEXT_VIEW_FACING]
    stations=sorted({s for z in zones for s in (z.start_s,z.end_s)})
    assert len(lines)==len(labels)==len(stations)==14   # + SIGNAL_1 and SIGNAL_4 (2026-09-29)
    assert [m.points[0].x for m in lines]==[s+100. for s in stations]
    shared=[m for m in labels if 'TUNNEL IN' in m.text]
    assert len(shared)==1 and 'TUNNEL IN' in shared[0].text and 'HIGHWAY' not in shared[0].text
    exit_labels=[m for m in labels if 'HIGHWAY OUT' in m.text]
    assert len(exit_labels)==1 and exit_labels[0].text=='FSM HIGHWAY OUT'
    assert all(m.text.startswith('FSM ') and 'CP' not in m.text for m in labels)
    assert all(m.header.frame_id=='map' for m in markers)


def test_unavailable_or_short_route_does_not_clamp_checkpoints_to_endpoint():
    from types import SimpleNamespace
    zones=[SimpleNamespace(name='TUNNEL',start_s=100.,end_s=200.)]
    route=Route(np.c_[np.arange(151.),np.zeros(151)])
    assert not checkpoint_markers(route,zones,None,Time()).markers
    markers=checkpoint_markers(route,zones,0.,Time()).markers
    assert len(markers)==2 and 'IN' in markers[-1].text


def test_official_checkpoint_is_shown_at_actual_position_with_acceptance_radius():
    from foscar_control.mission_config import read_checkpoints
    from foscar_planning.mission_markers import required_checkpoint_markers
    checkpoints=read_checkpoints(Path(__file__).parents[2]/'foscar_bringup/routes/checkpoints.txt')
    markers=required_checkpoint_markers(checkpoints,Time()).markers
    assert len(markers)==2*len(checkpoints)
    ring=next(m for m in markers if m.id==12 and m.type==Marker.LINE_STRIP)
    label=next(m for m in markers if m.id==12 and m.type==Marker.TEXT_VIEW_FACING)
    cp=checkpoints[12]
    assert label.text=='CP12' and 11 not in checkpoints
    assert np.allclose([label.pose.position.x,label.pose.position.y],cp['xy'])
    assert np.allclose([np.linalg.norm(np.array([p.x,p.y])-cp['xy']) for p in ring.points],3.)
    assert all(m.header.frame_id=='map' for m in markers)
