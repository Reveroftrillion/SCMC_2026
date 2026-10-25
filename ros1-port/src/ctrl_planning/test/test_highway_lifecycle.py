"""Ideal path/visibility policy exercise, not a vehicle dynamics validation."""
from pathlib import Path
from dataclasses import replace
import math
import numpy as np
import pytest
from ctrl_control.ctrl_mission_config import load_mission_config
from ctrl_planning.ctrl_mission import MissionSupervisor
from ctrl_planning.ctrl_highway_runtime import HighwayRuntime
from ctrl_planning.ctrl_state import Context, Target
from test_highway_route import highway
from highway_fixtures import calibrated_config


@pytest.mark.parametrize('speed_kmh,npc_bound_kmh,use_control_profile', [
    (54.,90.,False),(40.,60.,False),(50.,60.,False),(60.,60.,False),
    (40.,60.,True),(50.,60.,True)])
def test_three_changes_cp12_rejoin_and_cp13_mission_handoff(highway,tmp_path,speed_kmh,npc_bound_kmh,use_control_profile):
    # Synthetic bounds exercise planning only; 60 is the user's current scenario,
    # not a production assumption about competition NPC speeds.
    cfg=calibrated_config(tmp_path)
    cfg=replace(cfg,bounds=replace(cfg.bounds,npc_speed_bound_mps=npc_bound_kmh/3.6))
    if use_control_profile:
        from ctrl_planning.ctrl_highway_config import ControlProfile
        control=Path(__file__).resolve().parents[2]/'ctrl_bringup/config/low_speed.yaml'
        cfg=replace(cfg,control_profile=ControlProfile.load(control))
    runtime=HighwayRuntime(cfg,highway)
    mission=MissionSupervisor(load_mission_config(Path(__file__).parents[1]/'config/missions.yaml'))
    ref=None;changes=[];released=[];states=[]
    speed=speed_kmh/3.6;dt=.1
    for i,s in enumerate(np.arange(highway.start_s+1.,highway.end_s+3.,speed*dt)):
        t=1.+i*dt
        if ref is not None:
            q=ref.q_at_global(s);x,y=ref.at_q(q);yaw=ref.heading_at(q)
        elif s<highway.rejoin_s:
            x,y=highway.at(5,s);yaw=-math.pi/2
        else:
            x,y=highway.original.at(s);yaw=highway.original.heading_at(s)
        ctx=Context(s=s,x=x,y=y,yaw=yaw,v=speed,now=t)
        mission.update(ctx);mission.apply_context(ctx)
        runtime.accept(dict(stamp=t,frame='map',objects=[],scan_valid=True),t)
        target=runtime.apply(Target(v_max=ctx.mission_speed_mps),ctx)
        diag=runtime.diagnostic
        states.append((s,diag.get('state'),diag.get('reason')))
        if diag.get('reason')=='CHANGE_STARTED':
            m=runtime.policy.committed
            changes.append((m.source_lane,m.target_lane,s,runtime.policy.cp11))
        if target.reference is not None:ref=target.reference
        if target.reference_release:ref=None;released.append(s)
        if s>=highway.end_s:
            assert ctx.mission_name=='CRUISE' and not runtime.active
            assert target.v_max==60/3.6
    assert [(a,b) for a,b,_,_ in changes]==[(5,4),(4,3),(3,2)],(changes,states[-10:])
    assert all(passed for a,b,s,passed in changes if a!=5)
    assert released and runtime.policy.rejoin.end_s<=released[0]<runtime.policy.rejoin.end_s+speed*dt*2


def test_empty_road_progress_obeys_requested_speed_caps(highway):
    """Longitudinal feedback test; lateral pose is ideal, not simulator dynamics."""
    from ctrl_planning.ctrl_highway_config import HighwayConfig
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml')
    runtime=HighwayRuntime(replace(cfg,mode='control'),highway)
    s=highway.start_s+1;v=12.;ref=None;changes=[];dt=.1
    for i in range(2400):
        t=1+i*dt
        if ref is None:x,y=highway.at(5,s);yaw=-math.pi/2
        else:q=ref.q_at_global(s);x,y=ref.at_q(q);yaw=ref.heading_at(q)
        ctx=Context(s=s,x=x,y=y,yaw=yaw,v=v,now=t,mission_name='HIGHWAY')
        runtime.accept(dict(stamp=t,frame='map',scan_valid=True,objects=[]),t)
        target=runtime.apply(Target(),ctx)
        if runtime.diagnostic.get('reason')=='CHANGE_STARTED':
            changes.append((runtime.policy.committed.source_lane,runtime.policy.committed.target_lane))
        if target.reference is not None:ref=target.reference
        if target.reference_release:break
        desired=min(15.,target.v_max)
        v=max(0.,v+np.clip(desired-v,-1.5*dt,1.*dt))
        s+=v*dt
    assert target.reference_release,(s,v,runtime.diagnostic)
    assert changes==[(5,4),(4,3),(3,2)]
