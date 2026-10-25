import numpy as np
from ctrl_lidar.ctrl_highway_visibility import HighwaySensor

def test_active_only_inside_highway_span():
    s=HighwaySensor(100.,200.,60.,40.,20.)
    assert s.active(150.) and not s.active(99.) and not s.active(200.) and not s.active(None)

def test_highway_roi_includes_rear_only_on_road():
    from types import SimpleNamespace
    from ctrl_lidar.ctrl_obstacle_node import ObstacleNode
    n=SimpleNamespace(roi_x_min=0.,roi_x_max=40.,ego_s=150.,road=SimpleNamespace(outside=lambda p:abs(p[:,1])-2.),road_margin=0.,to_map=lambda p,t:p[:,:2],
        highway_sensor=HighwaySensor(100.,200.,60.,40.,20.))
    pts=np.array([[-20.,0.,1.],[-20.,10.,1.],[50.,0.,1.],[70.,0.,1.]])
    assert ObstacleNode.longitudinal_roi_mask(n,pts,1.).tolist()==[True,False,True,False]


def test_competition_config_observes_the_highway_before_cp10():
    # 2026-09-28: lane-4 speeds must be measured on arrival; only the lidar ROI moves earlier.
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]
    s=HighwaySensor.from_files(str(root/'ctrl_planning/config/highway.yaml'),str(root/'ctrl_planning/config/missions.yaml'))
    assert s.active(1118.7-50.) and not s.active(1118.7-70.)
