"""Extra approach observations remain restricted to mapped road surfaces."""
from types import SimpleNamespace
import numpy as np
from foscar_lidar.obstacle_node import ObstacleNode


def node(s=886.):
    return SimpleNamespace(roi_y_abs=20.,roundabout_roi_y_abs=30.,
        roundabout_observation_begin=860.,roundabout_observation_end=920.,ego_s=s,
        road_margin=.15,road=SimpleNamespace(outside=lambda xy:abs(xy[:,1]-24.)-1.75),
        to_map=lambda pts,t:pts[:,:2])


def test_roundabout_sees_incoming_lane_beyond_old_roi_but_not_guardrail():
    pts=np.array([[10.,19.,1.],[10.,24.,1.],[10.,26.5,1.],[10.,31.,1.]])
    assert ObstacleNode.lateral_roi_mask(node(),pts,1.).tolist()==[True,True,False,False]


def test_other_missions_and_missing_map_pose_keep_original_roi():
    pts=np.array([[10.,19.,1.],[10.,24.,1.]])
    for n in (node(700.),node(None),node(930.)):
        assert ObstacleNode.lateral_roi_mask(n,pts,1.).tolist()==[True,False]
    for attribute,value in [('road',None),('to_map',lambda p,t:None)]:
        n=node();setattr(n,attribute,value)
        assert ObstacleNode.lateral_roi_mask(n,pts,1.).tolist()==[True,False]


def test_rear_quarter_returns_require_mapped_road_in_roundabout():
    n=node();n.roi_x_min=0.;n.roi_x_max=40.;n.roundabout_roi_x_min=-15.
    pts=np.array([[-5.,24.,1.],[-5.,27.,1.],[-16.,24.,1.],[2.,24.,1.]])
    assert ObstacleNode.longitudinal_roi_mask(n,pts,1.).tolist()==[True,False,False,True]
    n.ego_s=700.
    assert ObstacleNode.longitudinal_roi_mask(n,pts,1.).tolist()==[False,False,False,True]
