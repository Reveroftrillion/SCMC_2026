from types import SimpleNamespace
import numpy as np
from foscar_planning.highway_config import HighwayConfig
from foscar_planning.highway_policy import HighwayPolicy
from foscar_planning.highway_observation import ObservationSnapshot
from foscar_planning.state import Context
from test_highway_route import highway
from pathlib import Path


def test_display_gap_uses_target_lane_front_and_rear_not_next_lane(highway):
    from foscar_planning.highway_scene import highway_scene
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml');p=HighwayPolicy(highway,cfg);p.current_lane=5
    x,y=highway.at(5,1150);ctx=Context(s=1150,x=x,y=y,yaw=-np.pi/2,v=12,now=1.)
    objects={}
    for key,lane,s in [('front',4,1170),('rear',4,1130),('other',3,1150)]:
        xy=highway.at(lane,s)+np.array([[-.8,-2],[.8,2]])
        objects[key]=SimpleNamespace(id=key,xy=xy,speed_ready=True,velocity_bounds=(0,0,-12,-12))
    snapshot=ObservationSnapshot(1.,True,'OK',objects)
    scene=highway_scene(p,ctx,snapshot)
    assert scene['gap']['front_id']=='front' and scene['gap']['rear_id']=='rear'
    assert scene['gap']['length_m']>30
    assert scene['gap']['status']=='UNASSESSED'
    assert scene['objects']['other']['lane']==3
    assert len(scene['lanes'])==2


def test_missing_scan_cannot_show_a_free_gap(highway):
    from foscar_planning.highway_scene import highway_scene
    cfg=HighwayConfig.load(Path(__file__).parents[1]/'config/highway.yaml');p=HighwayPolicy(highway,cfg);p.current_lane=5
    x,y=highway.at(5,1150);ctx=Context(s=1150,x=x,y=y,yaw=-np.pi/2,v=12,now=1.)
    scene=highway_scene(p,ctx,ObservationSnapshot(0.,False,'SCAN_UNKNOWN',{}))
    assert scene['gap'] is None
