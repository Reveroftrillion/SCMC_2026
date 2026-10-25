import numpy as np
from ctrl_lidar.ctrl_fragment_merge import merge_vehicle_fragments


def box(x0,x1,y0,y1):
    return dict(x=(x0+x1)/2,y=(y0+y1)/2,size_x=x1-x0,size_y=y1-y0,size_z=1.,z_max=1.4,
                n=20,footprint=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]])


def test_adjacent_vehicle_parts_become_one_measured_hull():
    result=merge_vehicle_fragments([box(0,1.5,0,1.8),box(2,4.5,0,1.8)])
    assert len(result)==1 and result[0]['n']==40
    assert result[0]['size_x']==4.5 and result[0]['size_y']==1.8


def test_separated_vehicles_and_long_rail_chain_are_not_collapsed():
    assert len(merge_vehicle_fragments([box(0,4.5,0,1.8),box(6,10.5,0,1.8)]))==2
    rails=[box(i,i+.6,0,.1) for i in range(7)]
    assert len(merge_vehicle_fragments(rails))==7


def test_marked_rail_fragments_keep_structure_evidence():
    rails=[box(0,1.5,0,.7),box(2,3.5,0,.7),box(4,5.5,0,.7)]
    for rail in rails:rail['structure']=True
    result=merge_vehicle_fragments(rails)
    assert len(result)==3
    assert all(d['structure'] for d in result)
