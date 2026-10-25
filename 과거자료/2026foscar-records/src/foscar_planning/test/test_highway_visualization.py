from types import SimpleNamespace
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
from foscar_planning.highway_markers import highway_markers

def test_markers_clear_missing_objects_in_next_scan():
    run=SimpleNamespace(active=True,diagnostic={'objects':[{'id':'a','xy':[[0,0],[1,0],[1,1]]}]})
    first=highway_markers(run,Time())
    assert len(first.markers)==2
    run.diagnostic={'objects':[]}
    second=highway_markers(run,Time())
    assert len(second.markers)==1 and second.markers[0].action==Marker.DELETEALL


def test_highway_hud_distinguishes_shadow_proposal_from_control():
    run=SimpleNamespace(active=True,diagnostic=dict(state='WAIT_GAP',
        reason='CALIBRATION_REQUIRED',mode='shadow',speed_cap_mps=0.,
        scan_age_s=.12,observed_count=3,global_s=1127.7,cp11_passed=False))
    markers=highway_markers(run,Time()).markers
    text=[m for m in markers if m.type==Marker.TEXT_VIEW_FACING]
    assert len(text)==1
    assert '[SHADOW]' in text[0].text
    assert 'WAIT_GAP' in text[0].text and 'CALIBRATION_REQUIRED' in text[0].text
    assert len(text[0].text.splitlines())==2
    assert 'objects=' not in text[0].text and 'scan age=' not in text[0].text
    assert text[0].scale.z==1.8
    assert text[0].header.frame_id=='base_link'
    run.active=False
    cleared=highway_markers(run,Time()).markers
    assert len(cleared)==1 and cleared[0].action==Marker.DELETEALL


def test_blocker_and_change_path_are_visible_without_long_object_ids():
    run=SimpleNamespace(active=True,diagnostic=dict(state='WAIT_GAP',mode='control',
        reason='PREDICTED_CONFLICT:long-id',blocked_by=['PREDICTED_CONFLICT:long-id'],
        ego_xy=[0.,0.],current_lane=5,target_lane=4,
        change_path=[[0.,0.],[10.,3.5],[20.,3.5]],
        objects=[dict(id='long-id',xy=[[-8.,2.5],[-4.,2.5],[-4.,4.5]],velocity=[12.,0.])]))
    markers=highway_markers(run,Time()).markers
    assert any(m.ns=='highway_change_path' and len(m.points)==3 for m in markers)
    assert any(m.ns=='highway_blocker' and m.color.r==1. and m.color.g<.3 for m in markers)
    assert any(m.type==Marker.ARROW for m in markers)
    assert not any('long-id' in m.text for m in markers)


def test_controller_fault_is_not_hidden_behind_rejoin_complete():
    run=SimpleNamespace(active=True,diagnostic=dict(state='REJOIN',mode='control',reason='REJOIN_COMPLETE',
        control=dict(state='FAULT',reason='overspeed for this route section')))
    labels=[m for m in highway_markers(run,Time()).markers if m.type==Marker.TEXT_VIEW_FACING]
    assert 'FAULT' in labels[0].text and 'overspeed' in labels[0].text


def test_scene_shows_gap_lane_edges_and_nearest_vehicle_roles():
    run=SimpleNamespace(active=True,diagnostic=dict(state='WAIT_GAP',reason='CLEAR_CONFIRM',mode='control',
        objects=[dict(id='rear',xy=[[0,0],[2,0],[2,1]],velocity=[5,0])],
        scene=dict(lanes=[dict(target=True,edges=[[[0,0],[20,0]],[[0,3],[20,3]]])],
            objects={'rear':dict(lane=4,role='REAR',speed_kmh=18.)},
            gap=dict(polygon=[[3,0],[15,0],[15,3],[3,3]],length_m=12.,bounded=True,
                     status='CLEAR_CONFIRM',front_id=None,rear_id='rear',occupied=False))))
    markers=highway_markers(run,Time()).markers
    assert any(m.ns=='highway_gap' and m.type==Marker.TRIANGLE_LIST for m in markers)
    assert any(m.ns=='highway_lane_edges' for m in markers)
    assert any('L4 REAR' in m.text for m in markers)
    assert any('GAP NOW' in m.text for m in markers)
    # A clear candidate still waiting for confirmation is not displayed as GO.
    border=next(m for m in markers if m.ns=='highway_gap_border')
    assert border.color.r>0.5
