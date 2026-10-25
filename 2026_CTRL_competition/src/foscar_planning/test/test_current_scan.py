import math
from dataclasses import replace
from test_circular_gap import scene


def scan(ring,stamp,ids):
    rows=[]
    for key in ids:
        e=ring.tracks[key].event
        rows.append(dict(id=key,xy=[[e.x-.2,e.y-.2],[e.x+.2,e.y+.2]]))
    return dict(frame='map',stamp=stamp,objects=rows)


def test_complete_scan_removes_missing_and_cached_events_cannot_resurrect():
    ring,pred,ctx=scene();ring.local_entry=True
    ring.on_footprints(scan(ring,1.01,['lidar:test:0']))
    assert 'lidar:test:1' not in ring.tracks
    ring.observe(pred,1.02)
    assert 'lidar:test:1' not in ring.tracks


def test_empty_scan_is_not_sensor_dropout_and_old_packet_cannot_restore_objects():
    ring,pred,ctx=scene();ring.local_entry=True
    old=scan(ring,1.01,list(ring.tracks))
    ring.on_footprints(dict(frame='map',stamp=1.1,objects=[]))
    assert not ring.tracks
    ring.on_footprints(old);ring.observe(pred,1.11)
    assert ring.scan_stamp==1.1 and not ring.tracks
    out=ring.evaluate(replace(ctx,now=2.),pred)
    assert out['decision']=='UNKNOWN' and 'scan' in out['reason']


def test_returning_same_id_does_not_inherit_old_motion():
    ring,pred,ctx=scene();ring.local_entry=True
    key='lidar:test:0';old=ring.tracks[key]
    ring.on_footprints(dict(frame='map',stamp=1.1,objects=[]))
    e=replace(old.event,stamp=1.2)
    pred.observe([e],1.2)
    ring.on_footprints(dict(frame='map',stamp=1.2,objects=[dict(id=key,xy=[[e.x-.2,e.y-.2],[e.x+.2,e.y+.2]])]))
    ring.observe(pred,1.2)
    assert ring.tracks[key] is not old
    assert not ring.tracks[key].ever_ready
    assert len(ring.tracks[key].samples)==1


def test_missing_or_malformed_object_list_does_not_clear_observations():
    ring,pred,ctx=scene();ring.local_entry=True
    for objects in (None,'broken',[dict(id='broken',xy=[]) ]):
        ring.on_footprints(dict(frame='map',stamp=2.,objects=objects))
        assert len(ring.tracks)==2
        assert ring.scan_stamp==-math.inf


def test_non_local_mode_retains_previous_memory_policy():
    ring,pred,ctx=scene()
    ring.on_footprints(dict(frame='map',stamp=1.1,objects=[]))
    assert len(ring.tracks)==2
