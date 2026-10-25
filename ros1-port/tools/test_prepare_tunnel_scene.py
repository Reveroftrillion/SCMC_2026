import copy
import math

import pytest

from prepare_tunnel_scene import build


def scene():
    return dict(mapInfo=dict(mapName='R_KR_PR_K-city_2025', eastOffset=302595., northOffset=4124145.),
                egoVehicle=dict(UNIQUEID=1, control='Manual'),
                pedestrianList=[dict(UNIQUEID=9, activeDistance=15.)],
                objectList=[dict(UNIQUEID=2, DataID=40100001, moveSpeed=0., waypointEventList=[],
                                 pos={'x': 1., 'y': 2., 'z': 3.}, attribution='source retained')])


def test_only_one_unique_static_actor_is_added_and_source_is_preserved():
    source = scene()
    before = copy.deepcopy(source)
    result, actor = build(source, 2, [10., 20., 28.4], -150., [3., 1., 1.])
    assert source == before
    assert result['objectList'][:-1] == before['objectList']
    assert dict(result, objectList=[]) == dict(before, objectList=[])
    assert actor['UNIQUEID'] == 10 and actor['DataID'] == 40100001
    assert actor['attribution'] == 'source retained'
    assert actor['pos']['_z'] == '28.400' and actor['scale']['x'] == 3.


@pytest.mark.parametrize('scale', [[0., 1., 1.], [1., -1., 1.], [math.nan, 1., 1.]])
def test_invalid_scale_is_rejected(scale):
    with pytest.raises(ValueError):
        build(scene(), 2, [1., 2., 3.], 0., scale)


def test_wrong_map_and_moving_template_are_rejected():
    wrong = scene()
    wrong['mapInfo']['mapName'] = 'different'
    with pytest.raises(ValueError):
        build(wrong, 2, [1., 2., 3.], 0., [1., 1., 1.])
    moving = scene()
    moving['objectList'][0]['moveSpeed'] = 1.
    with pytest.raises(ValueError):
        build(moving, 2, [1., 2., 3.], 0., [1., 1., 1.])


def test_explicit_native_id_registers_waypoint_actor_without_changing_existing_data():
    source = scene()
    source['objectList'][0]['isWaypointMode'] = True
    source['waypointDataList'] = [dict(pathName='ObstaclePath - 2', waypointData=[])]
    before = copy.deepcopy(source)
    result, actor = build(source, 2, [10., 20., 28.4], -150., [3., 1., 1.], actor_id=16)
    assert source == before
    assert actor['UNIQUEID'] == 16
    assert result['objectList'][:-1] == before['objectList']
    assert result['waypointDataList'] == before['waypointDataList'] + [
        dict(pathName='ObstaclePath - 16', waypointData=[])]
    assert {k: v for k, v in result.items() if k not in ('objectList', 'waypointDataList')} == {
        k: v for k, v in before.items() if k not in ('objectList', 'waypointDataList')}


@pytest.mark.parametrize('actor_id', [1, 2, 9, 0, -1, True, 1.5])
def test_colliding_or_invalid_explicit_id_is_rejected(actor_id):
    with pytest.raises(ValueError):
        build(scene(), 2, [1., 2., 3.], 0., [1., 1., 1.], actor_id=actor_id)


def test_orphan_waypoint_name_is_not_reused():
    source = scene()
    source['objectList'][0]['isWaypointMode'] = True
    source['waypointDataList'] = [dict(pathName='ObstaclePath - 16', waypointData=[])]
    with pytest.raises(ValueError):
        build(source, 2, [1., 2., 3.], 0., [1., 1., 1.], actor_id=16)
