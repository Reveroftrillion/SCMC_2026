"""Recorded wall probes must not borrow later scans or invalid evidence."""
import json
from types import SimpleNamespace

import pytest

import probe_tunnel_wall_clearance as tool


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


def message(topic, data, second):
    value = data if topic == '/localization/s' else json.dumps(data)
    return topic, SimpleNamespace(data=value), Stamp(second)


def footprint(second, **values):
    return message('/perception/footprints', dict(
        stamp=second, frame='map', tunnel_walls=[[0., 0., 'L']], **values), second)


def test_wall_reconstruction_stops_at_first_query_scan_and_filters_evidence(monkeypatch):
    calls = []
    corridor = object()

    class Memory:
        def add(self, walls, *args):
            calls.append(walls)
            return True

        def due(self, now):
            return True

        def corridor(self, road, end_s):
            return corridor

    monkeypatch.setattr(tool, 'WallMemory', Memory)
    runtime = SimpleNamespace(route=object(), road=object(), config={'tunnel.s_to': 1990.33})

    def messages():
        yield message('/localization/s', 1879., 1.)
        yield footprint(1.)  # no TUNNEL owner yet
        yield message('/behavior/mission', {'mission': 'TUNNEL'}, 1.1)
        yield footprint(1.2, scan_valid=False)
        yield footprint(1.3)
        yield message('/localization/s', 1880., 1.4)
        yield footprint(1.5)
        raise AssertionError('future evidence was consumed')

    result, evidence = tool.reconstruct(messages(), runtime, 1880.)
    assert result is corridor
    assert len(calls) == 2
    assert [row['stamp'] for row in evidence] == [1.3, 1.5]


def test_missing_wall_observations_do_not_invent_a_corridor():
    runtime = SimpleNamespace(route=object(), road=object(), config={'tunnel.s_to': 1990.33})
    messages = [message('/behavior/mission', {'mission': 'TUNNEL'}, 1.),
                message('/localization/s', 1880., 1.1),
                message('/perception/footprints', {'frame': 'map', 'stamp': 1.2,
                                                   'tunnel_walls': []}, 1.2)]
    with pytest.raises(ValueError, match='No preceding TUNNEL wall corridor'):
        tool.reconstruct(messages, runtime, 1880.)


def test_active_bag_is_rejected_before_creating_output(tmp_path):
    active = tmp_path / 'input.bag.active'
    active.write_bytes(b'not a closed bag')
    output = tmp_path / 'result'
    with pytest.raises(SystemExit):
        tool.main(['--bag', str(active), '--output', str(output)])
    assert not output.exists()
