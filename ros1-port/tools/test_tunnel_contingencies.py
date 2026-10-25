"""Verify that the probe rejects unsafe decisions and preserves previous evidence."""
import json
import math

import pytest

import probe_tunnel_contingencies as probe
from ctrl_planning.ctrl_state import Target


def test_stop_and_yield_contracts_reject_an_unconstrained_target():
    target = Target(v_max=50./3.6)
    assert not all(probe.evaluate('stop', target, {}).values())
    assert not all(probe.evaluate('yield', target, {}).values())
    assert not all(probe.evaluate('bypass', target, {}).values())


def test_move_contract_rejects_both_zero_cap_and_finite_stop():
    assert not all(probe.evaluate('move', Target(v_max=0.), {}).values())
    assert not all(probe.evaluate('move', Target(v_max=5., stop_s=1900.), {}).values())


def test_yield_curve_contract_rejects_cancelling_a_committed_curve():
    curve = Target(state='AVOID', v_max=5., lat_offset=3., entry_s=1880., entry_length_m=18.,
                   chain=((3., 1880., 18.), (0., 1950., 25.)))
    stopped = Target(state='YIELD', v_max=0., stop_s=1885.)
    checks = probe.evaluate('yield_with_curve', stopped, {'AVOID': curve})
    assert checks['yield_active'] and checks['stop_command']
    assert not checks['retained_avoidance_curve']


def test_result_cleanup_is_strict_json_and_does_not_change_values():
    original = {'nan': math.nan, 'inf': math.inf, 'finite': 3., 'chain': ((1., 2., 3.),)}
    result = json.loads(json.dumps(probe.clean(original), allow_nan=False))
    assert result == {'nan': None, 'inf': None, 'finite': 3., 'chain': [[1., 2., 3.]]}
    assert math.isnan(original['nan']) and math.isinf(original['inf'])


def test_previous_output_directory_is_never_overwritten(tmp_path):
    evidence = tmp_path / 'summary.json'
    evidence.write_text('original evidence\n')
    with pytest.raises(FileExistsError):
        probe.main(['--output', str(tmp_path)])
    assert evidence.read_text() == 'original evidence\n'
