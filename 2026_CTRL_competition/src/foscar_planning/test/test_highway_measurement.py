import importlib.util
from pathlib import Path

TOOLS=Path(__file__).resolve().parents[3]/'tools'

def load(name):
    spec=importlib.util.spec_from_file_location(name,TOOLS/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def test_measurement_summary_never_claims_calibration(tmp_path):
    tool=load('measure_highway')
    p=tmp_path/'drive.csv'
    p.write_text('elapsed_s,s_m,speed_mps,brake\n0,1100,8,0\n1,1120,6,0.2\n2,1125,4,0.2\n3,1128,2,0.2\n')
    report=tool.summarize_csv(p,start_s=1119.,end_s=1875.,brake_min=.1)
    assert report['verified'] is False
    assert report['braking_decel_mps2']['median']==2.
    assert report['highway_speed_mps']['max']==6.
    assert 'rear_range_m' in report['requires_measurement']
    assert report['source_sha256']
