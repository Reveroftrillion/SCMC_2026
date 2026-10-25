from pathlib import Path
import json
from ctrl_planning.ctrl_highway_config import HighwayConfig
from test_highway_measurement import load
from test_highway_route import highway

def test_replay_preserves_short_visibility_and_is_deterministic(highway):
    tool=load('replay_highway')
    cfg=HighwayConfig.load(Path(__file__).resolve().parents[1]/'config/highway.yaml')
    rows=[json.loads(line) for line in (Path(__file__).parent/'data/highway_replay.jsonl').read_text().splitlines()]
    a=tool.run_replay(rows,highway,cfg);b=tool.run_replay(rows,highway,cfg)
    assert a==b
    assert [r['observed_count'] for r in a]==[1,1,0,1]
    assert a[-1]['objects'][0]['speed_ready'] is False
    assert all('CALIBRATION' not in r['reason'] for r in a)

def test_replay_stops_highway_ownership_at_cp13(highway):
    tool=load('replay_highway')
    cfg=HighwayConfig.load(Path(__file__).resolve().parents[1]/'config/highway.yaml')
    rows=[]
    for i,s in enumerate((highway.rejoin_s+1,highway.end_s-1,highway.end_s+1)):
        x,y=highway.original.at(s);stamp=1.+i
        rows.append(dict(stamp=stamp,ego=dict(global_s=s,x=x,y=y,yaw=highway.original.heading_at(s),v=5.),
                         footprints=dict(stamp=stamp,frame='map',scan_valid=True,objects=[])))
    out=tool.run_replay(rows,highway,cfg)
    assert out[0]['reason']=='REJOIN_CURVE' and out[1]['reason']=='REJOIN_COMPLETE'
    assert out[2]['reason']=='OUTSIDE_HIGHWAY'
