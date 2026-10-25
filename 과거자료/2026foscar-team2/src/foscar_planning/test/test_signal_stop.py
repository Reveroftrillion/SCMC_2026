import math
from foscar_planning.signal_stop import SignalStopPolicy
from foscar_planning.state import Target

LINES=[dict(id='one',start_s=50.,stop_rear_s=100.,maneuver='straight'),dict(id='two',start_s=130.,stop_rear_s=180.,maneuver='left')]
def test_unknown_does_not_stop_and_red_survives_loss():
 p=SignalStopPolicy(LINES);assert math.isnan(p.apply(Target(),60,1).stop_s)
 assert p.observe('one','red',60,1)
 assert p.apply(Target(),90,100).stop_s==103
 assert not p.observe('one','UNKNOWN',90,101)
 assert p.apply(Target(),90,200).stop_s==103

def test_green_releases_and_unknown_keeps_it():
 p=SignalStopPolicy(LINES);p.observe('one','yellow',90,1)
 assert p.apply(Target(),90,1).stop_s==103
 p.observe('one','green',90,2)
 assert math.isnan(p.apply(Target(),90,99).stop_s)

def test_no_memory_across_intersections_or_backward_reset():
 p=SignalStopPolicy(LINES);p.observe('one','red',90,1);p.apply(Target(),90,1)
 assert math.isnan(p.apply(Target(),140,2).stop_s)
 assert not p.observe('one','red',140,3)
 p.observe('two','red',140,4);p.apply(Target(),140,4)
 assert math.isnan(p.apply(Target(),60,5).stop_s)

def test_directional_signals_and_preserve_other_stop():
 for color in ['left','red_left']:
  p=SignalStopPolicy(LINES);p.observe('one',color,90,1)
  assert p.apply(Target(stop_s=95.,v_max=1.,lat_offset=.5),90,1).stop_s==95
  p.observe('two',color,150,2)
  t=p.apply(Target(stop_s=160.,v_max=1.,lat_offset=.5),150,2)
  assert t.stop_s==160 and t.v_max==1 and t.lat_offset==.5
 p=SignalStopPolicy(LINES);p.observe('two','green',150,1)
 assert p.apply(Target(),150,1).stop_s==183

def test_out_of_range_and_stale_observation():
 p=SignalStopPolicy(LINES)
 assert not p.observe('one','red',0,1)
 assert not p.observe('one','red',90,-1,now=3)
 assert not p.observe('one','red',float('nan'),1)

def test_hold_after_small_stop_overshoot_and_release():
 p=SignalStopPolicy(LINES);p.observe('one','red',99,1)
 assert p.apply(Target(),100.7,2).v_max==0
 p.observe('one','green',100.7,3)
 assert math.isinf(p.apply(Target(),100.7,3).v_max)

import pytest
@pytest.mark.parametrize('key,s,color,stop',[
 ('one',90,'red',True),('one',90,'yellow',True),('one',90,'green',False),
 ('one',90,'left',True),('one',90,'red_left',True),('one',90,'green_left',False),
 ('two',150,'red',True),('two',150,'yellow',True),('two',150,'green',True),
 ('two',150,'left',False),('two',150,'red_left',False),('two',150,'green_left',False)])
def test_all_six_classes_for_both_maneuvers(key,s,color,stop):
 p=SignalStopPolicy(LINES);assert p.observe(key,color,s,1)
 assert math.isfinite(p.apply(Target(),s,2).stop_s)==stop

def test_wrong_intersection_and_old_stamp_cannot_release():
 p=SignalStopPolicy(LINES);p.observe('one','red',90,2)
 assert not p.observe('two','green',90,3)
 assert not p.observe('one','green',90,1)
 assert p.apply(Target(),90,4).stop_s==103
