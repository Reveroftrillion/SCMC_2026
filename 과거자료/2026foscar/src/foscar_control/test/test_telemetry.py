import math
from foscar_control.core import Output,Target
from foscar_control.telemetry import control_telemetry


def test_display_reports_actual_command_not_mission_ceiling():
    o=Output('CRUISE','CRUISE',target_speed=12.)
    d=control_telemetry(o,Target(v_max=math.inf),1.,10.,'GPS',22.)
    assert d['target_speed_mps']==12 and d['limit_source']=='PATH/STEERING'
    d=control_telemetry(Output('CRUISE','CRUISE',target_speed=5.),Target(v_max=5.),1.,10.,'GPS',22.)
    assert d['limit_source']=='MISSION'


def test_fault_stop_and_dead_reckoning_limits_are_distinct():
    for output,target,mode,label in [
        (Output('HOLD','sensor timeout'),Target(),'GPS','HOLD'),
        (Output('CRUISE','stopping'),Target(v_max=0.),'GPS','MISSION'),
        (Output('CRUISE','NO_TARGET'),None,'GPS','NO_TARGET'),
        (Output('CRUISE','CRUISE',target_speed=22/3.6),Target(),'DR','DR'),
    ]:
        assert control_telemetry(output,target,1.,2.,mode,22.)['limit_source']==label
