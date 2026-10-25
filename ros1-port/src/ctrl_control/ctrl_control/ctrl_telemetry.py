"""Read-only explanation of the issued speed command (not a second controller)."""
import math


def control_telemetry(output, target, stamp, speed, loc_mode, dr_speed_kmh):
    source='PATH/STEERING'
    if output.state!='CRUISE':source=output.state
    elif output.reason=='NO_TARGET':source='NO_TARGET'
    elif target is not None and output.target_speed>=target.v_max-.01:source='MISSION'
    elif target is not None and math.isfinite(target.stop_s):source='STOP/PATH'
    elif loc_mode=='DR' and dr_speed_kmh>0 and output.target_speed>=dr_speed_kmh/3.6-.01:source='DR'
    return dict(stamp=stamp,state=output.state,reason=output.reason,
                speed_mps=speed,target_speed_mps=output.target_speed,limit_source=source,
                accel=output.accel,brake=output.brake,steering_rad=output.steering)
