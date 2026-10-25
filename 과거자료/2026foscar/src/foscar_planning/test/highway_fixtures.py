import hashlib
from pathlib import Path
import yaml
from foscar_planning.highway_config import HighwayConfig

def calibrated_config(tmp_path,mode='control'):
    data=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/highway.yaml').read_text())
    data['mode']=mode
    data['policy']['rear_roi_m']=60.
    data['policy']['front_roi_m']=80.
    data['bounds']=dict(min_brake_mps2=2.,lat_accel_mps2=2.5,latency_s=.1,
        front_range_m=80.,rear_range_m=60.,npc_speed_bound_mps=25.,position_error_m=.1,velocity_error_mps=.5)
    return HighwayConfig.from_dict(data)
