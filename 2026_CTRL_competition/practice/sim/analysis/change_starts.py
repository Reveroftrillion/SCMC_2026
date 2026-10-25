"""Every highway lane-change start in a bag: ego speed, admission profile, and the other
bodies near the ego at that moment (along/across in the ego frame, speed along).

    python3 change_starts.py ~/foscar_logs/bags/run_<stamp>
"""
import sys, json, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = SequentialReader(); r.open(StorageOptions(uri=sys.argv[1], storage_id=''), ConverterOptions('', ''))
T = {t.name: t.type for t in r.get_all_topics_and_types()}; t0 = None
while r.has_next():
    tp, d, t = r.read_next()
    if t0 is None: t0 = t
    if tp != '/behavior/highway': continue
    h = json.loads(deserialize_message(d, get_message(T[tp])).data)
    if h['reason'] != 'CHANGE_STARTED': continue
    x, y = h['ego_xy']; yaw = h['ego_yaw']; c, s = math.cos(yaw), math.sin(yaw)
    print('%.1f s=%.0f v=%.1f km/h lane=%s profile=%s rear_gap=%s' % ((t-t0)*1e-9, h['global_s'], h['ego_speed']*3.6,
          h['current_lane'], h.get('admission_profile'), h.get('rear_gap_m') and round(h['rear_gap_m'], 1)))
    for o in h.get('objects') or []:
        px = sum(p[0] for p in o['xy'])/len(o['xy']); py = sum(p[1] for p in o['xy'])/len(o['xy'])
        dx, dy = px-x, py-y; along, across = dx*c+dy*s, -dx*s+dy*c
        if abs(along) < 30 and abs(across) < 6:
            vx, vy = o['velocity']; va = vx*c+vy*s
            print('    %s along=%+.1f across=%+.1f v=%s' % (o['id'][-4:], along, across,
                  '%.1f km/h' % (va*3.6) if o['speed_ready'] else 'unknown'))
