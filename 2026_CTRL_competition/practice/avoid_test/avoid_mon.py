"""상자 회피 실차 모니터: 오프셋 시작/완성/복귀 s, 상태, 충돌(MORAI CollisionData) 을 기록."""
import os, sys, socket, struct, threading, time, math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from foscar_interfaces.msg import DrivingTarget, Event

TAG = sys.argv[1] if len(sys.argv) > 1 else 'run'
import os
END_S = float(os.environ.get('END_S', 350.0))
LIMIT = float(os.environ.get('LIMIT', 170.0))
OUT = Path(os.environ.get('AVOID_LOG_DIR', str(Path.home() / 'foscar_logs' / 'avoid_tests'))) / f'avoid_{TAG}.txt'
OUT.parent.mkdir(parents=True, exist_ok=True)
collision_packets = 0

st = {'s': None, 'v': 0.0, 'state': '', 'off': 0.0, 'vmax': 0.0, 'stop': float('nan')}
rows, events, hits = [], [], []
stop = threading.Event()

def coll_rx():
    global collision_packets
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 9092)); s.settimeout(0.5)
    while not stop.is_set():
        try: d, _ = s.recvfrom(4096)
        except socket.timeout: continue
        if len(d) != 181 or d[:15] != b'#CollisionData$': continue
        collision_packets += 1
        for i in range(5):
            typ, oid = struct.unpack_from('<hh', d, 39 + i*28)
            if typ >= 0 and oid != 0:      # MORAI 는 '충돌 없음' 을 objType=-1 로 보낸다
                hits.append((time.time(), st['s'], typ, oid)); break
threading.Thread(target=coll_rx, daemon=True).start()

rclpy.init(); n = Node('avoid_mon')
n.create_subscription(Float64, '/localization/s', lambda m: st.__setitem__('s', float(m.data)), qos_profile_sensor_data)
n.create_subscription(Odometry, '/localization/odometry',
    lambda m: st.__setitem__('v', math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), qos_profile_sensor_data)
def on_t(m):
    st.update(state=m.state, off=float(m.lat_offset), vmax=float(m.v_max), stop=float(m.stop_s))
    rows.append((time.time(), st['s'], st['v'], m.state, float(m.lat_offset), float(m.v_max), float(m.stop_s), float(m.return_s), float(m.return_length_m), float(m.entry_s), float(m.entry_length_m)))
n.create_subscription(DrivingTarget, '/behavior/target', on_t, 50)
from std_msgs.msg import String
def on_cs(m):
    if m.data.startswith('FAULT') and not st.get('fault'):
        st['fault'] = m.data; print(f'  !! tracker FAULT: {m.data[:120]}', flush=True)
n.create_subscription(String, '/control/state', on_cs, 10)
n.create_subscription(Event, '/events',
    lambda m: events.append((time.time(), st['s'], m.type, m.x, m.y, m.size_x, m.size_y, m.s)), 100)

t0 = time.perf_counter(); last = 0.0; started = False
while time.perf_counter()-t0 < LIMIT and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.perf_counter()-t0
    if st['s'] is not None and st['v'] > 0.5: started = True
    if el-last > 5:
        last = el
        print(f'  {el:5.0f}s  s={st["s"] and round(st["s"],1)}  v={st["v"]*3.6:4.1f}km/h  {st["state"]:8s} off={st["off"]:+.2f}  충돌={len(hits)}', flush=True)
    if started and st['s'] is not None and st['s'] > END_S: break
    if st.get('fault') and time.perf_counter()-st.setdefault('fault_at', time.perf_counter()) > 3: break   # FAULT 면 3 s 뒤 종료
stop.set(); n.destroy_node(); rclpy.shutdown()

# ---- 요약 ----
if st.get('fault'): print(f'★ FAULT 로 종료: {st["fault"]}')
BOX_F, BOX_R = 329.0-1.5, 329.0+1.5
def first(pred):
    for r in rows:
        if r[1] is not None and pred(r): return r
    return None
r_req  = first(lambda r: abs(r[4]) > 1e-6)
peak = max((abs(r[4]) for r in rows), default=0.)
r_full = first(lambda r: abs(r[4]) >= peak - 1e-6 and peak > 1e-6)
r_rel  = first(lambda r: r_full and r[0] > r_full[0] and abs(r[4]) < 1e-6)
r_exit = first(lambda r: r_full and r[0] > r_full[0] and r[3] == 'CRUISE')
r_avd  = first(lambda r: r[3] == 'AVOID')
out = [f'=== {TAG} ===  targets {len(rows)}  events {len(events)}  상자 실제 s {BOX_F:.1f}~{BOX_R:.1f}']
for name, r in (('AVOID 진입', r_avd), ('오프셋 요청 시작', r_req), (f'오프셋 최대({peak:.2f})', r_full), ('오프셋 0 요청(완료)', r_rel), ('AVOID→CRUISE', r_exit)):
    if r: out.append(f'  {name:16s} s={r[1]:7.2f}  v={r[2]*3.6:4.1f}km/h  (상자 앞면까지 {BOX_F-r[1]:+6.2f} m / 뒤범퍼-상자뒤 {r[1]-0.835-BOX_R:+6.2f} m)')
    else: out.append(f'  {name:16s} 없음')
entries = [r for r in rows if r[10] > 0.]
if entries:
    entry = entries[0]
    out.append(f'  계획 진입 곡선: s={entry[9]:.2f} ~ {entry[9]+entry[10]:.2f} (요청 시점과 다름)')
scheduled = [r for r in rows if r[8] > 0.]
if scheduled:
    last = scheduled[-1]
    out.append(f'  계획 복귀 곡선: s={last[7]:.2f} ~ {last[7]+last[8]:.2f} (주시점은 이 구간을 미리 추종)')
vs = [r[2] for r in rows if r[1] and 300 < r[1] < 340]
if vs: out.append(f'  상자 구간(300~340) 속도 {min(vs)*3.6:.1f}~{max(vs)*3.6:.1f} km/h')
states = sorted(set(r[3] for r in rows)); out.append(f'  상태들: {states}')
stops = [r for r in rows if not math.isnan(r[6])]; out.append(f'  stop_s 나온 틱: {len(stops)}')
ev_box = [e for e in events if e[7] and 300 < e[7] < 345]
from collections import Counter
out.append(f'  상자 구간 이벤트 {len(ev_box)}건 타입 {dict(Counter(e[2] for e in ev_box))}')
out.append(f'  ★ 충돌(CollisionData, 수신 {collision_packets}패킷): {len(hits)}건' + (f'  첫 충돌 s={hits[0][1]} type={hits[0][2]} id={hits[0][3]}' if hits else ''))
if not collision_packets: out.append('  충돌 토픽 미수신: 무충돌 여부는 이 기록으로 검증할 수 없음')
txt = '\n'.join(out); print(txt); open(OUT, 'w').write(txt + '\n')
sys.stdout.flush(); os._exit(0)
