#!/usr/bin/env python3
"""시뮬 연결 확인. 실행 전에 한 번 돌려서 4 채널 다 OK 인지 본다.
    python3 tools/check_sim.py            # sim_ip 는 config/low_speed.yaml 에서 읽음
    python3 tools/check_sim.py 3          # 수신 대기 시간(초)
"""
import re, socket, select, subprocess, sys, time
from pathlib import Path

ws = Path(__file__).resolve().parent.parent
cfg = ws / 'src/foscar_bringup/config/low_speed.yaml'
sim_ip = re.search(r'sim_ip:\s*"([\d.]+)"', cfg.read_text()).group(1)
wait = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

print(f"sim_ip = {sim_ip}  (from {cfg.relative_to(ws)})")
my_ip = subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout.split()
print(f"my ip  = {' '.join(my_ip)}  ← 시뮬 Network 탭 목적지가 이 중 하나여야 함")

ok = subprocess.run(['ping', '-c', '1', '-W', '1', sim_ip], capture_output=True).returncode == 0
print(f"ping   = {'OK' if ok else 'FAIL  ← 케이블/IP 확인'}")

ports = {9099: 'status (Competition Vehicle Status)', 9102: 'gps', 9104: 'imu'}
socks = {}
for p in ports:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(('0.0.0.0', p))
    except OSError:
        print(f"  :{p} 이미 사용 중 — 다른 스택(run_*.sh)이 떠 있음. 먼저 끄고 다시")
        sys.exit(1)
    s.setblocking(False); socks[s] = p
cnt = {p: 0 for p in ports}; src = {}
end = time.time() + wait
while time.time() < end:
    r, _, _ = select.select(list(socks), [], [], 0.2)
    for s in r:
        _, a = s.recvfrom(65535); cnt[socks[s]] += 1; src[socks[s]] = a[0]

print(f"\nUDP 수신 {wait:.0f}s:")
bad = []
for p, name in ports.items():
    hz = cnt[p] / wait
    mark = 'OK ' if cnt[p] else 'NG '
    if not cnt[p]: bad.append(name)
    print(f"  {mark} :{p} {name:40s} {hz:5.1f} Hz  from {src.get(p, '-')}")

# ctrl cmd 는 우리가 보내는 쪽이라 수신 확인 불가. 시뮬 쪽 9093 이 Connect 인지 눈으로.
print(f"\n  ?   :9093 ctrl cmd (PC→시뮬)  — 시뮬 Network 탭에서 Ego Ctrl Cmd Host={my_ip[0]} Port=9093 Connect 확인")

if bad:
    print(f"\n[NG] 안 오는 채널: {', '.join(bad)}")
    print("     시뮬에서 F4 > Load 로 네트워크 설정 재로드 후, Network Settings 에서 해당 채널 목적지 IP/포트 확인")
    sys.exit(1)
print("\n[OK] 센서 3 채널 정상. bash run_competition.sh 로 출발")
