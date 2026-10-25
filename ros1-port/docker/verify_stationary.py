#!/usr/bin/env python3
"""Receive Competition status only; never send control or start ROS nodes."""
import argparse
import json
from pathlib import Path
import socket
import time

from ctrl_control.ctrl_core import parse_status

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds', type=float, default=5.)
p.add_argument('--output', type=Path, required=True)
args = p.parse_args()
samples, errors = [], []
start = time.monotonic()
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1', 909))
    sock.settimeout(.2)
    while time.monotonic()-start < args.seconds:
        try:
            raw, source = sock.recvfrom(65535)
        except socket.timeout:
            continue
        try:
            status = parse_status(raw)
            status.update(t=time.monotonic()-start, stamp_hex=raw[27:35].hex(), source=list(source))
            samples.append(status)
        except Exception as exc:
            errors.append(str(exc))
report = dict(scope='post-trial receive-only actual vehicle status', samples=samples, errors=errors,
    maximum_abs_speed_mps=max((abs(v['speed']) for v in samples),default=None),
    distinct_source_timestamps=len({v['stamp_hex'] for v in samples}),
    modes=sorted({v['mode'] for v in samples}),gears=sorted({v['gear'] for v in samples}))
report['stationary_confirmed'] = (len(samples)>20 and report['distinct_source_timestamps']>5
    and samples[-1]['t']-samples[0]['t']>args.seconds-.5 and not errors
    and report['maximum_abs_speed_mps']<.01 and report['modes']==[1] and report['gears']==[1])
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
raise SystemExit(0 if report['stationary_confirmed'] else 1)
