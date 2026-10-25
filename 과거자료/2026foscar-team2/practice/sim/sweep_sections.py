#!/usr/bin/env python3
"""PRACTICE ONLY: sweep controller speed settings over one route section, one run at a time.

Each setting temporarily rewrites keys in src/foscar_bringup/config/low_speed.yaml (restored on
exit), runs loop_runner once per repetition, and appends one JSON line per run with the section
time, max |cross-track| and seconds past the lane-touch proxy (0.8 m), max speed and 60 km/h
samples outside the exempt highway zone, and ground-truth ay / slip peaks.

    python3 practice/sim/sweep_sections.py Practice3_START 270 --reps 2 \
        --set a_lat_mps2=4.5 --set a_lat_mps2=5.5,accel_mps2=3.0 --out sweep.jsonl
"""
import argparse, csv, glob, json, os, re, shutil, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT/'src/foscar_bringup/config/low_speed.yaml'
EXEMPT = (1118.7, 1741.7)
TOUCH = .8


def apply(text, setting):
    for key, value in setting.items():
        pat = re.compile(r'^(    %s:)\s*[^#\n]*' % re.escape(key), re.M)
        if not pat.search(text):
            sys.exit(f'{key}: not in low_speed.yaml')
        text = pat.sub(lambda m: f'{m.group(1)} {value}' + ('' if value.endswith(']') else '  '), text, count=1)
    return text


def parse(spec):
    out = {}
    for part in spec.split(','):
        k, v = part.split('=', 1)
        out[k.strip()] = v.strip().replace(';', ',')
    return out


def metrics(follow, start_s, end_s):
    rows = [r for r in csv.DictReader(open(follow)) if r['state'] == 'CRUISE']
    sec = [r for r in rows if start_s <= float(r['s_m']) < end_s]
    if not sec:
        return {}
    t = [float(r['elapsed_s']) for r in sec]
    ct = [abs(float(r['cross_track_m'])) for r in sec]
    dt = (t[-1]-t[0])/max(1, len(t)-1)
    v = [float(r['speed_mps'])*3.6 for r in sec]
    outside = [(float(r['s_m']), float(r['speed_mps'])*3.6) for r in sec if not EXEMPT[0] <= float(r['s_m']) < EXEMPT[1]]
    return dict(section_s=round(t[-1]-t[0], 2), ct_max=round(max(ct), 2), touch_s=round(sum(c > TOUCH for c in ct)*dt, 2),
                v_max=round(max(v), 1), over60=sum(k > 60. for _, k in outside),
                v_max_outside=round(max([k for _, k in outside] or [0.]), 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario'); ap.add_argument('end_s', type=float)
    ap.add_argument('--start-s', type=float, default=None, help='section start for metrics (default: run start)')
    ap.add_argument('--reps', type=int, default=2)
    ap.add_argument('--set', action='append', required=True, help='key=value[,key=value]; use ; inside lists')
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=200.)
    a = ap.parse_args()
    backup = CFG.read_text()
    env = dict(os.environ)
    try:
        for spec in a.set:
            setting = parse(spec)
            CFG.write_text(apply(backup, setting))
            for rep in range(a.reps):
                before = set(glob.glob(os.path.expanduser('~/foscar_logs/follow_*.csv')))
                subprocess.run([sys.executable, '-u', str(ROOT/'practice/sim/loop_runner.py'), a.scenario, '--runs', '1',
                                '--end-s', str(a.end_s), '--watch-from', '0', '--stop-s', '20', '--timeout', str(a.timeout)],
                               cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                run = sorted(glob.glob(str(ROOT/'practice/sim/runs/*/results.jsonl')), key=os.path.getmtime)[-1]
                rec = [json.loads(l) for l in open(run)][-1]
                new = sorted(set(glob.glob(os.path.expanduser('~/foscar_logs/follow_*.csv')))-before, key=os.path.getmtime)
                start = a.start_s if a.start_s is not None else (rec.get('s_start') or 0.)
                m = metrics(new[-1], start, a.end_s) if new else {}
                row = dict(setting=setting, rep=rep, scenario=a.scenario, result=rec['result'], drive_s=rec['drive_s'],
                           s_end=rec['s_end'], collisions=len(rec.get('collisions', [])), ay=rec.get('truth_peaks', {}).get('ay'),
                           ay_at=rec.get('truth_peaks_at', {}).get('ay'), slip=rec.get('truth_peaks', {}).get('slip'),
                           slip_at=rec.get('truth_peaks_at', {}).get('slip'),
                           run=os.path.dirname(run), follow=new[-1] if new else None, **m)
                with open(a.out, 'a') as f:
                    f.write(json.dumps(row)+'\n')
                print(json.dumps(row), flush=True)
    finally:
        CFG.write_text(backup)


if __name__ == '__main__':
    main()
