"""Per-checkpoint-section report of one lap: time, max speed, max |cross-track|, seconds the
car was past the lane-touch proxy (|cross-track| > 0.8 m = 3.5 m lane - 1.9 m car), max
ground-truth lateral acceleration (needs the runner's ay_bins).

    python3 section_report.py ~/foscar_logs/follow_<stamp>.csv practice/sim/runs/<stamp>/results.jsonl
"""
import csv, json, sys
CP = [('START', 0.), ('CP1', 72.6), ('CP2', 186.3), ('CP3', 237.4), ('CP4', 346.7), ('CP5', 471.1), ('CP6', 580.9),
      ('CP7', 635.1), ('CP8', 858.5), ('CP9', 930.6), ('CP10', 1118.7), ('CP11', 1286.2), ('CP12', 1593.6),
      ('CP13', 1741.7), ('CP14', 1859.6), ('CP15', 1996.7), ('END', 2185.)]
TOUCH = .8
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r['state'] == 'CRUISE']
bins = {}
if len(sys.argv) > 2:
    rec = [json.loads(l) for l in open(sys.argv[2])][-1]
    bins = {float(k): v for k, v in rec.get('ay_bins', {}).items()}
print('%-11s %6s %6s %7s %7s %6s' % ('section', 'time', 'vmax', '|ct|max', 'touch_s', 'ay_max'))
for (a, sa), (b, sb) in zip(CP, CP[1:]):
    sec = [r for r in rows if sa <= float(r['s_m']) < sb]
    if not sec:
        continue
    t = [float(r['elapsed_s']) for r in sec]
    ct = [abs(float(r['cross_track_m'])) for r in sec]
    dt = (t[-1]-t[0])/max(1, len(t)-1)
    ay = max([v for k, v in bins.items() if sa-10 < k < sb] or [float('nan')])
    print('%-11s %6.1f %6.1f %7.2f %7.2f %6.2f' % (a+'-'+b, t[-1]-t[0], max(float(r['speed_mps']) for r in sec)*3.6,
                                                  max(ct), sum(c > TOUCH for c in ct)*dt, ay))
