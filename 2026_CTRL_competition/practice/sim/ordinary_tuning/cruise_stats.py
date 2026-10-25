#!/usr/bin/env python3
"""Descriptive measured-speed distribution; conditions differ, so not causal lap gain."""
import csv,json,sys
import numpy as np
for path in sys.argv[1:]:
    selected=[r for r in csv.DictReader(open(path)) if r['reason']=='CRUISE'
              and (1<=float(r['s_m'])<1108.7 or 1731.6<float(r['s_m'])<2155.3)
              and float(r['target_speed_mps'])*3.6>=59.5 and float(r['speed_mps'])*3.6>50]
    v=np.array([float(r['speed_mps'])*3.6 for r in selected])
    print(json.dumps(dict(csv=path,samples=len(v),p10_median_p90_kmh=np.percentile(v,[10,50,90]).tolist(),fraction_ge58_5=float(np.mean(v>=58.5)))))
