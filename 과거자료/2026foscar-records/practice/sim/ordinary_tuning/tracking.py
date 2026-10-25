#!/usr/bin/env python3
"""Compare ordinary CRUISE yaw against the global route; avoid/tunnel offsets excluded."""
import csv,json,sys
import numpy as np
from foscar_control.core import Route,wrap
route=Route(np.loadtxt('src/foscar_bringup/routes/global_path.txt'))
for path in sys.argv[1:]:
    rows=list(csv.DictReader(open(path)))
    groups={}
    for name,lo,hi in [('before',1,1108.7),('after',1731.6,2155.3)]:
        selected=[r for r in rows if lo<=float(r['s_m'])<hi and r['reason']=='CRUISE' and float(r['speed_mps'])>.5]
        errors=[abs(wrap(float(r['yaw_rad'])-route.heading_at(float(r['s_m']))))*180/np.pi for r in selected]
        groups[name]={'heading_max_deg':max(errors,default=0),'heading_p95_deg':float(np.percentile(errors,95)) if errors else 0,'samples':len(errors)}
    print(json.dumps({'csv':path,**groups}))
