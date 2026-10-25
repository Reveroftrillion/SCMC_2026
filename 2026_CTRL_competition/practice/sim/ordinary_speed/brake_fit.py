"""Brake onset fits from three first-signal approaches; practice CSV inputs only."""
import csv,glob,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import least_squares
results=[]
for stamp in ['145208','145607','150420']:
    filename=glob.glob('/home/foscar/foscar_logs/follow_20261006_'+stamp+'*.csv')[0]
    rows=[x for x in csv.DictReader(open(filename)) if 170<float(x['s_m'])<204 and float(x['elapsed_s'])<28]
    t=np.array([float(x['elapsed_s']) for x in rows])
    v=np.array([float(x['speed_mps']) for x in rows])
    b=np.array([float(x['brake']) for x in rows])
    u=np.array([float(x['accel']) for x in rows])
    grid=np.arange(t[0],t[-1],.05)
    vg=np.interp(grid,t,v)
    acc=savgol_filter(vg,7,2,deriv=1,delta=.05)
    onset=t[np.argmax(b>.1)]
    candidates=[]
    for delay in np.arange(0,.251,.025):
        bg=np.interp(grid-delay,t,b)
        ug=np.interp(grid-delay,t,u)
        mask=(vg>7)&(grid>onset-.1)&(grid<onset+1.5)
        fit=least_squares(lambda z:(3.47*ug-z[0]*bg+.0763-.001617*vg**2-acc)[mask],[15],loss='soft_l1',f_scale=.5)
        residual=(3.47*ug-fit.x[0]*bg+.0763-.001617*vg**2-acc)[mask]
        candidates.append(dict(delay=float(delay),gain=float(fit.x[0]),rmse=float(np.sqrt(np.mean(residual**2)))))
    mask=(grid>onset+.4)&(vg>7)
    bg=np.interp(grid-.125,t,b)
    gain=(-acc+.0763-.001617*vg**2)/np.maximum(bg,.01)
    results.append(dict(csv=filename,best=min(candidates,key=lambda x:x['rmse']),steady_gain_p10_p50_p90=np.percentile(gain[mask],[10,50,90]).tolist(),delay_scan=candidates))
text=json.dumps(results,indent=2)
Path(__file__).with_suffix('.json').write_text(text+'\n')
print(text)
