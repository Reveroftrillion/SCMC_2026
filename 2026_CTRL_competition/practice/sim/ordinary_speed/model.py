"""Offline throttle identification; units are SI. No simulator or production imports."""
import numpy as np
from scipy.optimize import least_squares


def fit_throttle(t,v,u,a,mask):
    candidates=[]
    for delay in np.arange(0,.401,.025):
        X=np.c_[np.interp(t-delay,t,u),np.ones(len(t)),v*v]
        fit=least_squares(lambda z:(X@z-a)[mask],[3.5,-.1,-.001],loss='soft_l1',f_scale=.3)
        residual=(X@fit.x-a)[mask]
        candidates.append(dict(delay=float(delay),coefficients=fit.x.tolist(),rmse=float(np.sqrt(np.mean(residual**2))),mae=float(np.mean(abs(residual))),n=int(sum(mask))))
    return min(candidates,key=lambda c:c['rmse'])
