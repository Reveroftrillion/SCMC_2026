import numpy as np
from model import fit_throttle

def test_recovers_known_gain_drag_delay():
    dt=.05
    t=np.arange(0,40,dt)
    u=.45+.25*np.sin(t*1.3)+.1*np.sin(t*3.1)
    v=9+4*np.sin(t*.3)
    a=3.5*np.interp(t-.15,t,u)-.08-.0012*v*v
    fit=fit_throttle(t,v,u,a,np.ones(len(t),dtype=bool))
    assert abs(fit['delay']-.15)<.001
    assert np.allclose(fit['coefficients'],[3.5,-.08,-.0012],atol=.001)
