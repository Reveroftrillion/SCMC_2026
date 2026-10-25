import math
import numpy as np
from foscar_planning.ring_contact import contact_arc


def test_outside_rear_corner_does_not_occupy_circulating_lane():
    center=np.array([-102.01770577,343.38164401]);radius=17.800235
    body=(-97.17819933193041,323.727013931097,1.1042372651785834,4.635,1.892)
    reference=-1.4334368895512306
    lo,hi=contact_arc(body,center,radius,-1.3,1.3,reference,1)
    x,y,h,l,w=body
    raw=[]
    for u,v in ((-l/2,-w/2),(-l/2,w/2),(l/2,-w/2),(l/2,w/2)):
        dx=x+u*math.cos(h)-v*math.sin(h)-center[0]
        dy=y+u*math.sin(h)+v*math.cos(h)-center[1]
        raw.append(math.atan2(math.sin(math.atan2(dy,dx)-reference),math.cos(math.atan2(dy,dx)-reference))*radius)
    assert lo>min(raw)+1.0
    assert hi<=max(raw)+1e-6


def test_no_contact_when_radially_separated():
    assert contact_arc((30.,0.,0.,4.,2.),np.zeros(2),20.,-1.,1.,0.,1) is None
    assert contact_arc((10.,0.,0.,4.,2.),np.zeros(2),20.,-1.,1.,0.,1) is None


def test_arc_encloses_every_sampled_point_inside_lane_in_both_directions():
    for direction in (-1,1):
        for h in np.linspace(-math.pi,math.pi,9):
            body=(-20.,0.,float(h),4.635,1.892)
            reference=math.pi*direction
            lo,hi=contact_arc(body,np.zeros(2),20.,-1.,1.,reference,direction)
            for u in np.linspace(-body[3]/2,body[3]/2,31):
                for v in np.linspace(-body[4]/2,body[4]/2,11):
                    x=body[0]+u*math.cos(h)-v*math.sin(h);y=u*math.sin(h)+v*math.cos(h)
                    if not 19.<=math.hypot(x,y)<=21.:continue
                    a=math.atan2(y,x)*direction-reference
                    s=math.atan2(math.sin(a),math.cos(a))*20.
                    assert lo-1e-7<=s<=hi+1e-7
