"""Angular extent of the part of an ego rectangle inside a lane annulus."""
import math
import numpy as np


def contact_arc(body,center,radius,radial_lo,radial_hi,reference,direction):
    x,y,h,length,width=body
    c,s=math.cos(h),math.sin(h)
    corners=np.array([[x+u*c-v*s-center[0],y+u*s+v*c-center[1]]
                      for u,v in ((-length/2,-width/2),(length/2,-width/2),
                                  (length/2,width/2),(-length/2,width/2))])
    inner=max(0.,radius+radial_lo);outer=radius+radial_hi
    norms=np.sum(corners*corners,axis=1)
    points=list(corners[(norms>=inner*inner-1e-9)&(norms<=outer*outer+1e-9)])
    # Endpoints of each rectangle-edge segment inside the annulus. Angular
    # extrema on a straight segment occur at its endpoints, not at an exterior
    # rectangle corner. Circle arcs between these intersections add no extrema.
    edges=np.roll(corners,-1,axis=0)-corners
    aa=np.sum(edges*edges,axis=1)
    bb=2*np.sum(corners*edges,axis=1)
    for r in (inner,outer):
        disc=bb*bb-4*aa*(norms-r*r)
        for sign in (-1.,1.):
            t=(-bb+sign*np.sqrt(np.maximum(0.,disc)))/(2*np.maximum(aa,1e-12))
            valid=(disc>=0.)&(aa>1e-12)&(t>=0.)&(t<=1.)
            points.extend(corners[valid]+t[valid,None]*edges[valid])
    if not points:return None
    pts=np.asarray(points)
    mid=math.atan2(y-center[1],x-center[0])*direction-reference
    mid=math.atan2(math.sin(mid),math.cos(mid))
    angles=np.arctan2(pts[:,1],pts[:,0])*direction-reference
    angles=mid+np.arctan2(np.sin(angles-mid),np.cos(angles-mid))
    return float(min(angles)*radius),float(max(angles)*radius)
