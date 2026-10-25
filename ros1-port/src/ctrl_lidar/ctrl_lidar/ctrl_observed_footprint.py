"""Compact outline of measured returns, not an inferred full vehicle shape."""
import numpy as np
from scipy.spatial import ConvexHull, QhullError


def oriented_dimensions(hull):
    """Minimum-area enclosing rectangle of an already convex measured outline."""
    pts=np.asarray(hull,dtype=float).reshape(-1,2)
    if len(pts)<2:return (0.,0.)
    edges=np.roll(pts,-1,axis=0)-pts
    edges=edges[np.linalg.norm(edges,axis=1)>1e-8]
    if not len(edges):return (0.,0.)
    axes=edges/np.linalg.norm(edges,axis=1)[:,None]
    along=np.ptp(pts@axes.T,axis=0)
    across=np.ptp(pts@np.column_stack((-axes[:,1],axes[:,0])).T,axis=0)
    best=int(np.argmin(along*across))
    return (float(max(along[best],across[best])),float(min(along[best],across[best])))


def observed_hull(points):
    pts=np.asarray(points,dtype=float)[:,:2]
    pts=pts[np.isfinite(pts).all(axis=1)]
    if len(pts)<3:
        return pts.tolist()
    try:
        return pts[ConvexHull(pts).vertices].tolist()
    except QhullError:
        axis=int(np.argmax(np.ptp(pts,axis=0)))
        return pts[[np.argmin(pts[:,axis]),np.argmax(pts[:,axis])]].tolist()


def observed_box(hull):
    """Center, yaw, length and width of the minimum-area observed rectangle."""
    pts=np.asarray(hull,dtype=float).reshape(-1,2)
    edges=np.roll(pts,-1,axis=0)-pts
    angles=np.arctan2(edges[:,1],edges[:,0])
    best=None
    for yaw in angles:
        c,s=np.cos(yaw),np.sin(yaw);axes=np.array([[c,-s],[s,c]])
        uv=pts@axes;lo=uv.min(axis=0);hi=uv.max(axis=0);size=hi-lo
        if best is None or np.prod(size)<best[0]:
            center=((lo+hi)/2)@axes.T
            best=(np.prod(size),(*center,float(yaw),float(size[0]),float(size[1])))
    return best[1]
