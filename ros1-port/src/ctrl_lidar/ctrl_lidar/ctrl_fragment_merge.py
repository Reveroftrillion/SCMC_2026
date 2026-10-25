"""Join adjacent measured fragments into one occupied hull; never erase returns."""
import numpy as np
from .ctrl_observed_footprint import observed_hull


def overlaps(a,b):
 if len(a)<3 or len(b)<3:return False
 for poly in (a,b):
  edges=np.roll(poly,-1,axis=0)-poly
  for edge in edges:
   axis=np.array([-edge[1],edge[0]])
   if np.linalg.norm(axis)<1e-8:continue
   aa=a@axis;bb=b@axis
   if max(aa)<min(bb) or max(bb)<min(aa):return False
 return True


def distance(a,b):
 if overlaps(a,b):return 0.
 best=float('inf')
 for pts,poly in ((a,b),(b,a)):
  edge=np.roll(poly,-1,axis=0)-poly
  delta=pts[:,None,:]-poly[None,:,:]
  t=np.clip(np.sum(delta*edge[None,:,:],axis=2)/np.maximum(1e-9,np.sum(edge*edge,axis=1)),0,1)
  best=min(best,float(np.min(np.linalg.norm(delta-t[:,:,None]*edge[None,:,:],axis=2))))
 return best


def merge_vehicle_fragments(dets):
 out=[dict(d) for d in dets]
 while True:
  best=None
  for i,a in enumerate(out):
   for j in range(i+1,len(out)):
    b=out[j]
    if a.get("structure",False) or b.get("structure",False):continue
    if np.hypot(a['x']-b['x'],a['y']-b['y'])>6.5:continue
    pa=np.asarray(a['footprint']);pb=np.asarray(b['footprint'])
    if len(pa)<2 or len(pb)<2:continue
    gap=distance(pa,pb)
    if gap>.85:continue
    points=np.vstack([pa,pb]);hull=np.asarray(observed_hull(points))
    edges=np.roll(hull,-1,axis=0)-hull;angles=np.arctan2(edges[:,1],edges[:,0])
    sizes=[]
    for angle in angles:
     c,s=np.cos(angle),np.sin(angle);size=np.ptp(hull@np.array([[c,-s],[s,c]]),axis=0)
     sizes.append((float(np.prod(size)),float(max(size)),float(min(size))))
    _,long,short=min(sizes)
    if not (long<=6.5 and .6<=short<=2.8):continue
    if best is None or gap<best[0]:best=(gap,i,j,hull)
  if best is None:return out
  _,i,j,hull=best;a,b=out[i],out[j];n=a['n']+b['n'];lo=hull.min(axis=0);hi=hull.max(axis=0)
  top=max(a['z_max'],b['z_max']);bottom=min(a['z_max']-a['size_z'],b['z_max']-b['size_z'])
  out[i]=dict(x=(a['x']*a['n']+b['x']*b['n'])/n,y=(a['y']*a['n']+b['y']*b['n'])/n,
   size_x=float(hi[0]-lo[0]),size_y=float(hi[1]-lo[1]),size_z=top-bottom,z_max=top,n=n,footprint=hull.tolist(),structure=False)
  out.pop(j)
