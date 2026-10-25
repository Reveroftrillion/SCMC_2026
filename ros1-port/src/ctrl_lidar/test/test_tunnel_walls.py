import numpy as np
from ctrl_lidar.ctrl_tunnel_walls import wall_faces


def test_innermost_zero_intensity_faces_per_side():
    xs=np.arange(5.,20.,.1)
    right=np.column_stack((xs,np.full_like(xs,-3.7),np.full_like(xs,1.)))
    right_back=right+[0.,-.3,0.]                     # thickness behind the face
    left=np.column_stack((xs,np.full_like(xs,7.4),np.full_like(xs,1.)))
    barricade=np.column_stack((np.full(20,12.),np.linspace(.8,3.8,20),np.full(20,.6)))
    pts=np.vstack((right,right_back,left,barricade))
    intensity=np.r_[np.zeros(len(right)*2+len(left)),np.full(20,40.)]
    faces=wall_faces(pts,intensity)
    r=[y for _,y,s in faces if s=='R'];l=[y for _,y,s in faces if s=='L']
    assert r and l
    assert max(abs(v+3.7) for v in r)<.05 and max(abs(v-7.4) for v in l)<.05   # reflective barricade ignored


def test_ground_and_single_stray_returns_are_not_walls():
    pts=np.array([[10.,-3.7,.05]]*10+[[10.,-1.,1.]])          # ground hits + one stray
    assert wall_faces(pts,np.zeros(len(pts)))==[]
