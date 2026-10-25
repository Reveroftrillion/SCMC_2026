import numpy as np
from foscar_camera.yolo_onnx import decode, choose_signal

def test_letterbox_inverse_and_class_scores():
 raw=np.zeros((1,11,2),np.float32)
 raw[0,:4,0]=[640,640,100,20];raw[0,4,0]=.9
 raw[0,:4,1]=[642,640,100,20];raw[0,4,1]=.8
 ds=decode(raw,1.,(0,280),(720,1280),.4)
 assert len(ds)==1 and ds[0]['color']=='red'
 assert np.allclose(ds[0]['xyxy'],[590,350,690,370])

def test_roi_and_conflict():
 ds=[dict(color='green',confidence=.9,xyxy=[100,100,200,120]),dict(color='red',confidence=.8,xyxy=[300,100,400,120])]
 assert choose_signal(ds,[0,0,1,1],(720,1280)) is None
 assert choose_signal(ds,[0,0,.2,1],(720,1280))=='green'
 assert choose_signal([dict(color='pedestrian',confidence=.9,xyxy=[100,100,120,130])],[0,0,1,1],(720,1280)) is None
