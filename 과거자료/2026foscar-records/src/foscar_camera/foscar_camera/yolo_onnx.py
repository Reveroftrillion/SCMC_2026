"""Fixed-shape YOLO26 export: RGB letterbox -> class scores -> NMS."""
import ast
import cv2
import numpy as np
NAMES=['red','yellow','green','green_left','left','pedestrian','red_left']

def decode(raw,ratio,pad,shape,confidence=.4):
    if raw.ndim!=3 or raw.shape[0]!=1 or raw.shape[1]!=11:
        raise ValueError(f'unsupported ONNX output: {raw.shape}')
    rows=raw[0].T;classes=rows[:,4:].argmax(1);scores=rows[np.arange(len(rows)),4+classes]
    keep=np.isfinite(rows).all(1)&(scores>=confidence)&(classes!=5)&(rows[:,2]>0)&(rows[:,3]>0)
    rows=rows[keep];classes=classes[keep];scores=scores[keep]
    boxes=[]
    for x,y,w,h in rows[:,:4]:boxes.append([float(x-w/2),float(y-h/2),float(w),float(h)])
    indices=cv2.dnn.NMSBoxes(boxes,scores.tolist(),confidence,.45) if boxes else []
    out=[]
    for i in np.asarray(indices,dtype=int).reshape(-1):
        x,y,w,h=boxes[i];xy=np.array([x-pad[0],y-pad[1],x+w-pad[0],y+h-pad[1]])/ratio
        xy[[0,2]]=np.clip(xy[[0,2]],0,shape[1]);xy[[1,3]]=np.clip(xy[[1,3]],0,shape[0])
        if xy[2]>xy[0] and xy[3]>xy[1]:out.append(dict(color=NAMES[int(classes[i])],confidence=float(scores[i]),xyxy=xy.tolist()))
    return out

def choose_signal(detections,roi,shape):
    h,w=shape[:2];colors=set()
    for d in detections:
        x1,y1,x2,y2=d['xyxy'];x=(x1+x2)/2/w;y=(y1+y2)/2/h
        if roi[0]<=x<=roi[2] and roi[1]<=y<=roi[3] and d['color']!='pedestrian':colors.add(d['color'])
    # Contradictory same-frame targets are unknown, never an invented permission.
    return next(iter(colors)) if len(colors)==1 else None

class Detector:
    def __init__(self,path,confidence=.4,threads=2):
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=threads
        # GPU first (user 2026-10-05: the competition laptop's RTX 4060 runs this model in ~25 ms a frame against ~536 ms
        # on the CPU; onnxruntime-gpu in .camera-deps), the CPU when no CUDA provider is there.
        providers=[p_ for p_ in ('CUDAExecutionProvider','CPUExecutionProvider') if p_ in ort.get_available_providers()]
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=providers)
        self.provider=self.session.get_providers()[0]
        inp=self.session.get_inputs()[0];self.input=inp.name
        if inp.shape!=[1,3,1280,1280] or inp.type!='tensor(float)':raise ValueError('expected float32 [1,3,1280,1280] model')
        names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        if [names[i] for i in range(7)]!=NAMES:raise ValueError('model class mapping differs')
        self.confidence=confidence
    def __call__(self,image):
        h,w=image.shape[:2];ratio=min(1280/w,1280/h);nw,nh=round(w*ratio),round(h*ratio)
        left,top=(1280-nw)//2,(1280-nh)//2
        resized=cv2.resize(image,(nw,nh));padded=np.full((1280,1280,3),114,np.uint8)
        padded[top:top+nh,left:left+nw]=resized
        tensor=np.ascontiguousarray(padded[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
        raw=self.session.run(None,{self.input:tensor})[0]
        return decode(raw,ratio,(left,top),image.shape,self.confidence)
