"""Fixed-shape YOLO26 export: RGB letterbox -> class scores -> NMS."""
import ast
import cv2
import numpy as np
NAMES=['red','yellow','green','green_left','left','pedestrian','red_left']

def decode(raw,ratio,pad,shape,confidence=.4):
    if raw.ndim!=3 or raw.shape[0]!=1 or raw.shape[1]!=11:
        raise ValueError(f'unsupported ONNX output: {raw.shape}')
    rows=raw[0].T;class_ids=rows[:,4:].argmax(1);scores=rows[np.arange(len(rows)),4+class_ids]
    # 유한한 box, confidence, 비보행자 클래스, 양수 크기를 모두 만족하는 행만 유지한다.
    valid_rows=np.isfinite(rows).all(1)&(scores>=confidence)&(class_ids!=5)&(rows[:,2]>0)&(rows[:,3]>0)
    rows=rows[valid_rows];class_ids=class_ids[valid_rows];scores=scores[valid_rows]
    boxes=[]
    for box_x,box_y,box_width,box_height in rows[:,:4]:boxes.append([float(box_x-box_width/2),float(box_y-box_height/2),float(box_width),float(box_height)])
    indices=cv2.dnn.NMSBoxes(boxes,scores.tolist(),confidence,.45) if boxes else []
    detections=[]
    for detection_index in np.asarray(indices,dtype=int).reshape(-1):
        box_x,box_y,box_width,box_height=boxes[detection_index];image_box=np.array([box_x-pad[0],box_y-pad[1],box_x+box_width-pad[0],box_y+box_height-pad[1]])/ratio
        image_box[[0,2]]=np.clip(image_box[[0,2]],0,shape[1]);image_box[[1,3]]=np.clip(image_box[[1,3]],0,shape[0])
        if image_box[2]>image_box[0] and image_box[3]>image_box[1]:detections.append(dict(color=NAMES[int(class_ids[detection_index])],confidence=float(scores[detection_index]),xyxy=image_box.tolist()))
    return detections

def choose_signal(detections,roi,shape):
    image_height,image_width=shape[:2];colors=set()
    for detection in detections:
        x1,y1,x2,y2=detection['xyxy'];normalized_center_x=(x1+x2)/2/image_width;normalized_center_y=(y1+y2)/2/image_height
        if roi[0]<=normalized_center_x<=roi[2] and roi[1]<=normalized_center_y<=roi[3] and detection['color']!='pedestrian':colors.add(detection['color'])
    # ROI 안의 서로 다른 색이 공존하거나 검출이 없으면 None이다. 같은 색 여러 box는 한 색으로 취급한다.
    return next(iter(colors)) if len(colors)==1 else None

class Detector:
    def __init__(self,path,confidence=.4,threads=2):
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=threads
        # GPU first (user 2026-10-05: the competition laptop's RTX 4060 runs this model in ~25 ms a frame against ~536 ms
        # on the CPU; onnxruntime-gpu in .camera-deps), the CPU when no CUDA provider is there.
        providers=[provider_name for provider_name in ('CUDAExecutionProvider','CPUExecutionProvider') if provider_name in ort.get_available_providers()]
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=providers)
        self.provider=self.session.get_providers()[0]
        model_input=self.session.get_inputs()[0];self.input=model_input.name
        if model_input.shape!=[1,3,1280,1280] or model_input.type!='tensor(float)':raise ValueError('expected float32 [1,3,1280,1280] model')
        names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        if [names[class_index] for class_index in range(7)]!=NAMES:raise ValueError('model class mapping differs')
        self.confidence=confidence
    def __call__(self,image):
        image_height,image_width=image.shape[:2];ratio=min(1280/image_width,1280/image_height);resized_width,resized_height=round(image_width*ratio),round(image_height*ratio)
        left,top=(1280-resized_width)//2,(1280-resized_height)//2
        resized=cv2.resize(image,(resized_width,resized_height));padded=np.full((1280,1280,3),114,np.uint8)
        padded[top:top+resized_height,left:left+resized_width]=resized
        tensor=np.ascontiguousarray(padded[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
        raw=self.session.run(None,{self.input:tensor})[0]
        return decode(raw,ratio,(left,top),image.shape,self.confidence)
