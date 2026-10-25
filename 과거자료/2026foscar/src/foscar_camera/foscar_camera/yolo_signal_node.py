#!/usr/bin/env python3
"""Camera 7 detections only. No control publisher, semantic or simulator API."""
import json,math,threading,time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool,Float64,String
from .yolo_onnx import Detector,choose_signal

class YoloSignalNode(Node):
    def __init__(self):
        super().__init__('yolo_signal_node');p=self.declare_parameter
        self.lines=json.loads(Path(p('stoplines_file','').value).read_text())['lines']
        log=str(p('detections_log','').value)
        self.log=open(Path(log).expanduser(),'a') if log else None
        self.detector=Detector(p('model_file','').value,float(p('confidence',.4).value),int(p('threads',2).value))
        self.confirm=int(p('confirm_frames',2).value);self.max_age=float(p('max_frame_age_s',1.).value)
        self.s=None;self.s_at=-math.inf;self.last_s=None;self.epoch=0
        self.pending=None;self.count=0;self.key=None
        self.lock=threading.Lock();self.wake=threading.Event();self.running=True;self.frame=None
        self.pub=self.create_publisher(String,'/perception/traffic_signal',10)
        self.debug=self.create_publisher(CompressedImage,'/perception/traffic_light/debug/compressed',1)
        # Off outside the signal missions: frames are dropped undecoded (missions.yaml signal).
        self.gate=bool(p('require_mission_gate',True).value);self.enabled_at=-math.inf
        self.create_subscription(Bool,'/behavior/signal_camera',self.on_gate,10)
        self.create_subscription(Float64,'/localization/s',self.on_s,qos_profile_sensor_data)
        self.create_subscription(CompressedImage,'/sensors/camera/traffic/compressed',self.on_image,qos_profile_sensor_data)
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def on_s(self,msg):
        if self.last_s is not None and (msg.data<self.last_s-10 or msg.data>self.last_s+80):self.epoch+=1
        self.s=self.last_s=float(msg.data);self.s_at=self.now()
    def current(self):
        if self.s is None or not math.isfinite(self.s) or self.now()-self.s_at>.5:return None
        line=next((x for x in self.lines if self.s<=x['stop_rear_s']+2),None)
        return line if line and self.s>=line['start_s'] else None
    def on_gate(self,msg):
        self.enabled_at=self.now() if msg.data else -math.inf
    def enabled(self):
        return not self.gate or self.now()-self.enabled_at<=.5
    def on_image(self,msg):
        if not self.enabled():return
        received=self.now();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.05<=received-stamp<=self.max_age:return
        with self.lock:self.frame=(bytes(msg.data),stamp,self.epoch)
        self.wake.set()
    def run(self):
        while self.running:
            self.wake.wait(.2);self.wake.clear()
            with self.lock:frame=self.frame;self.frame=None
            if frame is None:continue
            line=self.current()
            if not line or not self.enabled():continue
            data,stamp,epoch=frame
            if self.now()-stamp>self.max_age:continue
            key=(line['id'],epoch)
            if key!=self.key:self.key=key;self.pending=None;self.count=0
            try:
                image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                if image is None:continue
                started=time.monotonic()
                detections=self.detector(image);color=choose_signal(detections,line['roi'],image.shape)
                if self.log is not None:
                    h,w=image.shape[:2]
                    self.log.write(json.dumps(dict(t=self.now(),stamp=stamp,s=self.s,line=line['id'],
                        infer_ms=round((time.monotonic()-started)*1000),chosen=color,
                        dets=[(d['color'],round(d['confidence'],2),round((d['xyxy'][0]+d['xyxy'][2])/2/w,3),
                               round((d['xyxy'][1]+d['xyxy'][3])/2/h,3)) for d in detections]))+'\n');self.log.flush()
                current=self.current()
                if not current or current['id']!=line['id'] or epoch!=self.epoch or self.now()-stamp>self.max_age:
                    self.pending=None;self.count=0;continue
                self.count=self.count+1 if color and color==self.pending else 1 if color else 0
                self.pending=color
                if color and self.count>=self.confirm:
                    self.pub.publish(String(data=json.dumps(dict(id=line['id'],color=color,stamp=stamp))))
                for d in detections:
                    a,b,c,e=map(int,d['xyxy']);cv2.rectangle(image,(a,b),(c,e),(0,255,255),2)
                    cv2.putText(image,d['color'],(a,max(12,b-4)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                ok,jpeg=cv2.imencode('.jpg',image)
                if ok:
                    msg=CompressedImage();msg.header.stamp=self.get_clock().now().to_msg();msg.format='jpeg';msg.data=jpeg.tobytes();self.debug.publish(msg)
            except Exception as exc:
                self.pending=None;self.count=0;self.get_logger().error(f'ONNX inference: {exc}')
    def destroy_node(self):
        self.running=False;self.wake.set();self.worker.join(timeout=5);super().destroy_node()
def main(args=None):
    rclpy.init(args=args);node=YoloSignalNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
