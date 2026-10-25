#! /usr/bin/env python3

import os
import cv2
import time
import numpy as np

import rospy
import cv_bridge
import message_filters
from sensor_msgs.msg import Image, CompressedImage

from simul_msgs.msg import ObjectInfo

import rospkg
import onnxruntime as ort

class AnomDepthROS:
    def __init__(self):
        rospy.init_node("anom_depth_ros")
        self.pkg_path = rospkg.RosPack().get_path("perception")

        self.params = dict(
            ckpt_path = rospy.get_param("~ckpt", "model.onnx"),
            right_topic_name = rospy.get_param("~right_topic", "/image_jpeg/compressed"),
            left_topic_name = rospy.get_param("~left_topic", "/image_jpeg2/compressed"),
            show_heat = rospy.get_param("~show_heat", False),
            show_vis = rospy.get_param("~show_vis", False),
            out = rospy.get_param("~out", ""),

            thres_method = rospy.get_param("~thres_method", "percentile"), # ["percentile","otsu","abs"]
            thres = rospy.get_param("~thres", 98.8),
            score_thres = rospy.get_param("~score_thres", 0.42),
            box_score_method = rospy.get_param("~box_score_method", "p95"), # ["mean","max","p90","p95"]
            box_score_thres = rospy.get_param("~box_score_thres", 0.65),
            min_area = rospy.get_param("~min_area", 1800),

            use_sgbm = rospy.get_param("~use_sgbm", False),
            num_disp = rospy.get_param("~num_disp", 128),
            min_disp = rospy.get_param("~min_disp", 0),
            block = rospy.get_param("~block", 5),
            fx_px = rospy.get_param("~fx_px", 320.0),
            baseline_m = rospy.get_param("~baseline_m", 0.20),
            depth_scale = rospy.get_param("~depth_scale", 1.0),
            disp_min = rospy.get_param("~disp_min", 0.3),
            disp_max = rospy.get_param("~disp_max", 256.0),
            depth_debug = rospy.get_param("~depth_debug", False),
            fast_mode = rospy.get_param("~fast_mode", False),

            # BBox center filtering
            bbox_center_x_min = rospy.get_param("~bbox_center_x_min", 30),
            bbox_center_x_max = rospy.get_param("~bbox_center_x_max",460),
            bbox_center_y_min = rospy.get_param("~bbox_center_y_min", 140),
            bbox_center_y_max = rospy.get_param("~bbox_center_y_max", 620),
        )
        
        self.frame_count = 0
        self.last_log = time.time()
        self.hits_cache = {}
        self.cv_bridge = cv_bridge.CvBridge()
        self.sess, self.iname, self.onames = self.__load_onnx(os.path.join(self.pkg_path, "models", "abnormal", self.params["ckpt_path"]))

        self.stereo = self.__build_stereo(
            self.params["use_sgbm"],
            self.params["min_disp"],
            self.params["num_disp"],
            self.params["block"],
            self.params["fast_mode"]
        )
    
        self.vis_pub = rospy.Publisher("/visualization", Image, queue_size=1)
        self.object_pub = rospy.Publisher("/obstacle_info_static", ObjectInfo, queue_size=1)

        left_sub = message_filters.Subscriber(self.params["left_topic_name"], CompressedImage)
        right_sub = message_filters.Subscriber(self.params["right_topic_name"], CompressedImage)

        sync = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.005)
        sync.registerCallback(self.callback)

    def callback(self, left_msg, right_msg):
        self.frame_count += 1

        left = self.cv_bridge.compressed_imgmsg_to_cv2(left_msg)
        right = self.cv_bridge.compressed_imgmsg_to_cv2(right_msg)
        if left is None or right is None:
            return
        
        if left.shape[:2] != right.shape[:2]:
            right = cv2.resize(right, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_LINEAR)
        
        H, W = left.shape[:2]

        amap, frame_score = self.__infer_onnx(left)
        amap = np.squeeze(np.array(amap, dtype=np.float32))
        if amap.shape[:2] != (H,W):
            amap = cv2.resize(amap, (W,H), interpolation=cv2.INTER_LINEAR)

        amap01 = self.__normalize01(amap)
        amap01 = cv2.GaussianBlur(amap01, (0,0), 1.0)

        vis = left.copy()
        disp = self.__compute_disparity(left, right, self.stereo, self.params["fast_mode"]) if self.stereo[0] is not None else None
        if disp is not None:
            disp = self.__normalize_disparity(disp, disp_min=self.params["disp_min"], disp_max=self.params["disp_max"])
            if self.params["depth_debug"]:
                valid = np.isfinite(disp)
                vr = float(valid.mean()) * 100.0 if valid.size else 0.0
                d_med = float(np.nanmedian(disp)) if valid.any() else float('nan')
                print(f"[DBG] disp valid={vr:.1f}%  d_med={d_med:.3f}")

        if frame_score >= self.params["score_thres"]:
            mask = self.__make_mask(amap01, method=self.params["thres_method"], value=self.params["thres"]).astype(np.uint8)
            if mask.any():
                k = np.ones((3,3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

            contours,_ = cv2.findContours((mask * 255).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            boxes=[]
            for c in contours:
                if cv2.contourArea(c) < self.params["min_area"]: continue
                x,y,w,h = cv2.boundingRect(c)

                # 바운딩 박스 중심 좌표 필터링
                center_x = x + w // 2
                center_y = y + h // 2

                if not (self.params["bbox_center_x_min"] <= center_x < self.params["bbox_center_x_max"]):
                    continue
                if not (self.params["bbox_center_y_min"] <= center_y < self.params["bbox_center_y_max"]):
                    continue

                roi = amap01[y:y+h, x:x+w]
                bscore = self.__box_score_from_roi(roi, self.params["box_score_method"])
                if bscore < self.params["box_score_thres"]: continue

                depth_median=None
                if disp is not None:
                    d_roi = disp[y:y+h, x:x+w]
                    valid = d_roi[np.isfinite(d_roi) & (d_roi > 0)]
                    if valid.size > 30:
                        d_med = float(np.median(valid))
                        depth_median = (self.params["fx_px"] * self.params["baseline_m"]) / (d_med + 1e-6) * self.params["depth_scale"]
                        if self.params["depth_debug"]:
                            print(f"[DBG] ROI d_med={d_med:.3f} -> depth={depth_median:.2f}m")

                # ★★★ 거리 필터링: 8미터 이내만 표시 ★★★
                if depth_median is not None and depth_median > 10.0:
                    continue  # 8m 넘으면 스킵

                # 요청사항: 1미터 이내 장애물 무시
                if depth_median is not None and depth_median < 1.0:
                    continue # 1m 미만이면 스킵

                boxes.append((x,y,w,h,bscore,depth_median))

            # 개선된 깊이 기반 tracking (끊김 최소화 + 노이즈 필터링)
            new_hits={}
            object_info_msg = ObjectInfo()
            for (x,y,w,h,bscore,depth_median) in boxes:
                # depth와 대략적 위치(화면 영역) 기반 키
                # 화면을 좌/중/우 3개 영역으로 나누고, depth는 2m 단위로 그룹화 (더 넓게)
                screen_zone = 0 if x+w//2 < W//3 else (1 if x+w//2 < 2*W//3 else 2)
                if depth_median is not None:
                    # 2m 단위로 그룹화 (1m→2m, 경계 끊김 감소)
                    depth_group = int(depth_median / 2.0)
                    key = f"z{screen_zone}-d{depth_group}"
                else:
                    # depth 없으면 기존 방식
                    key = f"z{screen_zone}-xy{x//16}-{y//16}"

                # 기존 hits 찾기 - 여러 후보 중 가장 높은 hits 상속
                best_match_hits = 0
                best_match_frame = 0
                for old_key, old_data in self.hits_cache.items():
                    old_count = old_data['count']
                    old_frame = old_data['last_frame']

                    if depth_median is not None and old_key.startswith(f"z{screen_zone}-d"):
                        # 같은 zone이고 depth 추출 가능하면
                        try:
                            old_depth = int(old_key.split('-d')[1])
                            # ±2 그룹까지 허용 (실제로 ±4m 범위)
                            if abs(old_depth - depth_group) <= 2:
                                if old_count > best_match_hits:
                                    best_match_hits = old_count
                                    best_match_frame = old_frame
                        except:
                            pass
                    elif not depth_median and old_key.startswith(f"z{screen_zone}-xy"):
                        # depth 없을 때 xy 기반 매칭
                        if old_count > best_match_hits:
                            best_match_hits = old_count
                            best_match_frame = old_frame

                # 연속성 체크: 3프레임 이상 끊기면 hits 감소 (노이즈 필터링)
                frame_gap = self.frame_count - best_match_frame
                if frame_gap > 4:
                    # 끊긴 시간만큼 hits 감소
                    best_match_hits = max(0, best_match_hits - (frame_gap - 3))

                new_count = min(999, best_match_hits + 1)
                new_hits[key] = {'count': new_count, 'last_frame': self.frame_count}
                hits = new_count

                # hits >= 10으로 올림 (노이즈 강력 방지)
                if hits >= 6:
                    

                    color=(0,0,255)
                    cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)

                    # 박스 중앙 좌표 계산
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # 각도 계산 (카메라 중심 기준)
                    offset_x = center_x - (W / 2.0)  # 이미지 중심에서의 오프셋
                    angle_rad = np.arctan(offset_x / self.params["fx_px"])  # 라디안
                    angle_deg = angle_rad * (180.0 / np.pi)  # 도(degree)

                    # 박스 위 라벨 (거리 + 각도)
                    if depth_median is not None:
                        label_top = f"{depth_median:.1f}m {angle_deg:+.1f}°"
                    else:
                        label_top = f"{angle_deg:+.1f}°"

                    cv2.putText(vis, label_top, (x, max(0, y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

                    # 박스 안쪽 중심 좌표 표시
                    label_center = f"({center_x},{center_y})"
                    cv2.putText(vis, label_center, (center_x-40, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2, cv2.LINE_AA)
                        

                    if depth_median is not None:
                        # 1. 3D 중심 좌표 계산
                        # 1. 3D 중심 좌표 계산 (카메라 기준)
                        Z_cam = depth_median
                        X_cam = Z_cam * (center_x - W / 2.0) / self.params["fx_px"]
                        Y_cam = Z_cam * (center_y - H / 2.0) / self.params["fx_px"] # (fy=fx로 가정)

                        # 2. 차량 기준(base_link)으로 좌표 변환 (단순 덧셈)
                        #    (차량기준 카메라 위치: x=2.5, y=0.1, z=1.10)
                        X_base = Z_cam + 2.5
                        
                        # Y_base (차량 좌측) = -X_cam (카메라 우측 * -1) + 카메라 Y 오프셋
                        Y_base = X_cam + 0.1
                        
                        # Z_base (차량 상방) = -Y_cam (카메라 하방 * -1) + 카메라 Z 오프셋
                        Z_base = -Y_cam + 1.10

                        # length_m = 차량 기준 전방 크기 (ROS X축) - 1.0m로 고정
                        length_m = 2.0  
                        # width_m = 카메라 기준 수평 크기 (ROS Y축)
                        width_m = Z_cam * w / self.params["fx_px"] 
                        # height_m = 카메라 기준 수직 크기 (ROS Z축)
                        height_m = Z_cam * h / self.params["fx_px"] # (fy=fx로 가정)

                        
                        if(object_info_msg.objectCounts < 50):
                            object_info_msg.objectCounts += 1
                            
                            # 3D 좌표 (X, Y, Z)를 append (수정된 좌표)
                            object_info_msg.centerX.append(X_base)
                            object_info_msg.centerY.append(-Y_base)
                            object_info_msg.centerZ.append(Z_base)

                            # 3D 크기를 append (ROS 좌표계에 맞게)
                            object_info_msg.lengthX.append(length_m) # X축 (전방) 크기
                            object_info_msg.lengthY.append(width_m)  # Y축 (좌우) 크기
                            object_info_msg.lengthZ.append(height_m) # Z축 (상하) 크기

            self.vis_pub.publish(self.cv_bridge.cv2_to_imgmsg(vis, "bgr8"))
            self.object_pub.publish(object_info_msg)

            # 캐시 병합: 기존 캐시 유지 + 새로운 hits 업데이트 (끊김 방지)
            for key, data in new_hits.items():
                self.hits_cache[key] = data

            # 오래된 캐시 삭제 (15프레임 이상 안 본 것 제거)
            self.hits_cache = {k: v for k, v in self.hits_cache.items()
                        if self.frame_count - v['last_frame'] <= 15}

        t = time.time()
        if t - self.last_log > 2.0:
            print("[DBG] frame_score=%.3f" % frame_score, "mask_mean=%.1f%%" % (float(mask.mean()) * 100 if 'mask' in locals() else 0.0))
            self.last_log = t

    def __load_onnx(self, onnx_path):
        providers = ['CUDAExecutionProvider','CPUExecutionProvider'] if 'CUDAExecutionProvider' in ort.get_available_providers() else ['CPUExecutionProvider']

        sess = ort.InferenceSession(onnx_path, providers=providers)
        iname = sess.get_inputs()[0].name
        onames = [o.name for o in sess.get_outputs()]

        print("[ONNX] inputs:", [i.name for i in sess.get_inputs()])
        print("[ONNX] outputs:", onames)

        return sess, iname, onames

    def __build_stereo(self, sgbm=True, min_disp=0, num_disp=128, block=5, fast_mode=False):
        num_disp = max(16, num_disp // 16 * 16)
        if sgbm:
            # 개선된 SGBM 파라미터 (더 정교한 depth)
            P1 = 8 * 3 * (block ** 2)
            P2 = 32 * 3 * (block ** 2)
            left_matcher = cv2.StereoSGBM_create(
                minDisparity=min_disp,
                numDisparities=num_disp,
                blockSize=block,
                P1=P1,
                P2=P2,
                disp12MaxDiff=1,
                uniquenessRatio=5,  # 8→5: 더 많은 매칭 허용 (노이즈↑, 밀도↑)
                speckleWindowSize=150,  # 100→150: 더 강한 노이즈 제거
                speckleRange=2,
                preFilterCap=63,  # 명시적 설정
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )

            # WLS 필터용 right matcher 생성 (fast_mode에서는 비활성화)
            right_matcher = None if fast_mode else cv2.ximgproc.createRightMatcher(left_matcher)
            return (left_matcher, right_matcher)

        else:
            bm = cv2.StereoBM_create(numDisparities=num_disp, blockSize=max(5, block|1))
            bm.setSpeckleWindowSize(100); bm.setSpeckleRange(2); bm.setUniquenessRatio(10); bm.setMinDisparity(min_disp)
            return (bm, None)

    def __infer_onnx(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = np.transpose(rgb, (2,0,1))[None, ...]  # 1x3xHxW

        outs = self.sess.run(None, {self.iname: x})
        sizes = [int(np.prod(np.array(y).shape)) for y in outs]

        heat_idx = int(np.argmax(sizes))
        amap = np.array(outs[heat_idx]).squeeze().astype(np.float32)

        score_idx = int(np.argmin(sizes))
        score_arr = np.array(outs[score_idx]).squeeze()

        if score_arr.size == 1 and np.isfinite(score_arr).all():
            frame_score = float(score_arr)

        else:
            frame_score = float(np.nanmean(amap))

        return amap, frame_score

    def __normalize01(self, x):
        mn, mx = float(np.nanmin(x)), float(np.nanmax(x))
        if not np.isfinite(mn) or not np.isfinite(mx) or (mx-mn) < 1e-8:
            return np.zeros_like(x, dtype=np.float32)
        
        return ((x - mn)/(mx - mn)).astype(np.float32)

    def __compute_disparity(self, left_bgr, right_bgr, stereo, fast_mode=False):
        left_matcher, right_matcher = stereo

        # 그레이스케일 변환
        L = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        R = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)

        if fast_mode:
            # 경량화 모드: 간단한 전처리만
            L = cv2.equalizeHist(L)
            R = cv2.equalizeHist(R)
        else:
            # 고품질 모드: CLAHE + Gaussian blur
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            L = clahe.apply(L)
            R = clahe.apply(R)
            L = cv2.GaussianBlur(L, (5,5), 0.8)
            R = cv2.GaussianBlur(R, (5,5), 0.8)

        # Disparity 계산
        disp_left = left_matcher.compute(L, R).astype(np.float32)

        # WLS 필터 적용 (fast_mode에서는 스킵)
        if not fast_mode and right_matcher is not None:
            try:
                disp_right = right_matcher.compute(R, L).astype(np.float32)
                wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
                wls_filter.setLambda(8000.0)  # smoothness
                wls_filter.setSigmaColor(1.5)  # edge sensitivity
                disp = wls_filter.filter(disp_left, L, disparity_map_right=disp_right)
            except:
                # WLS 실패 시 fallback
                disp = disp_left
        else:
            disp = disp_left

        # 후처리
        disp = disp.astype(np.float32)
        disp[disp <= 0] = np.nan

        if fast_mode:
            # 경량화 모드: 빠른 median blur
            disp_for_filter = np.nan_to_num(disp, nan=0)
            disp = cv2.medianBlur(disp_for_filter.astype(np.float32), 5)
            disp[disp <= 0] = np.nan
        else:
            # 고품질 모드: bilateral filter (edge 보존)
            valid_mask = np.isfinite(disp).astype(np.uint8)
            disp_for_filter = np.nan_to_num(disp, nan=0)
            disp_filtered = cv2.bilateralFilter(disp_for_filter, 5, 75, 75)
            disp = np.where(valid_mask, disp_filtered, np.nan)

        return disp

    def __normalize_disparity(self, disp, disp_min=0.3, disp_max=256.0):
        d = disp.astype(np.float32)
        d[d <= 0] = np.nan
        
        med_abs = float(np.nanmedian(np.abs(d)))
        if med_abs > 64.0:  # 보통 *16 스케일
            d = d / 16.0
        
        med = float(np.nanmedian(d))
        if np.isfinite(med) and med < 0:  # 부호가 대부분 음수면 반전
            d = -d

        d[(d < disp_min) | (d > disp_max)] = np.nan
        
        return d

    def __make_mask(self, norm_map01, method="percentile", value=98.8):
        if method == "otsu":
            m8 = (norm_map01 * 255).astype(np.uint8)
            _, th = cv2.threshold(m8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            return (th > 0).astype(np.uint8)
        
        elif method == "abs":
            thr = float(value)
            return (norm_map01 >= thr).astype(np.uint8)
        
        else:
            thr = np.percentile(norm_map01, float(value))
            return (norm_map01 >= thr).astype(np.uint8)

    def __box_score_from_roi(self, roi, method="p95"):
        if roi.size == 0: return 0.0
        if method == "max": return float(np.nanmax(roi))
        if method == "p90": return float(np.nanquantile(roi, 0.90))
        if method == "p95": return float(np.nanquantile(roi, 0.95))
        return float(np.nanmean(roi))

if __name__ == "__main__":
    anom_depth_ros = AnomDepthROS()
    rospy.spin()
