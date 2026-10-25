#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time, argparse
import numpy as np
import cv2
import threading
import queue

# ==== ONNX ====
def load_onnx(onnx_path):
    import onnxruntime as ort
    # GPU가 있더라도 onnxruntime-gpu가 없을 수 있으니 안전하게 CPU 우선
    providers = ['CUDAExecutionProvider','CPUExecutionProvider'] if 'CUDAExecutionProvider' in ort.get_available_providers() else ['CPUExecutionProvider']
    sess = ort.InferenceSession(onnx_path, providers=providers)
    iname = sess.get_inputs()[0].name
    onames = [o.name for o in sess.get_outputs()]
    print("[ONNX] inputs:", [i.name for i in sess.get_inputs()])
    print("[ONNX] outputs:", onames)
    return sess, iname, onames

def infer_onnx(sess, iname, bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32)/255.0
    x = np.transpose(rgb, (2,0,1))[None, ...]  # 1x3xHxW
    outs = sess.run(None, {iname: x})
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

# ==== Utils ====
def to_numpy_map(amap):
    return np.squeeze(np.array(amap, dtype=np.float32))

def normalize01(x):
    mn, mx = float(np.nanmin(x)), float(np.nanmax(x))
    if not np.isfinite(mn) or not np.isfinite(mx) or (mx-mn) < 1e-8:
        return np.zeros_like(x, dtype=np.float32)
    return ((x - mn)/(mx - mn)).astype(np.float32)

def make_mask(norm_map01, method="percentile", value=99.2):
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

def box_score_from_roi(roi, method="p95"):
    if roi.size == 0: return 0.0
    if method == "max": return float(np.nanmax(roi))
    if method == "p90": return float(np.nanquantile(roi, 0.90))
    if method == "p95": return float(np.nanquantile(roi, 0.95))
    return float(np.nanmean(roi))

def put_text(img, text, org, color=(0,255,0)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

def display_thread_worker(img_queue, stop_event):
    """별도 스레드에서 cv2.imshow 처리 - GUI 블로킹 방지"""
    while not stop_event.is_set():
        try:
            if not img_queue.empty():
                vis = img_queue.get(timeout=0.01)
                cv2.imshow("anom+depth (ROS)", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                stop_event.set()
                break
        except:
            pass
    cv2.destroyAllWindows()

def build_stereo(sgbm=True, min_disp=0, num_disp=128, block=5, fast_mode=False):
    num_disp = max(16, int(num_disp/16)*16)
    if sgbm:
        # 개선된 SGBM 파라미터 (더 정교한 depth)
        P1=8*3*(block**2)
        P2=32*3*(block**2)
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
        bm=cv2.StereoBM_create(numDisparities=num_disp, blockSize=max(5, block|1))
        bm.setSpeckleWindowSize(100); bm.setSpeckleRange(2); bm.setUniquenessRatio(10); bm.setMinDisparity(min_disp)
        return (bm, None)


def _normalize_disparity(disp, disp_min=0.3, disp_max=256.0):
    import numpy as np
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

def compute_disparity(left_bgr, right_bgr, stereo, fast_mode=False):
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

def depth_from_disparity(disp, fx_px, baseline_m):
    Z = (fx_px * baseline_m) / (disp + 1e-6)
    Z[~np.isfinite(Z)] = np.nan
    return Z

# ==== ROS bridge ====
def decode_compressed(msg):
    # sensor_msgs/CompressedImage -> BGR
    np_arr = np.frombuffer(msg.data, np.uint8)
    bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return bgr

def main():
    ap = argparse.ArgumentParser(description="ROS1 topics → Anomaly+Depth")
    ap.add_argument("--ckpt", required=True, help="model.onnx 경로")
    ap.add_argument("--right_topic", default="/image_jpeg/compressed", help="오른쪽 토픽 (CompressedImage)")
    ap.add_argument("--left_topic",  default="/image_jpeg2/compressed", help="왼쪽 토픽 (CompressedImage)")
    ap.add_argument("--show_heat", action="store_true", help="히트맵 오버레이 표시")
    ap.add_argument("--show_vis", action="store_true", help="박스만 표시 (히트맵 없이)")
    ap.add_argument("--out", default="", help="저장 mp4 경로(옵션)")
    # thresholds (네가 쓰던 값 기본)
    ap.add_argument("--thres_method", default="percentile", choices=["percentile","otsu","abs"])
    ap.add_argument("--thres", type=float, default=99.2)
    ap.add_argument("--score_thres", type=float, default=0.45)
    ap.add_argument("--box_score_method", default="p95", choices=["mean","max","p90","p95"])
    ap.add_argument("--box_score_thres", type=float, default=0.72)
    ap.add_argument("--min_area", type=int, default=1800)
    # stereo
    ap.add_argument("--use_sgbm", action="store_true")
    ap.add_argument("--num_disp", type=int, default=128)
    ap.add_argument("--block", type=int, default=5)
    ap.add_argument("--fx_px", type=float, default=700.0)
    ap.add_argument("--baseline_m", type=float, default=0.20)
    ap.add_argument("--depth_scale", type=float, default=1.0, help="거리 보정 스케일 (실제거리/측정거리, 예: 8m/13m=0.615)")
    ap.add_argument("--disp_min", type=float, default=0.3, help="이 값 이하 시차는 무효(노이즈)")
    ap.add_argument("--disp_max", type=float, default=256.0, help="이 값 이상 시차는 무효(스케일 오검)")
    ap.add_argument("--depth_debug", action="store_true", help="뎁스/시차 통계 로그")
    ap.add_argument("--fast_mode", action="store_true", help="경량화 모드 (WLS/bilateral 필터 비활성화, 속도↑ 품질↓)")
    args = ap.parse_args()

    # ONNX 로드
    sess, iname, onames = load_onnx(args.ckpt)

    # Stereo 준비
    stereo = build_stereo(True if args.use_sgbm or True else False, 0, args.num_disp, args.block, args.fast_mode)

    # 출력 준비는 첫 프레임 들어오면 크기 보고 나중에 열자
    writer = None
    last_log = time.time()
    hits_cache = {}  # (key) -> {'count': hits, 'last_frame': frame_num}
    frame_count = 0
    last_display_time = 0.0

    # Display thread 준비 (GUI 블로킹 방지)
    img_queue = queue.Queue(maxsize=2)
    stop_event = threading.Event()
    if args.show_heat or args.show_vis:
        display_thread = threading.Thread(target=display_thread_worker, args=(img_queue, stop_event), daemon=True)
        display_thread.start()

    # ROS 준비
    import rospy
    from sensor_msgs.msg import CompressedImage
    import message_filters

    rospy.init_node("anom_depth_ros", anonymous=True)

    right_sub = message_filters.Subscriber(args.right_topic, CompressedImage)
    left_sub  = message_filters.Subscriber(args.left_topic,  CompressedImage)
    sync = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.005)
    
    def callback(left_msg, right_msg):
        nonlocal writer, last_log, hits_cache, frame_count, last_display_time
        frame_count += 1
        # decode
        left = decode_compressed(left_msg)
        right = decode_compressed(right_msg)
        if left is None or right is None:
            return
        if left.shape[:2] != right.shape[:2]:
            right = cv2.resize(right, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_LINEAR)
        H, W = left.shape[:2]

        # 추론
        amap, frame_score = infer_onnx(sess, iname, left)
        amap = to_numpy_map(amap)
        if amap.shape[:2] != (H,W):
            amap = cv2.resize(amap, (W,H), interpolation=cv2.INTER_LINEAR)
        amap01 = normalize01(amap)
        amap01 = cv2.GaussianBlur(amap01, (0,0), 1.0)

        vis = left.copy()
        disp = compute_disparity(left, right, stereo, args.fast_mode) if stereo[0] is not None else None
        if disp is not None:
            disp = _normalize_disparity(disp, disp_min=args.disp_min, disp_max=args.disp_max)
            if args.depth_debug:
                import numpy as np
                valid = np.isfinite(disp)
                vr = float(valid.mean())*100.0 if valid.size else 0.0
                d_med = float(np.nanmedian(disp)) if valid.any() else float('nan')
                print(f"[DBG] disp valid={vr:.1f}%  d_med={d_med:.3f}")

        if frame_score >= args.score_thres:
            mask = make_mask(amap01, method=args.thres_method, value=args.thres).astype(np.uint8)
            if mask.any():
                k = np.ones((3,3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

            contours,_ = cv2.findContours((mask*255).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            boxes=[]
            for c in contours:
                if cv2.contourArea(c) < args.min_area: continue
                x,y,w,h = cv2.boundingRect(c)

                # ★★★ Y좌표 필터링: 0~140 범위는 무시 (하늘/먼 배경) ★★★
                box_center_y = y + h // 2
                if box_center_y < 140:
                    continue  # y < 140이면 스킵

                roi = amap01[y:y+h, x:x+w]
                bscore = box_score_from_roi(roi, args.box_score_method)
                if bscore < args.box_score_thres: continue

                depth_median=None
                if disp is not None:
                    d_roi = disp[y:y+h, x:x+w]
                    valid = d_roi[np.isfinite(d_roi) & (d_roi > 0)]
                    if valid.size > 30:
                        d_med = float(np.median(valid))
                        depth_median = (args.fx_px * args.baseline_m) / (d_med + 1e-6) * args.depth_scale
                        if args.depth_debug:
                            print(f"[DBG] ROI d_med={d_med:.3f} -> depth={depth_median:.2f}m")

                # ★★★ 거리 필터링: 8미터 이내만 표시 ★★★
                if depth_median is not None and depth_median > 13.0:
                    continue  # 8m 넘으면 스킵

                boxes.append((x,y,w,h,bscore,depth_median))

            # 개선된 깊이 기반 tracking (끊김 최소화 + 노이즈 필터링)
            new_hits={}
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
                for old_key, old_data in hits_cache.items():
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
                frame_gap = frame_count - best_match_frame
                if frame_gap > 4:
                    # 끊긴 시간만큼 hits 감소
                    best_match_hits = max(0, best_match_hits - (frame_gap - 3))

                new_count = min(999, best_match_hits + 1)
                new_hits[key] = {'count': new_count, 'last_frame': frame_count}
                hits = new_count

                # hits >= 10으로 올림 (노이즈 강력 방지)
                if hits>=10:
                    color=(0,0,255)
                    cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)

                    # 박스 중앙 좌표 계산
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # 각도 계산 (카메라 중심 기준)
                    offset_x = center_x - (W / 2.0)  # 이미지 중심에서의 오프셋
                    angle_rad = np.arctan(offset_x / args.fx_px)  # 라디안
                    angle_deg = angle_rad * (180.0 / np.pi)  # 도(degree)

                    # 박스 위 라벨 (거리 + 각도)
                    if depth_median is not None:
                        label_top = f"{depth_median:.1f}m {angle_deg:+.1f}°"
                    else:
                        label_top = f"{angle_deg:+.1f}°"
                    put_text(vis, label_top, (x, max(0, y-6)), color)

                    # 박스 안쪽 중심 좌표 표시
                    label_center = f"({center_x},{center_y})"
                    cv2.putText(vis, label_center, (center_x-40, center_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2, cv2.LINE_AA)

            # 캐시 병합: 기존 캐시 유지 + 새로운 hits 업데이트 (끊김 방지)
            for key, data in new_hits.items():
                hits_cache[key] = data

            # 오래된 캐시 삭제 (15프레임 이상 안 본 것 제거)
            hits_cache = {k: v for k, v in hits_cache.items()
                         if frame_count - v['last_frame'] <= 15}

        # 히트맵 오버레이 (옵션)
        if args.show_heat:
            heat8=(amap01*255).astype(np.uint8)
            heat=cv2.applyColorMap(heat8, cv2.COLORMAP_JET)
            vis=cv2.addWeighted(vis,0.6,heat,0.4,0)

        # Anomaly score 표시 (히트맵 또는 박스 표시 모드일 때만)
        if args.show_heat or args.show_vis:
            put_text(vis, f"Anomaly score: {frame_score:.3f}", (10,24),
                     (0,255,255) if frame_score<args.score_thres else (255,255,255))

        if writer is None and args.out:
            fourcc=cv2.VideoWriter_fourcc(*"mp4v")
            writer=cv2.VideoWriter(args.out, fourcc, 20.0, (W,H))
        if writer is not None:
            writer.write(vis)

        # 미리보기 (원하면 q 종료) - 별도 스레드로 전송
        if args.show_heat or args.show_vis:
            if img_queue.full():
                img_queue.get()  # 오래된 프레임 버림
            img_queue.put(vis.copy())
            if stop_event.is_set():
                rospy.signal_shutdown("user quit")

        t=time.time()
        if t-last_log>2.0:
            print("[DBG] frame_score=%.3f"%frame_score, "mask_mean=%.1f%%"%(float(mask.mean())*100 if 'mask' in locals() else 0.0))
            last_log=t

    sync.registerCallback(callback)
    print("[INFO] Subscribed:", args.left_topic, args.right_topic)
    print("[INFO] Running... (Ctrl+C to stop)")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if args.show_heat or args.show_vis:
            display_thread.join(timeout=1.0)
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
