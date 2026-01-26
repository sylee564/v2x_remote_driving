#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, yaml, csv, math
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===================== utils =====================
def ensure_dir(p):
    if p and not os.path.exists(p):
        os.makedirs(p)

# numpy -> python 기본형
def _to_py(o):
    import numpy as _np
    if isinstance(o, (_np.generic,)): return o.item()
    if isinstance(o, _np.ndarray):    return [_to_py(x) for x in o.tolist()]
    if isinstance(o, (list, tuple)):  return [_to_py(x) for x in o]
    if isinstance(o, dict):           return {k:_to_py(v) for k,v in o.items()}
    return o

def save_yaml(path, data):
    ensure_dir(os.path.dirname(path))
    with open(path, 'w') as f:
        yaml.safe_dump(_to_py(data), f, default_flow_style=False, sort_keys=False)

_bridge = CvBridge()
def _pub_img(pub, img, frame_id="calib"):
    if pub is None: return
    msg = _bridge.cv2_to_imgmsg(img, encoding="bgr8")
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    pub.publish(msg)

def put_text(img, text, line=0, color=(255,255,255), scale=0.6, thick=2):
    org = (10, 26 + 22*line)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

# ================= chessboard ===================
def find_chess(gray, cols, rows, use_sb=False):
    if use_sb and hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
        if ok:
            return True, corners.reshape(-1,1,2).astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok: return False, None
    corners = cv2.cornerSubPix(
        gray, corners, (3,3), (-1,-1),
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    )
    return True, corners

def make_objpoints(cols, rows, square_m):
    objp = np.zeros((1, cols*rows, 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2)
    objp *= float(square_m)
    return objp  # (1,N,3)

# ============ diversity-aware selector ===========
class DiversitySelector:
    """
    위치(그리드), 스케일(크기), 회전(기울기) 버킷으로 프레임 선별.
    - 위치: 이미지 (grid_h x grid_w) 그리드의 '코너 중심'이 차지한 셀
    - 스케일: 코너 bbox의 대각 길이 / 이미지 대각 (작/중/대)
    - 회전: 체스보드 '가로 방향' 주축의 각도 (0~180) 를 bins로
    각 버킷별로 quota를 채워 target_frames에 도달.
    """
    def __init__(self, img_wh, grid_w=3, grid_h=3,
                 scale_edges=(0.22, 0.42),  # small <0.22 < mid <0.42 < large
                 rot_bins=6, quota_grid=1, quota_scale=6, quota_rot=6):
        self.W, self.H = img_wh
        self.grid_w, self.grid_h = grid_w, grid_h
        self.scale_edges = scale_edges
        self.rot_bins = rot_bins

        # quotas: 최소 충족 개수 (너무 빡세면 필요한 만큼만 만족하면 넘어감)
        self.quota_grid = quota_grid   # 각 grid cell당 최소 몇 장
        self.quota_scale = quota_scale # scale bin당 최소 몇 장
        self.quota_rot = quota_rot     # rotation bin당 최소 몇 장

        # counters
        self.grid_counts = np.zeros((grid_h, grid_w), dtype=int)
        self.scale_counts = np.zeros(3, dtype=int)  # small/mid/large
        self.rot_counts = np.zeros(rot_bins, dtype=int)

    def _centroid_cell(self, corners):
        c = corners.reshape(-1,2).mean(axis=0)
        cx, cy = float(c[0]), float(c[1])
        ix = min(self.grid_w-1, max(0, int(cx / self.W * self.grid_w)))
        iy = min(self.grid_h-1, max(0, int(cy / self.H * self.grid_h)))
        return iy, ix

    def _scale_bin(self, corners):
        pts = corners.reshape(-1,2)
        xmin, ymin = pts[:,0].min(), pts[:,1].min()
        xmax, ymax = pts[:,0].max(), pts[:,1].max()
        diag = math.hypot(xmax-xmin, ymax-ymin)
        maxdiag = math.hypot(self.W, self.H)
        r = diag / maxdiag
        if r < self.scale_edges[0]: return 0  # small
        if r < self.scale_edges[1]: return 1  # mid
        return 2                                # large

    def _rotation_bin(self, corners, cols, rows):
        # 체스보드 첫줄(가로) 방향벡터로 회전 추정
        pts = corners.reshape(rows, cols, 2)
        p0 = pts[0,0]; p1 = pts[0,min(1, cols-1)]
        v = p1 - p0
        ang = math.degrees(math.atan2(v[1], v[0])) % 180.0  # [0,180)
        bin_w = 180.0 / self.rot_bins
        b = int(ang // bin_w)
        return min(self.rot_bins-1, max(0, b))

    def want(self, corners, cols, rows):
        """ 프레임을 채택할지 판단하고 점수 반환(높을수록 선호) """
        gy, gx = self._centroid_cell(corners)
        sb = self._scale_bin(corners)
        rb = self._rotation_bin(corners, cols, rows)

        # 각 버킷이 quota 미달이면 점수 가중치↑
        score = 0.0
        if self.grid_counts[gy,gx] < self.quota_grid:
            score += 2.0
        if self.scale_counts[sb] < self.quota_scale:
            score += 1.5
        if self.rot_counts[rb] < self.quota_rot:
            score += 1.5

        # 추가 가중치: 화면 가장자리 가까울수록 +, 스케일 클수록 +
        pts = corners.reshape(-1,2)
        c = pts.mean(axis=0)
        edge_dist = min(c[0], c[1], self.W-c[0], self.H-c[1]) / min(self.W,self.H)
        score += (0.6 - edge_dist)  # 가장자리에 가까우면 score up

        return score, (gy,gx,sb,rb)

    def commit(self, key):
        gy,gx,sb,rb = key
        self.grid_counts[gy,gx] += 1
        self.scale_counts[sb] += 1
        self.rot_counts[rb] += 1

# ============ reporting (CSV only, optional) ============
def per_frame_reproj_report(images, objpoints_list, imgpoints_list, K, D, model, out_dir, save_csv=True):
    if not save_csv: return None
    ensure_dir(out_dir)
    csv_path = os.path.join(out_dir, "reprojection_report.csv")
    rows_csv=[]
    for i,(img,objp,imgp) in enumerate(zip(images, objpoints_list, imgpoints_list)):
        objp_use = objp[0] if (objp.ndim==3 and objp.shape[0]==1) else objp
        if model=="fisheye":
            rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
            proj,_ = cv2.fisheye.projectPoints(objp_use.astype(np.float64), rvec, tvec, K, D)
        else:
            ok_pnp, rvec, tvec = cv2.solvePnP(objp_use, imgp, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok_pnp:
                rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
            proj,_ = cv2.projectPoints(objp_use, rvec, tvec, K, D)
        err = np.linalg.norm(proj.reshape(-1,2)-imgp.reshape(-1,2), axis=1)
        rows_csv.append([i, float(np.mean(err)), float(np.median(err)), float(np.max(err)), len(err)])
    with open(csv_path,"w",newline="") as f:
        w=csv.writer(f); w.writerow(["frame_idx","mean_px","median_px","max_px","N"])
        w.writerows(rows_csv)
    return csv_path

# ============ YAML (requested format) ============
def make_camera_info_yaml_requested_format(K, D, imsize, camera_name, model):
    K = np.array(K, dtype=float).reshape(3,3)
    D = np.array(D, dtype=float).reshape(-1)
    if model=='fisheye':
        distortion_model='fisheye'; D_data=(D.tolist()+[0.0]*4)[:4]; dcols=4
    elif model=='rational':
        distortion_model='rational_polynomial'; D_data=(D.tolist()+[0.0]*8)[:8]; dcols=8
    else:
        distortion_model='plumb_bob'; D_data=(D.tolist()+[0.0]*5)[:5]; dcols=5
    fx,fy = float(K[0,0]), float(K[1,1]); cx,cy = float(K[0,2]), float(K[1,2])
    P = [fx,0.0,cx,0.0, 0.0,fy,cy,0.0, 0.0,0.0,1.0,0.0]
    return {
        'image_width':  int(imsize[0]),
        'image_height': int(imsize[1]),
        'camera_name':  str(camera_name),
        'camera_matrix': {'rows':3,'cols':3,'data':[float(x) for x in K.reshape(-1).tolist()]},
        'distortion_model': distortion_model,
        'distortion_coefficients': {'rows':1,'cols':dcols,'data':[float(x) for x in D_data]},
        'rectification_matrix': {'rows':3,'cols':3,'data':[1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]},
        'projection_matrix': {'rows':3,'cols':4,'data':[float(x) for x in P]}
    }

# ============ calibration backends ============
def calibrate_fisheye(images, cols, rows, square_m):
    objp = make_objpoints(cols, rows, square_m)
    objpoints, imgpoints, used_imgs = [], [], []
    imsize=None
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if imsize is None: imsize = gray.shape[::-1]
        ok, corners = find_chess(gray, cols, rows)
        if not ok: continue
        objpoints.append(objp); imgpoints.append(corners); used_imgs.append(img)
    if len(imgpoints)<8: raise RuntimeError("Not enough detections (>=8).")
    K=np.zeros((3,3)); D=np.zeros((4,1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
    rms,_,_,_,_ = cv2.fisheye.calibrate(objpoints, imgpoints, imsize, K, D, None, None, flags, (3,1e-6))
    return K,D,imsize,rms,used_imgs,objp, imgpoints

def calibrate_pinhole(images, cols, rows, square_m):
    objp = make_objpoints(cols, rows, square_m)[0]
    objpoints, imgpoints, used_imgs=[],[],[]
    imsize=None
    for img in images:
        gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if imsize is None: imsize=gray.shape[::-1]
        ok,corners = find_chess(gray, cols, rows)
        if not ok: continue
        objpoints.append(objp); imgpoints.append(corners); used_imgs.append(img)
    if len(imgpoints)<8: raise RuntimeError("Not enough detections (>=8).")
    ret,K,D,_,_ = cv2.calibrateCamera(objpoints, imgpoints, imsize, None, None)
    if D.size>5: D=D[:,:5]
    return K,D,imsize,ret,used_imgs,objp, imgpoints

def calibrate_rational(images, cols, rows, square_m):
    objp = make_objpoints(cols, rows, square_m)[0]
    objpoints, imgpoints, used_imgs=[],[],[]
    imsize=None
    for img in images:
        gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if imsize is None: imsize=gray.shape[::-1]
        ok,corners = find_chess(gray, cols, rows)
        if not ok: continue
        objpoints.append(objp); imgpoints.append(corners); used_imgs.append(img)
    if len(imgpoints)<8: raise RuntimeError("Not enough detections (>=8).")
    flags=cv2.CALIB_RATIONAL_MODEL
    ret,K,D,_,_ = cv2.calibrateCamera(objpints:=objpoints, imgpoints, imsize, None, None, flags=flags)
    if D.size>=8: D=D[:,:8]
    else:
        d=np.zeros((1,8), dtype=np.float64); d[0,:D.size]=D.ravel(); D=d
    return K,D,imsize,ret,used_imgs,objp, imgpoints

# ============ collection with guide enforcement ============
def collect_frames(topic, cols, rows, target_frames, timeout_sec, min_span_ratio, show_preview,
                   pub_raw=None, pub_corners=None, use_sb=False, show_candidates=True,
                   grid_w=3, grid_h=3, scale_edges=(0.22,0.42), rot_bins=6,
                   quota_grid=1, quota_scale=6, quota_rot=6):
    br=_bridge
    last_msg=[None]
    def cb(m): last_msg[0]=m
    sub=rospy.Subscriber(topic, Image, cb, queue_size=1)

    # warm-up frame to get size
    rospy.loginfo("Collecting from topic: %s", topic)
    t0=time.time(); first=None
    while not rospy.is_shutdown() and first is None and (time.time()-t0)<5.0:
        if last_msg[0] is not None: first=br.imgmsg_to_cv2(last_msg[0], "bgr8")
        else: rospy.sleep(0.01)
    if first is None:
        sub.unregister(); raise RuntimeError("No image received.")

    H,W = first.shape[:2]
    selector = DiversitySelector((W,H), grid_w, grid_h, scale_edges, rot_bins,
                                 quota_grid, quota_scale, quota_rot)

    kept_imgs=[]; kept_meta=[]
    kept=0; fps=0.0; tt=time.time()

    while not rospy.is_shutdown():
        if last_msg[0] is None:
            if time.time()-t0 > timeout_sec: break
            rospy.sleep(0.005); continue

        now=time.time(); fps = 0.9*fps + 0.1*(1.0/max(1e-6, now-tt)); tt=now
        img = br.imgmsg_to_cv2(last_msg[0], "bgr8")

        # always publish raw
        try: _pub_img(pub_raw, img)
        except: pass

        gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok,corners = find_chess(gray, cols, rows, use_sb=use_sb)

        vis = img.copy()
        status = ""
        if ok:
            xs, ys = corners[:,:,0], corners[:,:,1]
            span_x = float(xs.max()-xs.min())/W
            span_y = float(ys.max()-ys.min())/H
            cv2.drawChessboardCorners(vis, (cols,rows), corners, True)

            # diversity decision
            score, key = selector.want(corners, cols, rows)
            need_span = max(span_x, span_y) >= min_span_ratio
            if need_span and score>0.2:
                kept_imgs.append(img.copy()); kept_meta.append((corners, key))
                selector.commit(key); kept += 1
                status = f"KEEP ({kept}/{target_frames}) | span=({span_x:.2f},{span_y:.2f})"
            else:
                why=[]
                if not need_span: why.append("span")
                if score<=0.2:   why.append("diversity")
                status = "SKIP: need " + "/".join(why)
        else:
            status = "NO CHESSBOARD"

        # overlay HUD
        put_text(vis, status, line=0, color=(0,255,0) if status.startswith("KEEP") else (0,0,255))
        put_text(vis, f"FPS:{fps:4.1f}", line=1)
        put_text(vis, f"grid {selector.grid_counts.sum()}/{selector.grid_w*selector.grid_h*selector.quota_grid} "
                      f"scale {selector.scale_counts.tolist()} rot {selector.rot_counts.tolist()}",
                 line=2, color=(255,255,0))
        try: _pub_img(pub_corners, vis)
        except: pass

        if show_preview:
            cv2.imshow("collect(corners)", vis); cv2.waitKey(1)

        if kept >= target_frames or (time.time()-t0)>timeout_sec:
            break

        rospy.sleep(0.001)

    sub.unregister()
    if show_preview: cv2.destroyAllWindows()

    # unpack metas
    imgpoints=[]; objpoints=[]
    for corners, key in kept_meta:
        imgpoints.append(corners)
        # objp는 캘리브레이션 함수에서 다시 만들므로 자리만
        # (여기서는 diversity 메타만 사용)
    return kept_imgs

# ==================== main ======================
def main():
    rospy.init_node("image_calibrator", anonymous=False)

    # params
    image_topic    = rospy.get_param("~image_topic", rospy.get_param("/image_topic", "/camera/image_raw"))
    cols           = int(rospy.get_param("/board/cols"))
    rows           = int(rospy.get_param("/board/rows"))
    square_m       = float(rospy.get_param("/board/square_size_m"))
    target_frames  = int(rospy.get_param("/capture/target_frames", 80))
    timeout_sec    = int(rospy.get_param("/capture/timeout_sec", 300))
    min_span_ratio = float(rospy.get_param("/capture/min_span_ratio", 0.35))
    show_preview   = bool(rospy.get_param("/capture/show_preview", True))
    model          = rospy.get_param("/model", "rational")   # fisheye | pinhole | rational
    camera_name    = rospy.get_param("/output/camera_name", "camera")
    save_dir       = rospy.get_param("/output/save_dir", "/tmp/ros_image_calib")
    debug          = bool(rospy.get_param("/debug", True))
    stream_after   = bool(rospy.get_param("/stream_undist_after_calib", True))
    use_sb         = bool(rospy.get_param("/use_sb_detector", True))
    show_candidates= bool(rospy.get_param("/show_candidates", True))
    save_csv       = bool(rospy.get_param("/save_csv", True))
    width_hint     = int(rospy.get_param("/image_width_hint", 0))
    height_hint    = int(rospy.get_param("/image_height_hint", 0))

    # diversity params
    grid_w         = int(rospy.get_param("~grid_w", 3))
    grid_h         = int(rospy.get_param("~grid_h", 3))
    rot_bins       = int(rospy.get_param("~rot_bins", 6))
    quota_grid     = int(rospy.get_param("~quota_grid", 1))
    quota_scale    = int(rospy.get_param("~quota_scale", 6))
    quota_rot      = int(rospy.get_param("~quota_rot", 6))
    scale_edge_lo  = float(rospy.get_param("~scale_edge_lo", 0.22))
    scale_edge_hi  = float(rospy.get_param("~scale_edge_hi", 0.42))

    if model not in ("fisheye","pinhole","rational"):
        raise ValueError("model must be 'fisheye', 'pinhole', or 'rational'.")

    # debug publishers
    pub_raw     = rospy.Publisher("~debug/raw",     Image, queue_size=1, latch=True) if debug else None
    pub_corners = rospy.Publisher("~debug/corners", Image, queue_size=1, latch=True) if debug else None
    pub_und     = rospy.Publisher("~debug/undist",  Image, queue_size=1, latch=True) if debug else None

    # 1) guided collection
    imgs = collect_frames(
        image_topic, cols, rows, target_frames, timeout_sec, min_span_ratio, show_preview,
        pub_raw=pub_raw, pub_corners=pub_corners, use_sb=use_sb, show_candidates=show_candidates,
        grid_w=grid_w, grid_h=grid_h, scale_edges=(scale_edge_lo, scale_edge_hi),
        rot_bins=rot_bins, quota_grid=quota_grid, quota_scale=quota_scale, quota_rot=quota_rot
    )
    if len(imgs)==0: raise RuntimeError("No frames collected.")
    rospy.loginfo("Collected frames: %d", len(imgs))

    # 2) calibrate
    if model=="fisheye":
        K,D,imsize,rms,used_imgs,objp,imgL = calibrate_fisheye(imgs, cols, rows, square_m)
    elif model=="rational":
        K,D,imsize,rms,used_imgs,objp,imgL = calibrate_rational(imgs, cols, rows, square_m)
    else:
        K,D,imsize,rms,used_imgs,objp,imgL = calibrate_pinhole(imgs, cols, rows, square_m)
    rospy.loginfo("RMS reprojection error: %.4f", rms)

    if width_hint>0 and height_hint>0: imsize=(width_hint,height_hint)

    # 3) (optional) CSV only
    csv_path = per_frame_reproj_report(used_imgs, [objp]*len(imgL), imgL, K, D, model, save_dir, save_csv=save_csv)
    if csv_path: rospy.loginfo("Per-frame reprojection report CSV → %s", csv_path)

    # 4) save YAML (requested format)
    cam_yaml = make_camera_info_yaml_requested_format(K, D, imsize, camera_name, model)
    yaml_path = os.path.join(save_dir, "{}.yaml".format(camera_name))
    save_yaml(yaml_path, cam_yaml)
    rospy.loginfo("Saved CameraInfo YAML → %s", yaml_path)

    # 5) undist streaming
    size=(int(imsize[0]), int(imsize[1]))
    if model=="fisheye":
        map1,map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_32FC1)
    else:
        map1,map2 = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_32FC1)
    if stream_after and debug:
        br=_bridge; last=[None]
        def cb2(m): last[0]=m
        sub2=rospy.Subscriber(image_topic, Image, cb2, queue_size=1)
        rate=rospy.Rate(30); rospy.loginfo("Streaming ~debug/undist ... (Ctrl+C to stop)")
        while not rospy.is_shutdown():
            if last[0] is None: rate.sleep(); continue
            frame=br.imgmsg_to_cv2(last[0], "bgr8")
            und=cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            _pub_img(pub_und, und); rate.sleep()

if __name__=="__main__":
    try: main()
    except Exception as e:
        rospy.logerr("ERROR: %s", str(e)); raise
