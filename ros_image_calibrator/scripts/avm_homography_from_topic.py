#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, yaml
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def load_camera_info_yaml(path):
    with open(path,'r') as f:
        y = yaml.safe_load(f)
    K = np.array(y['K'], dtype=np.float64).reshape(3,3)
    D = np.array(y['D'], dtype=np.float64).reshape(-1,1)
    w, h = int(y['image_width']), int(y['image_height'])
    return K, D, (w,h), y

def ensure_dir(p):
    if not os.path.exists(p):
        os.makedirs(p)

def find_chess(gray, cols, rows):
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok: return False, None
    corners = cv2.cornerSubPix(
        gray, corners, (3,3), (-1,-1),
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    )
    return True, corners

def make_board_xy(cols, rows, square_m):
    # (0,0) ~ (cols-1, rows-1) 그리드의 (Xw,Yw) [m]
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1,2).astype(np.float32)
    xy = grid * float(square_m)
    return xy  # (N,2)

def image_once(topic, timeout):
    br = CvBridge()
    msg = rospy.wait_for_message(topic, Image, timeout=timeout)
    img = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    return img, msg.header

def init_maps(camera_model, K, D, imsize):
    size = (int(imsize[0]), int(imsize[1]))
    if camera_model == "fisheye":
        R = np.eye(3)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R, K, size, cv2.CV_32FC1)
    else:
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_32FC1)
    return map1, map2

def compute_homography(chess_uv, board_xy_m, bv_cfg, offset_m):
    """
    chess_uv : Nx2, undistorted image points (u,v)
    board_xy_m : Nx2, real XY on ground plane (meters), origin at board (0,0)
    bv_cfg : dict { width,height, meters_per_pixel, center_px }
    offset_m : [ox, oy] meters (where board origin sits relative to vehicle center)
    """
    cx, cy = bv_cfg['center_px']
    mpp = float(bv_cfg['meters_per_pixel'])
    ox, oy = float(offset_m[0]), float(offset_m[1])

    # (Xw,Yw)[m] -> target bird-view pixel (U,V)
    Xw = board_xy_m[:,0] + ox
    Yw = board_xy_m[:,1] + oy
    U = cx + (Xw / mpp)
    V = cy - (Yw / mpp)   # 화면에서 위가 -Y
    dst = np.stack([U,V], axis=1).astype(np.float32)

    src = chess_uv.astype(np.float32)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
    inliers = int(mask.sum()) if mask is not None else 0
    return H, inliers

def main():
    rospy.init_node("avm_homography_from_topic", anonymous=False)

    # --- load params ---
    image_topic = rospy.get_param("/image_topic", "/camera/image_raw")
    camera_model = rospy.get_param("/camera_model", "fisheye")
    camera_info_yaml = rospy.get_param("/camera_info_yaml")

    cols = int(rospy.get_param("/board/cols"))
    rows = int(rospy.get_param("/board/rows"))
    square_m = float(rospy.get_param("/board/square_size_m"))

    bv_cfg = rospy.get_param("/birdview")
    offset_m = rospy.get_param("/offset_m", [0.0, 0.0])

    timeout_sec = int(rospy.get_param("/capture/timeout_sec", 120))
    show_preview = bool(rospy.get_param("/capture/show_preview", True))

    out_dir = rospy.get_param("/output/save_dir", "/tmp/avm")
    name = rospy.get_param("/output/name", "front")
    avm_yaml_path = rospy.get_param("/output/avm_yaml_path", os.path.join(out_dir, "avm_result.yaml"))

    if not camera_info_yaml or not os.path.exists(camera_info_yaml):
        rospy.logerr("camera_info_yaml not found: %s", str(camera_info_yaml))
        raise RuntimeError("Missing camera_info_yaml")

    # --- load camera info + maps ---
    K, D, imsize, caminfo_raw = load_camera_info_yaml(camera_info_yaml)
    map1, map2 = init_maps(camera_model, K, D, imsize)

    # --- grab one frame with visible chessboard (retry until timeout) ---
    t0 = time.time()
    img = None; header = None
    uv = None
    while (time.time()-t0) < timeout_sec and not rospy.is_shutdown():
        try:
            frame, header = image_once(image_topic, timeout=5.0)
        except Exception:
            continue
        und = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        ok, corners = find_chess(gray, cols, rows)
        if ok:
            uv = corners.reshape(-1,2)
            img = und
            if show_preview:
                vis = und.copy()
                cv2.drawChessboardCorners(vis, (cols,rows), corners, True)
                cv2.imshow("chess(undist)", vis); cv2.waitKey(1)
            break

    if img is None or uv is None:
        raise RuntimeError("Failed to detect chessboard in undistorted image within timeout.")

    # --- build board XY (meters) ---
    board_xy = make_board_xy(cols, rows, square_m)

    # --- compute homography image->birdview ---
    H, inliers = compute_homography(uv, board_xy, bv_cfg, offset_m)
    if H is None:
        raise RuntimeError("Homography computation failed.")
    rospy.loginfo("Homography inliers: %d / %d", inliers, board_xy.shape[0])

    # --- warp sample and save outputs ---
    ensure_dir(out_dir)
    BVW, BVH = int(bv_cfg['width']), int(bv_cfg['height'])
    bird = cv2.warpPerspective(img, H, (BVW, BVH), flags=cv2.INTER_LINEAR)
    cv2.imwrite(os.path.join(out_dir, "{}_bird.png".format(name)), bird)
    cv2.imwrite(os.path.join(out_dir, "{}_undist.png".format(name)), img)

    # cumulative avm yaml (append/update this camera's H)
    avm_out = {}
    if os.path.exists(avm_yaml_path):
        with open(avm_yaml_path,'r') as f:
            try: avm_out = yaml.safe_load(f) or {}
            except Exception: avm_out = {}

    avm_out['birdview'] = {
        'width': BVW,
        'height': BVH,
        'meters_per_pixel': float(bv_cfg['meters_per_pixel']),
        'center_px': [int(bv_cfg['center_px'][0]), int(bv_cfg['center_px'][1])]
    }
    # 이름별로 H 저장 (front/left/right/rear 등)
    if 'homographies' not in avm_out: avm_out['homographies'] = {}
    avm_out['homographies'][name] = H.reshape(-1).astype(float).tolist()

    # 참조용으로 이 카메라의 CameraInfo 경로도 기록
    if 'camera_info_files' not in avm_out: avm_out['camera_info_files'] = {}
    avm_out['camera_info_files'][name] = camera_info_yaml

    with open(avm_yaml_path, 'w') as f:
        yaml.safe_dump(avm_out, f, default_flow_style=False, sort_keys=False)

    rospy.loginfo("Saved bird-view sample → %s", os.path.join(out_dir, "{}_bird.png".format(name)))
    rospy.loginfo("Updated AVM YAML       → %s", avm_yaml_path)
    rospy.loginfo("Done.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr("ERROR: %s", str(e))
        raise
