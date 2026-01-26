#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, yaml, numpy as np, cv2, rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def load_cam_yaml(path):
    with open(path,'r') as f: y = yaml.safe_load(f)
    K = np.array(y['K']).reshape(3,3).astype(np.float64)
    D = np.array(y['D']).reshape(1,-1).astype(np.float64)
    model = y.get('distortion_model','plumb_bob')
    w, h = int(y['image_width']), int(y['image_height'])
    return K, D, model, (w,h)

def chess(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols,rows), flags)
    if not ok: return False, None
    corners = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1),
                               (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.1))
    return True, corners

def undist_maps(K, D, model, size):
    if model=='fisheye':
        return cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_32FC1)
    else:
        return cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_32FC1)

def main():
    rospy.init_node("verify_calibration_from_topic", anonymous=False)
    topic = rospy.get_param("~image_topic", "/camera/image_raw")
    cam_yaml = rospy.get_param("~camera_info_yaml", "/home/lsy/ros_image_calib/camera.yaml")
    cols = int(rospy.get_param("~cols", 9))
    rows = int(rospy.get_param("~rows", 6))
    square_m = float(rospy.get_param("~square_size_m", 0.04))
    show = bool(rospy.get_param("~show_preview", True))

    K, D, model, (W,H) = load_cam_yaml(cam_yaml)
    map1, map2 = undist_maps(K, D, model, (W,H))
    br = CvBridge()
    print("[verify] model:", model, "image:", (W,H))

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message(topic, Image, timeout=5.0)
        img = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = chess(gray, cols, rows)
        if not ok:
            if show:
                cv2.imshow("raw", img); cv2.waitKey(1)
            continue

        # reprojection error via PnP (non-fisheye)
        objp = np.zeros((cols*rows,3), np.float32)
        grid = np.mgrid[0:cols,0:rows].T.reshape(-1,2).astype(np.float32)
        objp[:,0:2] = grid * square_m

        if model == 'fisheye':
            rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
            proj, _ = cv2.fisheye.projectPoints(objp.astype(np.float64), rvec, tvec, K, D)
        else:
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok_pnp:
                rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
            proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)

        proj2 = proj.reshape(-1,2); det2 = corners.reshape(-1,2)
        err = np.linalg.norm(proj2 - det2, axis=1)
        mean_e, med_e = float(np.mean(err)), float(np.median(err))
        print("[verify] reproj px: mean={:.3f}, median={:.3f}".format(mean_e, med_e))

        und = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        if show:
            vis1 = img.copy()
            cv2.drawChessboardCorners(vis1, (cols,rows), corners, True)
            for p in proj2.astype(int):
                cv2.circle(vis1, tuple(p), 2, (0,255,0), -1)
            cv2.imshow("raw + reproj(green)", vis1)
            cv2.imshow("undist", und)
            if cv2.waitKey(1)==27: break

if __name__ == "__main__":
    main()
