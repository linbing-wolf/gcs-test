# -*- coding: utf-8 -*-
"""
calib_distort.py —— 镜头畸变标定（PC 上跑，与比赛无关）

输入：calib/*.jpg（棋盘格照片，覆盖画面四角+中心+不同倾角）
输出：cam_K_dist.py（可直接拷进 maixcam_app_v2/config.py）

用法：
    python calib_distort.py                 # 默认 calib/ 目录, 9x6 内角点, 25mm
    python calib_distort.py --dir shots --cols 9 --rows 6 --square 25
"""
import argparse
import glob
import os

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="calib")
    ap.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数")
    ap.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数")
    ap.add_argument("--square", type=float, default=25.0, help="方格物理边长(mm)")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square

    obj_pts, img_pts, used = [], [], 0
    for f in sorted(glob.glob(os.path.join(args.dir, "*.jpg")) +
                    glob.glob(os.path.join(args.dir, "*.png"))):
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not ok:
            print("  跳过(未找到角点):", f)
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_pts.append(objp)
        img_pts.append(corners)
        used += 1

    if used < 8:
        print("有效图片太少(%d)，至少 8 张。请重拍。" % used)
        return

    h, w = gray.shape
    ret, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
    print("=" * 56)
    print("有效图片: %d，分辨率: %dx%d" % (used, w, h))
    print("重投影误差: %.4f px  (越小越好, <0.5 合格)" % ret)
    print("K =\n", K)
    print("dist =", dist.ravel())
    print("=" * 56)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cam_K_dist.py")
    with open(out, "w", encoding="utf-8") as fp:
        fp.write("# 自动生成：镜头内参/畸变，粘进 maixcam_app_v2/config.py\n")
        fp.write("UNDISTORT_ON = True\n")
        fp.write("CAM_K = [%s]\n" % ", ".join("%.8f" % v for v in K.ravel()))
        fp.write("CAM_DIST = [%s]   # k1,k2,p1,p2,k3\n"
                 % ", ".join("%.8f" % v for v in dist.ravel()))
    print("已写出:", out)


if __name__ == "__main__":
    main()