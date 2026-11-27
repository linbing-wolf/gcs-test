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
    ap.add_argument("--max-shift", type=float, default=48.0,
                    help="与 MaixCAM 配置一致的去畸变位移保护阈值(px)")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square

    obj_pts, img_pts, board_centers = [], [], []
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
        board_centers.append(corners.reshape(-1, 2).mean(axis=0))

    if len(board_centers) < 8:
        print("有效图片太少(%d)，至少 8 张。请重拍。" % len(board_centers))
        return

    h, w = gray.shape
    ret, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
    all_corners = np.concatenate([pts.reshape(-1, 2) for pts in img_pts], axis=0)
    centers = np.asarray(board_centers)
    u_min, v_min = all_corners.min(axis=0)
    u_max, v_max = all_corners.max(axis=0)

    # 全图网格会检查标定点之外的外推区域。中心附近的低误差不能代表四角和下沿可靠。
    grid_y, grid_x = np.mgrid[0:h:20, 0:w:20]
    grid = np.column_stack((grid_x.ravel(), grid_y.ravel())).astype(np.float32)
    undistorted = cv2.undistortPoints(
        grid.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
    shift = np.linalg.norm(undistorted - grid, axis=1)
    in_frame = ((undistorted[:, 0] >= 0.0) & (undistorted[:, 0] <= w) &
                (undistorted[:, 1] >= 0.0) & (undistorted[:, 1] <= h))
    trusted = in_frame & (shift <= args.max_shift)

    print("=" * 56)
    print("有效图片: %d，分辨率: %dx%d" % (len(board_centers), w, h))
    print("重投影误差: %.4f px  (越小越好, <0.5 合格)" % ret)
    print("角点覆盖: x %.1f..%.1f, y %.1f..%.1f"
          % (u_min, u_max, v_min, v_max))
    print("棋盘中心覆盖: x %.1f..%.1f, y %.1f..%.1f"
          % (centers[:, 0].min(), centers[:, 0].max(),
             centers[:, 1].min(), centers[:, 1].max()))
    print("整图网格预测: %.1f%% 需要回退原坐标，最大位移 %.1f px"
          % ((1.0 - trusted.mean()) * 100.0, shift.max()))
    if v_max < h * 0.90:
        print("警告: 棋盘角点没有覆盖画面下方 10%，下沿参数属于外推。")
    if v_max < h * 0.97:
        print("警告: 未覆盖最底部区域，D/E/H 若在下方工作必须补拍。")
    quadrants = (
        int(np.sum((centers[:, 0] < w / 2) & (centers[:, 1] < h / 2))),
        int(np.sum((centers[:, 0] >= w / 2) & (centers[:, 1] < h / 2))),
        int(np.sum((centers[:, 0] < w / 2) & (centers[:, 1] >= h / 2))),
        int(np.sum((centers[:, 0] >= w / 2) & (centers[:, 1] >= h / 2))),
    )
    print("棋盘中心象限数(左上/右上/左下/右下): %s"
          % "/".join(str(v) for v in quadrants))
    if min(quadrants) == 0:
        print("警告: 至少一个象限没有棋盘中心，四角畸变参数稳定性不足。")
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
