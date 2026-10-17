# -*- coding: utf-8 -*-
"""
calib_homography.py —— 手眼标定：像素 → 毫米（PC 上跑，与比赛无关）

输入：一张照片里若干"已知物理坐标"的标记点的【像素坐标】与【mm 坐标】
输出：homography.py（拷进 maixcam_app_v2/config.py）

用法：
  1) 台面摆点位，量出每个点的 mm 坐标（原点/轴向要和 STM32 一致）
  2) 拍照，读出这些点在图里的像素坐标
  3) 填进下面的 PX / MM 两个列表（至少 4 组，建议 9 组）
  4) python calib_homography.py
"""
import os

import cv2
import numpy as np

# ============================ 按你现场填 ============================
# 像素坐标 (u, v)
PX = [[120, 340], [520, 338], [118, 110], [522, 112],
      [320, 225], [320, 110], [320, 340], [120, 225], [520, 225]]
# 对应的物理坐标 (Xmm, Ymm)
MM = [[0, 0], [400, 0], [0, 230], [400, 230],
      [200, 115], [200, 230], [200, 0], [0, 115], [400, 115]]
# ===================================================================


def main():
    px = np.float32(PX)
    mm = np.float32(MM)
    H, mask = cv2.findHomography(px, mm)
    if H is None:
        print("求解失败：点太少或共线")
        return

    back = cv2.perspectiveTransform(px.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.abs(back - mm)
    print("=" * 56)
    print("H =\n", H)
    print("最大反算误差: %.3f mm" % err.max())
    print("平均反算误差: %.3f mm" % err.mean())
    print("=" * 56)
    if err.max() > 3.0:
        print("[!] 误差偏大，检查点位是否量准、是否共面、像素是否读对")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "homography.py")
    with open(out, "w", encoding="utf-8") as fp:
        fp.write("# 自动生成：手眼标定单应矩阵，粘进 maixcam_app_v2/config.py\n")
        fp.write("HOMOGRAPHY_ON = True\n")
        fp.write("HOMOGRAPHY = [%s]\n" % ", ".join("%.10f" % v for v in H.ravel()))
    print("已写出:", out)


if __name__ == "__main__":
    main()