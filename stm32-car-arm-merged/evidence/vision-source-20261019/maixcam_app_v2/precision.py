# -*- coding: utf-8 -*-
"""
precision.py —— 通用视觉精度库（与本比赛流程无关，可复用到任何 MaixCAM 项目）

提供四件套：
  1) CoordScaler     高分辨率检测 → 对外口径（640x480）坐标缩放
  2) DistortionModel 镜头畸变：坐标层反畸变（纯 Python 迭代，无 numpy/cv2 依赖）
  3) Homography      像素 → 毫米（手眼标定结果应用）
  4) fit_circle      圆的最小二乘亚像素拟合
  5) CenterFilter    多帧平均（降抖动）

设计原则：纯 Python、无第三方依赖，既能上板跑，也能在 PC 上单测。
"""

import math


# ======================================================================
# 1) 坐标缩放：高分辨率检测 → 对外口径
# ======================================================================
class CoordScaler:
    def __init__(self, det_w, det_h, out_w, out_h):
        self.sx = float(out_w) / float(det_w)
        self.sy = float(out_h) / float(det_h)

    def map(self, x, y):
        return x * self.sx, y * self.sy

    def map_roi(self, roi):
        if not roi:
            return []
        x, y, w, h = roi
        return [int(x * self.sx), int(y * self.sy), int(w * self.sx), int(h * self.sy)]


# ======================================================================
# 2) 镜头畸变：坐标层反畸变
#    正向模型（OpenCV 口径）：
#      x_d = x(1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x y + p2 (r^2 + 2 x^2)
#      y_d = y(1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y^2) + 2 p2 x y
#    反解用不动点迭代（对单个点足够快）。
# ======================================================================
class DistortionModel:
    def __init__(self, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0,
                 fx=1.0, fy=1.0, cx=0.0, cy=0.0):
        self.k1, self.k2, self.p1, self.p2, self.k3 = k1, k2, p1, p2, k3
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

    @classmethod
    def from_K_dist(cls, K, dist):
        """K: [fx,0,cx, 0,fy,cy, 0,0,1]；dist: [k1,k2,p1,p2,k3]"""
        k1, k2, p1, p2 = dist[0], dist[1], dist[2], dist[3]
        k3 = dist[4] if len(dist) > 4 else 0.0
        return cls(k1, k2, p1, p2, k3, K[0], K[4], K[2], K[5])

    def _undistort_norm(self, xd, yd, iters=10):
        x, y = xd, yd
        for _ in range(iters):
            r2 = x * x + y * y
            radial = 1.0 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2
            if abs(radial) < 1e-12:
                break
            dx = 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x)
            dy = self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        return x, y

    def undistort_point(self, u, v):
        """像素坐标 (u,v) → 去畸变后的像素坐标"""
        xd = (u - self.cx) / self.fx
        yd = (v - self.cy) / self.fy
        x, y = self._undistort_norm(xd, yd)
        return x * self.fx + self.cx, y * self.fy + self.cy


# ======================================================================
# 3) 单应：像素 → 毫米
# ======================================================================
class Homography:
    def __init__(self, H):
        """H: 3x3，行优先，长度为 9 的序列"""
        self.H = list(H)

    def apply(self, u, v):
        h = self.H
        w = h[6] * u + h[7] * v + h[8]
        if abs(w) < 1e-12:
            return None
        return (h[0] * u + h[1] * v + h[2]) / w, (h[3] * u + h[4] * v + h[5]) / w


# ======================================================================
# 4) 圆的最小二乘拟合（Kåsa 代数解，纯 Python 3x3 求解）
# ======================================================================
def fit_circle(points):
    """points: [(x,y), ...] 边缘点。返回 (xc, yc, r)，点太少/退化返回 None"""
    n = len(points)
    if n < 6:
        return None
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for (x, y) in points:
        z = x * x + y * y
        sx += x; sy += y; sz += z
        sxx += x * x; syy += y * y; sxy += x * y
        sxz += x * z; syz += y * z

    # 解 A·[a,b,c]^T = b   （法方程右端为负号）
    A = [[sxx, sxy, sx],
         [sxy, syy, sy],
         [sx,  sy,  float(n)]]
    B = [-sxz, -syz, -sz]

    det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
           - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
           + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
    if abs(det) < 1e-12:
        return None

    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    Aa = [row[:] for row in A]; Aa[0][0], Aa[1][0], Aa[2][0] = B
    Ab = [row[:] for row in A]; Ab[0][1], Ab[1][1], Ab[2][1] = B
    Ac = [row[:] for row in A]; Ac[0][2], Ac[1][2], Ac[2][2] = B
    a = det3(Aa) / det
    b = det3(Ab) / det
    c = det3(Ac) / det

    xc, yc = -a / 2.0, -b / 2.0
    r2 = xc * xc + yc * yc - c
    if r2 <= 0:
        return None
    return xc, yc, math.sqrt(r2)


# ======================================================================
# 5) 多帧平均：降抖动（σ_x̄ = σ_x / sqrt(N)）
# ======================================================================
class CenterFilter:
    def __init__(self, n=9):
        self.n = max(1, int(n))
        self.buf = []

    def reset(self):
        self.buf = []

    def push(self, cx, cy):
        self.buf.append((cx, cy))
        if len(self.buf) > self.n:
            self.buf.pop(0)
        k = len(self.buf)
        return (sum(p[0] for p in self.buf) / k,
                sum(p[1] for p in self.buf) / k)

    def std(self):
        if len(self.buf) < 2:
            return 0.0, 0.0
        mx = sum(p[0] for p in self.buf) / len(self.buf)
        my = sum(p[1] for p in self.buf) / len(self.buf)
        sx = math.sqrt(sum((p[0] - mx) ** 2 for p in self.buf) / len(self.buf))
        sy = math.sqrt(sum((p[1] - my) ** 2 for p in self.buf) / len(self.buf))
        return sx, sy


# ======================================================================
# 便捷：从 config.py 构建各组件（都带开关，默认不启用）
# ======================================================================
def build_scaler():
    import config
    return CoordScaler(getattr(config, "DET_W", config.W),
                       getattr(config, "DET_H", config.H),
                       config.W, config.H)


def build_distortion(det_w=None, det_h=None):
    """构建反畸变模型；det 与对外口径不同时自动缩放 CAM_K

    ⚠️ CAM_K/CAM_DIST 是在 W×H 下标定的。检测分辨率改成 DET_W×DET_H 后，
       fx/fy/cx/cy 必须按比例缩放，否则反畸变会把点推到错误位置。
    """
    import config
    if not getattr(config, "UNDISTORT_ON", False):
        return None
    if det_w and det_h and (det_w != config.W or det_h != config.H):
        kw = float(det_w) / float(config.W)
        kh = float(det_h) / float(config.H)
        K = [config.CAM_K[0] * kw, 0.0, config.CAM_K[2] * kw,
             0.0, config.CAM_K[4] * kh, config.CAM_K[5] * kh,
             0.0, 0.0, 1.0]
        return DistortionModel.from_K_dist(K, config.CAM_DIST)
    return DistortionModel.from_K_dist(config.CAM_K, config.CAM_DIST)


def build_homography():
    import config
    if not getattr(config, "HOMOGRAPHY_ON", False):
        return None
    return Homography(config.HOMOGRAPHY)


# ======================================================================
# 6) 坐标流水线：检测域 → 对外口径（缩放 → 反畸变 → 多帧平均）
# ======================================================================
class Pipeline:
    """把检测域坐标统一成对外口径。

    检测在 DET_W×DET_H 上做，但对 STM32 一律回 W×H 的坐标：

        ROI(对外)      --roi()-->   检测域 ROI        （喂给 find_blobs / find_circles）
        (x,y)(检测域)  --point()--> 对外口径 + 去畸变
        (w,h)(检测域)  --size()-->  对外口径尺寸
        N 帧中心       --smooth()--> 多帧平均（降抖动）

    PRECISION_ON=False 时用 build_pipeline() 得到恒等映射，行为与旧版一致。
    """

    def __init__(self, det_w, det_h, out_w, out_h, dist=None, filter_n=0,
                 max_shift=None):
        self.det_w, self.det_h = float(det_w), float(det_h)
        self.out_w, self.out_h = float(out_w), float(out_h)
        self.sx = self.out_w / self.det_w if self.det_w else 1.0
        self.sy = self.out_h / self.det_h if self.det_h else 1.0
        self.dist = dist
        self.filter_n = max(0, int(filter_n or 0))
        self.max_shift = max_shift
        self.dropped = 0
        self._filters = {}

    @property
    def active(self):
        return (abs(self.sx - 1.0) > 1e-9 or abs(self.sy - 1.0) > 1e-9
                or self.dist is not None or self.filter_n > 1)

    def describe(self):
        return ("DET %dx%d → %dx%d | 去畸变 %s | 多帧平均 %d"
                % (self.det_w, self.det_h, self.out_w, self.out_h,
                   "开" if self.dist is not None else "关", self.filter_n))

    # ---------------- 几何映射 ----------------
    def roi(self, roi):
        """对外口径 ROI [x,y,w,h] → 检测域 ROI"""
        if not roi:
            return []
        x, y, w, h = roi
        return [int(round(x / self.sx)), int(round(y / self.sy)),
                int(round(w / self.sx)), int(round(h / self.sy))]

    def point(self, x, y):
        """检测域坐标 → 对外口径（先去畸变，再缩放）

        反畸变保护：若反解后跑出画面、或位移超过 UNDISTORT_MAX_SHIFT，
        说明该区域的标定不可信（通常发生在四角），此时**退回原坐标**并计数，
        避免把负坐标/离谱坐标发给下位机。
        """
        if self.dist is not None:
            ux, uy = self.dist.undistort_point(x, y)
            if self._dist_trusted(ux, uy, x, y):
                x, y = ux, uy
            else:
                self.dropped += 1
        return int(round(x * self.sx)), int(round(y * self.sy))

    def _dist_trusted(self, ux, uy, x, y):
        if not (0.0 <= ux <= self.det_w and 0.0 <= uy <= self.det_h):
            return False
        if self.max_shift:
            if abs(ux - x) > self.max_shift or abs(uy - y) > self.max_shift:
                return False
        return True

    def size(self, w, h):
        if not w or not h:
            return 0, 0
        return (max(1, int(round(w * self.sx))),
                max(1, int(round(h * self.sy))))

    def area(self, pixels):
        """面积阈值跟着分辨率缩放（像素数按 (det/out)^2 放大）"""
        k = (self.det_w / self.out_w) * (self.det_h / self.out_h)
        return max(1, int(round(pixels * k)))

    # ---------------- 多帧平均 ----------------
    def smooth(self, key, x, y):
        """同一任务内多帧平均；filter_n<=1 时原样返回"""
        if self.filter_n <= 1:
            return x, y
        f = self._filters.get(key)
        if f is None:
            f = CenterFilter(self.filter_n)
            self._filters[key] = f
        mx, my = f.push(x, y)
        return int(round(mx)), int(round(my))

    def reset(self, key=None):
        if key is None:
            self._filters.clear()
        else:
            self._filters.pop(key, None)


def build_pipeline():
    """按 config 组装 Pipeline；PRECISION_ON=False → 恒等映射（旧行为）"""
    import config
    if not getattr(config, "PRECISION_ON", True):
        return Pipeline(config.W, config.H, config.W, config.H)
    det_w = getattr(config, "DET_W", config.W)
    det_h = getattr(config, "DET_H", config.H)
    return Pipeline(det_w, det_h, config.W, config.H,
                    dist=build_distortion(det_w, det_h),
                    filter_n=getattr(config, "CENTER_FILTER_N", 0),
                    max_shift=getattr(config, "UNDISTORT_MAX_SHIFT", 0))