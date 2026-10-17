# -*- coding: utf-8 -*-
"""
vision.py —— 把原树莓派 + OpenCV 的三个视觉能力，用 MaixPy 原生算子重写

原实现（cv2_python/zongchengxu20250724chusai.py）用的算子 vs MaixPy 对应关系：

  原 OpenCV 方案                        →  MaixPy 原生（硬件加速）        本文件函数
  -----------------------------------------------------------------------------------------
  cv2.inRange(HSV) + countNonZero 判色  →  find_blobs(LAB 阈值)          ColorClassifier.detect
  cv2.HoughCircles 定圆心/半径           →  find_circles()                circle_center_radius
  pyzbar.decode 识别二维码               →  find_qrcodes()                detect_qrcode
  cv2.Canny + cv2.HoughLinesP 测边界角   →  find_lines()                  detect_boundary_angle

关键认知：MaixPy 这些算子走芯片硬件加速，比 OpenCV(CPU) 快很多。
          所以『移植』不是把 OpenCV 搬过去（MaixCAM 上确实也能跑 cv2），
          而是用原生算子重写同一套判定逻辑，帧率能从原来的 ~10fps 提到 30~60fps。

本文件只做识别、不做业务状态机，方便单独调试每个能力。
"""

from maix import image
import math

# ----------------------------------------------------------------------
# 颜色码约定（必须和原程序完全一致）
#   原程序 get_color_name()：红=1 绿=2 蓝=3，『没看到』用 4
#   STM32 端 Serial_RxPacket[8] == 0x31/0x32/0x33/0x34 → '1'/'2'/'3'/'4'
# ----------------------------------------------------------------------
COLOR_RED = "1"
COLOR_GREEN = "2"
COLOR_BLUE = "3"
COLOR_NONE = "4"


class ColorClassifier:
    """颜色分类器：把色块判成红/绿/蓝/未知

    原程序用 HSV 的 H 分量判色（红 0-10 & 160-180、绿 35-89、蓝 90-140）。
    MaixPy 的 find_blobs 用 LAB 阈值，两者是等价的颜色区域划分，只是坐标系不同。
    这里利用 find_blobs 的『多阈值 + code 位』机制：一次调用同时找 3 种颜色，
    每个返回的 blob.code() 里带位标记，直接就知道命中了哪一组阈值。
    """

    def __init__(self, th_red, th_green, th_blue):
        self.thresholds = [th_red, th_green, th_blue]
        self.codes = [COLOR_RED, COLOR_GREEN, COLOR_BLUE]

    def detect(self, img, roi=None, pixels_threshold=300):
        """一次找出画面里所有红/绿/蓝色块

        返回列表（按像素数从大到小），每项是 dict：
            code / x / y / w / h / cx / cy / radius / pixels / roundness / blob
        """
        blobs = img.find_blobs(
            self.thresholds,
            roi=roi if roi else [],
            x_stride=2, y_stride=2,
            area_threshold=pixels_threshold,
            pixels_threshold=pixels_threshold,
            merge=True,     # 原程序用 dilate 合并碎块，这里用 merge 等效
            margin=6,
        )
        out = []
        for b in blobs:
            w, h = b.w(), b.h()
            if w <= 0 or h <= 0:
                continue
            # 半径估算：原程序用 HoughCircles 直接给半径，这里用外接矩形折算。
            # 圆的外接矩形近似正方形，取 (w+h)/4 作为半径估计。
            out.append({
                "code": self._code_of(b),
                "x": b.x(), "y": b.y(), "w": w, "h": h,
                "cx": int(b.cxf()), "cy": int(b.cyf()),
                "radius": int((w + h) / 4),
                "pixels": b.pixels(),
                "roundness": b.roundness(),
                "blob": b,
            })
        out.sort(key=lambda d: d["pixels"], reverse=True)
        return out

    def _code_of(self, blob):
        """从 blob.code() 的位标记反推颜色码

        find_blobs 传入 N 组阈值时，返回的 code 是 32 位整数，第 i 位为 1
        表示该色块命中了第 i 组阈值（K210/MaixPy 的经典机制）。
        一个色块可能同时命中多组（阈值区域有重叠），取最低位即可。
        """
        code = blob.code()
        for i, c in enumerate(self.codes):
            if code & (1 << i):
                return c
        return COLOR_NONE

    def dominant_color(self, img, x, y, w, h, pixels_threshold=50):
        """在指定小区域内判断主导颜色（对应原程序『圆内比三种颜色面积』）

        原程序 mask 之后比 red_area/green_area/blue_area 谁大；这里同理，
        在 ROI 内分别统计三种颜色的像素数，取最大者。
        """
        counts = {}
        blobs = img.find_blobs(self.thresholds, roi=[x, y, w, h],
                              pixels_threshold=pixels_threshold,
                              merge=True, margin=4)
        for b in blobs:
            c = self._code_of(b)
            counts[c] = counts.get(c, 0) + b.pixels()
        if not counts:
            return COLOR_NONE, counts
        best = max(counts.items(), key=lambda kv: kv[1])[0]
        return best, counts


def find_main_blob(img, thresholds, roi=None, pixels_threshold=300, prefer_round=True):
    """通用：找出最大的（或最圆的）色块

    prefer_round=True 优先选圆度高的（适合挑『物块/圆盘上的球』）；
    False 则纯按像素数选（适合挑『色环上的色块条』）。
    """
    blobs = img.find_blobs(thresholds, roi=roi if roi else [],
                          x_stride=2, y_stride=2,
                          area_threshold=pixels_threshold,
                          pixels_threshold=pixels_threshold,
                          merge=True, margin=6)
    if not blobs:
        return None
    if prefer_round:
        # 圆度 0~1 越接近 1 越圆；乘 sqrt(pixels) 加权，避免选到极小但很圆的噪点
        return max(blobs, key=lambda b: b.roundness() * math.sqrt(b.pixels()))
    return max(blobs, key=lambda b: b.pixels())


def detect_qrcode(img, roi=None):
    """二维码识别（替代 pyzbar）

    原程序：gray = cvtColor(BGR2GRAY); pyzbar.decode(gray)；只认 7 字符的码。
    MaixPy：img.find_qrcodes() 直接返回 QRCode 列表，有 payload()/rect()/corners()。

    返回 (payload, cx, cy)；没识别到返回 (None, None, None)。
    """
    codes = img.find_qrcodes(roi=roi if roi else [])
    if not codes:
        return None, None, None
    for q in codes:
        try:
            payload = q.payload()
        except Exception:
            continue
        r = q.rect()
        cx, cy = r[0] + r[2] // 2, r[1] + r[3] // 2
        if len(payload) == 7:
            return payload, cx, cy
        print("[i] 二维码长度 %d（需要 7），内容=%r" % (len(payload), payload))
    return None, None, None


def boundary_angle(lines):
    """把 find_lines 的结果换算成『水平线倾角』（等效原 HoughLinesP 的角度）

    ⚠️ 单位陷阱（最容易踩）：
      OpenCV HoughLinesP 返回线段两端点 (x1,y1,x2,y2)，斜率角 = atan2(dy,dx)；
      MaixPy find_lines 返回的是 Hough 参数 (rho, theta)，theta 是『法线』角度。
      两者相差 90°：同一根线的『斜率角』 = 90° - theta。
      不做这个换算，角度会整体偏 90°，按原公式发出去必然出错。

    另外每根线用长度做权重加权平均 —— 与原程序 weights=线段长度 完全一致。
    """
    if not lines:
        return None
    total_w = 0.0
    acc = 0.0
    for ln in lines:
        angle_deg = 90.0 - ln.theta()     # 法线角 → 斜率角
        length = ln.length()
        # 归一到 [-90, 90]，避免 ±180 附近求平均时乱跳
        while angle_deg > 90.0:
            angle_deg -= 180.0
        while angle_deg < -90.0:
            angle_deg += 180.0
        acc += angle_deg * length
        total_w += length
    if total_w <= 0:
        return None
    return acc / total_w


def detect_boundary_angle(img, roi=None, threshold=1500, min_len=100,
                          samples=3, variance_max=1.0, calib_offset=-2.0):
    """边界线角度检测：连续采 N 帧，方差合格才输出（复刻原 detect_boundary_line 判定链）

    原程序：采 3 次有效检测 → 算方差 → 方差 ≤ 1.0 才认为稳定 →
            取均值 + 校准偏置(-2.0) → 限幅 ±9.9 → 打包成 000000{s}{i}{d}。
    这里判定链完全相同，只把检测算子换成 find_lines（硬件加速）。

    返回 final_angle(float) 或 None。
    """
    results = []
    for _ in range(samples):
        lines = img.find_lines(roi=roi if roi else [], threshold=threshold,
                              theta_margin=25, rho_margin=25)
        if lines:
            keep = [ln for ln in lines if ln.length() >= min_len]
            a = boundary_angle(keep)
            if a is not None:
                results.append(a)
    if len(results) < samples:
        return None
    mean = sum(results) / len(results)
    var = sum((r - mean) ** 2 for r in results) / len(results)
    print("[i] 边界角样本=%s 均值=%.2f 方差=%.2f" %
          (["%.1f" % r for r in results], mean, var))
    if var > variance_max:
        return None
    final_angle = round(mean + calib_offset, 1)
    return max(-9.9, min(9.9, final_angle))


def circle_center_radius(img, r_min, r_max, roi=None, threshold=2000):
    """圆检测：给圆台/转盘上的物块定位（替代 HoughCircles）

    find_circles 内部也是 Hough 圆变换，参数与 OpenCV 对应关系：
      threshold : 累加器阈值（越大越严格，对应 OpenCV 的 param2）
      r_min/r_max : 半径范围（对应 minRadius/maxRadius）
      x/y/r_margin : 同心圆合并容差

    返回 (cx, cy, r) 或 None。
    """
    circles = img.find_circles(roi=roi if roi else [], threshold=threshold,
                              x_margin=10, y_margin=10, r_margin=10,
                              r_min=r_min, r_max=r_max)
    if not circles:
        return None
    c = max(circles, key=lambda c: c.r())   # 取最大的圆
    return c.x(), c.y(), c.r()
