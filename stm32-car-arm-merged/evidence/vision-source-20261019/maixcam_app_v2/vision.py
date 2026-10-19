# -*- coding: utf-8 -*-
"""
vision.py —— 新流程原生识别能力（MaixPy 硬件加速）

四个能力：
  1) 颜色分类（4 色：红/黄/蓝/绿，代码 1/2/3/4）  ColorClassifier
  2) 启停区定位（蓝色大区域）                      find_start_zone
  3) 圆盘中心定位（粗加工区 / 暂存区）              find_disc_center
  4) 码垛目标定位（暂存区最上层物料中心）            find_stack_top

说明：本文件只做识别，不做状态机/握手，方便单独调试每个能力。
"""

from maix import image
import config

# 颜色码（官方：红1 黄2 蓝3 绿4）
COLOR_RED = "1"
COLOR_YELLOW = "2"
COLOR_BLUE = "3"
COLOR_GREEN = "4"
COLOR_NONE = "0"


class ColorClassifier:
    """把色块判成红/黄/蓝/绿

    利用 find_blobs 的「多阈值 + code 位标记」机制：一次调用同时找 4 种颜色，
    每个 blob.code() 的第 i 位为 1 表示命中第 i 组阈值，直接查表得颜色码。

    可选接一个 AI 后端（ai_model.AI）：
      定位仍然用 LAB 阈值（快、稳），**判色**交给分类模型（抗现场色差/白平衡漂移）。
      ai=None 或模型没加载成功时，行为与纯 CV 版本完全一致。
    """

    def __init__(self, thresholds=config.THRESHOLDS, names=config.COLOR_NAMES, ai=None):
        self.thresholds = thresholds
        self.names = names
        self.codes = [config.COLOR_CODES[n] for n in names]
        self.ai = ai

    def detect(self, img, roi=None, pixels_threshold=config.MIN_PIXELS, largest_first=True):
        blobs = img.find_blobs(
            self.thresholds,
            roi=roi if roi else [],
            x_stride=2, y_stride=2,
            area_threshold=pixels_threshold,
            pixels_threshold=pixels_threshold,
            merge=True, margin=6,
        )
        out = []
        for b in blobs:
            w, h = b.w(), b.h()
            if w <= 0 or h <= 0:
                continue
            out.append({
                "code": self._code_of(b),
                "x": b.x(), "y": b.y(), "w": w, "h": h,
                "cx": int(b.cxf()), "cy": int(b.cyf()),
                "radius": int((w + h) / 4),
                "pixels": b.pixels(),
                "roundness": b.roundness(),
                "blob": b,
            })
        if largest_first:
            out.sort(key=lambda d: d["pixels"], reverse=True)
        return out

    def find_color(self, img, color_code, roi=None, pixels_threshold=config.MIN_PIXELS,
                   ai=None):
        """在 ROI 内找指定颜色码的最大色块；没有则 None

        判色顺序：AI 判色 →（不可用/低置信度/判成 background·other）→ 退回阈值判色。
        返回的 item 里会带：
            code  最终颜色码
            ai_code  AI 判出的码（"0"/None 表示没采用 AI）
            src   "ai" / "cv"，表明这个结果是谁给的
        """
        for it in self.detect(img, roi, pixels_threshold):
            if self.resolve_code(img, it, ai) == color_code:
                return it
        return None

    def resolve_code(self, img, item, ai=None):
        """决定一个已定位色块的颜色码（写回 item["code"] 并返回它）

        AI 优先；任何一步不满足条件就退回阈值判色，保证不比纯 CV 差。
        """
        ai = ai if ai is not None else self.ai
        item["ai_code"] = None
        item["src"] = "cv"
        if ai is None or getattr(config, "AI_PREFER_CV", False):
            return item["code"]
        try:
            if not ai.ready():
                return item["code"]
            m = int(getattr(config, "AI_LOCATE_MARGIN", 8))
            roi = [item["x"] - m, item["y"] - m, item["w"] + 2 * m, item["h"] + 2 * m]
            label, prob, idx = ai.classify_label(img, roi)
            if getattr(config, "AI_DEBUG_PROB", False):
                print("[AI] %-10s %.3f  (bbox=%s)" % (str(label), prob, roi))
            if idx is None:
                return item["code"]
            min_p = getattr(config, "AI_MIN_PROB", 0.60)
            if prob < min_p:
                return item["code"]                      # 置信度不足 → 信任阈值
            code = getattr(config, "AI_CLASS_TO_COLOR", {}).get(label)
            if code:
                item["ai_code"] = code
                item["ai_label"] = label
                item["ai_prob"] = prob
                item["code"] = code
                item["src"] = "ai"
                return code
            # 判成 background / other：严格模式下直接丢弃这个候选
            if getattr(config, "AI_VETO_OTHER", True):
                item["ai_label"] = label
                item["ai_prob"] = prob
                item["src"] = "ai-veto"
                return COLOR_NONE
            return item["code"]
        except Exception as e:
            print("[vision] AI 判色异常，退回阈值: %s" % e)
            return item["code"]

    def _code_of(self, blob):
        code = blob.code()
        for i, c in enumerate(self.codes):
            if code & (1 << i):
                return c
        return COLOR_NONE


def find_road(img, roi=None, th=None, min_ratio=None):
    """右侧有没有「灰色道路」

    做法：在 ROI 内用低饱和的灰色 LAB 阈值找连通块，统计灰色像素占 ROI 的比例。
    占比 ≥ min_ratio 判定有道路。

    返回 (has_road: bool, ratio: float)
    """
    roi = roi if roi else config.ROI_ROAD
    if not roi:
        return False, 0.0
    th = th if th is not None else config.TH_ROAD
    min_ratio = min_ratio if min_ratio is not None else config.ROAD_MIN_RATIO
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        return False, 0.0
    blobs = img.find_blobs(
        [th],
        roi=roi,
        x_stride=4, y_stride=4,
        area_threshold=min(w * h // 50, 400),
        pixels_threshold=min(w * h // 50, 400),
        merge=True, margin=8,
    )
    if not blobs:
        return False, 0.0
    area = sum(b.pixels() for b in blobs)
    ratio = float(area) / float(w * h)
    return ratio >= min_ratio, ratio


def find_start_zone(img, roi=None, min_pixels=config.MIN_PIXELS_START):
    """定位蓝色启停区：全画面最大的蓝色块（面积阈值很大，避免误认蓝色物料）

    返回 dict 或 None。
    """
    blobs = img.find_blobs(
        [config.TH_BLUE],
        roi=roi if roi else [],
        x_stride=4, y_stride=4,
        area_threshold=min_pixels,
        pixels_threshold=min_pixels,
        merge=True, margin=10,
    )
    if not blobs:
        return None
    b = max(blobs, key=lambda x: x.pixels())
    return {
        "x": b.x(), "y": b.y(), "w": b.w(), "h": b.h(),
        "cx": int(b.cxf()), "cy": int(b.cyf()),
        "pixels": b.pixels(), "blob": b,
    }


def find_disc_center(img, roi=None, r_min=config.DISC_R_MIN, r_max=config.DISC_R_MAX,
                     threshold=config.DISC_THRESHOLD):
    """定位圆盘中心（粗加工区/暂存区）：找最大的圆

    视觉只给圆盘中心；各环位置由 STM32 按 config.RING_SPACING_PX 平行推算。
    返回 (cx, cy, r) 或 None。
    """
    circles = img.find_circles(roi=roi if roi else [], threshold=threshold,
                               x_margin=10, y_margin=10, r_margin=10,
                               r_min=r_min, r_max=r_max)
    if not circles:
        return None
    c = max(circles, key=lambda c: c.r())
    return c.x(), c.y(), c.r()


def find_ring_center(img, roi=None,
                     r_min=config.STACK_RING_R_MIN, r_max=config.STACK_RING_R_MAX,
                     threshold=config.STACK_RING_THRESHOLD):
    """找圆环中心（码垛时没有底座，用来回"空环中心"）

    取离 ROI 中心最近的圆。返回 (cx, cy, r) 或 None。
    """
    roi = roi if roi else config.ROI_STACK
    if not roi:
        return None
    x, y, w, h = roi
    cx0, cy0 = x + w / 2.0, y + h / 2.0
    circles = img.find_circles(roi=roi, threshold=threshold,
                               x_margin=10, y_margin=10, r_margin=10,
                               r_min=r_min, r_max=r_max)
    if not circles:
        return None
    c = min(circles, key=lambda c: (c.x() - cx0) ** 2 + (c.y() - cy0) ** 2)
    return c.x(), c.y(), c.r()


def find_stack_base(clf, img, roi=None, color_code=None, ai=None):
    """码垛底座：按颜色找物料；同色多个取离 ROI 中心最近的；没有就回空环中心

    返回 dict：{"cx","cy","src","code"}
        src = "base"（找到同色物料）/ "ring"（空环）/ "roi"（兜底 ROI 中心）
    返回 None 只在"没有底座、也没找到环、且不允许兜底"时出现。
    """
    roi = roi if roi else config.ROI_STACK
    cx0, cy0 = (roi[0] + roi[2] / 2.0, roi[1] + roi[3] / 2.0) if roi else (0, 0)

    if color_code:
        best = None
        for it in clf.detect(img, roi, config.MIN_PIXELS):
            code = clf.resolve_code(img, it, ai)          # 阈值 + AI 判色
            if code != color_code:
                continue
            d = (it["cx"] - cx0) ** 2 + (it["cy"] - cy0) ** 2
            if best is None or d < best[0]:
                best = (d, it)
        if best is not None:
            it = best[1]
            return {"cx": it["cx"], "cy": it["cy"], "x": it["x"], "y": it["y"],
                    "w": it["w"], "h": it["h"], "src": "base", "code": color_code}

    c = find_ring_center(img, roi)
    if c:
        cx, cy = int(c[0]), int(c[1])
        return {"cx": cx, "cy": cy, "x": max(0, cx - 10), "y": max(0, cy - 10),
                "w": 20, "h": 20, "src": "ring", "code": None}

    if getattr(config, "STACK_RING_FALLBACK", True) and roi:
        return {"cx": int(cx0), "cy": int(cy0),
                "x": max(0, int(cx0) - 10), "y": max(0, int(cy0) - 10),
                "w": 20, "h": 20, "src": "roi", "code": None}
    return None


def find_stack_top(clf, img, roi=None, pixels_threshold=config.MIN_PIXELS):
    """定位码垛目标：暂存区里最上面（y 最小）的那个物料

    返回 dict 或 None。
    """
    items = clf.detect(img, roi, pixels_threshold)
    if not items:
        return None
    return min(items, key=lambda d: d["cy"])


def find_obstacle(img, roi=None, min_pixels=config.MIN_PIXELS_OBST,
                  th=config.TH_BLACK, margin=config.OBST_MERGE_MARGIN):
    """前方障碍物检测（黑色物体，官方 φ50×100mm）—— 供"前进状态"使用

    返回最大的黑色色块 dict 或 None：
        x / y / w / h / cx / cy / pixels
    注意：赛道/阴影里也可能有黑色，靠 MIN_PIXELS_OBST 面积阈值 + ROI 收紧来抗误检。
    """
    blobs = img.find_blobs(
        [th],
        roi=roi if roi else [],
        x_stride=4, y_stride=4,
        area_threshold=min_pixels,
        pixels_threshold=min_pixels,
        merge=True, margin=margin,
    )
    if not blobs:
        return None
    b = max(blobs, key=lambda x: x.pixels())
    w, h = b.w(), b.h()
    if w <= 0 or h <= 0:
        return None
    return {
        "x": b.x(), "y": b.y(), "w": w, "h": h,
        "cx": int(b.cxf()), "cy": int(b.cyf()),
        "pixels": b.pixels(),
    }


def estimate_distance_mm(width_px, real_width_mm=config.OBST_WIDTH_MM):
    """单目测距：dist = fx * 真实宽 / 像素宽。未标定(无 fx)时返回 0"""
    if not width_px or width_px <= 0:
        return 0
    fx = None
    try:
        if getattr(config, "UNDISTORT_ON", False):
            fx = config.CAM_K[0]
    except Exception:
        fx = None
    if not fx or fx <= 0:
        return 0
    return int(round(fx * float(real_width_mm) / float(width_px)))