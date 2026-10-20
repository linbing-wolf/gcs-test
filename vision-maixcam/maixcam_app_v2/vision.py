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
import math
import precision
from timing import ticks_ms

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


def _pixel_luma(img, x, y):
    """读一个像素的亮度（0~255）。灰度图返回 [GRAY]，RGB 图返回 [R,G,B]。"""
    try:
        p = img.get_pixel(int(x), int(y), True)
    except TypeError:
        p = img.get_pixel(int(x), int(y))
    if isinstance(p, (list, tuple)):
        if len(p) >= 3:
            return (p[0] * 299 + p[1] * 587 + p[2] * 114) / 1000.0
        if len(p) >= 1:
            return float(p[0])
        return None
    try:
        return float(p)
    except Exception:
        return None


def _radial_edge_points(img, cx, cy, r, rays=72, band=10, step=2, min_grad=6.0):
    """沿 rays 条射线，在 r±band 内找亮度梯度最大的点（圆周边缘点）。

    纯坐标采样，不生成边缘图，比 find_edges 省内存，也方便在板端控制耗时。
    """
    pts = []
    w, h = img.width(), img.height()
    rays = max(8, int(rays))
    step = max(0.5, float(step))
    r0 = max(2.0, float(r) - float(band))
    r1 = max(r0 + step, float(r) + float(band))
    for i in range(rays):
        a = 2.0 * math.pi * i / rays
        ca, sa = math.cos(a), math.sin(a)
        rr = r0
        prev = None
        best_g, best_xy = -1.0, None
        while rr <= r1:
            x = int(round(cx + ca * rr))
            y = int(round(cy + sa * rr))
            if 0 <= x < w and 0 <= y < h:
                v = _pixel_luma(img, x, y)
                if v is not None and prev is not None:
                    g = abs(v - prev)
                    if g > best_g:
                        best_g, best_xy = g, (x, y)
                prev = v
            else:
                prev = None
            rr += step
        if best_xy is not None and best_g >= min_grad:
            pts.append(best_xy)
    return pts


def refine_disc_center(img, cx, cy, r, rays=None, band=None, step=None,
                       min_points=None, max_shift=None, max_rms=None, meta=None):
    """径向边缘点 + 最小二乘拟合，细化圆心/半径。

    返回 (cx, cy, r) 或 None（点太少 / 拟合退化 / 偏移或残差超限 → 调用方用粗结果）。
    meta 不为 None 时写入 refine_reason / refine_points，供调试抓图分析。
    """
    if meta is not None:
        meta["refine_reason"] = "unknown"
        meta["refine_points"] = 0
    if not hasattr(img, "get_pixel"):
        if meta is not None:
            meta["refine_reason"] = "no-pixel-api"
        return None                      # PC 单测的假图没有像素接口
    rays = int(rays if rays is not None else getattr(config, "DISC_REFINE_RAYS", 72))
    band = float(band if band is not None else getattr(config, "DISC_REFINE_BAND", 10))
    step = float(step if step is not None else getattr(config, "DISC_REFINE_STEP", 2))
    min_points = int(min_points if min_points is not None
                     else getattr(config, "DISC_REFINE_MIN_POINTS", 24))
    max_shift = float(max_shift if max_shift is not None
                      else getattr(config, "DISC_REFINE_MAX_SHIFT", 6))
    max_rms = float(max_rms if max_rms is not None
                    else getattr(config, "DISC_REFINE_MAX_RMS", 3.0))
    min_grad = float(getattr(config, "DISC_REFINE_MIN_GRAD", 6.0))
    try:
        pts = _radial_edge_points(img, cx, cy, r, rays, band, step, min_grad)
        if meta is not None:
            meta["refine_points"] = len(pts)
        if len(pts) < max(6, min_points):
            if meta is not None:
                meta["refine_reason"] = "points=%d<%d" % (len(pts), max(6, min_points))
            return None
        fit = precision.fit_circle(pts)
        if fit is None:
            if meta is not None:
                meta["refine_reason"] = "fit-degenerate"
            return None
        xc, yc, rc = fit
        if abs(xc - cx) > max_shift or abs(yc - cy) > max_shift:
            if meta is not None:
                meta["refine_reason"] = "shift=%.1f,%.1f>%.1f" % (xc - cx, yc - cy, max_shift)
            return None
        if abs(rc - r) > band + 2.0:
            if meta is not None:
                meta["refine_reason"] = "radius=%.1f->%.1f" % (r, rc)
            return None
        s = 0.0
        for (x, y) in pts:
            d = math.hypot(x - xc, y - yc) - rc
            s += d * d
        rms = (s / len(pts)) ** 0.5
        if rms > max_rms:
            if meta is not None:
                meta["refine_reason"] = "rms=%.2f>%.2f" % (rms, max_rms)
            return None
        if meta is not None:
            meta["refine_reason"] = "ok(rms=%.2f,n=%d)" % (rms, len(pts))
        return int(round(xc)), int(round(yc)), int(round(rc))
    except Exception as e:
        if meta is not None:
            meta["refine_reason"] = "exception:%s" % e
        return None


def _circle_mag(c):
    try:
        return int(c.magnitude())
    except Exception:
        return 0


def find_disc_center_black(img, roi=None, th=None, min_pixels=None,
                           sub_ratio=None, top_n=None):
    """用黑色同心环的像素质心定位靶心（对 1/2/3 号环靶最稳）。

    ROI 上部找黑色 blob → 取最大的 N 个按像素数加权求质心；
    半径取最大 blob 的外接框半径。没有足够黑色像素时返回 None。
    """
    fixed = getattr(config, "DISC_BLACK_ROI", None)
    if fixed:
        sub = [int(v) for v in fixed]
    else:
        roi = roi if roi else config.ROI_DISC_ROUGH
        if not roi:
            return None
        x, y, w, h = roi
        if w <= 8 or h <= 8:
            return None
        ratio = float(sub_ratio if sub_ratio is not None
                      else getattr(config, "DISC_BLACK_SUB_RATIO", 0.65))
        sub = [int(x), int(y), int(w), max(8, int(h * ratio))]
    if len(sub) != 4:
        return None
    sx, sy, sw, sh = sub
    sx, sy = max(0, sx), max(0, sy)
    sw = min(sw, img.width() - sx)
    sh = min(sh, img.height() - sy)
    if sw <= 8 or sh <= 8:
        return None
    sub = [sx, sy, sw, sh]
    th = th if th is not None else config.TH_BLACK
    min_px = int(min_pixels if min_pixels is not None
                 else getattr(config, "DISC_BLACK_MIN_PIXELS", 300))
    top_n = int(top_n if top_n is not None
                else getattr(config, "DISC_BLACK_TOP_N", 8))
    blobs = img.find_blobs([th], roi=sub, x_stride=2, y_stride=2,
                           area_threshold=min_px, pixels_threshold=min_px,
                           merge=False, margin=0)
    if not blobs:
        return None
    total = sum(b.pixels() for b in blobs)
    if total < min_px:
        return None
    blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:max(1, top_n)]
    total = sum(b.pixels() for b in blobs)
    if total <= 0:
        return None
    cx = sum(b.cxf() * b.pixels() for b in blobs) / float(total)
    cy = sum(b.cyf() * b.pixels() for b in blobs) / float(total)
    b0 = blobs[0]
    r = max(b0.w(), b0.h()) / 2.0
    return int(round(cx)), int(round(cy)), max(2, int(round(r)))


def find_disc_center_inner(img, roi=None, meta=None):
    """用最内圈白块（靶心小圆）定位中心。

    同心环靶的最内圈是一个亮色小圆盘；找到它 → 圆心就是靶心，
    不会再受外层 Hough 圆（不同层环）影响。
    """
    fixed = getattr(config, "DISC_BLACK_ROI", None)
    if fixed:
        sub = [int(v) for v in fixed]
    else:
        roi = roi if roi else config.ROI_DISC_ROUGH
        if not roi:
            return None
        x, y, w, h = roi
        ratio = float(getattr(config, "DISC_BLACK_SUB_RATIO", 0.65))
        sub = [int(x), int(y), int(w), max(8, int(h * ratio))]
    if len(sub) != 4:
        return None
    sx, sy, sw, sh = sub
    sx, sy = max(0, sx), max(0, sy)
    sw = min(sw, img.width() - sx)
    sh = min(sh, img.height() - sy)
    if sw <= 8 or sh <= 8:
        return None
    sub = [sx, sy, sw, sh]

    # 参考点：黑色质心（没有就用子区域中心）
    ref = find_disc_center_black(img, roi)
    if ref is not None:
        bx, by = ref[0], ref[1]
    else:
        bx, by = sx + sw / 2.0, sy + sh / 2.0
    if meta is not None:
        meta["inner_ref"] = [int(bx), int(by)]

    th = getattr(config, "TH_CENTER_LIGHT", [55, 100, -18, 18, -18, 18])
    min_px = int(getattr(config, "DISC_CENTER_MIN_PIXELS", 200))
    dmin = int(getattr(config, "DISC_CENTER_MIN_D", 25))
    dmax = int(getattr(config, "DISC_CENTER_MAX_D", 140))
    pick_r = float(getattr(config, "DISC_CENTER_PICK_RADIUS", 20))
    blobs = img.find_blobs([th], roi=sub, x_stride=2, y_stride=2,
                           area_threshold=min_px, pixels_threshold=min_px,
                           merge=False, margin=0)
    if meta is not None:
        meta["inner_blobs"] = [[int(b.w()), int(b.h()), int(b.cxf()), int(b.cyf()),
                                int(b.pixels())] for b in (blobs or [])[:12]]
    if not blobs:
        if meta is not None:
            meta["inner_reason"] = "no-light-blobs"
        return None
    cands = []
    for b in blobs:
        w, h = b.w(), b.h()
        if w < dmin or h < dmin or w > dmax or h > dmax or h <= 0:
            continue
        ar = w / float(h)
        if ar < 0.7 or ar > 1.4:
            continue
        cx, cy = b.cxf(), b.cyf()
        d = ((cx - bx) ** 2 + (cy - by) ** 2) ** 0.5
        cands.append((d, w * h, cx, cy, max(w, h) / 2.0))
    if not cands:
        if meta is not None:
            meta["inner_reason"] = "no-candidate(%d blobs)" % len(blobs)
        return None
    near = [c for c in cands if c[0] <= pick_r]
    if near:
        c = min(near, key=lambda t: (t[1], t[0]))   # 优先最小白块 = 最内圈
    else:
        c = min(cands, key=lambda t: t[0])          # 否则取离黑心最近的
    if meta is not None:
        meta["inner_reason"] = "ok"
    return int(round(c[2])), int(round(c[3])), max(2, int(round(c[4])))


_TPL_CACHE = None


def _load_templates():
    """加载数字“2”模板（带缓存）；PC 单测没有 image.load 时返回空。"""
    global _TPL_CACHE
    if _TPL_CACHE is not None:
        return _TPL_CACHE
    _TPL_CACHE = []
    if not bool(getattr(config, "DISC_TEMPLATE_ON", True)):
        return _TPL_CACHE
    try:
        if not hasattr(image, "load"):
            return _TPL_CACHE
        base = image.load(getattr(config, "DISC_TEMPLATE_PATH",
                                  "/maixapp/apps/yyb_mover/assets/target2.png"))
        try:
            base = base.to_format(image.Format.FMT_GRAYSCALE)
        except Exception:
            pass
        w, h = base.width(), base.height()
        scales = getattr(config, "DISC_TEMPLATE_SCALES", [1.0])
        for s in scales:
            s = float(s)
            if abs(s - 1.0) < 0.01:
                _TPL_CACHE.append(base)
            else:
                _TPL_CACHE.append(base.resize(max(4, int(round(w * s))),
                                              max(4, int(round(h * s)))))
        print("[视觉] 已加载目标模板: %s (%dx%d, %d 个尺度)"
              % (getattr(config, "DISC_TEMPLATE_PATH", "?"), w, h, len(_TPL_CACHE)))
    except Exception as e:
        print("[视觉] 模板加载失败，跳过内容校验: %s" % e)
        _TPL_CACHE = []
    return _TPL_CACHE


def verify_target2(img, cx, cy, meta=None):
    """校验候选圆内部确实有数字“2”。

    只有模板匹配通过才认为目标有效；模板缺失时返回 True（不阻断旧行为）。
    """
    tpls = _load_templates()
    if not tpls:
        return True
    rw = int(getattr(config, "DISC_TEMPLATE_ROI_W", 160))
    rh = int(getattr(config, "DISC_TEMPLATE_ROI_H", 140))
    dx = int(getattr(config, "DISC_TEMPLATE_ROI_DX", 0))
    dy = int(getattr(config, "DISC_TEMPLATE_ROI_DY", 0))
    x = int(cx) - rw // 2 + dx
    y = int(cy) - rh // 2 + dy
    x = max(0, min(x, img.width() - rw))
    y = max(0, min(y, img.height() - rh))
    if rw <= 4 or rh <= 4 or x < 0 or y < 0 or x + rw > img.width() or y + rh > img.height():
        return False
    roi = [x, y, rw, rh]
    try:
        crop = img.crop(x, y, rw, rh)
        try:
            crop = crop.to_format(image.Format.FMT_GRAYSCALE)
        except Exception:
            pass
    except Exception as e:
        if meta is not None:
            meta["tpl_error"] = "crop:%s" % e
        return False
    th = float(getattr(config, "DISC_TEMPLATE_THRESHOLD", 0.65))
    scales = getattr(config, "DISC_TEMPLATE_SCALES", [1.0])
    for i, tpl in enumerate(tpls):
        try:
            r = crop.find_template(tpl, th, step=2)
        except Exception as e:
            if meta is not None:
                meta["tpl_error"] = str(e)
            continue
        if r:
            if meta is not None:
                meta["tpl"] = [int(r[0]) + x, int(r[1]) + y, int(r[2]), int(r[3])]
                meta["tpl_scale"] = float(scales[i]) if i < len(scales) else 1.0
                meta["tpl_roi"] = roi
            return True
    if meta is not None:
        meta["tpl"] = None
        meta["tpl_roi"] = roi
    return False


def find_disc_center(img, roi=None, r_min=config.DISC_R_MIN, r_max=config.DISC_R_MAX,
                     threshold=config.DISC_THRESHOLD, refine=None,
                     ring_fallback=None, prefer=None, meta=None):
    """定位圆盘中心（粗加工区/暂存区），三级策略：

      1) find_circles 找候选圆（粗定位）
      2) DISC_REFINE_ON：径向边缘点 + 最小二乘拟合，细化圆心/半径
      3) DISC_RING_FALLBACK：连大圆都找不到时，用同心圆环找中心

    prefer=(x,y)：上一次的圆心（检测域）。有它时优先在 DISC_TRACK_MAX_JUMP
    范围内选候选，避免在多个环/假圆之间来回跳。
    meta：调试字典，写入 circles/coarse/refined/ring/src/ms 等字段。

    视觉只给圆盘中心；各环位置由 STM32 按 config.RING_SPACING_PX 平行推算。
    返回 (cx, cy, r) 或 None。
    """
    if refine is None:
        refine = bool(getattr(config, "DISC_REFINE_ON", True))
    if ring_fallback is None:
        ring_fallback = bool(getattr(config, "DISC_RING_FALLBACK", True))
    if meta is not None:
        meta["src"] = "none"
        meta["circles"] = []
        meta["coarse"] = None
        meta["refined"] = None
        meta["ring"] = None
        meta["refine_reason"] = ""
        meta["refine_points"] = 0
    t0 = ticks_ms()

    def _accept(p, src):
        """内容校验：候选圆附近必须能匹配到数字“2”，否则丢弃。"""
        if p is None:
            return None
        if not bool(getattr(config, "DISC_TEMPLATE_ON", True)):
            return p
        if verify_target2(img, p[0], p[1], meta=meta):
            return p
        if meta is not None:
            meta["tpl_reject"] = src
        return None

    # 0) 最优先：最内圈白块（靶心小圆）。只认这一圈，避免在多层环之间跳。
    if bool(getattr(config, "DISC_CENTER_ON", True)):
        inner = find_disc_center_inner(img, roi, meta=meta)
        if inner is not None:
            p = _accept(inner, "inner")
            if p is not None:
                if meta is not None:
                    meta["inner"] = [int(p[0]), int(p[1]), int(p[2])]
                    meta["coarse"] = [int(p[0]), int(p[1]), int(p[2])]
                    meta["src"] = "inner"
                    meta["ms"] = ticks_ms() - t0
                return p

    # 1) 次优先：黑色同心环质心。Hough 在粗糙/手绘环上会拟合出偏心假圆，
    #    而黑色环的像素质心实测抖动只有 ~3px。
    if bool(getattr(config, "DISC_BLACK_CENTER_ON", True)):
        b = find_disc_center_black(img, roi)
        if b is not None:
            if meta is not None:
                meta["black"] = [int(b[0]), int(b[1]), int(b[2])]
                meta["coarse"] = [int(b[0]), int(b[1]), int(b[2])]
            if refine and b[2] >= 40:
                m2 = {}
                f = refine_disc_center(img, b[0], b[1], b[2], meta=m2)
                if meta is not None:
                    meta.update(m2)
                if f is not None:
                    p = _accept(f, "black-refine")
                    if p is not None:
                        if meta is not None:
                            meta["refined"] = [int(p[0]), int(p[1]), int(p[2])]
                            meta["src"] = "black-refine"
                            meta["ms"] = ticks_ms() - t0
                        return p
            p = _accept(b, "black")
            if p is not None:
                if meta is not None:
                    meta["black"] = [int(p[0]), int(p[1]), int(p[2])]
                    meta["coarse"] = [int(p[0]), int(p[1]), int(p[2])]
                    meta["src"] = "black"
                    meta["ms"] = ticks_ms() - t0
                return p

    circles = img.find_circles(roi=roi if roi else [], threshold=threshold,
                               x_margin=10, y_margin=10, r_margin=10,
                               r_min=r_min, r_max=r_max)
    if meta is not None:
        meta["circles"] = [[int(c.x()), int(c.y()), int(c.r()), _circle_mag(c)]
                           for c in sorted(circles, key=lambda c: c.r(), reverse=True)[:12]]

    if circles:
        pick = circles
        # 连续性：上一次圆心附近至少有两个候选时才用，避免锁死单个假圆
        max_jump = float(getattr(config, "DISC_TRACK_MAX_JUMP", 0) or 0)
        if prefer is not None and max_jump > 0:
            near = [c for c in circles
                    if ((c.x() - prefer[0]) ** 2 + (c.y() - prefer[1]) ** 2) ** 0.5 <= max_jump]
            if len(near) >= 2:
                pick = near
        pick = sorted(pick, key=lambda c: c.r(), reverse=True)
        if meta is not None:
            meta["near_count"] = len(pick)

        # ---- 圆心聚类：同心环的圆心应该重合，成员最多的簇才是目标 ----
        # 旧逻辑取「半径最大的圆」；ROI 裁掉靶子上半部时会拟合出偏下的大假圆。
        max_in = max(8, int(getattr(config, "DISC_CLUSTER_MAX_INPUT", 80)))
        if len(pick) > max_in:
            pick = pick[:max_in]
        cluster_px = float(getattr(config, "DISC_CLUSTER_PX", 15))
        best_cluster = None
        for c in pick:
            members = [o for o in pick
                       if ((o.x() - c.x()) ** 2 + (o.y() - c.y()) ** 2) ** 0.5 <= cluster_px]
            if best_cluster is None or len(members) > len(best_cluster):
                best_cluster = members
        if best_cluster:
            pick = best_cluster
        if meta is not None:
            meta["cluster"] = [[int(c.x()), int(c.y()), int(c.r()), _circle_mag(c)]
                               for c in pick[:12]]
            meta["cluster_n"] = len(pick)

        # 簇内中位数作为粗中心/半径（比最大圆稳得多）
        xs = sorted(c.x() for c in pick)
        ys = sorted(c.y() for c in pick)
        rs = sorted(c.r() for c in pick)
        cx = xs[len(xs) // 2]
        cy = ys[len(ys) // 2]
        r = rs[len(rs) // 2]

        attempts = []
        # 先拟合簇中位数
        if refine:
            m2 = {}
            f = refine_disc_center(img, cx, cy, r, meta=m2)
            attempts.append({"c": [int(cx), int(cy), int(r)], "mag": 0,
                             "fit": [int(f[0]), int(f[1]), int(f[2])] if f else None,
                             "why": m2.get("refine_reason", "")})
            if f is not None:
                p = _accept(f, "cluster-refine")
                if p is not None:
                    if meta is not None:
                        meta.update(m2)
                        meta["coarse"] = [int(cx), int(cy), int(r)]
                        meta["picked"] = [int(cx), int(cy), int(r), 0]
                        meta["refined"] = [int(p[0]), int(p[1]), int(p[2])]
                        meta["src"] = "cluster-refine"
                        meta["attempts"] = attempts
                        meta["ms"] = ticks_ms() - t0
                    return p

        # 簇中位数拟合失败：再试簇内前几个候选
        try_n = max(1, int(getattr(config, "DISC_REFINE_TRY", 2)))
        if refine:
            for c in pick[:try_n]:
                m2 = {}
                f = refine_disc_center(img, c.x(), c.y(), c.r(), meta=m2)
                attempts.append({"c": [int(c.x()), int(c.y()), int(c.r())],
                                 "mag": _circle_mag(c),
                                 "fit": [int(f[0]), int(f[1]), int(f[2])] if f else None,
                                 "why": m2.get("refine_reason", "")})
                if f is not None:
                    p = _accept(f, "refine")
                    if p is not None:
                        if meta is not None:
                            meta.update(m2)
                            meta["coarse"] = [int(c.x()), int(c.y()), int(c.r())]
                            meta["picked"] = [int(c.x()), int(c.y()), int(c.r()), _circle_mag(c)]
                            meta["refined"] = [int(p[0]), int(p[1]), int(p[2])]
                            meta["src"] = "refine"
                            meta["attempts"] = attempts
                            meta["ms"] = ticks_ms() - t0
                        return p

        # 都失败：用簇中位数（至少是多个同心环的共同中心），但仍要过“2”校验
        p = _accept((cx, cy, r), "cluster")
        if p is not None:
            if meta is not None:
                meta["coarse"] = [int(p[0]), int(p[1]), int(p[2])]
                meta["picked"] = [int(p[0]), int(p[1]), int(p[2]), 0]
                meta["src"] = "cluster"
                meta["attempts"] = attempts
                meta["ms"] = ticks_ms() - t0
            return p

    if ring_fallback:
        c = find_ring_center(
            img, roi,
            r_min=getattr(config, "DISC_RING_R_MIN", 20),
            r_max=getattr(config, "DISC_RING_R_MAX", 140),
            threshold=getattr(config, "DISC_RING_THRESHOLD", 1200))
        if c:
            cx, cy, r = c
            if meta is not None:
                meta["ring"] = [int(cx), int(cy), int(r)]
            if refine:
                f = refine_disc_center(img, cx, cy, r, meta=meta)
                if f is not None:
                    p = _accept(f, "ring-refine")
                    if p is not None:
                        if meta is not None:
                            meta["refined"] = [int(p[0]), int(p[1]), int(p[2])]
                            meta["src"] = "ring-refine"
                            meta["ms"] = ticks_ms() - t0
                        return p
            p = _accept((cx, cy, r), "ring")
            if p is not None:
                if meta is not None:
                    meta["ring"] = [int(p[0]), int(p[1]), int(p[2])]
                    meta["src"] = "ring"
                    meta["ms"] = ticks_ms() - t0
                return p

    if meta is not None:
        meta["ms"] = ticks_ms() - t0
    return None


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