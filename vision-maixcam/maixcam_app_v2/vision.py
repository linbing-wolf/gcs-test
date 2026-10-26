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


def _ellipse_radius(a, b, ca, sa):
    """轴对齐椭圆在单位方向 (ca, sa) 上的半径：r = a·b / √((b·ca)² + (a·sa)²)。"""
    return a * b / math.sqrt((b * ca) ** 2 + (a * sa) ** 2)


def _ellipse_rms(pts, cx, cy, a, b):
    """轴对齐椭圆 (cx,cy,a,b) 的径向残差 RMS（原始像素坐标）。"""
    if a <= 0 or b <= 0:
        return 1e9
    s = 0.0
    n = 0
    for (x, y) in pts:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            continue
        re = _ellipse_radius(a, b, dx / d, dy / d)
        s += (d - re) ** 2
        n += 1
    return math.sqrt(s / n) if n else 1e9


def fit_arc_ellipse(pts, k_min=0.80, k_max=1.15, k_step=0.01, max_pts=None):
    """轴对齐椭圆的最小二乘拟合（用「拉伸空间拟合圆」把 5 参数降到 4 参数）。

    为什么不用 cv2.fitEllipse：物料盘只能看到下弧（约 220°），直接拟合完整椭圆
    是病态的（上半部分无约束，实测 12 张真实图里有 2 张直接飞掉、其余角度也怪）。
    盘是平面圆、镜头斜视 → 投影为轴对齐椭圆（水平长轴 a、竖直短轴 b=k·a）。
    把点按 (x, y/k) 拉伸后在拉伸空间拟合圆，再映射回椭圆，兼顾精度与数值稳定。

    返回 (cx, cy, a, b, k, rms)：
      · 椭圆中心 (cx, cy)、水平半轴 a、竖直半轴 b(=k·a)、比例 k、径向残差 rms
      · 盘最低点 = (cx, cy + b)
    点太少/拟合退化返回 None。

    提速：max_pts>0 时先均匀抽稀到该点数再拟合（实测 87 点→40 点，耗时降一半，
    最低点几乎不变）；板端 numpy.lstsq 在这么小的矩阵上反而更慢，故用纯 Python 解析解。
    """
    if len(pts) < 12:
        return None
    n = len(pts)
    if max_pts and n > int(max_pts):
        stride = int(math.ceil(n / float(max_pts)))
        pts = pts[::stride]
        n = len(pts)
    best = None
    k = float(k_min)
    while k <= float(k_max) + 1e-9:
        fit = precision.fit_circle([(x, y / k) for (x, y) in pts])
        if fit is not None:
            X, Y, R = fit
            rms = _ellipse_rms(pts, X, k * Y, R, k * R)
            if best is None or rms < best[0]:
                best = (rms, k, X, k * Y, R, k * R)
        k += float(k_step)
    if best is None:
        return None
    rms, k, cx, cy, a, b = best
    return cx, cy, a, b, k, rms


def _reject_by_ellipse(pts, cx, cy, a, b, tol):
    """剔除径向残差超 tol 的边缘点（抗中间黑条/夹具/阴影等离群）。"""
    out = []
    for (x, y) in pts:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            continue
        if abs(d - _ellipse_radius(a, b, dx / d, dy / d)) <= tol:
            out.append((x, y))
    return out


def _gray_np(img):
    """把整帧转成 numpy 灰度数组 (h, w) uint8；不可用返回 None。

    为什么：板端 get_pixel 约 12µs/次，一次检测要 6000 次 ≈ 71ms；
    整帧转灰度只要 ~2.2ms，之后用 numpy 向量化采样 ~1ms（实测快 27 倍）。
    """
    try:
        import numpy as np
        if not hasattr(img, "to_format") or not hasattr(img, "to_bytes"):
            return None
        g = img.to_format(image.Format.FMT_GRAYSCALE)
        b = g.to_bytes()
        n = int(g.width()) * int(g.height())
        if len(b) < n:
            return None
        return np.frombuffer(b, np.uint8, count=n).reshape(g.height(), g.width())
    except Exception:
        return None


def _frame_level(gray, img=None, pct=90.0):
    """估计「盘面亮度」：优先 numpy 灰度（快）；否则用 get_pixel 稀疏采样（PC 单测用）。"""
    try:
        if gray is not None:
            import numpy as np
            return float(np.percentile(gray[::8, ::8], pct))
    except Exception:
        pass
    if img is None or not hasattr(img, "get_pixel"):
        return None
    vals = []
    w, h = img.width(), img.height()
    for y in range(0, h, 24):
        for x in range(0, w, 24):
            v = _pixel_luma(img, x, y)
            if v is not None:
                vals.append(v)
    if not vals:
        return None
    vals.sort()
    return float(vals[min(len(vals) - 1, int(len(vals) * pct / 100.0))])


def check_inside_bright(img, cx, cy, a, b, meta=None, gray=None,
                        n_rays=None, min_ratio=None):
    """校验拟合出的椭圆**内侧**确实是盘面亮度。

    为什么：电缆/立柱/阴影外边界也能拟合出一条"看起来合理"的弧（实测 rms 6~7，
    最低点偏 +60~+80px）。但它们的"内侧"是阴影或桌面（实测 123~154），
    而真盘缘内侧是盘面（196~211）——这条判据能把假弧整体否掉，
    又**不会**像"逐点过滤"那样把真盘缘的点裁掉（那会把最低点往里拉）。

    返回 (内侧达标比例, 需要达到的亮度, 有效射线数)；没有像素接口时返回 None。
    """
    b = float(b)
    a = float(a) if a else b
    if a <= 0 or b <= 0:
        return None
    if gray is None:
        gray = _gray_np(img)
    try:
        W, H = int(img.width()), int(img.height())
    except Exception:
        return None
    n_rays = int(n_rays if n_rays is not None
                 else getattr(config, "PLATE_INSIDE_RAYS", 40))
    min_ratio = float(min_ratio if min_ratio is not None
                      else getattr(config, "PLATE_INSIDE_MIN_RATIO", 0.60))
    lo = float(getattr(config, "PLATE_IN_BACK_LO", 14.0))
    hi = float(getattr(config, "PLATE_IN_BACK_HI", 34.0))
    arc = getattr(config, "PLATE_ARC_DEG", (-20.0, 200.0))
    a0, a1 = math.radians(float(arc[0])), math.radians(float(arc[1]))
    pct = float(getattr(config, "PLATE_IN_LEVEL_PCT", 90))
    lvl = _frame_level(gray, img, pct=pct)
    need = max(float(getattr(config, "PLATE_IN_MIN_ABS", 175.0)),
               (lvl - float(getattr(config, "PLATE_IN_MARGIN", 18.0)))
               if lvl is not None else 0.0)
    ok = tot = 0
    for i in range(max(6, n_rays)):
        ang = a0 + (a1 - a0) * i / max(1, n_rays - 1)
        ca, sa = math.cos(ang), math.sin(ang)
        rc = a if abs(a - b) < 1e-6 else _ellipse_radius(a, b, ca, sa)
        vals = []
        rr = rc - hi
        while rr <= rc - lo:
            x = int(round(cx + ca * rr))
            y = int(round(cy + sa * rr))
            if 0 <= x < W and 0 <= y < H:
                if gray is not None:
                    vals.append(float(gray[y, x]))
                else:
                    v = _pixel_luma(img, x, y)
                    if v is not None:
                        vals.append(v)
            rr += 2.0
        if not vals:
            continue
        tot += 1
        if sum(vals) / len(vals) >= need:
            ok += 1
    if tot <= 0:
        return None
    if meta is not None:
        meta["inside_need"] = round(need, 1)
        meta["inside_rays"] = tot
    return (float(ok) / float(tot), need, tot)


def _arc_edge_points_np(gray, cx, cy, r, band, rays, step, min_grad, a0, a1,
                        skip_bottom_deg=0.0):
    """`_arc_edge_points` 的 numpy 向量化版本（语义一致，快 ~20 倍）。

    精度上的两点改进（都是为了「最低点更准」）：
      1) **亚像素**：找到最大梯度后用相邻梯度做抛物线插值，不再量化到整像素
         （整像素量化带来的 0.3~0.5px 抖动会直接进最低点）；
      2) **底部加密**：在最低点附近（90°±PLATE_BOTTOM_DEG）额外加射线。
         盘最低点就是这段弧的切点，这里点密了，cx 与 b 才被「钉住」，
         而不会被两侧被物料挡住的点带偏。
    """
    import numpy as np
    h, w = gray.shape
    rays = max(8, int(rays))
    step = max(0.5, float(step))
    if isinstance(r, (tuple, list)) and len(r) == 2:
        ea, eb = float(r[0]), float(r[1])
    else:
        ea = eb = float(r)
    ang = a0 + (a1 - a0) * np.arange(rays, dtype=np.float64) / max(1, rays - 1)
    b_rays = int(getattr(config, "PLATE_RAYS_BOTTOM", 60))
    b_deg = float(getattr(config, "PLATE_BOTTOM_DEG", 25.0))
    if b_rays > 0 and b_deg > 0.5:
        c0 = math.pi / 2.0 - math.radians(b_deg)
        c1 = math.pi / 2.0 + math.radians(b_deg)
        add = c0 + (c1 - c0) * np.arange(max(2, b_rays), dtype=np.float64) / max(
            1, max(2, b_rays) - 1)
        ang = np.unique(np.concatenate([ang, add]))
    if skip_bottom_deg > 0:
        keep = np.abs(ang - math.pi / 2.0) > math.radians(float(skip_bottom_deg))
        ang = ang[keep]
    if ang.size == 0:
        return []
    ca, sa = np.cos(ang), np.sin(ang)
    if ea == eb:
        rc = np.full(ang.shape, ea)
    else:
        rc = ea * eb / np.sqrt((eb * ca) ** 2 + (ea * sa) ** 2)
    r0 = np.maximum(2.0, rc - float(band))
    r1 = np.maximum(r0 + step, rc + float(band))
    K = int(math.floor((2.0 * float(band)) / step)) + 2       # 每条射线的采样点数
    rr = r0[:, None] + step * np.arange(K, dtype=np.float64)[None, :]
    valid = rr <= (r1[:, None] + 1e-6)
    xs = np.rint(cx + ca[:, None] * rr).astype(np.int32)
    ys = np.rint(cy + sa[:, None] * rr).astype(np.int32)
    inb = valid & (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    xs_c = np.clip(xs, 0, w - 1)
    ys_c = np.clip(ys, 0, h - 1)
    vals = np.where(inb, gray[ys_c, xs_c].astype(np.int16), -1)
    both = inb[:, 1:] & inb[:, :-1]
    grad = np.where(both, vals[:, 1:] - vals[:, :-1], 0).astype(np.float32)
    ag = np.abs(grad)
    idx = np.argmax(ag, axis=1)
    rows = np.arange(ag.shape[0])
    best = ag[rows, idx]

    # ---- 内侧亮度约束：候选边往里 14~34px 必须是「盘面亮度」----
    # 电缆/立柱/阴影内部边界上方是阴影或桌面（实测 123~154），盘缘上方是盘面（196~211），
    # 两者天然分开 40+ 灰度，这条能把电缆这类强边缘直接排除。
    ok_in = np.ones(ag.shape[0], dtype=bool)
    if bool(getattr(config, "PLATE_IN_BRIGHT_ON", True)):
        try:
            pct = float(getattr(config, "PLATE_IN_LEVEL_PCT", 90))
            lvl = _frame_level(gray, pct=pct)
            need = max(float(getattr(config, "PLATE_IN_MIN_ABS", 175.0)),
                       (lvl - float(getattr(config, "PLATE_IN_MARGIN", 18.0)))
                       if lvl is not None else 0.0)
            b_lo = max(1, int(round(float(getattr(config, "PLATE_IN_BACK_LO", 14.0)) / step)))
            b_hi = max(b_lo + 1, int(round(float(getattr(config, "PLATE_IN_BACK_HI", 34.0)) / step)))
            kk = idx + 1                      # 最强梯度对应的采样列
            for i in range(ag.shape[0]):
                j1 = min(K, kk[i] - b_lo + 1)
                j0 = max(0, kk[i] - b_hi + 1)
                if j1 <= j0:
                    ok_in[i] = False
                    continue
                seg = vals[i, j0:j1]
                good = seg[seg >= 0]
                ok_in[i] = good.size > 0 and float(good.mean()) >= need
        except Exception:
            ok_in[:] = True
    best = np.where(ok_in, best, -1)
    if not bool(getattr(config, "PLATE_SUBPIXEL", False)):
        # 关闭亚像素 → 与旧版逐像素一致（保住现场已标定的落点口径）
        pts = []
        for i in np.nonzero(best >= float(min_grad))[0]:
            k = int(idx[i]) + 1
            pts.append((float(xs[i, k]), float(ys[i, k])))
        return pts
    # 亚像素插值（抛物线顶点，偏移量在 ±0.5 个采样步长内）
    il = np.clip(idx - 1, 0, ag.shape[1] - 1)
    ir = np.clip(idx + 1, 0, ag.shape[1] - 1)
    gl = grad[rows, il]
    g0 = grad[rows, idx]
    gr = grad[rows, ir]
    den = gl - 2.0 * g0 + gr
    den = np.where(np.abs(den) < 1e-6, 1e-6, den)
    sub = np.clip(0.5 * (gl - gr) / den, -0.5, 0.5)
    sub = np.where((idx > 0) & (idx < ag.shape[1] - 1), sub, 0.0)
    pts = []
    for i in np.nonzero(best >= float(min_grad))[0]:
        k = int(idx[i])
        ri = float(rr[i, k + 1]) + step * float(sub[i])
        pts.append((float(cx) + float(ca[i]) * ri,
                    float(cy) + float(sa[i]) * ri))
    return pts


def _arc_edge_points(img, cx, cy, r, band, rays, step, min_grad, a0, a1,
                     skip_bottom_deg=0.0, gray=None):
    """沿 a0..a1 的圆弧射线，在 r±band 内找亮度梯度最大的点（边缘点）。

    和 _radial_edge_points 的区别：只采样指定角度范围（物料盘只有下半弧可见），
    且每条约200点内取最大梯度。返回 [(x,y), ...]（像素坐标）。

    r 为标量时是正圆；为 (a, b) 二元组时是轴对齐椭圆（水平半轴 a、竖直半轴 b），
    每条射线按该方向上的椭圆半径采样——不然用单一半径去搜椭圆盘缘，两侧会搜不到。

    skip_bottom_deg>0 时跳过「正下方 ±该角度」的射线（夹具/阴影遮挡时用）。
    gray 传入整帧 numpy 灰度数组时走向量化路径（快 ~20 倍）；否则逐点 get_pixel。
    """
    if gray is None:
        gray = _gray_np(img)
    if gray is not None:
        try:
            return _arc_edge_points_np(gray, cx, cy, r, band, rays, step,
                                       min_grad, a0, a1, skip_bottom_deg)
        except Exception:
            pass
    if isinstance(r, (tuple, list)) and len(r) == 2:
        ea, eb = float(r[0]), float(r[1])
    else:
        ea = eb = float(r)
    pts = []
    w, h = img.width(), img.height()
    rays = max(8, int(rays))
    step = max(0.5, float(step))
    span = a1 - a0
    skip = math.radians(max(0.0, float(skip_bottom_deg)))
    b_rays = int(getattr(config, "PLATE_RAYS_BOTTOM", 60))
    b_deg = float(getattr(config, "PLATE_BOTTOM_DEG", 25.0))
    angles = [a0 + span * i / (rays - 1) for i in range(rays)]
    if b_rays > 0 and b_deg > 0.5:
        c0 = math.pi / 2.0 - math.radians(b_deg)
        c1 = math.pi / 2.0 + math.radians(b_deg)
        nb = max(2, b_rays)
        angles += [c0 + (c1 - c0) * i / (nb - 1) for i in range(nb)]
        angles.sort()
    for a in angles:
        if skip > 0.0 and abs(a - math.pi / 2.0) <= skip:
            continue
        ca, sa = math.cos(a), math.sin(a)
        rc = ea if ea == eb else _ellipse_radius(ea, eb, ca, sa)
        r0 = max(2.0, rc - float(band))
        r1 = max(r0 + step, rc + float(band))
        rr = r0
        prev = None
        gs = []          # [(半径, 带符号梯度)]
        vs = []          # [(半径, 亮度)]
        while rr <= r1:
            x = int(round(cx + ca * rr))
            y = int(round(cy + sa * rr))
            if 0 <= x < w and 0 <= y < h:
                v = _pixel_luma(img, x, y)
                if v is not None:
                    vs.append((rr, float(v)))
                    if prev is not None:
                        gs.append((rr, float(v - prev)))
                prev = v
            else:
                prev = None
            rr += step
        if not gs:
            continue
        bi, bg = 0, -1.0
        for i, (_, g) in enumerate(gs):
            if abs(g) > bg:
                bi, bg = i, abs(g)
        if bg < min_grad:
            continue
        sub = 0.0
        if 0 < bi < len(gs) - 1 and bool(getattr(config, "PLATE_SUBPIXEL", False)):
            gl, g0, gr = gs[bi - 1][1], gs[bi][1], gs[bi + 1][1]
            den = gl - 2.0 * g0 + gr
            if abs(den) > 1e-6:
                sub = max(-0.5, min(0.5, 0.5 * (gl - gr) / den))
        ri = gs[bi][0] + step * sub     # gs[bi][0] 已是「最强梯度那对里靠外」的采样点
        # 内侧亮度约束（与 numpy 路径同判据）
        if bool(getattr(config, "PLATE_IN_BRIGHT_ON", True)) and vs:
            lo = ri - float(getattr(config, "PLATE_IN_BACK_HI", 34.0))
            hi = ri - float(getattr(config, "PLATE_IN_BACK_LO", 14.0))
            seg = [v for (r, v) in vs if lo <= r <= hi]
            if seg:
                lvl = _frame_level(gray, img, pct=float(
                    getattr(config, "PLATE_IN_LEVEL_PCT", 90)))
                need = max(float(getattr(config, "PLATE_IN_MIN_ABS", 175.0)),
                           (lvl - float(getattr(config, "PLATE_IN_MARGIN", 18.0)))
                           if lvl is not None else 0.0)
                if sum(seg) / len(seg) < need:
                    continue
        if not bool(getattr(config, "PLATE_SUBPIXEL", False)):
            pts.append((float(int(round(cx + ca * ri))),
                        float(int(round(cy + sa * ri)))))
        else:
            pts.append((float(cx) + ca * ri, float(cy) + sa * ri))
    return pts


def _fit_bottom_parabola(pts, A, rounds=1):
    """给定曲率系数 A，最小二乘求底点 (x0, y0)：y = y0 − A·(x−x0)²。

    为什么把 A 固定：盘半径 330px 时，底点 ±26px 内的「拱高」只有 ~1px，
    自由拟合二次项会让顶点乱跑（三个系数彼此抵消）。A 由椭圆半轴给定
    （A = b / 2a²，正圆时 A = 1/2r），只留两个未知量 → 稳定得多。

    把 y + A·x² 看成 x 的线性函数：u = p + q·x → x0 = q/(2A)，y0 = p + A·x0²。
    返回 (x0, y0, npts, rms) 或 None。
    """
    A = float(A)
    if A <= 1e-9 or len(pts) < 6:
        return None
    cur = [(float(p[0]), float(p[1])) for p in pts]
    x0 = y0 = rms = 0.0
    for rd in range(max(0, int(rounds)) + 1):
        n = float(len(cur))
        if n < 6:
            return None
        sx = sum(p[0] for p in cur)
        sxx = sum(p[0] * p[0] for p in cur)
        su = sum(p[1] + A * p[0] * p[0] for p in cur)
        sxu = sum((p[1] + A * p[0] * p[0]) * p[0] for p in cur)
        den = n * sxx - sx * sx
        if abs(den) < 1e-9:
            return None
        q = (n * sxu - sx * su) / den
        p0 = (su - q * sx) / n
        x0 = q / (2.0 * A)
        y0 = p0 + A * x0 * x0
        rms = math.sqrt(sum((p[1] - (y0 - A * (p[0] - x0) ** 2)) ** 2
                            for p in cur) / n)
        if rd >= int(rounds):
            break
        keep = [p for p in cur
                if abs(p[1] - (y0 - A * (p[0] - x0) ** 2)) <= max(1.2, 2.5 * rms)]
        if len(keep) < 6 or len(keep) >= len(cur):
            break
        cur = keep
    return x0, y0, len(cur), rms


def measure_bottom_point(img, cx, cy, a, b, meta=None, gray=None,
                         half_w=None, up=None, down=None, min_grad=None,
                         min_pts=None):
    """直接测盘的**最低点**（不靠椭圆外推 cy+b）。

    为什么需要：全局拟合里 cy 与 b 是强相关的一对（只见下弧时更病态），
    两者误差会互相抵消一部分，但**底点本身**其实是个局部特征，可以单独测准。

    做法：以椭圆外推的底点为中心，在 ±half_w 列做**竖向**扫描（底边近水平，
    竖扫最灵敏），取最强向下梯度并做亚像素抛物插值，再按已知曲率 A 拟合
    抛物线，顶点即 (x0, y0)。返回 (x0, y0, npts, rms) 或 None。
    """
    try:
        W, H = int(img.width()), int(img.height())
    except Exception:
        return None
    if gray is None:
        gray = _gray_np(img)
    half_w = int(half_w if half_w is not None
                 else getattr(config, "PLATE_LOW_HALF_W", 26))
    up = float(up if up is not None else getattr(config, "PLATE_LOW_UP", 26.0))
    down = float(down if down is not None else getattr(config, "PLATE_LOW_DOWN", 10.0))
    min_grad = float(min_grad if min_grad is not None
                     else getattr(config, "PLATE_LOW_MIN_GRAD", 8.0))
    min_pts = max(6, int(min_pts if min_pts is not None
                         else getattr(config, "PLATE_LOW_MIN_PTS", 18)))
    a = float(a) if a else float(b)
    b = float(b)
    if a <= 0.0 or b <= 0.0:
        return None
    A = b / (2.0 * a * a)          # 底点附近的曲率系数
    y_exp = float(cy) + b
    y_lo = max(1, int(math.floor(y_exp - up)))
    y_hi = min(H - 2, int(math.ceil(y_exp + down)))
    x_lo = max(0, int(math.floor(float(cx) - half_w)))
    x_hi = min(W - 1, int(math.ceil(float(cx) + half_w)))
    if y_hi - y_lo < 4 or x_hi - x_lo < 8:
        return None

    pts = []
    if gray is not None:
        try:
            import numpy as np
            ys = np.arange(y_lo, y_hi + 1)
            xs = np.arange(x_lo, x_hi + 1)
            gm = gray[ys - 1][:, xs].astype(np.int32)
            gp = gray[ys + 1][:, xs].astype(np.int32)
            grad = gp - gm                       # 向下变亮为正
            absg = np.abs(grad)
            idx = np.argmax(absg, axis=0)
            cols = np.arange(xs.size)
            ok = (idx > 0) & (idx < ys.size - 1) & (absg[idx, cols] >= min_grad)
            if int(ok.sum()) >= min_pts:
                ic = idx[ok]
                cc = cols[ok]
                vm = grad[ic - 1, cc].astype(np.float32)
                v0 = grad[ic, cc].astype(np.float32)
                vp = grad[ic + 1, cc].astype(np.float32)
                den = vm - 2.0 * v0 + vp
                den = np.where(np.abs(den) < 1e-6, 1e-6, den)
                sub = np.clip(0.5 * (vm - vp) / den, -0.5, 0.5)
                yy = ys[ic].astype(np.float32) + sub
                pts = [(float(xs[c]), float(v)) for c, v in zip(cc, yy)]
        except Exception:
            pts = []

    if not pts:
        # 纯 Python 兜底（PC 单测的假图只有 get_pixel）
        for x in range(x_lo, x_hi + 1):
            gs = []
            for y in range(y_lo, y_hi + 1):
                va = _pixel_luma(img, x, y - 1)
                vb = _pixel_luma(img, x, y + 1)
                if va is None or vb is None:
                    gs.append(None)
                else:
                    gs.append(float(vb - va))
            bi, bg = -1, -1.0
            for i, g in enumerate(gs):
                if g is None or not (0 < i < len(gs) - 1):
                    continue
                if abs(g) > bg:
                    bi, bg = i, abs(g)
            if bi < 0 or bg < min_grad:
                continue
            vm = gs[bi - 1] or 0.0
            v0 = gs[bi] or 0.0
            vp = gs[bi + 1] or 0.0
            den = vm - 2.0 * v0 + vp
            sub = 0.0 if abs(den) < 1e-6 else max(-0.5, min(0.5, 0.5 * (vm - vp) / den))
            pts.append((float(x), float(y_lo + bi) + sub))

    if len(pts) < min_pts:
        if meta is not None:
            meta["low_direct_pts"] = len(pts)
        return None
    res = _fit_bottom_parabola(pts, A)
    if res is None:
        if meta is not None:
            meta["low_direct_pts"] = len(pts)
            meta["low_direct_reason"] = "parabola-degenerate"
        return None
    x0, y0, n, rms = res
    if n < min_pts:
        if meta is not None:
            meta["low_direct_pts"] = n
        return None
    if meta is not None:
        meta["low_direct_pts"] = n
        meta["low_direct_rms"] = round(rms, 2)
    return x0, y0, n, rms


def _circle_from_3(p1, p2, p3):
    """三点定圆（外接圆），退化返回 None。"""
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-6:
        return None
    a = x1 * x1 + y1 * y1
    b = x2 * x2 + y2 * y2
    c = x3 * x3 + y3 * y3
    ux = (a * (y2 - y3) + b * (y3 - y1) + c * (y1 - y2)) / d
    uy = (a * (x3 - x2) + b * (x1 - x3) + c * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    return ux, uy, r


def rim_scan_points(img, x_step=16, y0=24, y_step=3, dy=4, min_drop=25.0,
                    need_in=0.0, gray=None):
    """先验无关的全图粗扫盘缘：逐列找「亮→暗」最强的下降沿。

    盘缘是一条明显的暗线（实测亮→暗落差 50~150），桌面上没有这么强的边，
    所以这个扫描能在盘横向偏移很大（先验已经偏出搜索带）时，仍给出粗圆心的依据。
    返回 [(x, y), ...]；图像没有像素接口时返回 []。

    need_in>0 时追加**内侧亮度**约束：候选沿上方 14~34px 的均值必须 ≥ need_in。
    为什么必须加：现场的电缆/机械臂也是「上亮下暗」的强边，而且电缆是**弯的**，
    RANSAC 会把它当成盘缘拟出一个"合理"的圆（最低点偏 +60~80px）。
    盘面内侧实测 196~211，电缆上方只有 123~154，这条能把它们分开。

    注意：板端 get_pixel 约 15µs/次，本函数是「贵」操作（~200ms），
    find_disc_arc 只在「配置先验那轮不可信」时才调用它（惰性）。
    """
    if not hasattr(img, "get_pixel"):
        return []
    w, h = img.width(), img.height()
    pts = []
    ys = max(1, int(y_step))
    lo = float(getattr(config, "PLATE_IN_BACK_LO", 14.0))
    hi = float(getattr(config, "PLATE_IN_BACK_HI", 34.0))

    # 预计算「内侧带均值」图：in_mean[y, x] = mean(gray[y-hi : y-lo, x])
    # 这样每列每个候选 y 都能 O(1) 判断内侧亮不亮，不必逐点采样。
    in_mean = None
    if need_in > 0.0:
        try:
            import numpy as np
            if gray is not None:
                g = gray.astype(np.float32)
            else:
                g = np.array([[float(_pixel_luma(img, x, y) or 0)
                               for x in range(w)] for y in range(h)],
                             dtype=np.float32)
            cs = np.concatenate([np.zeros((1, g.shape[1]), np.float32),
                                 np.cumsum(g, axis=0)], axis=0)
            iy = np.arange(g.shape[0])
            a = np.clip(iy - int(round(hi)), 0, g.shape[0])
            b = np.clip(iy - int(round(lo)), 0, g.shape[0])
            cnt = np.maximum(b - a, 1).astype(np.float32)
            in_mean = (cs[b] - cs[a]) / cnt[:, None]
        except Exception:
            in_mean = None

    for x in range(int(x_step), int(w) - 1, int(x_step)):
        best_g, by = -1.0, None
        y = int(y0)
        while y + dy < h:
            va = _pixel_luma(img, x, y)
            vb = _pixel_luma(img, x, y + dy)
            if va is not None and vb is not None:
                g = va - vb
                if g > best_g:
                    yy = y + dy // 2
                    if need_in > 0.0:
                        # 只有「内侧是盘面亮度」的候选才参与竞争
                        ok = False
                        if in_mean is not None:
                            ok = float(in_mean[yy, x]) >= float(need_in)
                        else:
                            vals = []
                            for vv in range(max(0, yy - int(round(hi))),
                                            max(1, yy - int(round(lo))), 2):
                                v = _pixel_luma(img, x, vv)
                                if v is not None:
                                    vals.append(float(v))
                            ok = bool(vals) and (sum(vals) / len(vals)) >= float(need_in)
                        if not ok:
                            y += ys
                            continue
                    best_g, by = g, yy
            y += ys
        if by is None or best_g < float(min_drop):
            continue
        pts.append((x, by))
    return pts


def coarse_disc_prior(img, pts=None, iters=120, tol=8.0, min_inliers=12,
                      r_min=60.0, r_max=900.0, seed=12345,
                      x_step=16, y0=24, y_step=3, need_in=0.0, gray=None):
    """粗定位盘（先验无关）：边缘点 + RANSAC 圆拟合 → (cx, cy, r, ninl) 或 None。

    只用圆做「粗」定位（RANSAC 抗离群：中间黑条/夹具/线缆/桌面边）；
    后续仍由 find_disc_arc 用椭圆模型在它附近精修。
    need_in>0 时把「内侧必须是盘面亮度」传给 rim_scan_points（挡掉电缆/机械臂）。
    """
    if pts is None:
        pts = rim_scan_points(img, x_step=x_step, y0=y0, y_step=y_step,
                              need_in=need_in, gray=gray)
    n = len(pts)
    if n < max(4, int(min_inliers)):
        return None
    try:
        import random
        rnd = random.Random(int(seed))
    except Exception:
        return None
    best = None
    for _ in range(max(10, int(iters))):
        i1, i2, i3 = rnd.sample(range(n), 3)
        fit = _circle_from_3(pts[i1], pts[i2], pts[i3])
        if fit is None:
            continue
        cx, cy, r = fit
        if not (float(r_min) <= r <= float(r_max)):
            continue
        k = 0
        for (x, y) in pts:
            if abs(math.hypot(x - cx, y - cy) - r) <= float(tol):
                k += 1
        if best is None or k > best[3]:
            best = (cx, cy, r, k)
    if best is None or best[3] < int(min_inliers):
        return None
    cx, cy, r, k = best
    inl = [p for p in pts if abs(math.hypot(p[0] - cx, p[1] - cy) - r) <= float(tol)]
    f2 = precision.fit_circle(inl)
    if f2 is not None:
        return f2[0], f2[1], f2[2], len(inl)
    return cx, cy, r, k


def _fit_arc_once(img, cx, cy, r, bands=None, rays=None, step=None,
                  min_grad=None, arc_deg=None, meta=None,
                  skip_bottom_deg=None, gray=None):
    """物料盘下弧检测：从粗圆心沿下半弧找边缘点，迭代拟合「轴对齐椭圆」。

    为什么需要：物料盘（白色带孔大圆盘）常常只拍得到下半弧，圆心落在画面外，
    find_circles / Hough 找不到整圆；但可见的下弧足够外推出圆心和半径。

    模型：盘是平面圆，镜头斜视 → 投影是**透视椭圆**（水平半轴 a ≥ 竖直半轴 b）。
    用圆去套这条弧会系统性带偏（实测：边缘半径随角度极差约 12px，圆拟合残差
    ~3~4px；最低点比真实低 ~3.5px，且绿圆在两侧明显翘离盘缘）。
    故默认用 fit_arc_ellipse（PLATE_ARC_MODEL="ellipse"），可退回 "circle" 对比。

    参数 cx/cy/r 是「粗圆心 + 粗半径」先验（圆心可为负），bands 是几轮搜索带半径
    （大→小逐步精修，可容忍物料盘移动）。

    返回 (cx, cy, r)，其中 r 已换成**竖直半轴 b**：盘最低点 = (cx, cy + r)，
    这样调用方（tasks.task_plate）的最低点公式无需改动。
    meta 会写入 model/a/b/k/rms/npts，供绘制与调试输出使用。
    """
    if not hasattr(img, "get_pixel"):
        if meta is not None:
            meta["reason"] = "no-pixel-api"
        return None
    bands = bands if bands is not None else getattr(
        config, "PLATE_SEARCH_BANDS", (60.0, 28.0, 16.0))
    if isinstance(bands, (int, float)):
        bands = (float(bands),)
    rays = int(rays if rays is not None else getattr(config, "PLATE_RAYS", 140))
    step = float(step if step is not None else getattr(config, "PLATE_STEP", 2.0))
    min_grad = float(min_grad if min_grad is not None
                     else getattr(config, "PLATE_MIN_GRAD", 8.0))
    arc = arc_deg if arc_deg is not None else getattr(
        config, "PLATE_ARC_DEG", (-20.0, 200.0))
    a0, a1 = math.radians(float(arc[0])), math.radians(float(arc[1]))
    model = str(getattr(config, "PLATE_ARC_MODEL", "ellipse")).strip().lower()
    robust = bool(getattr(config, "PLATE_ARC_ROBUST", True))
    out_px = float(getattr(config, "PLATE_ARC_OUTLIER_PX", 6.0))
    k_min = float(getattr(config, "PLATE_ELLIPSE_K_MIN", 0.80))
    k_max = float(getattr(config, "PLATE_ELLIPSE_K_MAX", 1.15))
    k_step = float(getattr(config, "PLATE_ELLIPSE_K_STEP", 0.01))
    use_ellipse = model in ("ellipse", "ellip", "conic")
    skip_bottom = float(skip_bottom_deg if skip_bottom_deg is not None
                        else getattr(config, "PLATE_ARC_SKIP_BOTTOM_DEG", 0.0))
    fit_max_pts = int(getattr(config, "PLATE_FIT_MAX_PTS", 48))
    robust_rounds = max(0, int(getattr(config, "PLATE_ROBUST_ROUNDS", 1)))
    gray = gray if gray is not None else _gray_np(img)   # 一帧只转一次灰度
    cx, cy, r = float(cx), float(cy), float(r)
    ea = eb = None
    for band in bands:
        shape = (ea, eb) if (use_ellipse and ea is not None) else r
        pts = _arc_edge_points(img, cx, cy, shape, float(band), rays, step,
                               min_grad, a0, a1, skip_bottom, gray=gray)
        if meta is not None:
            meta["npts"] = len(pts)
        if len(pts) < 12:
            if meta is not None:
                meta["reason"] = "points=%d<12" % len(pts)
            return None
        if use_ellipse:
            fit = fit_arc_ellipse(pts, k_min, k_max, k_step, max_pts=fit_max_pts)
            if fit is None:
                if meta is not None:
                    meta["reason"] = "ellipse-degenerate"
                return None
            cx, cy, ea, eb, kk, rms = fit
            # 鲁棒重拟合：剔除离群边缘点（中间黑条/夹具/阴影）再拟合一次
            if robust:
                for _ in range(robust_rounds):
                    keep = _reject_by_ellipse(pts, cx, cy, ea, eb,
                                              max(out_px, 2.0 * rms))
                    if len(keep) < 12 or len(keep) >= len(pts):
                        break
                    f2 = fit_arc_ellipse(keep, k_min, k_max, k_step, max_pts=fit_max_pts)
                    if f2 is None:
                        break
                    cx, cy, ea, eb, kk, rms = f2
                    pts = keep
            r = eb
            if meta is not None:
                meta["model"] = "ellipse"
                meta["a"] = round(ea, 1)
                meta["b"] = round(eb, 1)
                meta["k"] = round(kk, 3)
                meta["rms"] = round(rms, 2)
        else:
            fit = precision.fit_circle(pts)
            if fit is None:
                if meta is not None:
                    meta["reason"] = "fit-degenerate"
                return None
            cx, cy, r = fit
            ea = eb = None
            if meta is not None:
                meta["model"] = "circle"
                meta["rms"] = round(_circle_rms(pts, cx, cy, r), 2)
                meta["npts"] = len(pts)
    if meta is not None:
        meta["reason"] = "ok(%s,npts=%d)" % (meta.get("model", "?"),
                                             meta.get("npts", 0))
    return cx, cy, r


def _circle_rms(pts, cx, cy, r):
    s = 0.0
    for (x, y) in pts:
        s += (math.hypot(x - cx, y - cy) - r) ** 2
    return math.sqrt(s / len(pts)) if pts else 1e9


def find_disc_arc(img, cx, cy, r, bands=None, rays=None, step=None,
                  min_grad=None, arc_deg=None, meta=None):
    """物料盘下弧检测（对外入口）：粗定位 + 多先验尝试 + 质量闸门。

    为什么这么做（现场实测）：H 的配置先验是固定的，但盘相对车会**横向偏移很大**
    （用户拍的 other_0422~0444）。先验一旦偏出搜索带（±PLATE_SEARCH_BANDS[0]），
    算法会沿旧弧扫到桌面/夹具/线缆，拟出**假圆**（rms 6~11 却照样回 DISC）并驱动
    错误的 MOVEXY。所以：

      1) 先验无关的**粗定位**（rim_scan_points + RANSAC 圆）→ 抗横向大偏移；
      2) 粗定位先验与配置先验**各试一轮**，取「可信且 rms 最小」的那个；
      3) **质量闸门**（rms>PLATE_MAX_RMS 或 npts<PLATE_MIN_PTS）→ 判不可信，
         返回 None（上层回 NONE,H，让上位机重摆），绝不回假圆。

    实测 35 张真实图：可信拟合 rms ≤ 3.0 / npts ≥ 44；假圆 rms ≥ 5.96。闸门取
    rms ≤ 4.0（判据干净）＋ npts ≥ 30（只防退化）。

    参数 cx/cy/r 是配置先验（圆心可为负）。返回 (cx, cy, r)，其中 r 在椭圆模型下
    是**竖直半轴 b**：盘最低点 = (cx, cy + r)。meta 写入 model/a/b/k/rms/npts/
    coarse/prior_used/tries，供绘制与调试使用。
    """
    if not hasattr(img, "get_pixel"):
        if meta is not None:
            meta["reason"] = "no-pixel-api"
        return None
    cx0, cy0, r0 = float(cx), float(cy), float(r)
    d = {} if meta is None else meta
    gray_all = _gray_np(img)      # 整帧灰度只转一次（拟合 + 直测底点共用）
    gate = bool(getattr(config, "PLATE_QUALITY_GATE", True))
    max_rms = float(getattr(config, "PLATE_MAX_RMS", 4.0))
    min_pts = int(getattr(config, "PLATE_MIN_PTS", 30))
    tries = []

    # ---- 几何硬校验（与「准不准」无关）：最低点跑出画面 / 半径离谱 → 直接作废 ----
    _W, _H = img.width(), img.height()
    geom_on = bool(getattr(config, "PLATE_GEOM_CHECK", True))
    _ly_lo = float(getattr(config, "PLATE_LOW_Y_MIN_RATIO", 0.25)) * _H
    _ly_hi = _H - 1.0
    _lxr = float(getattr(config, "PLATE_LOW_X_EDGE_RATIO", 0.05))
    _b_lo = r0 * float(getattr(config, "PLATE_GEOM_B_RATIO_MIN", 0.40))
    _b_hi = r0 * float(getattr(config, "PLATE_GEOM_B_RATIO_MAX", 2.00))

    def _geom_ok(res):
        if not geom_on:
            return True, ""
        cx_, cy_, b_ = float(res[0]), float(res[1]), float(res[2])
        low_y = cy_ + b_
        if not (_ly_lo <= low_y <= _ly_hi):
            return False, "low_y=%.0f 不在 %.0f~%.0f" % (low_y, _ly_lo, _ly_hi)
        if not (_lxr * _W <= cx_ <= (1.0 - _lxr) * _W):
            return False, "low_x=%.0f 越界" % cx_
        if not (_b_lo <= b_ <= _b_hi):
            return False, "b=%.0f 不在 %.0f~%.0f" % (b_, _b_lo, _b_hi)
        return True, ""

    def _try(tag, px, py, pr, skip=None):
        """试一轮：返回 (rank, rms, tag, res, meta_of_this_try)；失败返回 None。

        rank: 0=可信且几何合理；1=低置信但几何合理；2=几何不合理（不可用）。
        """
        m2 = {}
        res = _fit_arc_once(img, px, py, pr, bands, rays, step, min_grad,
                            arc_deg, m2, skip_bottom_deg=skip, gray=gray_all)
        label = tag if not skip else "%s/skip%g" % (tag, skip)
        if res is None:
            tries.append({"prior": label, "reason": str(m2.get("reason", ""))[:40]})
            return None
        rms = float(m2.get("rms", 0.0))
        npts = int(m2.get("npts", 0))
        credible = (not gate) or (rms <= max_rms and npts >= min_pts)
        g_ok, g_why = _geom_ok(res)
        tries.append({"prior": label, "rms": rms, "npts": npts,
                      "credible": bool(credible), "geom": g_ok,
                      "low": [round(res[0], 1), round(res[1] + res[2], 1)],
                      **({} if g_ok else {"geom_why": g_why})})
        rank = 0 if (credible and g_ok) else (1 if g_ok else 2)
        return (rank, rms, label, res, m2)

    base_skip = float(getattr(config, "PLATE_ARC_SKIP_BOTTOM_DEG", 0.0))
    alt_skip = float(getattr(config, "PLATE_ARC_SKIP_BOTTOM_ALT", 30.0))
    skips = [base_skip]
    if bool(getattr(config, "PLATE_ARC_SKIP_ALT_ON", True)) and alt_skip > base_skip:
        skips.append(alt_skip)

    def _best_of(priors):
        """按优先级试若干 (tag, cx, cy, r) × skips，返回可信且 rms 最小的候选。"""
        out = None
        for (tag, px, py, pr) in priors:
            for sk in skips:
                e = _try(tag, px, py, pr, sk)
                if e is not None and (out is None or (e[0], e[1]) < (out[0], out[1])):
                    out = e
                if out is not None and out[0] == 0:
                    return out        # 已经可信 → 不再试（省耗时）
        return out

    # ① 先试「配置先验」——正常位姿走这条，零额外开销
    best = _best_of([("config", cx0, cy0, r0)])
    # ①b 内侧亮度门槛（粗定位/直测共用）：帧内盘面亮度 p90 往下留 18 灰阶
    _lvl = _frame_level(gray_all, pct=float(getattr(config, "PLATE_IN_LEVEL_PCT", 90)))
    _need_in = max(float(getattr(config, "PLATE_IN_MIN_ABS", 175.0)),
                   (_lvl - float(getattr(config, "PLATE_IN_MARGIN", 18.0)))
                   if _lvl is not None else 0.0) \
        if bool(getattr(config, "PLATE_IN_BRIGHT_ON", True)) or \
        bool(getattr(config, "PLATE_INSIDE_CHECK", True)) else 0.0
    d["plate_level"] = None if _lvl is None else round(_lvl, 1)
    d["need_in"] = round(_need_in, 1)
    # ② 仍不可信时才做「先验无关的粗定位」补救（贵，惰性）
    if (best is None or best[0] != 0) and bool(getattr(config, "PLATE_COARSE_LOCATE", True)):
        cp = coarse_disc_prior(
            img,
            iters=int(getattr(config, "PLATE_COARSE_ITERS", 120)),
            tol=float(getattr(config, "PLATE_COARSE_TOL", 8.0)),
            min_inliers=int(getattr(config, "PLATE_COARSE_MIN_INLIERS", 12)),
            r_min=float(getattr(config, "PLATE_COARSE_R_MIN", 150.0)),
            r_max=float(getattr(config, "PLATE_COARSE_R_MAX", 560.0)),
            x_step=int(getattr(config, "PLATE_COARSE_X_STEP", 16)),
            y0=int(getattr(config, "PLATE_COARSE_Y0", 24)),
            y_step=int(getattr(config, "PLATE_COARSE_Y_STEP", 3)),
            need_in=_need_in if bool(getattr(config, "PLATE_COARSE_INSIDE", True)) else 0.0,
            gray=gray_all)
        lo = r0 * float(getattr(config, "PLATE_COARSE_R_RATIO_MIN", 0.55))
        hi = r0 * float(getattr(config, "PLATE_COARSE_R_RATIO_MAX", 1.60))
        if cp is not None and lo <= cp[2] <= hi:
            d["coarse"] = [round(cp[0], 1), round(cp[1], 1), round(cp[2], 1),
                           int(cp[3])]
            e2 = _best_of([("coarse", float(cp[0]), float(cp[1]), float(cp[2]))])
            if e2 is not None and (best is None or
                                   (e2[0], e2[1]) < (best[0], best[1])):
                best = e2
        else:
            d["coarse_reject"] = ("none" if cp is None
                                  else "r=%.0f 不在 %.0f~%.0f" % (cp[2], lo, hi))
    d["tries"] = tries
    if best is None:
        d["reason"] = "no-fit"
        return None
    if best[0] == 2:
        # 几何不合理（最低点跑出画面/半径离谱）→ 任何模式都不执行
        d["reason"] = "geom-reject(最好 rms=%.2f low=%s)" % (
            best[1], tries[-1].get("low") if tries else None)
        d["rejected"] = True
        return None
    rank, rms, tag, res, m2 = best
    for k, v in m2.items():
        d[k] = v
    d["prior_used"] = tag

    # ---- 底点交叉校验：局部直测 vs 椭圆外推 ----
    # 两个独立估计（全局拟合 / 局部直测）差得太多，说明这一帧的拟合可疑 →
    # 标 low_conf（report 模式下仍执行，只是日志里能看到）。
    # PLATE_LOW_USE_DIRECT=True 时直接用直测值（会改变落点口径，需重新标定）。
    _mismatch = False
    if bool(getattr(config, "PLATE_LOW_CHECK", True)):
        _b = float(res[2])
        _ea = float(d.get("a") or _b)
        d["low_ellipse"] = [round(float(res[0]), 1), round(float(res[1]) + _b, 1)]
        lo = measure_bottom_point(img, float(res[0]), float(res[1]), _ea, _b,
                                  meta=d, gray=gray_all)
        if lo is None:
            d["low_direct"] = None
        else:
            x0, y0 = float(lo[0]), float(lo[1])
            d["low_direct"] = [round(x0, 1), round(y0, 1), int(lo[2]),
                               round(float(lo[3]), 2)]
            _dxm = x0 - float(res[0])
            _dym = y0 - (float(res[1]) + _b)
            d["low_delta"] = [round(_dxm, 1), round(_dym, 1)]
            _cx_ = float(getattr(config, "PLATE_LOW_CHECK_DX", 15.0))
            _cy_ = float(getattr(config, "PLATE_LOW_CHECK_DY", 12.0))
            if abs(_dxm) > _cx_ or abs(_dym) > _cy_:
                d["low_check"] = "mismatch"
                d["low_conf"] = True       # 两个估计不一致 → 可疑（下面会被覆盖）
                _mismatch = True
            else:
                d["low_check"] = "ok"
                _mismatch = False
            if bool(getattr(config, "PLATE_LOW_USE_DIRECT", False)):
                _mx = float(getattr(config, "PLATE_LOW_MAX_DX", 45.0))
                _my = float(getattr(config, "PLATE_LOW_MAX_DY", 25.0))
                cand = (x0, float(res[1]), y0 - float(res[1]))
                _ok2, _why2 = _geom_ok(cand)
                if abs(_dxm) <= _mx and abs(_dym) <= _my and _ok2:
                    res = cand
                    d["low_used"] = "direct"
                else:
                    d["low_used"] = "ellipse"
                    d["low_reject"] = "dx=%.1f dy=%.1f geom=%s%s" % (
                        abs(_dxm), abs(_dym), _ok2,
                        "" if _ok2 else ("/" + _why2))

    d["low_conf"] = bool(gate and rank != 0) or bool(_mismatch)

    # ---- 内侧亮度校验：拟合出的弧内侧必须是盘面 ----
    # 电缆/立柱/阴影也能拟合出"合理"的弧（最低点偏 +60~80px），但内侧是暗的。
    if bool(getattr(config, "PLATE_INSIDE_CHECK", True)):
        chk = check_inside_bright(img, float(res[0]), float(res[1]),
                                  float(d.get("a") or res[2]), float(res[2]),
                                  meta=d, gray=gray_all)
        if chk is not None:
            ratio, need, tot = chk
            d["inside_ratio"] = round(ratio, 2)
            _minr = float(getattr(config, "PLATE_INSIDE_MIN_RATIO", 0.60))
            if ratio < _minr:
                d["inside_check"] = "fail"
                mode2 = str(getattr(config, "PLATE_INSIDE_ON_FAIL",
                                    "reject")).strip().lower()
                if mode2 in ("reject", "none", "strict"):
                    d["reason"] = "inside-fail(%.2f<%.2f, need=%.0f, n=%d)" % (
                        ratio, _minr, need, tot)
                    d["rejected"] = True
                    return None
                d["low_conf"] = True
            else:
                d["inside_check"] = "ok"
    if gate and rank != 0:
        # 现场要求：不准确也要执行完 → 默认 "report"（只标记低置信，照常返回最佳猜测）
        # 想恢复「不可信就回 NONE」的严格模式：PLATE_GATE_MODE="reject"
        mode = str(getattr(config, "PLATE_GATE_MODE", "report")).strip().lower()
        d["reject_reason"] = "rms=%.2f npts=%d" % (rms, m2.get("npts", 0))
        if mode in ("reject", "none", "strict"):
            d["reason"] = "reject(最好 rms=%.2f npts=%d)" % (
                rms, m2.get("npts", 0))
            d["rejected"] = True
            return None
        d["reason"] = "lowconf(%s,rms=%.2f,npts=%d,prior=%s)" % (
            d.get("model", "?"), rms, m2.get("npts", 0), tag)
        return res
    d["reason"] = "ok(%s,npts=%d,prior=%s)" % (d.get("model", "?"),
                                               d.get("npts", 0), tag)
    return res


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


def verify_target2(img, cx, cy, meta=None, max_shift=None):
    """校验候选圆内部确实有数字“2”。

    只有模板匹配通过才认为目标有效；模板缺失时返回 True（不阻断旧行为）。

    max_shift: 匹配到的“2”中心离候选圆心超过这个距离就**不算匹配**
    （环距只有 RING_SPACING_PX=22px，没有这个限制时会锁到旁边别的环上）。
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
    far = None
    for i, tpl in enumerate(tpls):
        try:
            r = crop.find_template(tpl, th, step=2)
        except Exception as e:
            if meta is not None:
                meta["tpl_error"] = str(e)
            continue
        if r:
            tx = int(r[0]) + x
            ty = int(r[1]) + y
            tw, thh = int(r[2]), int(r[3])
            mx = tx + tw / 2.0
            my = ty + thh / 2.0
            if max_shift is not None:
                d = math.hypot(mx - float(cx), my - float(cy))
                if d > float(max_shift):
                    # 离候选圆心太远 → 大概率锁到旁边别的环，换下一个尺度再试
                    far = [round(mx, 1), round(my, 1), round(d, 1)]
                    continue
            if meta is not None:
                meta["tpl"] = [tx, ty, tw, thh]
                meta["tpl_scale"] = float(scales[i]) if i < len(scales) else 1.0
                meta["tpl_roi"] = roi
                meta.pop("tpl_far", None)
            return True
    if meta is not None:
        meta["tpl"] = None
        meta["tpl_roi"] = roi
        if far is not None:
            meta["tpl_far"] = far
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
        """内容校验：候选圆附近必须能匹配到数字“2”，否则丢弃。

        校验通过时**直接用数字“2”的匹配位置作为环组中心**：
        数字“2”画在环组正中，实测模板 score>0.9、比黑环质心 / 霍夫大圈都稳；
        这样无论前面走的是 inner / black / cluster，最终中心都会统一到“2”的位置，
        彻底消除同一目标在不同路径间来回跳变（D 中心不稳的根因）。
        """
        if p is None:
            return None
        if not bool(getattr(config, "DISC_TEMPLATE_ON", True)):
            return p
        m2 = {}
        _sh = float(getattr(config, "DISC_TEMPLATE_MAX_SHIFT", 22.0))
        _lim = min(_sh, max(12.0, 0.35 * float(p[2])))
        if verify_target2(img, p[0], p[1], meta=m2, max_shift=_lim):
            if meta is not None:
                meta.update(m2)
            tpl = m2.get("tpl")
            if tpl and len(tpl) == 4:
                tx, ty, tw, th = tpl
                cx2 = int(round(tx + tw / 2.0))
                cy2 = int(round(ty + th / 2.0))
                if meta is not None:
                    meta["center_src"] = "tpl2"
                return (cx2, cy2, p[2])
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

    try:
        # find_circles 的霍夫累加器占用「fast frame buffer stack」，**ROI 越大越吃**。
        # 现场实测：ROI 放大到 448×400 后日志开始刷
        #   MemoryError: Out of fast Frame Buffer Stack Memory
        # 之后进程直接被系统干掉（无 APP EXIT 记录）。所以给霍夫单独限幅，
        # blob 路径（inner/black）仍用完整大 ROI，不受影响。
        hroi = _cap_roi(roi)
        if meta is not None and hroi != roi:
            meta["hough_roi"] = hroi
        circles = img.find_circles(roi=hroi if hroi else [], threshold=threshold,
                                   x_margin=10, y_margin=10, r_margin=10,
                                   r_min=r_min, r_max=r_max)
    except Exception as e:
        # ROI 放大后 find_circles 的累加器缓冲也变大，内存紧张时会抛
        # MemoryError（Out of fast Frame Buffer Stack Memory）——
        # 这里兜住，退到圆环兜底，绝不让整个 D/E 任务挂掉。
        if meta is not None:
            meta["circles_error"] = str(e)[:60]
        print("[视觉] find_circles 失败，走兜底: %s" % e)
        circles = []
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


def _cap_roi(roi, max_w=None, max_h=None):
    """把 ROI 居中裁剪到上限尺寸。

    为什么：find_circles 的霍夫累加器按 ROI 面积吃「fast frame buffer stack」。
    现场实测 448×400 会把内存池耗干（日志刷 MemoryError，
    随后堆被破坏 → refine_disc_center 里一个属性访问就段错误、进程被杀）。
    所以凡是调霍夫的地方都先限幅，回到历史稳定尺寸。
    """
    if not roi or len(roi) != 4:
        return roi
    try:
        _cap = getattr(config, "DISC_HOUGH_ROI_MAX", (340, 320))
        cw = int(max_w if max_w is not None else _cap[0])
        ch = int(max_h if max_h is not None else _cap[1])
        x, y, w, h = [int(v) for v in roi]
        if w <= cw and h <= ch:
            return roi
        nw, nh = min(w, cw), min(h, ch)
        return [x + (w - nw) // 2, y + (h - nh) // 2, nw, nh]
    except Exception:
        return roi


def find_ring_center(img, roi=None,
                     r_min=config.STACK_RING_R_MIN, r_max=config.STACK_RING_R_MAX,
                     threshold=config.STACK_RING_THRESHOLD):
    """找圆环中心（码垛时没有底座，用来回"空环中心"）

    取离 ROI 中心最近的圆。返回 (cx, cy, r) 或 None。
    """
    roi = roi if roi else config.ROI_STACK
    if not roi:
        return None
    roi = _cap_roi(roi)          # 霍夫必须限幅，否则耗干内存池（见 _cap_roi 注释）
    x, y, w, h = roi
    cx0, cy0 = x + w / 2.0, y + h / 2.0
    try:
        circles = img.find_circles(roi=roi, threshold=threshold,
                                   x_margin=10, y_margin=10, r_margin=10,
                                   r_min=r_min, r_max=r_max)
    except Exception as e:
        print("[视觉] find_ring_center 霍夫失败: %s" % e)
        return None
    if not circles:
        return None
    c = min(circles, key=lambda c: (c.x() - cx0) ** 2 + (c.y() - cy0) ** 2)
    return c.x(), c.y(), c.r()


def find_stack_color_center_ai(ai, img, roi, color_code,
                               tile=None, stride=None,
                               min_prob=None, min_tiles=None):
    """用 AI 分类器在 ROI 内分块扫描，找指定颜色的中心（CV 找不到底座时兜底）

    为什么需要：F,<颜色> 原来先用 LAB 阈值找色块，再用 AI 判色。
    如果现场色差/反光导致阈值一个候选都没有，AI 根本没机会介入，最后退到空环中心。
    这里直接让 AI 在画面上滑动分类，命中的块取加权中心，作为颜色中心。

    返回 dict（cx/cy/x/y/w/h/src="ai"/code/ai_prob/ai_tiles），找不到返回 None。
    """
    if ai is None or not roi or not color_code:
        return None
    try:
        if not ai.ready():
            return None
    except Exception:
        return None

    try:
        iw, ih = img.width(), img.height()
    except Exception:
        return None
    x0, y0, rw, rh = [int(v) for v in roi]
    x0 = max(0, min(x0, iw - 1))
    y0 = max(0, min(y0, ih - 1))
    rw = max(1, min(rw, iw - x0))
    rh = max(1, min(rh, ih - y0))

    tile = int(tile if tile is not None else getattr(config, "STACK_AI_TILE", 96))
    tile = max(24, min(tile, rw, rh))
    stride = int(stride if stride is not None else getattr(config, "STACK_AI_STRIDE", tile // 2))
    stride = max(8, min(stride, tile))
    min_prob = float(min_prob if min_prob is not None
                     else getattr(config, "STACK_AI_MIN_PROB", 0.70))
    min_tiles = max(1, int(min_tiles if min_tiles is not None
                           else getattr(config, "STACK_AI_MIN_TILES", 1)))

    code_map = getattr(config, "AI_CLASS_TO_COLOR", {})
    target = str(color_code)
    # 蓝色最容易把浅蓝/白底/反光误判进来，单独提高置信度门槛
    if target == COLOR_BLUE:
        min_prob = max(min_prob, float(getattr(config, "STACK_AI_BLUE_MIN_PROB", 0.90)))
    hits = []
    best = (0.0, None)
    xs = list(range(x0, x0 + rw - tile + 1, stride))
    ys = list(range(y0, y0 + rh - tile + 1, stride))
    if not xs or xs[-1] != x0 + rw - tile:
        xs.append(max(x0, x0 + rw - tile))
    if not ys or ys[-1] != y0 + rh - tile:
        ys.append(max(y0, y0 + rh - tile))

    for ty in ys:
        for tx in xs:
            box = [tx, ty, tile, tile]
            try:
                label, prob, idx = ai.classify_label(img, box)
            except Exception:
                continue
            if idx is None:
                continue
            code = code_map.get(label)
            if prob > best[0]:
                best = (prob, (label, code, tx + tile // 2, ty + tile // 2))
            if code == target and prob >= min_prob:
                hits.append((tx + tile / 2.0, ty + tile / 2.0, float(prob),
                             tx, ty))

    if len(hits) < min_tiles:
        # 单块高置信也认可：物料在 96px 网格里经常只压住一个块，
        # 硬要求 2 块会让「明明认得对」的情况一直未命中（现场 F,1 实测 108 次）。
        if len(hits) == 1 and hits[0][2] >= float(
                getattr(config, "STACK_AI_SINGLE_TILE_PROB", 0.90)):
            min_tiles = 1
    if len(hits) < min_tiles:
        if getattr(config, "STACK_AI_DEBUG", False):
            print("[AI][F] 未命中 color=%s hits=%d best=%s prob=%.3f" %
                  (target, len(hits), best[1] if best[1] else None, best[0]))
        return None

    # 相邻命中块聚成一簇；优先块数多的簇，再优先离 ROI 中心近的
    groups = []
    link = max(stride * 1.6, 12.0)
    for h in hits:
        for g in groups:
            if any((h[0] - q[0]) ** 2 + (h[1] - q[1]) ** 2 <= link * link for q in g):
                g.append(h)
                break
        else:
            groups.append([h])
    cx0, cy0 = x0 + rw / 2.0, y0 + rh / 2.0
    groups.sort(key=lambda g: (-len(g),
                               min((q[0] - cx0) ** 2 + (q[1] - cy0) ** 2 for q in g)))
    g = groups[0]
    if len(g) < min_tiles:
        return None
    # 蓝色电控板/电池常占一大片，命中块数明显多；蓝色单独限制最大块数
    if target == COLOR_BLUE:
        max_tiles = max(1, int(getattr(config, "STACK_AI_BLUE_MAX_TILES", 6)))
        if len(g) > max_tiles:
            if getattr(config, "STACK_AI_DEBUG", False):
                print("[AI][F] 拒绝蓝色大块 hits=%d > %d center=(%d,%d)"
                      % (len(g), max_tiles, int(g[0][0]), int(g[0][1])))
            return None

    wsum = sum(q[2] for q in g)
    cx = sum(q[0] * q[2] for q in g) / wsum
    cy = sum(q[1] * q[2] for q in g) / wsum
    gx0 = min(q[3] for q in g)
    gy0 = min(q[4] for q in g)
    gx1 = max(q[3] + tile for q in g)
    gy1 = max(q[4] + tile for q in g)
    out = {
        "cx": int(round(cx)), "cy": int(round(cy)),
        "x": int(max(0, gx0)), "y": int(max(0, gy0)),
        "w": int(max(1, gx1 - gx0)), "h": int(max(1, gy1 - gy0)),
        "src": "ai", "code": target,
        "ai_prob": round(max(q[2] for q in g), 3), "ai_tiles": len(g),
    }
    if target == COLOR_BLUE:
        max_cy = int(getattr(config, "STACK_AI_BLUE_MAX_CY", 240))
        if out["cy"] > max_cy:
            if getattr(config, "STACK_AI_DEBUG", False):
                print("[AI][F] 拒绝蓝色下半区 center=(%d,%d) > y=%d"
                      % (out["cx"], out["cy"], max_cy))
            return None
    if getattr(config, "STACK_AI_DEBUG", False):
        print("[AI][F] 命中 color=%s center=(%d,%d) tiles=%d prob=%.3f" %
              (target, out["cx"], out["cy"], len(g), out["ai_prob"]))
    return out


def refine_stack_top_center(img, item, margin=None,
                            r_min=None, r_max=None, threshold=None):
    """AI 粗定颜色框内精修物料颜色中心。

    默认 color 模式：在 AI 框内按目标颜色取最大色块，使用色块质心；
    比旧霍夫圆稳，不会轻易锁到旁边的黑环。
    circle 模式保留旧行为，便于现场对比。
    """
    if not item:
        return item
    if not getattr(config, "STACK_CV_FINE", True):
        return item
    margin = int(margin if margin is not None
                 else getattr(config, "STACK_CV_SEARCH_MARGIN", 24))
    try:
        iw, ih = img.width(), img.height()
    except Exception:
        return item
    x = max(0, int(item.get("x", item["cx"] - 20)) - margin)
    y = max(0, int(item.get("y", item["cy"] - 20)) - margin)
    x2 = min(iw, int(item.get("x", item["cx"] - 20)) + int(item.get("w", 40)) + margin)
    y2 = min(ih, int(item.get("y", item["cy"] - 20)) + int(item.get("h", 40)) + margin)
    roi = [x, y, max(1, x2 - x), max(1, y2 - y)]
    ax, ay = int(item["cx"]), int(item["cy"])

    mode = str(getattr(config, "STACK_CV_FINE_MODE", "color")).lower()
    if mode == "circle":
        r_min = int(r_min if r_min is not None else getattr(config, "STACK_TOP_R_MIN", 18))
        r_max = int(r_max if r_max is not None else getattr(config, "STACK_TOP_R_MAX", 90))
        threshold = int(threshold if threshold is not None
                        else getattr(config, "STACK_TOP_THRESHOLD", 1500))
        c = find_ring_center(img, roi, r_min=r_min, r_max=r_max, threshold=threshold)
        if not c:
            return item
        cx, cy, r = int(c[0]), int(c[1]), int(c[2])
        max_jump = float(getattr(config, "STACK_CV_MAX_JUMP", 55))
        if (cx - ax) ** 2 + (cy - ay) ** 2 > max_jump * max_jump:
            return item
        out = dict(item)
        out.update({"cx": cx, "cy": cy, "r": r, "src": "ai+cv",
                    "ai_cx": ax, "ai_cy": ay})
        if getattr(config, "STACK_AI_DEBUG", False):
            print("[AI][F] CV圆精修 (%d,%d) r=%d -> (%d,%d)"
                  % (ax, ay, r, cx, cy))
        return out

    # color 模式：目标颜色的最大色块质心
    code = str(item.get("code") or "")
    try:
        idx = [config.COLOR_CODES[n] for n in config.COLOR_NAMES].index(code)
        th = config.THRESHOLDS[idx]
    except Exception:
        return item
    min_px = max(1, int(getattr(config, "STACK_COLOR_MIN_PIXELS", 100)))
    max_jump = float(getattr(config, "STACK_COLOR_MAX_JUMP", 25))
    try:
        blobs = img.find_blobs(
            [th], roi=roi, x_stride=2, y_stride=2,
            area_threshold=min_px, pixels_threshold=min_px,
            merge=True, margin=4)
    except Exception:
        return item
    best = None
    for b in blobs or []:
        try:
            area = int(b.pixels())
        except Exception:
            continue
        if area < min_px:
            continue
        try:
            bx, by = float(b.cx()), float(b.cy())
        except Exception:
            bx, by = float(b.cxf()), float(b.cyf())
        d2 = (bx - ax) ** 2 + (by - ay) ** 2
        # 优先离 AI 粗中心近的，面积作为次条件
        key = (d2, -area)
        if best is None or key < best[0]:
            best = (key, bx, by, area, b)
    if best is None:
        return item
    _, bx, by, area, b = best
    if (bx - ax) ** 2 + (by - ay) ** 2 > max_jump * max_jump:
        return item
    cx, cy = int(round(bx)), int(round(by))
    r = int(round(math.sqrt(max(1.0, area / math.pi))))
    out = dict(item)
    out.update({"cx": cx, "cy": cy,
                "x": int(b.x()), "y": int(b.y()),
                "w": int(b.w()), "h": int(b.h()),
                "r": r, "src": "ai+color",
                "ai_cx": ax, "ai_cy": ay,
                "color_area": area})
    if getattr(config, "STACK_AI_DEBUG", False):
        print("[AI][F] 颜色质心 (%d,%d) -> (%d,%d) area=%d"
              % (ax, ay, cx, cy, area))
    return out


def find_stack_base(clf, img, roi=None, color_code=None, ai=None):
    """码垛底座：按颜色找物料；同色多个取离 ROI 中心最近的；没有就回空环中心

    返回 dict：{"cx","cy","src","code"}
        src = "base"（找到同色物料）/ "ai"（AI 分块找到颜色中心）/
              "ring"（空环）/ "roi"（兜底 ROI 中心）
    返回 None 只在"没有底座、也没找到环、且不允许兜底"时出现。
    """
    roi = roi if roi else config.ROI_STACK
    cx0, cy0 = (roi[0] + roi[2] / 2.0, roi[1] + roi[3] / 2.0) if roi else (0, 0)

    if color_code and getattr(config, "STACK_AI_ONLY", False):
        # 码垛颜色中心只信 AI：先粗扫颜色，再用经典 CV 在 AI 框内精修顶部圆心
        it = find_stack_color_center_ai(ai, img, roi, color_code)
        if it is not None and getattr(config, "STACK_CV_FINE", True):
            it = refine_stack_top_center(img, it)
        return it

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

        # CV 阈值一个同色候选都没找到：让 AI 在 ROI 内分块找颜色中心
        if getattr(config, "STACK_AI_LOCATE", True):
            it = find_stack_color_center_ai(ai, img, roi, color_code)
            if it is not None:
                return it

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