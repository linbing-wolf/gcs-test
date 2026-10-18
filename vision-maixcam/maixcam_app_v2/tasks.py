# -*- coding: utf-8 -*-
"""
tasks.py —— 各视觉任务（对应新流程里下位机要"询问"的每一项）

每个任务：
  - 有超时（超时回 NONE），不会像旧程序那样死等
  - 用 StabilityTracker 做「连续 N 帧中心不动（+ 颜色投票）」的稳定性判定
  - 返回一段已经打包好的回复字符串（由 main.py 直接发出去）
"""

from maix import image, app
from timing import ticks_ms, sleep_ms
import config
import precision
import vision
from protocol import (pack_start, pack_task, pack_pick, pack_disc, pack_stack,
                      pack_none, parse_task_code, pack_obstacle, pack_clear,
                      pack_ready, pack_busy, pack_move, parse_command, CMD_RESET)


def log(*a):
    if config.VERBOSE:
        print(*a)


# 没传 pipeline 时用恒等映射（触屏调试 / 单测路径），行为与旧版一致
_IDENTITY = precision.Pipeline(config.W, config.H, config.W, config.H)


def _pipeline(p):
    return p if p is not None else _IDENTITY


def _confirm_road(ai, img, roi):
    """右侧道路二次确认：灰色占比过线后，再用分类模型确认是 other。

    返回 (ok, why)：
      · AI 没启用/没加载 → 只用灰色结论（ok=True）
      · AI 判成 other 且置信度够 → ok=True
      · 其它 → ok=False
    """
    if ai is None or not ai.ready():
        return True, "gray-only(AI off)"
    idx, prob = ai.classify_top1(img, roi)
    if idx is None:
        return False, "ai-err"
    label = ai.labels[idx] if 0 <= idx < len(ai.labels) else "?"
    want = getattr(config, "ROAD_AI_LABEL", "other")
    min_p = getattr(config, "AI_MIN_PROB", 0.6)
    if label == want and prob >= min_p:
        return True, "gray+ai(%s %.2f)" % (label, prob)
    return False, "ai=%s %.2f" % (label, prob)


class TaskCtl:
    """任务执行期间的串口看门（一问一答的补充）

    视觉端跑任务时是阻塞的，但每帧会「看一眼」串口：
      · 收到 R  → 立刻中断任务，调用方回 READY
      · 收到其它 → 回一帧 BUSY（别重复发），任务继续
    这样下位机不会因为超时误判，同时保留 R 的紧急复位能力。
    """

    def __init__(self, link=None, max_per_frame=4):
        self.link = link
        self.reset_flag = False
        self.max_per_frame = max(1, int(max_per_frame))

    def tick(self):
        """True=继续任务；False=收到 R，应当中断"""
        if self.link is None:
            return True
        if self.reset_flag:
            return False
        for _ in range(self.max_per_frame):
            raw = self.link.poll_command()
            if raw is None:
                break
            cmd, _arg = parse_command(raw)
            if cmd == CMD_RESET:
                log("[看门] 收到 R，中断当前任务")
                self.reset_flag = True
                return False
            log("[看门] 任务中收到 %r → 回 BUSY" % raw)
            self.link.send_payload(pack_busy())
        return True


# ======================================================================
# 稳定性判定：连续多帧确认同一个目标（中心不动 + 可选颜色投票）
# ======================================================================
class StabilityTracker:
    def __init__(self, stable_frames, max_move=config.MAX_PIXEL_MOVE,
                 color_window=None, confidence=None):
        self.stable_frames = stable_frames
        self.max_move = max_move
        self.color_window = color_window if color_window is not None else config.COLOR_WINDOW
        self.confidence = confidence if confidence is not None else config.COLOR_CONFIDENCE
        self.reset()

    def reset(self):
        self.first_center = None
        self.stable_count = 0
        self.color_history = []
        self.last = None

    def update(self, item):
        """喂一帧结果，返回 (是否稳定, 最终颜色或None, item)

        item=None 表示这帧没看到（不清零，温和策略）。
        item 没带 'code' 时视为「非颜色任务」，跳过颜色投票。
        """
        if item is None:
            return False, None, None
        self.last = item
        code = item.get("code")

        if code is not None:
            self.color_history.append(code)
            if len(self.color_history) > self.color_window:
                self.color_history.pop(0)

        cx, cy = item["cx"], item["cy"]
        if self.first_center is None:
            self.first_center = (cx, cy)
            self.stable_count = 1
        else:
            dx, dy = cx - self.first_center[0], cy - self.first_center[1]
            if (dx * dx + dy * dy) ** 0.5 < self.max_move:
                self.stable_count += 1
            else:
                self.stable_count = 1
                self.first_center = (cx, cy)
                self.color_history = []

        final_color = None
        if code is not None and len(self.color_history) >= self.color_window:
            counts = {}
            for c in self.color_history:
                counts[c] = counts.get(c, 0) + 1
            top = max(counts.items(), key=lambda kv: kv[1])
            if top[1] / float(len(self.color_history)) >= self.confidence:
                final_color = top[0]

        color_ok = (code is None) or (final_color is not None)
        if self.stable_count >= self.stable_frames and color_ok:
            return True, (final_color if code is not None else None), item
        return False, final_color, item


# ======================================================================
# 绘制辅助（调试用）
# ======================================================================
def draw_roi(img, roi, color=image.COLOR_BLUE):
    if roi:
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], color, 1)


def draw_item(img, item, label, color=image.COLOR_GREEN):
    if not item:
        return
    if "w" in item and "h" in item:
        img.draw_rect(item["x"], item["y"], item["w"], item["h"], color, 2)
    img.draw_cross(item["cx"], item["cy"], image.COLOR_RED, 6, 2)
    txt = label if "code" not in item else "%s c=%s" % (label, item["code"])
    img.draw_string(item["x"], max(0, item["y"] - 16), txt, image.COLOR_YELLOW, 1)


# ======================================================================
# 任务 A：启停区识别
# ======================================================================
def task_start(cam, disp, timeout_ms=config.TASK_TIMEOUT_MS, p=None, ctl=None, clf=None):
    """任务 A：启停区

    主输出是**区号**：看画面右侧有没有灰色道路 → 有=START_ZONE_HAS_ROAD(1)，无=2。
    同时尽量给出蓝色启停区的位置（看不到就给 0,0,0,0，区号仍然有效）。
    道路结论要连续 ROAD_STABLE_FRAMES 帧一致才确认。
    """
    P = _pipeline(p)
    road_roi = P.roi(config.ROI_ROAD)
    start_roi = P.roi(config.ROI_START)
    log("[A] 启停区识别（右侧道路 → 区号）")
    tracker = StabilityTracker(config.STABLE_FRAMES)
    t0 = ticks_ms()
    last_road = None
    road_count = 0
    ratio = 0.0
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()

        # --- 右侧道路 → 启停区编号 ---
        has_road, ratio = vision.find_road(img, road_roi)
        why = "gray=no"
        if has_road:
            ai = getattr(clf, "ai", None) if clf is not None else None
            has_road, why = _confirm_road(ai, img, road_roi)
        if last_road is None or has_road == last_road:
            road_count += 1
        else:
            road_count = 1
        last_road = has_road
        road_ok = road_count >= max(1, int(getattr(config, "ROAD_STABLE_FRAMES", 5)))

        # --- 蓝色启停区位置（可选，看不到不影响区号）---
        z = vision.find_start_zone(img, start_roi, P.area(config.MIN_PIXELS_START))
        if z:
            draw_item(img, z, "START")
        tracker.update(z)

        draw_roi(img, road_roi, image.COLOR_YELLOW)
        img.draw_string(2, 2, "ROAD %s %.2f %s" % ("YES" if has_road else "no", ratio, why),
                        image.COLOR_GREEN if has_road else image.COLOR_RED, 1)

        if road_ok:
            zone = (config.START_ZONE_HAS_ROAD if last_road
                    else config.START_ZONE_NO_ROAD)
            if z:
                cx, cy = P.smooth("A", *P.point(z["cx"], z["cy"]))
                w, h = P.size(z["w"], z["h"])
            else:
                cx = cy = w = h = 0
            log("[A] 启停区=%d号（右侧道路=%s ratio=%.2f %s）位置(%d,%d) %dx%d"
                % (zone, "有" if last_road else "无", ratio, why, cx, cy, w, h))
            if disp:
                disp.show(img)
            return pack_start(zone, cx, cy, w, h)

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[A] 超时（道路结论未稳定）")
            return pack_none("A")
    return pack_none("A")


# ======================================================================
# 任务 T：接收任务码（扫码模块 → 单片机 → 上位机），解析并确定颜色
#       纯解析，不用摄像头
# ======================================================================
def task_task(code):
    r = parse_task_code(code)
    if r is None:
        log("[T] 任务码格式非法: %r" % code)
        return pack_none("T")
    log("[T] 任务码 OK: %s  第一批=%s 第二批=%s" % (r["raw"], r["g1"], r["g3"]))
    # G1/G3 是颜色+顺序，G2/G4 是放置位置
    return pack_task(r["g1"], r["g3"])


# ======================================================================
# 任务 C：原料区"三选一" —— 找指定颜色码的物料并返回坐标
# ======================================================================
def task_pick(cam, clf, disp, color_code, timeout_ms=config.TASK_TIMEOUT_MS, p=None, ctl=None):
    P = _pipeline(p)
    roi = P.roi(config.ROI_MATERIAL)
    log("[C] 原料区找颜色=%s 的物料" % color_code)
    tracker = StabilityTracker(config.STABLE_FRAMES, color_window=config.COLOR_WINDOW,
                               confidence=config.COLOR_CONFIDENCE)
    t0 = ticks_ms()
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        draw_roi(img, roi)
        it = clf.find_color(img, color_code, roi, P.area(config.MIN_PIXELS))
        if it:
            draw_item(img, it, "PICK")
            if it.get("src") == "ai":
                img.draw_string(2, 16, "AI c=%s" % it.get("ai_code"), image.COLOR_GREEN, 1)
        else:
            img.draw_string(2, 2, "PICK c=%s: not found" % color_code, image.COLOR_RED, 1)
        ok, _, it = tracker.update(it)
        if ok:
            cx, cy = P.smooth("C", *P.point(it["cx"], it["cy"]))
            log("[C] 找到 (%d,%d) 颜色=%s 来源=%s" %
                (cx, cy, color_code, it.get("src", "cv")))
            if disp:
                disp.show(img)
            return pack_pick(color_code, cx, cy)
        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[C] 超时")
            return pack_none("C")
    return pack_none("C")


# ======================================================================
# 任务 D / E：圆盘中心（粗加工区 / 暂存区）
# ======================================================================
def task_disc(cam, disp, roi, tag, timeout_ms=config.TASK_TIMEOUT_MS, p=None, ctl=None):
    """任务 D/E：圆盘中心 + 环2 水平对齐

    目标是让「环 2（同心环组的中心）」落在画面水平中心 |x-W/2| <= ALIGN_TOL：
      · ALIGN_ON=True  → 视觉端主动发 MOVE,<axis>,<dx> 让下位机微调，自己闭环
      · ALIGN_ON=False → 只等待，够稳定就带当前偏差回复（退化为旧行为）
    成功：DISC,x,y,dx,1    未对齐/超次数：DISC,x,y,dx,0    看不到圆：NONE,<tag>
    """
    P = _pipeline(p)
    droi = P.roi(roi)
    r_min = max(1, int(round(config.DISC_R_MIN * P.sx)))
    r_max = max(2, int(round(config.DISC_R_MAX * P.sx)))
    link = getattr(ctl, "link", None) if ctl is not None else None
    align_on = bool(getattr(config, "ALIGN_ON", True)) and link is not None
    tol = int(getattr(config, "ALIGN_TOL", 5))
    need_stable = max(1, int(getattr(config, "ALIGN_STABLE_FRAMES", 5)))
    step_max = max(1, int(getattr(config, "ALIGN_STEP_MAX", 40)))
    settle = max(0, int(getattr(config, "ALIGN_SETTLE_MS", 300)))
    max_iter = max(1, int(getattr(config, "ALIGN_MAX_ITER", 20)))
    center_x = config.W // 2

    log("[%s] 圆盘中心定位（对齐容差 ±%dpx，MOVE %s）"
        % (tag, tol, "开" if align_on else "关"))
    tracker = StabilityTracker(config.STABLE_FRAMES)
    t0 = ticks_ms()
    stable = 0
    iters = 0
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        draw_roi(img, droi)
        c = vision.find_disc_center(img, droi, r_min, r_max)
        item = None
        dx = 0
        cx = cy = 0
        if c:
            cx, cy = P.point(c[0], c[1])
            dx = cx - center_x
            item = {"cx": cx, "cy": cy}
            img.draw_circle(c[0], c[1], int(c[2]), image.COLOR_GREEN, 2)
            img.draw_cross(cx, cy, image.COLOR_RED, 6, 2)
            img.draw_line(center_x, 0, center_x, config.H - 1, image.COLOR_YELLOW, 1)
            img.draw_string(2, 2, "dx=%+d %s" % (dx, "OK" if abs(dx) <= tol else "MOVE"),
                            image.COLOR_GREEN if abs(dx) <= tol else image.COLOR_YELLOW, 2)

        if item is not None and abs(dx) <= tol:
            stable += 1
            tracker.update(item)
            if stable >= need_stable:
                log("[%s] 圆盘中心 (%d,%d) dx=%+d 已对齐（%d 次微调）"
                    % (tag, cx, cy, dx, iters))
                if disp:
                    disp.show(img)
                return pack_disc(cx, cy, dx, 1)
        else:
            stable = 0
            if item is not None and align_on and iters < max_iter:
                step = max(-step_max, min(step_max, dx))
                if getattr(config, "ALIGN_INVERT", False):
                    step = -step
                log("[%s] dx=%+d 超差 → MOVE,%d%s" %
                    (tag, dx, step, "（已反转）" if getattr(config, "ALIGN_INVERT", False) else ""))
                link.send_payload(pack_move(step))
                iters += 1
                if disp:
                    disp.show(img)
                if settle:
                    sleep_ms(settle)
                continue
            if item is not None and align_on and iters >= max_iter:
                log("[%s] 微调 %d 次仍未对齐（dx=%+d），带偏差回复" % (tag, iters, dx))
                if disp:
                    disp.show(img)
                return pack_disc(cx, cy, dx, 0)

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[%s] 超时" % tag)
            if item is not None:
                return pack_disc(cx, cy, dx, 0)
            return pack_none(tag)
    return pack_none(tag)


# ======================================================================
# 任务 F：码垛目标中心 + 固定物料高度
# ======================================================================
def task_stack(cam, clf, disp, timeout_ms=config.TASK_TIMEOUT_MS, p=None,
               ctl=None, color_code=None):
    """任务 F：码垛

    F,<颜色码>：找**同色**的底座；同色多个取离 ROI 中心最近的；
                没有同色底座 → 回"空环中心"（连环都没有则退 ROI 中心）。
    F（不带参数）：兼容旧行为，取 ROI 内最靠上的色块。

    回复：STACK,x,y（视觉不测高度，高度由下位机按物料/层数自行计算）
    """
    P = _pipeline(p)
    roi = P.roi(config.ROI_STACK)
    ai = getattr(clf, "ai", None)
    log("[F] 码垛底座定位（颜色=%s）" % (color_code or "任意"))
    tracker = StabilityTracker(config.STABLE_FRAMES)
    t0 = ticks_ms()
    src = None
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        draw_roi(img, roi)

        if color_code:
            it = vision.find_stack_base(clf, img, roi, color_code, ai)
            src = it["src"] if it else None
        else:
            it = vision.find_stack_top(clf, img, roi, P.area(config.MIN_PIXELS))
            src = "base" if it else None

        vote = bool(getattr(config, "STACK_COLOR_VOTE", False))
        if it:
            draw_item(img, it, "STACK:" + (src or "?"))
            track_item = it if vote else {"cx": it["cx"], "cy": it["cy"]}
        else:
            track_item = None

        ok, _, _ = tracker.update(track_item)
        if ok:
            it = it or track_item
            cx, cy = P.smooth("F", *P.point(it["cx"], it["cy"]))
            log("[F] 码垛 %s (%d,%d)" % (src or "?", cx, cy))
            if disp:
                disp.show(img)
            return pack_stack(cx, cy)

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[F] 超时")
            return pack_none("F")
    return pack_none("F")


# ======================================================================
# 任务 G：前进状态 —— 检测前方是否有黑色障碍物，给出避障信息
#   有障碍物 → OBST,x,y,w,h,dist ；连续多帧没有 → CLEAR
# ======================================================================
def task_forward(cam, disp, timeout_ms=config.OBST_TIMEOUT_MS,
                 roi=None, min_pixels=config.MIN_PIXELS_OBST, p=None, ctl=None):
    P = _pipeline(p)
    log("[G] 前进状态：检测前方障碍物")
    roi = config.ROI_FORWARD if roi is None else roi
    droi = P.roi(roi)
    min_px = P.area(min_pixels)
    # 障碍物没有颜色码，StabilityTracker 会自动跳过颜色投票，只做中心稳定判定
    # 行进中连续询问，用更少的稳定帧数（默认 5 帧 ≈ 83ms）
    tracker = StabilityTracker(int(getattr(config, "OBST_STABLE_FRAMES", 5)))
    t0 = ticks_ms()
    clear_count = 0
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        if droi:
            draw_roi(img, droi)
        ob = vision.find_obstacle(img, droi, min_px)
        if ob:
            clear_count = 0
            # 障碍物用绿色框标注
            img.draw_rect(ob["x"], ob["y"], ob["w"], ob["h"], image.COLOR_GREEN, 2)
            img.draw_cross(ob["cx"], ob["cy"], image.COLOR_RED, 8, 2)
        else:
            clear_count += 1
            img.draw_string(2, 2, "FORWARD: clear", image.COLOR_GREEN, 1)

        # 稳定性判定（障碍物无颜色码 → 只做中心稳定）
        ok, _, it = tracker.update(ob)
        if ok:
            x, y = P.point(it["x"], it["y"])
            w, h = P.size(it["w"], it["h"])
            # 测距用「对外口径」的像素宽（fx 是在 W×H 上标定的）
            dist = vision.estimate_distance_mm(w)
            log("[G] 有障碍物 (%d,%d) %dx%d 约%dmm" % (x, y, w, h, dist))
            if disp:
                disp.show(img)
            return pack_obstacle(x, y, w, h, dist)

        # 连续多帧没看到障碍物 → 前方畅通，尽快回 CLEAR
        if clear_count >= config.OBST_CLEAR_FRAMES:
            log("[G] 前方无障碍物")
            if disp:
                disp.show(img)
            return pack_clear()

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[G] 超时，按无障碍处理")
            return pack_clear()
    return pack_clear()