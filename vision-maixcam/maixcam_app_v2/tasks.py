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
import os
import config
import precision
import vision
import disc_debug
import screen_echo
from protocol import (pack_start, pack_task, pack_pick, pack_disc, pack_stack,
                      pack_none, parse_task_code, pack_obstacle, pack_clear,
                      pack_ready, pack_busy, pack_move, pack_movey,
                      pack_movexy, parse_command, CMD_RESET)


def log(*a):
    if config.VERBOSE:
        print(*a)


# 没传 pipeline 时用恒等映射（触屏调试 / 单测路径），行为与旧版一致
_IDENTITY = precision.Pipeline(config.W, config.H, config.W, config.H)


def _pipeline(p):
    return p if p is not None else _IDENTITY


# ======================================================================
# D/E 跑飞保护：连续同方向 + |误差| 持续增大 → 锁死，不再发对应轴的指令
# 水平(MOVE)和垂直(MOVEY)各自独立计数，避免一个轴正常、另一个轴跑飞时互相干扰。
# ======================================================================
_GUARDS = {
    "x": {"sign": 0, "abs": 0, "count": 0, "latched": False},
    "y": {"sign": 0, "abs": 0, "count": 0, "latched": False},
}


def reset_guard(axis=None):
    """清掉跑飞锁；axis=None 清两轴，axis='x'/'y' 只清一个轴。"""
    names = [axis] if axis in _GUARDS else list(_GUARDS.keys())
    for name in names:
        g = _GUARDS[name]
        g["sign"] = 0
        g["abs"] = 0
        g["count"] = 0
        g["latched"] = False


def _runaway_check(value, axis="x"):
    """连续同方向且 |value| 持续增大 → 判定方向反了/跑飞。

    水平轴传 dx（默认 axis='x'），垂直轴传 dy 且 axis='y'。
    触发后该轴锁死，直到 R/Z 或成功对齐调用 reset_guard()。
    """
    name = axis if axis in _GUARDS else "x"
    g = _GUARDS[name]
    if not bool(getattr(config, "DISC_RUNAWAY_GUARD", True)):
        return False
    if g["latched"]:
        return True
    sign = 1 if value > 0 else (-1 if value < 0 else 0)
    absv = abs(int(value))
    grow = int(getattr(config, "DISC_RUNAWAY_GROW_PX", 2))
    if sign == 0:
        reset_guard(name)
        return False
    if sign == g["sign"] and absv > g["abs"] + grow:
        g["count"] += 1
    elif sign == g["sign"]:
        # 同方向但误差没有继续变大（机构还没动/正在收敛）→ 不算跑飞
        g["count"] = 0
    else:
        g["sign"] = sign
        g["count"] = 1
    g["abs"] = absv
    limit = max(2, int(getattr(config, "DISC_RUNAWAY_COUNT", 4)))
    if g["count"] >= limit:
        g["latched"] = True
        return True
    return False


def _axis_step(err, gain, step_min, step_max, scale, invert):
    """误差 → 单轴步长（增益、最小/最大限幅、方向反转、数值系数）。"""
    s = int(round(err * gain))
    if s == 0:
        s = 1 if err > 0 else -1
    if step_max > 0:
        s = max(-step_max, min(step_max, s))
    if abs(s) < step_min:
        s = step_min if s > 0 else -step_min
    if invert:
        s = -s
    if abs(scale - 1.0) > 1e-9:
        s = int(round(s * scale))
    return s


def _align_movexy_mode():
    """True = 用一条 MOVEXY,dx,dy 并行发两轴；False = 老的分发 MOVE/MOVEY。"""
    return str(getattr(config, "ALIGN_MODE", "movexy")).strip().lower() in (
        "movexy", "xy", "combined")


_VERIFY_CACHE = {"t": None, "ov": None, "src": ""}


def _align_verify_mode_from_config():
    """仅看 config.ALIGN_REPLY_MODE：True=verify（回读），False=direct（去回读）。"""
    return str(getattr(config, "ALIGN_REPLY_MODE", "direct")).strip().lower() == "verify"


def _scan_verify_override():
    """扫触发文件，返回 (override, src)；override ∈ (True=强制回读, False=强制去回读, None=无覆盖)。

    触发文件给现场诊断用：SSH 上 `touch /tmp/align_verify` 就能让 D/E/F/H
    临时回读一次、拿到「移动后偏差」，用完 `rm` 即恢复 config 默认，免重编包。
    """
    try:
        vf = getattr(config, "ALIGN_VERIFY_TRIGGER_FILE", "")
        df = getattr(config, "ALIGN_DIRECT_TRIGGER_FILE", "")
        if vf and os.path.exists(vf):
            return True, vf
        if df and os.path.exists(df):
            return False, df
    except Exception:
        pass
    return None, ""


def _align_verify_on():
    """发完 MOVEXY 后是否回读一次再回复。

    True  = 回读，用调整后的 x/y/dx 回复（准但慢）
    False = 去回读，立即回动作前的 x/y/dx（快）

    优先级：触发文件 > config.ALIGN_REPLY_MODE。config 每次即时生效；
    触发文件的扫描结果按 ALIGN_MODE_CACHE_MS 缓存，避免在逐帧循环里频繁 stat。
    """
    c = _VERIFY_CACHE
    now = ticks_ms()
    ttl = int(getattr(config, "ALIGN_MODE_CACHE_MS", 500))
    if c["t"] is None or ttl <= 0 or now < c["t"] or now - c["t"] >= ttl:
        ov, src = _scan_verify_override()
        if (ov, src) != (c["ov"], c["src"]):
            if ov is None:
                log("[ALIGN] 无触发文件，回复模式按 config：%s"
                    % ("verify 回读" if _align_verify_mode_from_config()
                       else "direct 去回读"))
            else:
                log("[ALIGN] 触发文件 %s → 强制 %s"
                    % (src, "verify 回读" if ov else "direct 去回读"))
        c["ov"], c["src"], c["t"] = ov, src, now
    if c["ov"] is None:
        return _align_verify_mode_from_config()
    return c["ov"]


def _make_align_actions(x_out, y_out, sx_step, sy_step):
    """按当前模式组装指令；返回 (name, step, payload) 列表。"""
    if _align_movexy_mode():
        if not x_out and not y_out:
            return []
        return [("MOVEXY", (sx_step, sy_step), pack_movexy(sx_step, sy_step))]
    actions = []
    if x_out:
        actions.append(("MOVE", sx_step, pack_move(sx_step)))
    if y_out:
        actions.append(("MOVEY", sy_step, pack_movey(sy_step)))
    return actions


def _actions_sent_axis(actions, axis):
    """这次实际发出的指令里，指定轴是否有非零步长。"""
    idx = 0 if axis == "x" else 1
    for name, step, _ in actions:
        if name == "MOVEXY" and abs(step[idx]) > 0:
            return True
        if name == ("MOVE" if axis == "x" else "MOVEY") and abs(step) > 0:
            return True
    return False


def _actions_sent_x(actions):
    return _actions_sent_axis(actions, "x")


def _actions_sent_y(actions):
    return _actions_sent_axis(actions, "y")


def _wait_stable_post(cam, ctl, detect, initial_ms, max_ms, tol, need,
                      baseline=None, require_x=False, require_y=False):
    """one-shot 后只读等待：先等 initial_ms，再等到目标连续 need 帧基本不动。

    baseline=(x,y) 是动作前中心；require_x/require_y=True 时，对应轴必须
    相对 baseline 真的移动过，不能把“还没开始动”误判成“已经停稳”。
    detect(img) → (cx, cy, meta) 或 None；本函数绝不发 MOVE/MOVEY。
    """
    t_start = ticks_ms()
    while ticks_ms() - t_start < max(0, int(initial_ms)):
        if app.need_exit() or (ctl is not None and not ctl.tick()):
            return None
        sleep_ms(20)

    last = None
    stable = 0
    moved_x = moved_y = False
    base_x = base_y = None
    if baseline is not None:
        base_x, base_y = float(baseline[0]), float(baseline[1])
    deadline = ticks_ms() + max(0, int(max_ms))
    while ticks_ms() < deadline:
        if app.need_exit() or (ctl is not None and not ctl.tick()):
            break
        img = cam.read()
        r = detect(img)
        if r is None:
            stable = 0
            sleep_ms(50)
            continue
        cx, cy, meta = r
        if base_x is None:
            base_x, base_y = cx, cy
        if abs(cx - base_x) > tol:
            moved_x = True
        if abs(cy - base_y) > tol:
            moved_y = True
        if last is not None and abs(cx - last["cx"]) <= tol and abs(cy - last["cy"]) <= tol:
            stable += 1
        else:
            stable = 1
        last = {"cx": cx, "cy": cy, "meta": meta, "img": img,
                "stable": False, "elapsed_ms": ticks_ms() - t_start,
                "moved_x": moved_x, "moved_y": moved_y}
        if (stable >= max(1, int(need))
                and (moved_x or not require_x)
                and (moved_y or not require_y)):
            last["stable"] = True
            return last
        sleep_ms(50)
    return last


def _frame_sig(img, cols=24, rows=18):
    """画面的降采样灰度签名（24x18=432 点），用来比较相邻帧是否变化。

    纯 get_pixel 采样，不依赖 cv2/numpy；img 没有像素接口时返回 None。
    """
    if not hasattr(img, "get_pixel"):
        return None
    try:
        w, h = img.width(), img.height()
    except Exception:
        return None
    vals = []
    for r in range(rows):
        y = int((r + 0.5) * h / rows)
        for c in range(cols):
            x = int((c + 0.5) * w / cols)
            v = vision._pixel_luma(img, x, y)
            vals.append(0.0 if v is None else float(v))
    return vals


def _sig_diff(a, b):
    if not a or not b:
        return 1e9
    n = min(len(a), len(b))
    s = 0.0
    for i in range(n):
        s += abs(a[i] - b[i])
    return s / n


def _wait_scene_settle(cam, ctl, disp, max_ms, tag=""):
    """等画面稳定：连续 SCENE_SETTLE_FRAMES 帧画面变化 < 阈值就立即返回 True。

    为什么要它：任务开头原来死等固定毫秒（D/E=200、F=500）。
    但机构（机械臂/转盘）什么时候停不确定——固定值要么等太久、要么没停就开始识别。
    改成判稳后：还在动就继续等，一停就马上读第一帧。

    max_ms 是「最长等待」上限（沿用各任务原来的 *_START_DELAY_MS）。
    SCENE_SETTLE_ON=False 时退回固定等待（行为与旧版一致）。
    返回 True=可以开始识别；False=被 R 中断（调用方应回 READY）。
    """
    max_ms = max(0, int(max_ms))
    if max_ms <= 0:
        return True
    if not bool(getattr(config, "SCENE_SETTLE_ON", True)):
        t0 = ticks_ms()                       # 退回固定等待
        while ticks_ms() - t0 < max_ms:
            if app.need_exit():
                return False
            if ctl is not None and not ctl.tick():
                return False
            sleep_ms(20)
        return True

    need = max(1, int(getattr(config, "SCENE_SETTLE_FRAMES", 3)))
    thr = float(getattr(config, "SCENE_SETTLE_DIFF", 2.0))
    min_ms = max(0, int(getattr(config, "SCENE_SETTLE_MIN_MS", 100)))
    t0 = ticks_ms()
    prev = None
    stable = 0
    while True:
        if app.need_exit():
            return False
        if ctl is not None and not ctl.tick():
            return False
        img = cam.read()
        if disp is not None:
            try:
                disp.show(img)
            except Exception:
                pass
        sig = _frame_sig(img)
        if sig is not None and prev is not None:
            d = _sig_diff(prev, sig)
            if d <= thr:
                stable += 1
            else:
                stable = 0
        prev = sig
        el = ticks_ms() - t0
        if stable >= need and el >= min_ms:
            log("[%s] 画面已稳定（%d 帧变化<%.1f，用时 %dms）" % (tag, stable, thr, el))
            return True
        if el >= max_ms:
            log("[%s] 画面稳定等待到上限（%dms），继续" % (tag, max_ms))
            return True
        sleep_ms(8)


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


class StopDetector:
    """判断目标是否"停稳"（用于转盘：必须停下才回复）

    判据：与 hold_ms 之前那一帧相比，位移不超过 tol_px。
    转盘慢转时相邻帧只差 1~2px，但 300ms 累计就会超阈值，所以比"逐帧阈值"可靠。

    speed_px_s 是允许的残余速度（像素/秒，对外 640x480 口径）：
        tol_px = speed_px_s * hold_ms / 1000，且不小于 3px（抗抖动）
    """

    def __init__(self, speed_px_s=8.0, hold_ms=300, min_tol_px=3.0):
        self.hold_ms = max(0, int(hold_ms))
        self.tol = max(float(min_tol_px), float(speed_px_s) * self.hold_ms / 1000.0)
        self.buf = []
        self.speed = 0.0

    def reset(self):
        self.buf = []
        self.speed = 0.0

    def update(self, x, y, now_ms):
        """喂一帧位置 → (是否停稳, 当前速度 px/s)"""
        if self.hold_ms <= 0:
            return True, 0.0
        self.buf.append((now_ms, x, y))
        cut = now_ms - 2 * self.hold_ms - 200
        while len(self.buf) > 1 and self.buf[0][0] < cut:
            self.buf.pop(0)
        ref = None
        for t, px, py in self.buf:          # 取"至少 hold_ms 之前"的最后一帧
            if now_ms - t >= self.hold_ms:
                ref = (t, px, py)
            else:
                break
        if ref is None:
            self.speed = 0.0
            return False, 0.0
        t0, x0, y0 = ref
        dt = now_ms - t0
        d = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
        self.speed = d * 1000.0 / dt if dt else 0.0
        return (d <= self.tol), self.speed


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
    tracker = StabilityTracker(config.STABLE_FRAMES, color_window=config.COLOR_WINDOW,
                               confidence=config.COLOR_CONFIDENCE)
    # 转盘：必须等物料真的停稳才回复（stop_only=False 可退回旧行为）
    stop_only = bool(getattr(config, "PICK_STOP_ONLY", True))
    stop_det = StopDetector(getattr(config, "PICK_STOP_SPEED_PX_S", 8.0),
                            getattr(config, "PICK_STOP_HOLD_MS", 300))
    log("[C] 原料区找颜色=%s 的物料（停止才回复=%s）" % (color_code, stop_only))
    t0 = ticks_ms()
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        draw_roi(img, roi)
        it = clf.find_color(img, color_code, roi, P.area(config.MIN_PIXELS))
        stopped, speed = False, 0.0
        if it:
            draw_item(img, it, "PICK")
            if it.get("src") == "ai":
                img.draw_string(2, 16, "AI c=%s" % it.get("ai_code"), image.COLOR_GREEN, 1)
            px, py = P.point(it["cx"], it["cy"])
            stopped, speed = stop_det.update(px, py, ticks_ms())
            img.draw_string(2, 32, "%s %.0fpx/s" % ("STOPPED" if stopped else "MOVING", speed),
                            image.COLOR_GREEN if stopped else image.COLOR_YELLOW, 1)
        else:
            stop_det.reset()
            img.draw_string(2, 2, "PICK c=%s: not found" % color_code, image.COLOR_RED, 1)
        ok, _, it = tracker.update(it)
        if ok and (stopped or not stop_only):
            cx, cy = P.smooth("C", *P.point(it["cx"], it["cy"]))
            log("[C] 找到 (%d,%d) 颜色=%s 来源=%s 已停稳(%.1fpx/s)" %
                (cx, cy, color_code, it.get("src", "cv"), speed))
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
def task_disc(cam, disp, roi, tag, timeout_ms=None, p=None, ctl=None):
    """任务 D/E：圆盘中心 + 环2 水平/垂直对齐

    识别：find_circles 候选 → 径向边缘 + 最小二乘细化 → 找不到再用圆环兜底。
    对齐：视觉端发 MOVE,<step> 和/或 MOVEY,<step> 闭环微调；
          |dx| <= ALIGN_TOL 且 |dy| <= ALIGN_DY_TOL 连续 N 帧 → ok=1。
          ALIGN_ON=False 时不发指令，中心稳定就带当前偏差回复。
    回复：DISC,x,y,dx,1  对齐完成
          DISC,x,y,dx,0  未对齐（超次数/超时/只报告模式偏差过大）
          NONE,<tag>     连续 DISC_LOST_FRAMES 帧看不到圆

    调试：DISC_DEBUG_SAVE=True 时，每次任务在 DISC_DEBUG_DIR 下保存
          关键帧 JPEG + meta.jsonl（所有候选圆/拟合结果/每帧 dx/dy）。
    """
    P = _pipeline(p)
    droi = P.roi(roi)
    # 半径配置是「对外 640x480 口径」，检测分辨率更高时要换算到检测域
    sx = getattr(P, "sx", 1.0) or 1.0
    r_min = max(1, int(round(config.DISC_R_MIN / sx)))
    r_max = max(2, int(round(config.DISC_R_MAX / sx)))
    link = getattr(ctl, "link", None) if ctl is not None else None
    align_on = bool(getattr(config, "ALIGN_ON", True)) and link is not None
    # D 和 E 的 MOVE / MOVEY 方向可能不同
    invert = (bool(getattr(config, "ALIGN_INVERT_D", True)) if tag == "D"
              else bool(getattr(config, "ALIGN_INVERT_E", False)))
    dy_invert = (bool(getattr(config, "ALIGN_INVERT_DY_D", False)) if tag == "D"
                 else bool(getattr(config, "ALIGN_INVERT_DY_E", False)))
    tol = int(getattr(config, "ALIGN_TOL", 8))
    dy_on = bool(getattr(config, "ALIGN_DY_ON", True))
    dy_tol = int(getattr(config, "ALIGN_DY_TOL", tol))
    need_stable = max(1, int(getattr(config, "ALIGN_STABLE_FRAMES", 3)))
    gain = float(getattr(config, "ALIGN_GAIN", 0.8))
    step_min = max(1, int(getattr(config, "ALIGN_STEP_MIN", 2)))
    step_max = int(getattr(config, "ALIGN_STEP_MAX", 0))
    if step_max > 0:
        step_max = max(step_min, step_max)
    # D/E 可分别设置 dx 偏置；None 表示回退到全局 ALIGN_DX_OFFSET
    dx_offset = None
    if tag == "D":
        dx_offset = getattr(config, "ALIGN_DX_OFFSET_D", None)
    elif tag == "E":
        dx_offset = getattr(config, "ALIGN_DX_OFFSET_E", None)
    if dx_offset is None:
        dx_offset = getattr(config, "ALIGN_DX_OFFSET", 0)
    dx_offset = int(dx_offset)
    # D/E 可分别设置 dy 偏置；None 表示回退到全局 ALIGN_DY_OFFSET
    dy_offset = None
    if tag == "D":
        dy_offset = getattr(config, "ALIGN_DY_OFFSET_D", None)
    elif tag == "E":
        dy_offset = getattr(config, "ALIGN_DY_OFFSET_E", None)
    if dy_offset is None:
        dy_offset = getattr(config, "ALIGN_DY_OFFSET", 0)
    dy_offset = int(dy_offset)
    settle = max(0, int(getattr(config, "ALIGN_SETTLE_MS", 200)))
    max_iter = max(1, int(getattr(config, "ALIGN_MAX_ITER", 8)))
    lost_limit = max(1, int(getattr(config, "DISC_LOST_FRAMES", 2)))
    refine = bool(getattr(config, "DISC_REFINE_ON", True))
    ring_fb = bool(getattr(config, "DISC_RING_FALLBACK", True))
    if timeout_ms is None:
        timeout_ms = int(getattr(config, "DISC_TIMEOUT_MS", config.TASK_TIMEOUT_MS))
    center_x = config.W // 2
    center_y = config.H // 2
    det_center_x = int(getattr(P, "det_w", config.W) // 2)
    det_center_y = int(getattr(P, "det_h", config.H) // 2)

    # ---- 调试抓图：每次 D/E 在 DISC_DEBUG_DIR 下建一个任务目录 ----
    rec = None
    if bool(getattr(config, "DISC_DEBUG_SAVE", False)):
        rec = disc_debug.DiscDebug(
            tag,
            getattr(config, "DISC_DEBUG_DIR", "/root/yyb_mover_data/disc_debug"),
            max_frames=getattr(config, "DISC_DEBUG_MAX_FRAMES", 24),
            quality=getattr(config, "DISC_DEBUG_QUALITY", 85),
            keep_tasks=getattr(config, "DISC_DEBUG_KEEP_TASKS", 20),
            min_interval_ms=getattr(config, "DISC_DEBUG_MIN_INTERVAL_MS", 100))
        rec.start()

    def finish(payload):
        if rec is not None:
            rec.close(payload)
        return payload

    def fields_of(c, cx, cy, dx, dy, meta, extra=None):
        f = {"iter": iters, "lost": lost, "stable": stable}
        if c is not None:
            f.update({"cx": int(cx), "cy": int(cy), "r": int(c[2]),
                      "dx": int(dx), "dy": int(dy)})
        if meta:
            for k in ("src", "circles", "coarse", "refined", "ring",
                      "refine_reason", "refine_points", "ms", "near_count",
                      "picked", "attempts", "inner", "inner_ref",
                      "inner_blobs", "inner_reason",
                      "tpl", "tpl_scale", "tpl_roi", "tpl_reject", "tpl_error"):
                if k in meta:
                    f[k] = meta[k]
        if extra:
            f.update(extra)
        return f

    def axis_step(err, scale, do_invert):
        return _axis_step(err, gain, step_min, step_max, scale, do_invert)

    log("[%s] 圆盘中心定位（对齐容差 dx±%dpx / dy%s，MOVE %s，超时 %dms）"
        % (tag, tol, ("±%dpx" % dy_tol) if dy_on else "关",
           "开" if align_on else "关", timeout_ms))
    log("[%s] 指令模式=%s, MOVE 系数=%.1fx, MOVEY 系数=%.1fx, dx 修正=%+dpx, dy 修正=%+dpx, 步长上限=%s"
        % (tag, "MOVEXY" if _align_movexy_mode() else "MOVE+MOVEY",
           float(getattr(config, "MOVE_SCALE", 1.0)),
           float(getattr(config, "MOVEY_SCALE", 1.0)), dx_offset, dy_offset,
           "无" if step_max <= 0 else str(step_max)))
    # 收到 D/E 后等画面稳定（上限 DISC_START_DELAY_MS），再读第一帧
    start_delay = max(0, int(getattr(config, "DISC_START_DELAY_MS", 0)))
    if start_delay > 0:
        log("[%s] 等待画面稳定（上限 %dms）" % (tag, start_delay))
        if not _wait_scene_settle(cam, ctl, disp, start_delay, tag):
            return finish(pack_ready())
    P.reset(tag)                         # 清掉上一次同名任务的多帧平均缓存
    tracker = StabilityTracker(need_stable)
    t0 = ticks_ms()
    stable = 0
    iters = 0
    lost = 0
    last_center = None
    first_frame = True
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return finish(pack_ready())
        img = cam.read()
        draw_roi(img, droi)
        meta = {}
        c = vision.find_disc_center(img, droi, r_min, r_max,
                                    refine=refine, ring_fallback=ring_fb,
                                    prefer=last_center, meta=meta)

        # ---- 看不到圆：连续 N 帧就立即回 NONE，不再干等到超时 ----
        if c is None:
            lost += 1
            stable = 0
            img.draw_string(2, 2, "%s: no disc %d/%d" % (tag, lost, lost_limit),
                            image.COLOR_RED, 2)
            f = fields_of(None, 0, 0, 0, 0, meta, {"action": "no_disc"})
            if rec:
                rec.snap(img, f, "no_disc" if not first_frame else "start",
                         force=first_frame)
                first_frame = False
            if disp:
                disp.show(img)
            if lost >= lost_limit:
                log("[%s] 连续 %d 帧看不到圆 → NONE" % (tag, lost))
                if rec:
                    rec.snap(img, dict(f, action="final_none"), "final_none", force=True)
                return finish(pack_none(tag))
            if ticks_ms() - t0 > timeout_ms:
                log("[%s] 超时（看不到圆）" % tag)
                if rec:
                    rec.snap(img, dict(f, action="final_none_timeout"),
                             "final_none_timeout", force=True)
                return finish(pack_none(tag))
            continue

        lost = 0
        last_center = (c[0], c[1])
        cx, cy = P.point(c[0], c[1])
        dx = cx - center_x + dx_offset
        dy = cy - center_y + dy_offset
        item = {"cx": cx, "cy": cy}

        # 画所有候选圆（蓝=候选，绿=采用，红=最终中心）——抓图时一眼能看出是不是选错圆
        for cc in (meta.get("circles") or [])[:8]:
            try:
                img.draw_circle(int(cc[0]), int(cc[1]), int(cc[2]),
                                image.COLOR_BLUE, 1)
            except Exception:
                pass
        img.draw_circle(c[0], c[1], int(c[2]), image.COLOR_GREEN, 2)
        img.draw_cross(c[0], c[1], image.COLOR_RED, 6, 2)
        img.draw_line(det_center_x, 0, det_center_x, img.height() - 1,
                      image.COLOR_YELLOW, 1)
        img.draw_line(0, det_center_y, img.width() - 1, det_center_y,
                      image.COLOR_YELLOW, 1)
        ok_xy = abs(dx) <= tol and (not dy_on or abs(dy) <= dy_tol)
        img.draw_string(2, 2, "dx=%+d dy=%+d %s src=%s" %
                        (dx, dy, "OK" if ok_xy else "MOVE",
                         meta.get("src", "?")),
                        image.COLOR_GREEN if ok_xy else image.COLOR_YELLOW, 2)
        img.draw_string(2, 24, "r=%d n=%d %s" %
                        (int(c[2]), len(meta.get("circles") or []),
                         str(meta.get("refine_reason", ""))[:24]),
                        image.COLOR_YELLOW, 1)

        f = fields_of(c, cx, cy, dx, dy, meta)
        if rec:
            rec.snap(img, f, "start" if first_frame else "frame",
                     force=first_frame)
            first_frame = False

        # ---- 只报告模式：中心连续稳定就回，ok 表示是否在容差内 ----
        if not align_on:
            okc, _, _ = tracker.update(item)
            if okc:
                sx_out, sy_out = P.smooth(tag, cx, cy)
                log("[%s] 圆盘中心 (%d,%d) dx=%+d dy=%+d（只报告）"
                    % (tag, sx_out, sy_out, dx, dy))
                if rec:
                    rec.snap(img, dict(f, action="report"), "final_report", force=True)
                if disp:
                    disp.show(img)
                return finish(pack_disc(sx_out, sy_out, dx, 1))

        # ---- 容差内：连续 N 帧稳定才算完成 ----
        if abs(dx) <= tol and (not dy_on or abs(dy) <= dy_tol):
            reset_guard()          # 已经进容差，清掉两轴跑飞锁
            stable += 1
            if stable >= need_stable:
                sx_out, sy_out = P.smooth(tag, cx, cy)
                log("[%s] 圆盘中心 (%d,%d) dx=%+d dy=%+d 已对齐（%d 次微调）"
                    % (tag, sx_out, sy_out, dx, dy, iters))
                if rec:
                    rec.snap(img, dict(f, action="ok"), "final_ok", force=True)
                if disp:
                    disp.show(img)
                return finish(pack_disc(sx_out, sy_out, dx, 1))
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[%s] 超时（等待稳定），带偏差回复" % tag)
                if rec:
                    rec.snap(img, dict(f, action="timeout_stable"),
                             "final_timeout", force=True)
                return finish(pack_disc(cx, cy, dx, 1))
            continue

        # ---- 超差：发 MOVEXY（并行）或 MOVE / MOVEY（拆分）----
        stable = 0
        if align_on and iters < max_iter:
            guard_hit = None
            x_out = abs(dx) > tol
            y_out = bool(dy_on and abs(dy) > dy_tol)
            sx_step = 0
            sy_step = 0
            if x_out:
                if _runaway_check(dx, "x"):
                    guard_hit = ("x", dx)
                else:
                    sx_step = axis_step(dx, float(getattr(config, "MOVE_SCALE", 1.0)),
                                        invert)
            else:
                _runaway_check(dx, "x")     # 水平已收敛 → 清水平跑飞计数
            if dy_on:
                if y_out:
                    if _runaway_check(dy, "y"):
                        guard_hit = guard_hit or ("y", dy)
                    else:
                        sy_step = axis_step(dy, float(getattr(config, "MOVEY_SCALE", 1.0)),
                                            dy_invert)
                else:
                    _runaway_check(dy, "y")  # 垂直已收敛 → 清垂直跑飞计数
            if guard_hit:
                axis, val = guard_hit
                log("[%s] [保护] %s 连续同方向且 |%s| 持续增大（%s=%+d），"
                    "停止发指令，回 DISC,...,1；发 R 可解除"
                    % (tag, "MOVE" if axis == "x" else "MOVEY",
                       "dx" if axis == "x" else "dy",
                       "dx" if axis == "x" else "dy", val))
                if rec:
                    rec.snap(img, dict(f, action="runaway_guard_%s" % axis),
                             "guard_%s" % axis, force=True)
                if disp:
                    disp.show(img)
                return finish(pack_disc(cx, cy, dx, 1))
            actions = _make_align_actions(x_out, y_out, sx_step, sy_step)
            parts = []
            if _align_movexy_mode():
                if actions:
                    name, step, payload = actions[0]
                    log("[%s] dx=%+d dy=%+d 超差(±%d/±%d) → MOVEXY,%d,%d"
                        % (tag, dx, dy, tol, dy_tol, step[0], step[1]))
                    if rec:
                        rec.snap(img, dict(f, action="move_xy",
                                           step_x=step[0], step_y=step[1]),
                                 "iter%d_before_movexy" % iters, force=True)
                    link.send_payload(payload)
                    parts.append(payload)
            else:
                for name, step, payload in actions:
                    if name == "MOVE":
                        log("[%s] dx=%+d 超差(±%d) → MOVE,%d%s"
                            % (tag, dx, tol, step,
                               "（已反转）" if invert else ""))
                    else:
                        log("[%s] dy=%+d 超差(±%d) → MOVEY,%d%s"
                            % (tag, dy, dy_tol, step,
                               "（已反转）" if dy_invert else ""))
                    if rec:
                        rec.snap(img, dict(f, action="move_%s" % name.lower(),
                                           step=step),
                                 "iter%d_before_%s" % (iters, name.lower()),
                                 force=True)
                    link.send_payload(payload)
                    parts.append(payload)
            iters += 1
            # 屏幕回显本轮所有指令，并停留 MOVE_ECHO_HOLD_MS，让现场看得到
            screen_echo.set_tx(" ".join(parts))
            if disp:
                disp.show(img)
            hold = int(getattr(config, "MOVE_ECHO_HOLD_MS", 250))
            if hold > 0:
                sleep_ms(hold)
            if bool(getattr(config, "ALIGN_ONE_SHOT", False)):
                sx_out, sy_out = P.smooth(tag, cx, cy)
                ok = 1  # 回复 ok 固定为 1
                reply_x, reply_y, reply_dx = sx_out, sy_out, dx
                log("[%s] 一次性模式：已发 %s，%s"
                    % (tag, " ".join(parts),
                       "回读调整后值再回 DISC" if _align_verify_on()
                       else "去回读：直接回动作前偏差"))
                if rec:
                    rec.snap(img, dict(f, action="move_oneshot",
                                       steps=[s for _, s, _ in actions]),
                             "final_oneshot", force=True)

                # ALIGN_REPLY_MODE="verify" 时才回读；"direct" 直接回动作前的值。
                if _align_verify_on():
                    def detect_post(pimg):
                        pmeta = {}
                        pc = vision.find_disc_center(
                            pimg, droi, r_min, r_max, refine=refine,
                            ring_fallback=ring_fb, prefer=None, meta=pmeta)
                        if pc is None:
                            return None
                        pcx, pcy = P.point(pc[0], pc[1])
                        return pcx, pcy, {"c": pc, "meta": pmeta}

                    expect_x = _actions_sent_x(actions)
                    expect_y = _actions_sent_y(actions)
                    post = _wait_stable_post(
                        cam, ctl, detect_post,
                        int(getattr(config, "ONE_SHOT_VERIFY_MS", 1200)),
                        int(getattr(config, "ONE_SHOT_VERIFY_MAX_MS", 4000)),
                        int(getattr(config, "ONE_SHOT_STOP_TOL", 3)),
                        int(getattr(config, "ONE_SHOT_STOP_FRAMES", 3)),
                        baseline=(cx, cy),
                        require_x=expect_x, require_y=expect_y)
                    if post is not None:
                        pc = post["meta"]["c"]
                        pmeta = post["meta"]["meta"]
                        pcx, pcy = post["cx"], post["cy"]
                        pdx = pcx - center_x + dx_offset
                        pdy = pcy - center_y + dy_offset
                        reply_x, reply_y, reply_dx = pcx, pcy, pdx
                        if post["stable"]:
                            stop = "已停稳"
                        elif expect_y and not post.get("moved_y"):
                            stop = "未检测到垂直位移"
                        else:
                            stop = "未停稳(超时)"
                        log("[%s] 调整后偏差 dx=%+d dy=%+d 中心=(%d,%d) %s（%dms，回传此值，不再发 MOVE）"
                            % (tag, pdx, pdy, pcx, pcy, stop,
                               int(post["elapsed_ms"])))
                        if rec:
                            pf = fields_of(
                                pc, pcx, pcy, pdx, pdy, pmeta,
                                {"action": "post_move",
                                 "post_dx": int(pdx), "post_dy": int(pdy),
                                 "post_src": pmeta.get("src", "?"),
                                 "post_stable": bool(post["stable"]),
                                 "post_moved_x": bool(post.get("moved_x")),
                                 "post_moved_y": bool(post.get("moved_y"))})
                            rec.snap(post["img"], pf, "final_post", force=True)
                    else:
                        log("[%s] 调整后偏差：未检测到目标，回传移动前值" % tag)
                return finish(pack_disc(reply_x, reply_y, reply_dx, ok))
            if settle:
                sleep_ms(settle)
            if ticks_ms() - t0 > timeout_ms:
                log("[%s] 超时（微调中），带偏差回复" % tag)
                if rec:
                    rec.snap(img, dict(f, action="timeout_move"),
                             "final_timeout", force=True)
                return finish(pack_disc(cx, cy, dx, 1))
            continue

        if align_on and iters >= max_iter:
            log("[%s] 微调 %d 次仍未对齐（dx=%+d dy=%+d），带偏差回复"
                % (tag, iters, dx, dy))
            if rec:
                rec.snap(img, dict(f, action="fail_max_iter"),
                         "final_fail", force=True)
            if disp:
                disp.show(img)
            return finish(pack_disc(cx, cy, dx, 1))

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[%s] 超时" % tag)
            if rec:
                rec.snap(img, dict(f, action="timeout"), "final_timeout", force=True)
            return finish(pack_disc(cx, cy, dx, 1))
    return finish(pack_none(tag))


# ======================================================================
# 任务 H：物料盘下弧对齐（盘太大只见下弧，以「最低点」为基准）
# ======================================================================
def task_plate(cam, disp, timeout_ms=None, p=None, ctl=None):
    """任务 H：把物料盘的「最低点」对齐到目标位置。

    识别：vision.find_disc_arc 从粗圆心沿下弧找边缘 → 最小二乘拟合圆 → 最低点 (cx, cy+r)。
    对齐：与 D/E 同一套，发 MOVEXY,dx,dy 微调（ALIGN_MODE="split" 时退回 MOVE+MOVEY）。
    回复：DISC,x,y,dx,1（x,y = 盘最低点，dx = 水平偏差；dy 只随 MOVEXY 下发）
          NONE,H  连续 PLATE_LOST_FRAMES 帧看不到下弧 / 超时
    """
    P = _pipeline(p)
    link = getattr(ctl, "link", None) if ctl is not None else None
    align_on = bool(getattr(config, "PLATE_ON", True)) and link is not None
    sx = getattr(P, "sx", 1.0) or 1.0
    sy = getattr(P, "sy", 1.0) or 1.0
    init_cx = float(getattr(config, "PLATE_INIT_CX", 310.0)) / sx
    init_cy = float(getattr(config, "PLATE_INIT_CY", -55.0)) / sy
    init_r = float(getattr(config, "PLATE_INIT_R", 330.0)) / sx
    bands = getattr(config, "PLATE_SEARCH_BANDS", (60.0, 28.0, 16.0))
    bands = tuple(float(b) / sx for b in bands)
    tx = float(getattr(config, "PLATE_TARGET_X", 311.0))
    ty = float(getattr(config, "PLATE_TARGET_Y", 267.0))
    dx_offset = int(getattr(config, "PLATE_DX_OFFSET", 0))
    dy_offset = int(getattr(config, "PLATE_DY_OFFSET", 0))
    tol_x = int(getattr(config, "PLATE_TOL_X", 6))
    tol_y = int(getattr(config, "PLATE_TOL_Y", 6))
    need_stable = max(1, int(getattr(config, "PLATE_STABLE_FRAMES", 3)))
    lost_limit = max(1, int(getattr(config, "PLATE_LOST_FRAMES", 2)))
    gain = float(getattr(config, "PLATE_GAIN", getattr(config, "ALIGN_GAIN", 0.65)))
    step_min = max(1, int(getattr(config, "ALIGN_STEP_MIN", 2)))
    step_max = int(getattr(config, "ALIGN_STEP_MAX", 0))
    if step_max > 0:
        step_max = max(step_min, step_max)
    inv_x = bool(getattr(config, "PLATE_INVERT_X", False))
    inv_y = bool(getattr(config, "PLATE_INVERT_Y", False))
    oneshot = bool(getattr(config, "ALIGN_ONE_SHOT", True))  # 跟随 D/E 的一次/闭环模式
    max_iter = max(1, int(getattr(config, "PLATE_MAX_ITER", 3)))
    settle = max(0, int(getattr(config, "ALIGN_SETTLE_MS", 800)))
    if timeout_ms is None:
        timeout_ms = int(getattr(config, "PLATE_TIMEOUT_MS", 8000))

    def _detect(img):
        """一帧：检测下弧 → 返回 (最低点x, 最低点y, meta)（对外口径）或 None"""
        meta = {}
        c = vision.find_disc_arc(img, init_cx, init_cy, init_r, bands=bands, meta=meta)
        if c is None:
            return None
        cx, cy, r = c
        lx, ly = P.point(cx, cy + r)        # 检测域最低点 → 对外口径（含去畸变）
        return lx, ly, {"c": c, "meta": meta}

    log("[H] 物料盘下弧对齐（粗圆心 %.0f,%.0f r=%.0f | 目标最低点 %.0f,%.0f | 容差 ±%d/±%d | %s | 超时 %dms）"
        % (init_cx, init_cy, init_r, tx, ty, tol_x, tol_y,
           "MOVEXY 双轴" if _align_movexy_mode() else "MOVE+MOVEY", timeout_ms))

    # 收到 H 后等画面稳定（上限 PLATE_START_DELAY_MS）再读第一帧
    start_delay = max(0, int(getattr(config, "PLATE_START_DELAY_MS", 0)))
    if start_delay > 0:
        log("[H] 等待画面稳定（上限 %dms）" % start_delay)
        if not _wait_scene_settle(cam, ctl, disp, start_delay, "H"):
            return pack_ready()

    P.reset("H")
    t0 = ticks_ms()
    iters = 0
    stable = 0
    lost = 0
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        r = _detect(img)

        # ---- 看不到下弧：连续 N 帧立即回 NONE ----
        if r is None:
            lost += 1
            stable = 0
            img.draw_string(2, 2, "H: no arc %d/%d" % (lost, lost_limit),
                            image.COLOR_RED, 2)
            if disp:
                disp.show(img)
            if lost >= lost_limit:
                log("[H] 连续 %d 帧看不到下弧 → NONE" % lost)
                return pack_none("H")
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（看不到下弧）")
                return pack_none("H")
            continue

        lost = 0
        lx, ly, info = r
        cx, cy, rr = info["c"]
        flx, fly = P.smooth("H", lx, ly)
        dx = int(round(flx - tx + dx_offset))
        dy = int(round(fly - ty + dy_offset))
        ok_xy = abs(dx) <= tol_x and abs(dy) <= tol_y

        # 画出拟合圆 + 最低点 + 目标十字（调试抓图/预览都看得到）
        try:
            img.draw_circle(int(round(cx)), int(round(cy)), int(round(rr)),
                            image.COLOR_GREEN, 2)
            img.draw_cross(int(round(cx)), int(round(cy + rr)), image.COLOR_RED, 6, 2)
            img.draw_line(0, int(round(ty)), img.width() - 1, int(round(ty)),
                          image.COLOR_YELLOW, 1)
            img.draw_line(int(round(tx)), 0, int(round(tx)), img.height() - 1,
                          image.COLOR_YELLOW, 1)
        except Exception:
            pass
        img.draw_string(2, 2, "dx=%+d dy=%+d %s n=%d" %
                        (dx, dy, "OK" if ok_xy else "MOVE",
                         info["meta"].get("npts", 0)),
                        image.COLOR_GREEN if ok_xy else image.COLOR_YELLOW, 2)

        # ---- 容差内：连续 N 帧稳定才回 ok=1 ----
        if ok_xy:
            reset_guard()
            stable += 1
            if stable >= need_stable:
                log("[H] 物料盘最低点 (%d,%d) dx=%+d dy=%+d 已对齐（%d 次微调）"
                    % (flx, fly, dx, dy, iters))
                if disp:
                    disp.show(img)
                return pack_disc(flx, fly, dx, 1)
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（等待稳定），带偏差回复")
                return pack_disc(flx, fly, dx, 1)
            continue

        stable = 0

        # ---- 超差：发 MOVEXY（并行）或 MOVE / MOVEY（拆分）----
        if align_on and iters < max_iter:
            x_out = abs(dx) > tol_x
            y_out = abs(dy) > tol_y
            sx_step = _axis_step(dx, gain, step_min, step_max, 1.0, inv_x) if x_out else 0
            sy_step = _axis_step(dy, gain, step_min, step_max, 1.0, inv_y) if y_out else 0
            actions = _make_align_actions(x_out, y_out, sx_step, sy_step)
            parts = []
            for name, step, payload in actions:
                log("[H] dx=%+d dy=%+d 超差(±%d/±%d) → %s"
                    % (dx, dy, tol_x, tol_y, payload))
                link.send_payload(payload)
                parts.append(payload)
            iters += 1
            if parts:
                screen_echo.set_tx(" ".join(parts))
            if disp:
                disp.show(img)
            if oneshot:
                rx, ry, rdx = flx, fly, dx
                if _align_verify_on():
                    # 只读回读：先等一会，再等最低点连续 N 帧基本不动；绝不补发
                    post = _wait_stable_post(
                        cam, ctl, _detect,
                        int(getattr(config, "ONE_SHOT_VERIFY_MS", 600)),
                        int(getattr(config, "ONE_SHOT_VERIFY_MAX_MS", 2500)),
                        int(getattr(config, "ONE_SHOT_STOP_TOL", 3)),
                        int(getattr(config, "ONE_SHOT_STOP_FRAMES", 3)),
                        baseline=(lx, ly),
                        require_x=x_out, require_y=y_out)
                    if post is not None:
                        rx, ry = post["cx"], post["cy"]
                        rdx = int(round(rx - tx + dx_offset))
                        log("[H] 调整后最低点 (%d,%d) dx=%+d（回传此值，不再发 MOVEXY）"
                            % (rx, ry, rdx))
                    else:
                        log("[H] 调整后偏差：未检测到目标，回传移动前值")
                else:
                    log("[H] 去回读：直接回动作前偏差 dx=%+d" % dx)
                return pack_disc(rx, ry, rdx, 1)
            if settle:
                sleep_ms(settle)
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（微调中），带偏差回复")
                return pack_disc(flx, fly, dx, 1)
            continue

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[H] 超时")
            return pack_disc(flx, fly, dx, 1)
    return pack_none("H")


# ======================================================================
# 调试：连续抓一段画面（不发 MOVE，不需要 STM32）
# ======================================================================
def task_snapshot(cam, disp, roi, tag, duration_ms=None, p=None):
    """连续记录一段时间的圆盘检测结果，用于现场分析「为什么选错圆/跳变」。

    与 task_disc 的区别：不发 MOVE、不做稳定判定，只按固定节奏抓图 + 记数据。
    触发方式：SSH 上 `touch /tmp/run_disc_once`（见 config.DISC_DEBUG_TRIGGER_FILE）。
    """
    P = _pipeline(p)
    droi = P.roi(roi)
    sx = getattr(P, "sx", 1.0) or 1.0
    r_min = max(1, int(round(config.DISC_R_MIN / sx)))
    r_max = max(2, int(round(config.DISC_R_MAX / sx)))
    refine = bool(getattr(config, "DISC_REFINE_ON", True))
    ring_fb = bool(getattr(config, "DISC_RING_FALLBACK", True))
    dur = int(duration_ms if duration_ms is not None
              else getattr(config, "DISC_DEBUG_TRIGGER_MS", 2000))
    rec = None
    if bool(getattr(config, "DISC_DEBUG_SAVE", False)):
        rec = disc_debug.DiscDebug(
            tag,
            getattr(config, "DISC_DEBUG_DIR", "/root/yyb_mover_data/disc_debug"),
            max_frames=getattr(config, "DISC_DEBUG_MAX_FRAMES", 24),
            quality=getattr(config, "DISC_DEBUG_QUALITY", 85),
            keep_tasks=getattr(config, "DISC_DEBUG_KEEP_TASKS", 20),
            min_interval_ms=min(50, int(getattr(config, "DISC_DEBUG_MIN_INTERVAL_MS", 100))))
        rec.start()

    log("[%s] 抓图模式：%dms，检测候选圆/拟合结果（不发 MOVE）" % (tag, dur))
    t0 = ticks_ms()
    n = 0
    last = None
    f = {}
    img = None
    while not app.need_exit() and (ticks_ms() - t0) < dur:
        img = cam.read()
        draw_roi(img, droi)
        meta = {}
        c = vision.find_disc_center(img, droi, r_min, r_max,
                                    refine=refine, ring_fallback=ring_fb,
                                    prefer=last, meta=meta)
        f = {"n": n}
        if c:
            last = (c[0], c[1])
            cx, cy = P.point(c[0], c[1])
            dx = cx - config.W // 2
            dy = cy - config.H // 2
            f.update({"cx": int(cx), "cy": int(cy), "r": int(c[2]),
                      "dx": int(dx), "dy": int(dy)})
            for cc in (meta.get("circles") or [])[:8]:
                try:
                    img.draw_circle(int(cc[0]), int(cc[1]), int(cc[2]),
                                    image.COLOR_BLUE, 1)
                except Exception:
                    pass
            img.draw_circle(c[0], c[1], int(c[2]), image.COLOR_GREEN, 2)
            img.draw_cross(c[0], c[1], image.COLOR_RED, 6, 2)
            img.draw_string(2, 2, "SNAP dx=%+d dy=%+d src=%s" %
                            (dx, dy, meta.get("src", "?")), image.COLOR_YELLOW, 2)
        else:
            img.draw_string(2, 2, "SNAP: no disc", image.COLOR_RED, 2)
        for k in ("src", "circles", "coarse", "refined", "ring",
                  "refine_reason", "refine_points", "ms", "near_count",
                  "picked", "attempts", "inner", "inner_ref",
                  "inner_blobs", "inner_reason",
                  "tpl", "tpl_scale", "tpl_roi", "tpl_reject", "tpl_error"):
            if k in meta:
                f[k] = meta[k]
        if rec:
            rec.snap(img, f, "snap%02d" % n)
        if disp:
            disp.show(img)
        n += 1
        sleep_ms(50)

    if rec:
        if img is not None:
            rec.snap(img, dict(f, action="final"), "final", force=True)
        rec.close("SNAPSHOT,%d" % n)
    log("[%s] 抓图完成：%d 帧" % (tag, n))
    return pack_none(tag)


# ======================================================================
# 任务 F：码垛目标中心 + 固定物料高度
# ======================================================================
def task_stack(cam, clf, disp, timeout_ms=config.TASK_TIMEOUT_MS, p=None,
               ctl=None, color_code=None):
    """任务 F：码垛（与 D/E 相同的 MOVE/MOVEY 双轴闭环）

    F,<颜色码>：找**同色**的底座；同色多个取离 ROI 中心最近的；
                没有同色底座 → 回"空环中心"（连环都没有则退 ROI 中心）。
    F（不带参数）：兼容旧行为，取 ROI 内最靠上的色块。

    对齐：视觉端发 MOVE,<dx> / MOVEY,<dy> 闭环微调；
          |dx| <= STACK_ALIGN_TOL 且 |dy| <= STACK_ALIGN_DY_TOL 连续 N 帧稳定
          → 回 STACK,x,y,dx,1。
          STACK_ALIGN_ON=False（或无串口链路）时只报告，不发指令。

    回复：STACK,x,y,dx,1（高度由下位机按物料/层数自行计算）
          STACK,x,y,dx,1（超次数/超时也带残差回复，与 D/E 的 ok 约定一致）
          NONE,F（超时仍看不到底座/环）
    """
    P = _pipeline(p)
    roi = P.roi(config.ROI_STACK)
    ai = getattr(clf, "ai", None)
    link = getattr(ctl, "link", None) if ctl is not None else None
    align_on = bool(getattr(config, "STACK_ALIGN_ON", True)) and link is not None
    gain = float(getattr(config, "ALIGN_GAIN", 0.8))
    step_min = max(1, int(getattr(config, "ALIGN_STEP_MIN", 2)))
    step_max = int(getattr(config, "ALIGN_STEP_MAX", 0))
    if step_max > 0:
        step_max = max(step_min, step_max)
    tol = int(getattr(config, "STACK_ALIGN_TOL", 5))
    dy_tol = int(getattr(config, "STACK_ALIGN_DY_TOL", 5))
    dx_offset = int(getattr(config, "STACK_ALIGN_DX_OFFSET", 0))
    dy_offset = int(getattr(config, "STACK_ALIGN_DY_OFFSET", 0))
    invert_x = bool(getattr(config, "STACK_INVERT_X", False))
    invert_y = bool(getattr(config, "STACK_INVERT_Y", False))
    need_stable = max(1, int(getattr(config, "ALIGN_STABLE_FRAMES", 3)))
    settle = max(0, int(getattr(config, "ALIGN_SETTLE_MS", 800)))
    max_iter = max(1, int(getattr(config, "ALIGN_MAX_ITER", 2)))
    hold = int(getattr(config, "MOVE_ECHO_HOLD_MS", 100))
    center_x = config.W // 2
    center_y = config.H // 2
    det_center_x = int(getattr(P, "det_w", config.W) // 2)
    det_center_y = int(getattr(P, "det_h", config.H) // 2)

    log("[F] 码垛底座定位（颜色=%s）" % (color_code or "任意"))
    log("[F] 对齐容差 dx±%dpx / dy±%dpx, 指令=%s, %s, 最多 %d 轮"
        % (tol, dy_tol, "MOVEXY" if _align_movexy_mode() else "MOVE+MOVEY",
           "开" if align_on else "关", max_iter))

    rec = None
    if bool(getattr(config, "STACK_DEBUG_SAVE", False)):
        rec = disc_debug.DiscDebug(
            "F%s" % (color_code or "any"),
            getattr(config, "STACK_DEBUG_DIR", "/root/yyb_mover_data/stack_debug"),
            max_frames=getattr(config, "STACK_DEBUG_MAX_FRAMES", 24),
            quality=getattr(config, "STACK_DEBUG_QUALITY", 85),
            keep_tasks=getattr(config, "STACK_DEBUG_KEEP_TASKS", 20),
            min_interval_ms=getattr(config, "STACK_DEBUG_MIN_INTERVAL_MS", 100))
        rec.start()

    def finish(payload):
        if rec is not None:
            rec.close(payload)
        return payload

    start_delay = max(0, int(getattr(config, "STACK_START_DELAY_MS", 0)))
    if start_delay > 0:
        log("[F] 等待画面稳定（上限 %dms）" % start_delay)
        if not _wait_scene_settle(cam, ctl, disp, start_delay, "F"):
            return finish(pack_ready())

    P.reset("F")
    tracker = StabilityTracker(config.STABLE_FRAMES)
    reset_guard()
    t0 = ticks_ms()
    stable = 0
    iters = 0
    ai_logged = False
    first_frame = True
    raw_start_saved = False
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return finish(pack_ready())
        img = cam.read()
        draw_roi(img, roi)
        # 即使这一帧完全没识别到目标，也保留一张原始校准图
        if rec is not None and not raw_start_saved:
            rec.snap(img, {"action": "raw_start"}, "raw_start", force=True)
            raw_start_saved = True

        if color_code:
            it = vision.find_stack_base(clf, img, roi, color_code, ai)
            src = it["src"] if it else None
            if src in ("ai", "ai+cv") and not ai_logged:
                log("[F] AI 颜色中心 color=%s (%d,%d) src=%s prob=%.3f tiles=%s"
                    % (color_code, it["cx"], it["cy"], src,
                       float(it.get("ai_prob", 0.0)), it.get("ai_tiles", "?")))
                ai_logged = True
        else:
            it = vision.find_stack_top(clf, img, roi, P.area(config.MIN_PIXELS))
            src = "base" if it else None

        if it is None:
            stable = 0
            tracker.update(None)
            img.draw_string(2, 2, "F: no stack", image.COLOR_RED, 2)
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[F] 超时（看不到底座/环）")
                return finish(pack_none("F"))
            continue

        vote = bool(getattr(config, "STACK_COLOR_VOTE", False))
        draw_item(img, it, "STACK:" + (src or "?"))
        cx, cy = P.point(it["cx"], it["cy"])
        dx = cx - center_x + dx_offset
        dy = cy - center_y + dy_offset

        img.draw_line(det_center_x, 0, det_center_x, img.height() - 1,
                      image.COLOR_YELLOW, 1)
        img.draw_line(0, det_center_y, img.width() - 1, det_center_y,
                      image.COLOR_YELLOW, 1)
        ok_xy = abs(dx) <= tol and abs(dy) <= dy_tol
        img.draw_string(2, 2, "F dx=%+d dy=%+d %s src=%s"
                        % (dx, dy, "OK" if ok_xy else "MOVE", src or "?"),
                        image.COLOR_GREEN if ok_xy else image.COLOR_YELLOW, 2)

        fields = {"cx": int(cx), "cy": int(cy), "dx": int(dx), "dy": int(dy),
                  "src": src, "color": color_code, "code": it.get("code"),
                  "r": it.get("r"), "ai_cx": it.get("ai_cx"),
                  "ai_cy": it.get("ai_cy"), "ai_prob": it.get("ai_prob"),
                  "ai_tiles": it.get("ai_tiles")}
        if rec is not None:
            rec.snap(img, dict(fields, action="detect"),
                     "start" if first_frame else "frame", force=first_frame)
            first_frame = False

        # ---- 只报告模式：保留原来的中心稳定判定 ----
        if not align_on:
            track_item = it if vote else {"cx": it["cx"], "cy": it["cy"]}
            ok, _, _ = tracker.update(track_item)
            if ok:
                sx_out, sy_out = P.smooth("F", cx, cy)
                log("[F] 码垛 %s (%d,%d) dx=%+d dy=%+d（只报告）"
                    % (src or "?", sx_out, sy_out, dx, dy))
                if disp:
                    disp.show(img)
                return finish(pack_stack(sx_out, sy_out, dx, 1))
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[F] 超时（等待稳定）")
                return finish(pack_stack(cx, cy, dx, 1))
            continue

        # ---- 双轴容差内：连续 N 帧稳定才算完成 ----
        if ok_xy:
            reset_guard()
            stable += 1
            if stable >= need_stable:
                sx_out, sy_out = P.smooth("F", cx, cy)
                log("[F] 码垛 %s (%d,%d) dx=%+d dy=%+d 已对齐（%d 轮）"
                    % (src or "?", sx_out, sy_out, dx, dy, iters))
                if disp:
                    disp.show(img)
                return finish(pack_stack(sx_out, sy_out, dx, 1))
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[F] 超时（等待稳定）")
                return finish(pack_stack(cx, cy, dx, 1))
            continue

        # ---- 超差：发 MOVEXY（并行）或 MOVE/MOVEY（拆分）----
        stable = 0
        if iters < max_iter:
            guard_hit = None
            x_out = abs(dx) > tol
            y_out = abs(dy) > dy_tol
            sx_step = 0
            sy_step = 0
            if x_out:
                if _runaway_check(dx, "x"):
                    guard_hit = ("x", dx)
                else:
                    sx_step = _axis_step(dx, gain, step_min, step_max,
                                         float(getattr(config, "MOVE_SCALE", 1.0)),
                                         invert_x)
            else:
                _runaway_check(dx, "x")
            if y_out:
                if _runaway_check(dy, "y"):
                    guard_hit = guard_hit or ("y", dy)
                else:
                    sy_step = _axis_step(dy, gain, step_min, step_max,
                                         float(getattr(config, "MOVEY_SCALE", 1.0)),
                                         invert_y)
            else:
                _runaway_check(dy, "y")

            if guard_hit:
                axis, val = guard_hit
                log("[F] [保护] %s 连续同方向且 |%s| 持续增大（%+d），"
                    "停止发指令；发 R 可解除"
                    % ("MOVE" if axis == "x" else "MOVEY",
                       "dx" if axis == "x" else "dy", val))
                if disp:
                    disp.show(img)
                return finish(pack_stack(cx, cy, dx, 1))

            actions = _make_align_actions(x_out, y_out, sx_step, sy_step)
            parts = []
            if _align_movexy_mode():
                if actions:
                    name, step, payload = actions[0]
                    log("[F] dx=%+d dy=%+d 超差(±%d/±%d) → MOVEXY,%d,%d"
                        % (dx, dy, tol, dy_tol, step[0], step[1]))
                    if rec is not None:
                        rec.snap(img, dict(fields, action="move_xy",
                                           step_x=step[0], step_y=step[1]),
                                 "iter%d_before_movexy" % iters, force=True)
                    link.send_payload(payload)
                    parts.append(payload)
            else:
                for name, step, payload in actions:
                    if name == "MOVE":
                        log("[F] dx=%+d 超差(±%d) → MOVE,%d%s"
                            % (dx, tol, step, "（已反转）" if invert_x else ""))
                    else:
                        log("[F] dy=%+d 超差(±%d) → MOVEY,%d%s"
                            % (dy, dy_tol, step, "（已反转）" if invert_y else ""))
                    if rec is not None:
                        rec.snap(img, dict(fields, action="move_%s" % name.lower(),
                                           step=step),
                                 "iter%d_before_%s" % (iters, name.lower()),
                                 force=True)
                    link.send_payload(payload)
                    parts.append(payload)
            iters += 1
            screen_echo.set_tx(" ".join(parts))
            if disp:
                disp.show(img)
            if hold > 0:
                sleep_ms(hold)
            if bool(getattr(config, "ALIGN_ONE_SHOT", False)):
                sx_out, sy_out = P.smooth("F", cx, cy)
                reply_x, reply_y, reply_dx = sx_out, sy_out, dx
                log("[F] 一次性模式：已发 %s，%s"
                    % (" ".join(parts),
                       "回读调整后值再回 STACK" if _align_verify_on()
                       else "去回读：直接回动作前偏差"))
                if disp:
                    disp.show(img)

                # ALIGN_REPLY_MODE="verify" 时才回读；"direct" 直接回动作前的值。
                if _align_verify_on():
                    def detect_post(pimg):
                        if color_code:
                            pit = vision.find_stack_base(clf, pimg, roi, color_code, ai)
                            psrc = pit["src"] if pit else None
                        else:
                            pit = vision.find_stack_top(clf, pimg, roi,
                                                       P.area(config.MIN_PIXELS))
                            psrc = "base" if pit else None
                        if pit is None:
                            return None
                        pcx, pcy = P.point(pit["cx"], pit["cy"])
                        # 每次回读都留一张校准照片，便于复盘 F 的跳点/过冲
                        if rec is not None:
                            pdx = pcx - center_x + dx_offset
                            pdy = pcy - center_y + dy_offset
                            rec.snap(
                                pimg,
                                {"cx": int(pcx), "cy": int(pcy),
                                 "dx": int(pdx), "dy": int(pdy),
                                 "src": psrc, "color": color_code,
                                 "code": pit.get("code"),
                                 "r": pit.get("r"),
                                 "ai_cx": pit.get("ai_cx"),
                                 "ai_cy": pit.get("ai_cy"),
                                 "ai_prob": pit.get("ai_prob"),
                                 "ai_tiles": pit.get("ai_tiles"),
                                 "action": "post_frame"},
                                "post_frame")
                        return pcx, pcy, {"it": pit, "src": psrc}

                    expect_x = _actions_sent_x(actions)
                    expect_y = _actions_sent_y(actions)
                    post = _wait_stable_post(
                        cam, ctl, detect_post,
                        int(getattr(config, "ONE_SHOT_VERIFY_MS", 1200)),
                        int(getattr(config, "ONE_SHOT_VERIFY_MAX_MS", 4000)),
                        int(getattr(config, "ONE_SHOT_STOP_TOL", 3)),
                        int(getattr(config, "ONE_SHOT_STOP_FRAMES", 3)),
                        baseline=(cx, cy),
                        require_x=expect_x, require_y=expect_y)
                    if post is not None:
                        pit = post["meta"]["it"]
                        psrc = post["meta"]["src"]
                        pcx, pcy = post["cx"], post["cy"]
                        pdx = pcx - center_x + dx_offset
                        pdy = pcy - center_y + dy_offset
                        reply_x, reply_y, reply_dx = pcx, pcy, pdx
                        if post["stable"]:
                            stop = "已停稳"
                        elif expect_y and not post.get("moved_y"):
                            stop = "未检测到垂直位移"
                        else:
                            stop = "未停稳(超时)"
                        log("[F] 调整后偏差 dx=%+d dy=%+d 中心=(%d,%d) %s（%dms，回传此值，不再发 MOVE）"
                            % (pdx, pdy, pcx, pcy, stop, int(post["elapsed_ms"])))
                        if rec is not None:
                            rec.snap(
                                post["img"],
                                {"cx": int(pcx), "cy": int(pcy),
                                 "dx": int(pdx), "dy": int(pdy),
                                 "src": psrc, "color": color_code,
                                 "code": pit.get("code"),
                                 "r": pit.get("r"),
                                 "ai_cx": pit.get("ai_cx"),
                                 "ai_cy": pit.get("ai_cy"),
                                 "ai_prob": pit.get("ai_prob"),
                                 "ai_tiles": pit.get("ai_tiles"),
                                 "action": "post_move",
                                 "post_dx": int(pdx), "post_dy": int(pdy),
                                 "post_stable": bool(post["stable"]),
                                 "post_moved_x": bool(post.get("moved_x")),
                                 "post_moved_y": bool(post.get("moved_y"))},
                                "final_post", force=True)
                    else:
                        log("[F] 调整后偏差：未检测到目标，回传移动前值")
                return finish(pack_stack(reply_x, reply_y, reply_dx, 1))
            if settle:
                sleep_ms(settle)
            if ticks_ms() - t0 > timeout_ms:
                log("[F] 超时（微调中）")
                return finish(pack_stack(cx, cy, dx, 1))
            continue

        log("[F] 微调 %d 轮仍未对齐（dx=%+d dy=%+d）" % (iters, dx, dy))
        if disp:
            disp.show(img)
        return finish(pack_stack(cx, cy, dx, 1))
    return finish(pack_none("F"))


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