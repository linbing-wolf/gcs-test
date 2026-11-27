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
import math
import config
import h_material_config
import h_direction
import h_packet
from material_state import MotionWindow
import cam_light
import precision
import vision
import disc_debug
import screen_echo
from protocol import (pack_start, pack_task, pack_pick, pack_disc, pack_stack,
                      pack_none, parse_task_code, pack_obstacle, pack_clear,
                      pack_ready, pack_busy, pack_move, pack_movey,
                      pack_movexy, pack_h_colors, pack_h_disc, pack_material_state,
                      is_color_code, parse_command, CMD_RESET)


def log(*a):
    if config.VERBOSE:
        print(*a)


# 没传 pipeline 时用恒等映射（触屏调试 / 单测路径），行为与旧版一致
_IDENTITY = precision.Pipeline(config.W, config.H, config.W, config.H)


def _pipeline(p):
    return p if p is not None else _IDENTITY


# 只有已经完成停稳判断的 J 才能给下一轮 J 预热。H 的即时终包是运动
# 快照，不能作为稳定种子，否则下一轮 J 可能跳过真实的观察窗口。
_MATERIAL_STATE_SEED = {}


def clear_material_state_seed():
    _MATERIAL_STATE_SEED.clear()


def _remember_material_state_seed(positions, now):
    _MATERIAL_STATE_SEED.clear()
    for code, x, y in positions:
        _MATERIAL_STATE_SEED[str(code)] = (int(now), int(x), int(y))


def _seed_material_state_window(window, code, now):
    entry = _MATERIAL_STATE_SEED.get(str(code))
    if entry is None:
        return False
    seen_ms, x, y = entry
    max_age = max(0, int(getattr(config, "MATERIAL_STATE_SEED_MAX_AGE_MS", 5000)))
    if now - seen_ms > max_age:
        return False
    step_ms = max(1, min(50, int(config.MATERIAL_STATE_MAX_GAP_MS) - 1))
    window.seed_stable(now, x, y, step_ms=step_ms)
    log("[J] 复用上一轮停稳坐标 color=%s age=%dms pos=(%d,%d)"
        % (code, now - seen_ms, x, y))
    return True


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


_BIAS_CACHE = {"t": None, "vals": None, "sig": "", "src": "config"}


def _bias_default(key):
    value = getattr(config, key, None)
    if value is None and key.startswith("ALIGN_"):
        value = getattr(config, key.rsplit("_", 1)[0], 0)
    return float(0 if value is None else value)


def _plate_bias():
    """D/E/F/H 的目标/偏置：config 默认值，可被运行时文件覆盖。

    覆盖文件逐行 `KEY=VALUE`（见 config.PLATE_BIAS_FILE / PLATE_BIAS_KEYS），
    只写要改的项即可；文件不存在/解析失败的行 → 该键仍用 config 默认。
    结果按 PLATE_BIAS_CACHE_MS 缓存；各任务在开始时读取一次，任务内固定。
    """
    c = _BIAS_CACHE
    now = ticks_ms()
    ttl = int(getattr(config, "PLATE_BIAS_CACHE_MS", 500))
    if c["t"] is not None and ttl > 0 and now >= c["t"] and now - c["t"] < ttl:
        return c["vals"]
    keys = tuple(getattr(config, "PLATE_BIAS_KEYS",
                          ("PLATE_TARGET_X", "PLATE_TARGET_Y",
                           "PLATE_DX_OFFSET", "PLATE_DY_OFFSET")))
    vals = {k: _bias_default(k) for k in keys}
    src = "config"
    path = str(getattr(config, "PLATE_BIAS_FILE", "") or "")
    try:
        if path and os.path.exists(path):
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or "=" not in line:
                        continue
                    k, _, v = line.partition("=")
                    k = k.strip()
                    if k not in vals:
                        continue
                    try:
                        value = float(v.split("#")[0].strip())
                        if math.isfinite(value):
                            vals[k] = value
                    except ValueError:
                        continue
            src = path
    except Exception:
        pass
    for k in keys:                      # 偏置按整数用
        if "_OFFSET" in k:
            vals[k] = int(round(vals[k]))
    sig = ",".join("%s=%s" % (k, vals[k]) for k in keys)
    if c["t"] is None or sig != c["sig"] or src != c["src"]:
        log("[ALIGN] 落点参数（%s）：%s" % (src, sig))
    c["vals"], c["sig"], c["src"], c["t"] = vals, sig, src, now
    return vals


def _disc_offsets(tag):
    """Return the effective X/Y target offsets used by one D/E task."""
    dx_key = "ALIGN_DX_OFFSET_" + str(tag)
    dy_key = "ALIGN_DY_OFFSET_" + str(tag)
    dx_offset = getattr(config, dx_key, None)
    if dx_offset is None:
        dx_offset = getattr(config, "ALIGN_DX_OFFSET", 0)
    dy_offset = getattr(config, dy_key, None)
    if dy_offset is None:
        dy_offset = getattr(config, "ALIGN_DY_OFFSET", 0)
    bias = _plate_bias()
    return (
        int(bias.get(dx_key, int(dx_offset))),
        int(bias.get(dy_key, int(dy_offset))),
    )


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


def _wait_scene_settle(cam, ctl, disp, max_ms, tag="", frames=None):
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

    need = max(1, int(frames if frames is not None
                      else getattr(config, "SCENE_SETTLE_FRAMES", 3)))
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
        self.disc_pending_capture = None

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
    names = {"1": "RED", "2": "YELLOW", "3": "BLUE", "4": "GREEN"}
    txt = "%s %s (%d,%d)" % (label, names.get(str(item.get("code")), ""),
                               item["cx"], item["cy"])
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
    P.reset("A")  # 每次任务独立，避免上次启停区坐标混入本次回包
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
    P.reset("C")  # 不同颜色/车位的原料任务不能共享历史坐标
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
    """任务 D/E 入口：只在任务期间打开补光灯（见 config.DISC_LIGHT_ON）。

    D/E 与 H 一样是「近距离对盘」，现场要求也点上灯；其余命令不亮。
    """
    light_on = bool(getattr(config, "DISC_LIGHT_ON", True)) and cam_light.on()
    try:
        return _task_disc_impl(cam, disp, roi, tag, timeout_ms, p, ctl)
    finally:
        if light_on:
            cam_light.off()


def _is_disc_terminal_payload(payload, tag):
    payload = str(payload)
    return payload.startswith("DISC,") or payload == pack_none(tag)


def capture_disc_terminal_followup(cam, roi, tag, payload, p=None,
                                   delay_ms=None, target_delay_ms=None,
                                   actual_delay_ms=None, rec=None,
                                   close_record=True):
    """Capture one post-terminal D/E frame without changing the reply packet.

    The caller must send ``payload`` before invoking this helper. The helper
    sleeps for the configured settle delay and then records a raw JPEG plus
    the current disc detection, terminal packet, and effective target offsets.
    """
    if str(tag) not in ("D", "E") or not _is_disc_terminal_payload(payload, tag):
        return None
    if not bool(getattr(config, "DISC_TERMINAL_CAPTURE_ENABLED", True)):
        return None

    if delay_ms is None:
        delay_ms = getattr(config, "DISC_TERMINAL_CAPTURE_DELAY_MS", 600)
    delay_ms = max(0, int(delay_ms))
    if target_delay_ms is None:
        target_delay_ms = delay_ms
    target_delay_ms = max(0, int(target_delay_ms))
    wait_started_ms = ticks_ms()
    if delay_ms:
        sleep_ms(delay_ms)
    if actual_delay_ms is None:
        actual_delay_ms = ticks_ms() - wait_started_ms
    actual_delay_ms = max(0, int(actual_delay_ms))

    created_record = rec is None
    if created_record:
        rec = disc_debug.DiscDebug(
            "%s-post" % tag,
            getattr(config, "DISC_DEBUG_DIR", "/root/yyb_mover_data/disc_debug"),
            max_frames=1,
            quality=getattr(config, "DISC_DEBUG_QUALITY", 85),
            keep_tasks=getattr(config, "DISC_DEBUG_KEEP_TASKS", 20),
            min_interval_ms=0,
            enabled=True,
        )
        rec.start()
    previous_light_on = cam_light.is_on()
    light_changed = False
    detected = False
    captured_error = ""
    try:
        light_changed = (
            bool(getattr(config, "DISC_TERMINAL_CAPTURE_LIGHT_ON", True))
            and not previous_light_on
            and cam_light.on()
        )
        img = cam.read()
        pipeline = _pipeline(p)
        detection_roi = pipeline.roi(roi)
        scale = getattr(pipeline, "sx", 1.0) or 1.0
        r_min = max(1, int(round(config.DISC_R_MIN / scale)))
        r_max = max(2, int(round(config.DISC_R_MAX / scale)))
        meta = {}
        center = vision.find_disc_center(
            img,
            detection_roi,
            r_min,
            r_max,
            refine=bool(getattr(config, "DISC_REFINE_ON", True)),
            ring_fallback=bool(getattr(config, "DISC_RING_FALLBACK", True)),
            meta=meta,
            template_paths=vision.disc_template_paths(tag),
        )
        dx_offset, dy_offset = _disc_offsets(tag)
        fields = {
            "terminal_payload": str(payload),
            "delay_ms": target_delay_ms,
            "actual_delay_ms": actual_delay_ms,
            "dx_offset": dx_offset,
            "dy_offset": dy_offset,
            "roi": list(detection_roi),
        }
        if center is not None:
            raw_x, raw_y = center[0], center[1]
            cx, cy = pipeline.point(raw_x, raw_y)
            dx = int(cx) - config.W // 2 + dx_offset
            dy = int(cy) - config.H // 2 + dy_offset
            fields.update({
                "cx": int(cx),
                "cy": int(cy),
                "dx": dx,
                "dy": dy,
                "r": int(center[2]),
                "detected": True,
            })
            if meta:
                fields["detection_meta"] = meta
            draw_roi(img, detection_roi)
            img.draw_circle(raw_x, raw_y, int(center[2]), image.COLOR_GREEN, 2)
            img.draw_cross(raw_x, raw_y, image.COLOR_RED, 6, 2)
            img.draw_string(
                2,
                2,
                "%s post dx=%+d dy=%+d" % (tag, dx, dy),
                image.COLOR_GREEN,
                2,
            )
        else:
            fields["detected"] = False
            img.draw_string(
                2,
                2,
                "%s post: no disc" % tag,
                image.COLOR_RED,
                2,
            )

        image_name = rec.snap(
            img,
            fields,
            "after_terminal_%dms" % target_delay_ms,
            force=True,
        )
        detected = center is not None
        log("[%s] 终包后目标 %dms / 实际 %dms 拍照完成: %s" %
            (tag, target_delay_ms, actual_delay_ms,
             image_name or "未存图"))
        return image_name
    except Exception as exc:
        captured_error = str(exc)
        log("[%s] 终包后拍照失败，不影响回包: %s" % (tag, exc))
        return None
    finally:
        if light_changed:
            cam_light.off()
        extra = {
            "capture_kind": "post_terminal",
            "delay_ms": target_delay_ms,
            "actual_delay_ms": actual_delay_ms,
            "detected": detected,
        }
        if captured_error:
            extra["error"] = captured_error
        if close_record or created_record:
            try:
                rec.close(payload, extra)
            except Exception:
                pass


def _disc_capture_delay_ms(tag):
    key = "DISC_TERMINAL_CAPTURE_DELAY_%s_MS" % str(tag)
    value = getattr(config, key, None)
    if value is None:
        value = getattr(config, "DISC_TERMINAL_CAPTURE_DELAY_MS", 600)
    return max(0, int(value))


def _disc_capture_delays_ms(tag):
    configured = getattr(config, "DISC_TERMINAL_CAPTURE_DELAYS_MS", None)
    if configured is None:
        return (_disc_capture_delay_ms(tag),)
    if not isinstance(configured, (list, tuple)):
        configured = (configured,)
    delays = []
    for value in configured:
        delay_ms = max(0, int(value))
        if delay_ms > 0 and delay_ms not in delays:
            delays.append(delay_ms)
    return tuple(sorted(delays))


def schedule_disc_terminal_followup(ctl, rec, roi, tag, payload, p=None):
    """Queue planned D/E captures; the packet itself is never delayed here."""
    if not _is_disc_terminal_payload(payload, tag):
        if rec is not None:
            rec.close(payload)
        return False
    delays_ms = _disc_capture_delays_ms(tag)
    if (ctl is None
            or not bool(getattr(config, "DISC_TERMINAL_CAPTURE_ENABLED", True))
            or not delays_ms):
        if rec is not None:
            rec.close(payload)
        return False
    if getattr(ctl, "disc_pending_capture", None) is not None:
        cancel_disc_terminal_followup(ctl, "replaced")
    ctl.disc_pending_capture = {
        "tag": str(tag),
        "roi": tuple(roi),
        "payload": str(payload),
        "pipeline": p,
        "rec": rec,
        "delays_ms": delays_ms,
        "next_delay_index": 0,
        "captured_delays_ms": [],
        "armed_ms": None,
        "due_ms": None,
    }
    return True


def arm_disc_terminal_followup(ctl, payload):
    """Start the post-terminal delay after the reply has been sent."""
    pending = getattr(ctl, "disc_pending_capture", None) if ctl is not None else None
    if pending is None or pending["payload"] != str(payload):
        return False
    now = ticks_ms()
    pending["armed_ms"] = now
    pending["due_ms"] = now + pending["delays_ms"][0]
    return True


def service_disc_terminal_followup(cam, ctl):
    """Capture a due follow-up frame without waiting in a timer thread."""
    pending = getattr(ctl, "disc_pending_capture", None) if ctl is not None else None
    if pending is None or pending["due_ms"] is None:
        return False
    if ticks_ms() < pending["due_ms"]:
        return False
    index = pending["next_delay_index"]
    target_delay_ms = pending["delays_ms"][index]
    actual_ms = ticks_ms() - pending["armed_ms"]
    is_last = index + 1 >= len(pending["delays_ms"])
    log("[%s] 终包后第 %d/%d 张迟拍到期（目标 %dms / 实际 %dms）"
        % (pending["tag"], index + 1, len(pending["delays_ms"]),
           target_delay_ms, actual_ms))
    capture_disc_terminal_followup(
        cam,
        pending["roi"],
        pending["tag"],
        pending["payload"],
        p=pending["pipeline"],
        delay_ms=0,
        target_delay_ms=target_delay_ms,
        actual_delay_ms=actual_ms,
        rec=pending["rec"],
        close_record=is_last,
    )
    pending["captured_delays_ms"].append(target_delay_ms)
    if is_last:
        ctl.disc_pending_capture = None
    else:
        pending["next_delay_index"] = index + 1
        pending["due_ms"] = pending["armed_ms"] + pending["delays_ms"][index + 1]
    return True


def cancel_disc_terminal_followup(ctl, reason="next_command"):
    """Close a pending capture before a new command changes the scene."""
    pending = getattr(ctl, "disc_pending_capture", None) if ctl is not None else None
    if pending is None:
        return False
    ctl.disc_pending_capture = None
    rec = pending.get("rec")
    if rec is not None:
        try:
            rec.close(
                pending["payload"],
                {
                    "capture_kind": "post_terminal",
                    "cancelled": str(reason),
                    "planned_delays_ms": list(pending["delays_ms"]),
                    "captured_delays_ms": list(pending["captured_delays_ms"]),
                },
            )
        except Exception:
            pass
    log("[%s] 终包后迟拍取消: %s" % (pending["tag"], reason))
    return True


def _task_disc_impl(cam, disp, roi, tag, timeout_ms=None, p=None, ctl=None):
    """任务 D/E：圆盘中心 + 环2 水平/垂直对齐

    识别：find_circles 候选 → 径向边缘 + 最小二乘细化 → 找不到再用圆环兜底。
    对齐：视觉端发 MOVE,<step> 和/或 MOVEY,<step> 闭环微调；
          |dx| <= ALIGN_TOL 且 |dy| <= ALIGN_DY_TOL 连续 N 帧 → ok=1。
          ALIGN_ON=False 时不发指令，中心稳定就带当前偏差回复。
    回复：DISC,x,y,dx,1  对齐完成
          DISC,x,y,dx,0  未对齐（超次数/超时/只报告模式偏差过大）
          看不到圆超过 DISC_FALLBACK_AFTER_MS：先发 MOVEXY,0,0，
          再用配置的数值化 DISC 兜底；关闭兜底时才回 NONE,<tag>

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
    # D/E 可分别设置 dy 偏置；None 表示回退到全局 ALIGN_DY_OFFSET
    dx_offset, dy_offset = _disc_offsets(tag)
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
        if _is_disc_terminal_payload(payload, tag):
            schedule_disc_terminal_followup(ctl, rec, roi, tag, payload, p=P)
        elif rec is not None:
            rec.close(payload)
        return payload

    def finish_unaligned(cx, cy, dx):
        accepted = bool(getattr(config, "DISC_OK_ON_DETECTION", False))
        if accepted:
            log("[%s] 联调模式：识别到坐标，未完成对齐复核也回 ok=1" % tag)
        return finish(pack_disc(cx, cy, dx, int(accepted)))

    def finish_missing(reason, img=None, fields=None):
        fallback = bool(getattr(config, "DISC_FALLBACK_ON_MISS", True))
        if not fallback:
            return finish(pack_none(tag))
        x = int(getattr(config, "DISC_FALLBACK_X", center_x))
        y = int(getattr(config, "DISC_FALLBACK_Y", center_y))
        dx = int(getattr(config, "DISC_FALLBACK_DX", 0))
        ok = int(getattr(config, "DISC_FALLBACK_OK", 1))
        payload = pack_movexy(0, 0)
        if link is not None:
            link.send_payload(payload)
        screen_echo.set_tx(payload)
        log("[%s] %s，识别失败兜底：先发 MOVEXY,0,0，再回 %s"
            % (tag, reason, pack_disc(x, y, dx, ok)))
        if rec is not None and img is not None:
            rec.snap(img, dict(fields or {}, action="fallback_missing",
                               fallback_payload=pack_disc(x, y, dx, ok)),
                     "final_fallback", force=True)
        return finish(pack_disc(x, y, dx, ok))

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
                      "tpl", "tpl_scale", "tpl_roi", "tpl_reject", "tpl_error",
                      "tpl_far", "center_src", "circles_error",
                      "hough_roi", "hough_tries", "hough_use", "hough_thr",
                      "hough_batches"):
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
        _sf = int(getattr(config, "SCENE_SETTLE_FRAMES_D", 5)) if str(tag) == "D" else None
        log("[%s] 等待画面稳定（上限 %dms，判稳 %d 帧）"
            % (tag, start_delay,
               _sf if _sf is not None else int(getattr(config, "SCENE_SETTLE_FRAMES", 3))))
        if not _wait_scene_settle(cam, ctl, disp, start_delay, tag, frames=_sf):
            return finish(pack_ready())
    P.reset(tag)                         # 清掉上一次同名任务的多帧平均缓存
    tracker = StabilityTracker(need_stable)
    t0 = ticks_ms()
    stable = 0
    iters = 0
    lost = 0
    oneshot = bool(getattr(config, "ALIGN_ONE_SHOT", False))
    if oneshot:
        max_iter = 1
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
                                    prefer=last_center, meta=meta,
                                    template_paths=vision.disc_template_paths(tag))

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
            miss_ms = ticks_ms() - t0
            fallback_after = max(
                0, int(getattr(config, "DISC_FALLBACK_AFTER_MS", 1200)))
            if lost >= lost_limit and miss_ms >= fallback_after:
                _tm = meta
                _why = str(_tm.get("reason") or "?" )[:40]
                if _tm.get("tpl_reject"):
                    _why += " tpl_reject=%s" % _tm.get("tpl_reject")
                if _tm.get("tpl_far"):
                    _why += " tpl_far=%s" % (_tm.get("tpl_far"),)
                if _tm.get("tpl") is not None:
                    _why += " tpl=%s" % (_tm.get("tpl"),)
                if _tm.get("circles_error"):
                    _why += " circles_error=%s" % _tm.get("circles_error")
                if _tm.get("inner_reason"):
                    _why += " inner=%s" % _tm.get("inner_reason")
                log("[%s] 连续 %d 帧看不到圆（原因：%s）"
                    % (tag, lost, _why[:150]))
                return finish_missing("无法识别圆盘", img, dict(f, reason=_why))
            if miss_ms > timeout_ms:
                log("[%s] 超时（看不到圆）" % tag)
                return finish_missing("识别圆盘超时", img,
                                      dict(f, reason="timeout_no_disc"))
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

        # ---- 只报告模式：保留检测成功的回包语义 ----
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
                return finish_unaligned(cx, cy, dx)
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
                    "停止发指令；发 R 可解除"
                    % (tag, "MOVE" if axis == "x" else "MOVEY",
                       "dx" if axis == "x" else "dy",
                       "dx" if axis == "x" else "dy", val))
                if rec:
                    rec.snap(img, dict(f, action="runaway_guard_%s" % axis),
                             "guard_%s" % axis, force=True)
                if disp:
                    disp.show(img)
                return finish_unaligned(cx, cy, dx)
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
            if oneshot:
                log("[%s] 单次模式：已发 %s，立即带动作前坐标回 ok=1"
                    % (tag, " ".join(parts)))
                return finish(pack_disc(cx, cy, dx, 1))
            hold = int(getattr(config, "MOVE_ECHO_HOLD_MS", 250))
            wait_until = min(ticks_ms() + max(0, hold) + settle, t0 + timeout_ms)
            while ticks_ms() < wait_until:
                if app.need_exit():
                    return finish(pack_none(tag))
                if ctl is not None and not ctl.tick():
                    return finish(pack_ready())
                sleep_ms(max(0, min(20, wait_until - ticks_ms())))
            # A move invalidates pre-move observations. One-shot still verifies
            # fresh frames, but the iteration limit prevents a second move.
            P.reset(tag)
            tracker.reset()
            last_center = None
            if ticks_ms() - t0 >= timeout_ms:
                log("[%s] 超时（微调中），带偏差回复" % tag)
                if rec:
                    rec.snap(img, dict(f, action="timeout_move"),
                             "final_timeout", force=True)
                return finish_unaligned(cx, cy, dx)
            continue

        if align_on and iters >= max_iter and not oneshot:
            log("[%s] 微调 %d 次仍未对齐（dx=%+d dy=%+d），带偏差回复"
                % (tag, iters, dx, dy))
            if rec:
                rec.snap(img, dict(f, action="fail_max_iter"),
                         "final_fail", force=True)
            if disp:
                disp.show(img)
            return finish_unaligned(cx, cy, dx)

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[%s] 超时" % tag)
            if rec:
                rec.snap(img, dict(f, action="timeout"), "final_timeout", force=True)
            return finish_unaligned(cx, cy, dx)
    return finish(pack_none(tag))


# ======================================================================
# 任务 H：物料盘下弧对齐（盘太大只见下弧，以「最低点」为基准）
# ======================================================================
def task_material_state(cam, clf, disp, p=None, ctl=None):
    """J entry: observe materials with the camera light on by default."""
    light_on = bool(getattr(config, "J_LIGHT_ON", True)) and cam_light.on()
    try:
        return _task_material_state_impl(cam, clf, disp, p=p, ctl=ctl)
    finally:
        if light_on:
            cam_light.off()


def _task_material_state_impl(cam, clf, disp, p=None, ctl=None):
    """J observes indefinitely and replies only after all visible colors stop."""
    pipeline = _pipeline(p)
    tracks = {}
    next_report = 0
    report_ms = max(50, int(config.MATERIAL_STATE_REPORT_MS))
    confirm_ms = max(0, int(getattr(config, "MATERIAL_STATE_CONFIRM_MS", 1000)))
    stable_since = None
    stable_codes = ()
    while not app.need_exit():
        if ctl is not None and not ctl.tick():
            return pack_ready()
        img = cam.read()
        grouped = {}
        for item in clf.detect(img, None, pipeline.area(config.MIN_PIXELS)):
            code = clf.resolve_code(img, item)
            if is_color_code(code):
                grouped.setdefault(code, []).append(item)
        now = ticks_ms()
        records = []
        next_tracks = {}
        drawings = []
        for code in sorted(grouped):
            candidates = grouped[code]
            item = max(candidates, key=lambda candidate: candidate["pixels"])
            window = tracks.get(code)
            if window is None:
                window = MotionWindow(config.MATERIAL_STATE_HOLD_MS,
                                      config.MATERIAL_STATE_TOL_PX,
                                      config.MATERIAL_STATE_MIN_FRAMES,
                                      config.MATERIAL_STATE_MAX_GAP_MS)
                _seed_material_state_window(window, code, now)
            x, y = pipeline.point(item["cx"], item["cy"])
            state = window.update(now, x, y, trustworthy=len(candidates) == 1)
            next_tracks[code] = window
            records.append((code, x, y, state))
            drawings.append((item, state))
        tracks = next_tracks

        codes = tuple(sorted(record[0] for record in records))
        all_stopped = bool(records) and all(record[3] == 1 for record in records)
        if all_stopped and (stable_since is None or codes != stable_codes):
            stable_since = now
            stable_codes = codes
        elif not all_stopped:
            stable_since = None
            stable_codes = ()

        done = (all_stopped and stable_since is not None
                and now - stable_since >= confirm_ms)
        if disp is not None:
            for item, state in drawings:
                draw_item(img, item, "J state=%d" % state)
            disp.show(img)
        if done:
            payload = pack_material_state(records, done=True)
            _remember_material_state_seed(
                [(record[0], record[1], record[2]) for record in records], now)
            log("[J] 全部可见物料停稳，返回 %s" % payload)
            return payload
        if now >= next_report:
            # Keep diagnostics visible without creating or sending a
            # non-terminal MSTATE packet.
            detail = " ".join("%s:(%d,%d),state=%d" % record for record in records)
            log("[J] 等待中 %s" % (detail or "无有效物料"))
            next_report = now + report_ms
    return pack_none("J")


def task_h_material(cam, clf, disp, p=None, ctl=None, settings=None):
    """Observe two material arrivals, with a full-frame shortcut during motion."""
    link = ctl.link if ctl is not None else None
    if link is None:
        return pack_none("H")

    settings = h_material_config.defaults(config) if settings is None else settings
    clear_material_state_seed()
    pipeline = _pipeline(p)
    roi = pipeline.roi(config.H_MATERIAL_ROI)
    min_pixels = pipeline.area(config.MIN_PIXELS)
    color_frames = settings["color_frames"]
    # Older board profiles may still contain 3. Keep the deployed refresh
    # baseline while allowing profiles to choose a lower frame count.
    center_frames = min(
        settings["center_frames"],
        int(getattr(config, "H_MATERIAL_CENTER_FRAMES", 2)),
    )
    target_x = config.W // 2
    target_y = settings["target_y"]
    tolerance = settings["tol_y"]
    max_moves = settings["max_moves"]
    extended_packet = bool(
        getattr(config, "H_MATERIAL_EXTENDED_PACKET_ENABLED", False))
    single_stop_mode = (
        extended_packet
        or bool(getattr(config, "H_MATERIAL_SINGLE_STOP_MODE", False)))
    rec = disc_debug.DiscDebug(
        "H-material", config.PLATE_DEBUG_DIR,
        max_frames=config.PLATE_DEBUG_MAX_FRAMES,
        quality=config.PLATE_DEBUG_QUALITY,
        keep_tasks=config.PLATE_DEBUG_KEEP_TASKS,
        min_interval_ms=config.PLATE_DEBUG_MIN_INTERVAL_MS,
        enabled=config.PLATE_DEBUG_SAVE,
    )
    rec.start()
    result = pack_none("H")
    colors = []
    direction_enabled = bool(getattr(config, "H_DIRECTION_PACKET_ENABLED", False))
    direction_tracker = h_direction.DirectionTracker(getattr(config, "H_DIRECTION_FALLBACK", 1))
    direction_frozen = False
    tracks = {}
    stopped_colors = []
    already_stopped = set()
    moving_pair = ()
    pair_frames = 0
    shortcut = False
    colors_sent = False
    move_after = 0
    moves = 0
    arrival_moving_seen = False
    pending_extended = None
    rec.data({"target_x": target_x, "target_y": target_y, "roi": roi, "tolerance": tolerance,
              "gain": settings["gain"], "invert": settings["invert_move"],
              "max_step": settings["step_max"]}, "settings")
    log("[H-material] 物料中心目标 (%s,%s)，开始采集颜色" % (target_x, target_y))
    light_on = config.PLATE_LIGHT_ON and cam_light.on()
    try:
        while not app.need_exit():
            if not ctl.tick():
                result = pack_ready()
                break
            img = cam.read()
            now = ticks_ms()
            by_color = {}
            pair_candidates = {}
            width, height = img.width(), img.height()
            edge_margin = 8
            display_items = []
            for item in clf.detect(img, None, min_pixels):
                code = clf.resolve_code(img, item)
                if not is_color_code(code):
                    continue
                display_items.append(item)
                x, y = pipeline.point(item["cx"], item["cy"])
                inside = (item["x"] >= edge_margin and item["y"] >= edge_margin
                          and item["x"] + item["w"] <= width - edge_margin
                          and item["y"] + item["h"] <= height - edge_margin)
                candidate = {"code": code, "x": x, "y": y, "pixels": item["pixels"],
                             "src": item.get("src"), "raw": (item["cx"], item["cy"]),
                             "pair_eligible": inside}
                if code not in by_color or item["pixels"] > by_color[code]["pixels"]:
                    by_color[code] = candidate
                if inside and (code not in pair_candidates or item["pixels"] > pair_candidates[code]["pixels"]):
                    pair_candidates[code] = candidate

            if direction_enabled:
                direction_tracker.update(list(by_color.values()), now,
                                         allow_motion=not direction_frozen)

            if pending_extended is not None:
                if now >= move_after:
                    ordered = sorted(by_color.values(),
                                     key=lambda candidate: candidate["x"])
                    positions = [(item["code"], item["x"], item["y"])
                                 for item in ordered]
                    current = {code: (x, y) for code, x, y in positions}
                    pending_extended["detectors"] = {
                        code: detector
                        for code, detector in pending_extended["detectors"].items()
                        if code in current
                    }
                    all_stable = bool(positions)
                    for code, x, y in positions:
                        detector = pending_extended["detectors"].get(code)
                        if detector is None:
                            detector = StopDetector(
                                config.PICK_STOP_SPEED_PX_S,
                                getattr(config, "H_MATERIAL_SETTLE_HOLD_MS", 240))
                            pending_extended["detectors"][code] = detector
                        stable, _ = detector.update(x, y, now)
                        all_stable = all_stable and stable

                    codes = tuple(code for code, _, _ in positions)
                    if codes != pending_extended["stable_codes"]:
                        pending_extended["stable_codes"] = codes
                        pending_extended["stable_frames"] = 0
                    pending_extended["stable_frames"] = (
                        pending_extended["stable_frames"] + 1 if all_stable else 0)

                    if (not pending_extended["motion_seen"]
                            and pending_extended["move_requested"]):
                        move_px = float(getattr(
                            config, "H_MATERIAL_SETTLE_MOTION_PX", 4.0))
                        for code, (x, y) in current.items():
                            anchor = pending_extended["anchors"].get(code)
                            if anchor is None:
                                continue
                            ax, ay = anchor
                            if math.hypot(x - ax, y - ay) >= move_px:
                                pending_extended["motion_seen"] = True
                                log("[H-material] 检测到纠偏运动，转入停稳确认")
                                break

                    motion_ready = pending_extended["motion_seen"]
                    # H only needs one real displacement after MOVEXY. Do not
                    # wait for the scene to settle again before sending HDISC2.
                    if motion_ready:
                        result = h_packet.pack_h_disc2(
                            pending_extended["arrival_stopped"],
                            positions,
                        )
                        rec.data({
                            "phase": "extended_final",
                            "result": result,
                            "arrival_stopped": pending_extended["arrival_stopped"],
                            "positions": positions,
                            "elapsed_ms": now - pending_extended["started_ms"],
                            "motion_seen": pending_extended["motion_seen"],
                        })
                        log("[H-material] 校准后视野颜色 %d 个，稳定确认 %dms，返回 %s"
                            % (len(positions),
                               now - pending_extended["started_ms"], result))
                        break
                continue

            stopped = []
            moving_colors = set()
            next_tracks = {}
            for code, item in by_color.items():
                detector, count = tracks.get(code, (None, 0))
                if detector is None:
                    detector = StopDetector(
                        config.PICK_STOP_SPEED_PX_S,
                        getattr(config, "H_MATERIAL_STOP_HOLD_MS",
                                config.PICK_STOP_HOLD_MS))
                stable, speed = detector.update(item["x"], item["y"], now)
                count = count + 1 if stable and now >= move_after else 0
                next_tracks[code] = (detector, count)
                if not stable and speed > config.PICK_STOP_SPEED_PX_S:
                    moving_colors.add(code)
                    arrival_moving_seen = True
                rx, ry = item["raw"]
                if count >= center_frames and (not roi or
                        roi[0] <= rx < roi[0] + roi[2] and roi[1] <= ry < roi[1] + roi[3]):
                    stopped.append(item)
            tracks = next_tracks
            stopped.sort(key=lambda item: item["raw"][0] if direction_enabled
                         and direction_tracker.effective_direction == h_direction.CW else -item["x"])

            if single_stop_mode and stopped:
                current = max(stopped, key=lambda item: item["y"])
                current_code = current["code"]
                first_code = next(code for code in ("1", "2", "3", "4")
                                  if code != current_code)
                colors[:] = [first_code, current_code]
                stopped_colors[:] = [first_code, current_code]
                direction = direction_tracker.effective_direction
                color_payload = (h_direction.pack_colors(first_code, current_code, direction)
                                 if direction_enabled
                                 else pack_h_colors(first_code, current_code))
                if not colors_sent:
                    link.send_payload(color_payload)
                    screen_echo.set_tx(color_payload)
                    colors_sent = True
                    log("[H-material] 单件停稳：当前=%s（y=%d），补先到色=%s，发 %s"
                        % (current_code, current["y"], first_code, color_payload))

                dx = current["x"] - target_x
                dy = current["y"] - target_y
                limit = settings["step_max"]
                sx_step = (_axis_step(dx, settings["gain"], 1, limit,
                                      1.0, config.PLATE_INVERT_X) if dx else 0)
                sy_step = (_axis_step(dy, settings["gain"], 1, limit,
                                      1.0, settings["invert_move"]) if dy else 0)
                if extended_packet and not (sx_step or sy_step):
                    # The extended terminal packet must follow an observable
                    # move, even when the measured center is already exact.
                    sx_step = 1
                move_payload = pack_movexy(sx_step, sy_step)
                link.send_payload(move_payload)
                screen_echo.set_tx(move_payload)
                direction_frozen = True

                result = (h_direction.pack_disc(
                    current["x"], current["y"], dx, first_code, current_code, direction)
                    if direction_enabled
                    else pack_h_disc(
                        current["x"], current["y"], dx, first_code, current_code))
                moves += 1
                fields = {
                    "candidates": list(by_color.values()),
                    "center": (current["x"], current["y"]),
                    "dx": dx,
                    "dy": dy,
                    "distance_error": dy,
                    "colors": list(colors),
                    "stopped_colors": list(stopped_colors),
                    "moving_pair": False,
                    "moves": moves,
                    "direction_enabled": direction_enabled,
                    "direction": direction,
                    "direction_assumed": direction_tracker.direction is h_direction.UNKNOWN,
                    "sent": move_payload,
                    "single_stop": True,
                }
                if extended_packet:
                    pending_extended = {
                        "arrival_stopped": 0 if arrival_moving_seen else 1,
                        "started_ms": ticks_ms(),
                        "move_requested": bool(sx_step or sy_step),
                        "anchors": {
                            item["code"]: (item["x"], item["y"])
                            for item in by_color.values()
                        },
                        "detectors": {},
                        "stable_codes": (),
                        "stable_frames": 0,
                        "motion_seen": False,
                    }
                    move_after = ticks_ms() + max(
                        0, int(getattr(config, "H_MATERIAL_SETTLE_MIN_MS", 200)))
                    tracks.clear()
                    log("[H-material] 已发 %s，动态等车停稳后采集颜色坐标"
                        % move_payload)
                    continue
                try:
                    rec.snap(img, fields, "single_stop_final", force=True)
                except Exception as exc:
                    log("[H-material] 终帧抓图失败，不阻断回包：%s" % exc)
                log("[H-material] 单次校正 %s，立即返回 %s"
                    % (move_payload, result))
                if disp is not None:
                    try:
                        draw_roi(img, roi)
                        raw_x, raw_y = current["raw"]
                        img.draw_cross(raw_x, raw_y, image.COLOR_RED, 6, 2)
                        img.draw_string(
                            2, 2, "H single stop colors=%s,%s dx=%d dy=%d%s"
                            % (first_code, current_code, dx, dy,
                               " ASSUMED" if direction_tracker.direction is h_direction.UNKNOWN else ""),
                            image.COLOR_GREEN, 1)
                        disp.show(img)
                    except Exception as exc:
                        log("[H-material] 屏幕绘制失败，不阻断回包：%s" % exc)
                break

            if not single_stop_mode and not colors and not stopped:
                ordered = sorted(pair_candidates.values(), key=lambda item: -item["x"])
                pair = (direction_tracker.order_pair(list(pair_candidates.values()))
                        if direction_enabled else
                        tuple(item["code"] for item in ordered) if len(ordered) == 2 else ())
                if pair and pair == moving_pair:
                    pair_frames += 1
                else:
                    moving_pair = pair if moving_colors.intersection(pair_candidates) else ()
                    pair_frames = 1 if moving_pair else 0
                if moving_pair and pair_frames >= color_frames:
                    colors.extend(moving_pair)
                    shortcut = True

            terminal = None
            correction = None
            if len(colors) == 2:
                terminal = next((item for item in stopped if item["code"] == colors[1]), None)
            else:
                if now >= move_after:
                    already_stopped.intersection_update(by_color)
                    already_stopped.difference_update(moving_colors)
                if stopped and not stopped_colors:
                    first = stopped[0]["code"]
                    stopped_colors.append(first)
                    colors.append(first)
                    already_stopped.update(item["code"] for item in stopped[1:])
                for item in stopped:
                    code = item["code"]
                    if code == stopped_colors[0]:
                        correction = item
                    elif code not in already_stopped:
                        stopped_colors.append(code)
                        colors.append(code)
                        terminal = item
                        break

            if len(colors) == 2 and not colors_sent:
                payload = (h_direction.pack_colors(*colors, direction_tracker.effective_direction)
                           if direction_enabled else pack_h_colors(*colors))
                link.send_payload(payload)
                screen_echo.set_tx(payload)
                colors_sent = True
                if direction_enabled:
                    direction_frozen = True
                log("[H-material] 已发送 %s" % payload)

            item = terminal or correction
            center = (item["x"], item["y"]) if item is not None else None
            dx = center[0] - target_x if center is not None else None
            dy = center[1] - target_y if center is not None else None
            x_out = dx is not None and abs(dx) > tolerance
            y_out = dy is not None and abs(dy) > tolerance
            fields = {"candidates": list(by_color.values()), "center": center,
                      "dx": dx, "dy": dy, "distance_error": dy,
                      "colors": list(colors), "stopped_colors": list(stopped_colors),
                      "moving_pair": shortcut, "moves": moves,
                      "direction_enabled": direction_enabled,
                      "direction": direction_tracker.effective_direction,
                      "direction_assumed": direction_tracker.direction is h_direction.UNKNOWN}
            rec.snap(img, fields, "detect")
            if disp is not None:
                try:
                    draw_roi(img, roi)
                    for display_item in display_items:
                        draw_item(img, display_item,
                                  display_item.get("src", "cv").upper())
                    img.draw_string(
                        2, 2, "H colors=%s dx=%s dy=%s"
                        % (",".join(colors), dx, dy),
                        image.COLOR_GREEN, 1)
                    disp.show(img)
                except Exception as exc:
                    log("[H-material] 屏幕绘制失败，不阻断回包：%s" % exc)

            if terminal is not None and (not (x_out or y_out) or max_moves == 0):
                result = (h_direction.pack_disc(center[0], center[1], dx, *colors, direction_tracker.effective_direction)
                          if direction_enabled else pack_h_disc(center[0], center[1], dx, *colors))
                break
            # Keep one correction available for the final material, which owns the result.
            move_limit = max_moves if terminal is not None else max(0, max_moves - 1)
            if (x_out or y_out) and now >= move_after and moves < move_limit:
                gain = settings["gain"]
                limit = settings["step_max"]
                sx_step = _axis_step(dx, gain, 1, limit, 1.0, config.PLATE_INVERT_X) if x_out else 0
                sy_step = _axis_step(dy, gain, 1, limit, 1.0, settings["invert_move"]) if y_out else 0
                payload = pack_movexy(sx_step, sy_step)
                link.send_payload(payload)
                screen_echo.set_tx(payload)
                direction_frozen = True
                moves += 1
                tracks.clear()
                move_after = ticks_ms() + settings["settle_ms"]
                rec.data(dict(fields, sent=payload), "move")
                log("[H-material] dx=%+d dy=%+d 超差，发 %s" % (dx, dy, payload))
                if terminal is not None and moves >= max_moves:
                    result = (h_direction.pack_disc(center[0], center[1], dx, *colors, direction_tracker.effective_direction)
                          if direction_enabled else pack_h_disc(center[0], center[1], dx, *colors))
                    rec.data(dict(fields, moves=moves, reason="budget_exhausted"), "final_budget")
                    log("[H-material] 纠偏次数用尽，带末轮动作前坐标回 %s" % result)
                    break
        return result
    finally:
        try:
            rec.close(result, {"colors": colors, "moves": moves})
        finally:
            if light_on:
                cam_light.off()


def task_plate(cam, disp, timeout_ms=None, p=None, ctl=None):
    """任务 H：把物料盘的「最低点」对齐到目标位置（H 期间打开补光灯）。

    识别：vision.find_disc_arc 从粗圆心沿下弧找边缘 → 拟合轴对齐椭圆 → 最低点。
    对齐：与 D/E 同一套，发 MOVEXY,dx,dy 微调（ALIGN_MODE="split" 时退回 MOVE+MOVEY）。
    回复：DISC,x,y,dx,1（x,y = 盘最低点，dx = 水平偏差；dy 只随 MOVEXY 下发）
          NONE,H  连续 PLATE_LOST_FRAMES 帧看不到下弧（且几何上不可能）/ 超时

    补光灯：现场要求「只给 H 加灯」→ 进来点亮、退出（含中断/超时/异常）关掉。
    """
    light_on = bool(getattr(config, "PLATE_LIGHT_ON", True)) and cam_light.on()
    try:
        return _task_plate_impl(cam, disp, timeout_ms, p, ctl)
    finally:
        if light_on:
            cam_light.off()


def _task_plate_impl(cam, disp, timeout_ms=None, p=None, ctl=None):
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
    bias = _plate_bias()
    tx = float(bias.get("PLATE_TARGET_X", 311.0))
    ty = float(bias.get("PLATE_TARGET_Y", 267.0))
    dx_offset = int(bias.get("PLATE_DX_OFFSET", 0))
    dy_offset = int(bias.get("PLATE_DY_OFFSET", 0))
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

    _detect_meta = {}

    def _detect(img):
        """一帧：检测下弧 → 返回 (最低点x, 最低点y, meta)（对外口径）或 None"""
        meta = {}
        _detect_meta["m"] = meta
        c = vision.find_disc_arc(img, init_cx, init_cy, init_r, bands=bands, meta=meta)
        if c is None:
            return None
        cx, cy, r = c
        lx, ly = P.point(cx, cy + r)        # 检测域最低点 → 对外口径（含去畸变）
        return lx, ly, {"c": c, "meta": meta}

    # ---- 调试抓图：每次 H 在 PLATE_DEBUG_DIR 下建一个任务目录 ----
    rec = None
    if bool(getattr(config, "PLATE_DEBUG_SAVE", False)):
        rec = disc_debug.DiscDebug(
            "H",
            getattr(config, "PLATE_DEBUG_DIR", "/root/yyb_mover_data/plate_debug"),
            max_frames=getattr(config, "PLATE_DEBUG_MAX_FRAMES", 16),
            quality=getattr(config, "PLATE_DEBUG_QUALITY", 85),
            keep_tasks=getattr(config, "PLATE_DEBUG_KEEP_TASKS", 10),
            min_interval_ms=getattr(config, "PLATE_DEBUG_MIN_INTERVAL_MS", 100))
        rec.start()

    def finish(payload):
        if rec is not None:
            rec.close(payload)
        return payload

    def dbg_fields(c, fflx, ffly, dx, dy, meta, extra=None):
        f = {"iter": iters, "lost": lost, "stable": stable,
             "low_x": int(round(fflx)), "low_y": int(round(ffly)),
             "dx": int(dx), "dy": int(dy)}
        if c is not None:
            f["cx"], f["cy"], f["r"] = int(c[0]), int(c[1]), int(c[2])
        if meta:
            for k in ("model", "a", "b", "k", "rms", "npts", "reason",
                      "low_ellipse", "low_direct", "low_delta", "low_check",
                      "low_used", "low_conf", "inside_ratio", "inside_check",
                      "inside_need", "inside_rays", "tries", "prior_used",
                      "coarse", "coarse_reject", "plate_level", "need_in",
                      "inside_evidence", "arc_support", "occlusion",
                      "inside_reason", "rim_support", "rim_span", "rim_sides",
                      "rim_longest_run", "rim_contrast", "rim_drop",
                      "arc_span", "arc_bins", "arc_sides", "rim_check",
                      "rim_visible_sides"):
                if k in meta:
                    f[k] = meta[k]
        if extra:
            f.update(extra)
        return f

    log("[H] 物料盘下弧对齐（粗圆心 %.0f,%.0f r=%.0f | 目标最低点 %.0f,%.0f → 有效 %.0f,%.0f | 容差 ±%d/±%d | %s | 模型 %s | 落点来源 %s | 超时 %dms）"
        % (init_cx, init_cy, init_r, tx, ty, tx - dx_offset, ty - dy_offset,
           tol_x, tol_y,
           "MOVEXY 双轴" if _align_movexy_mode() else "MOVE+MOVEY",
           getattr(config, "PLATE_ARC_MODEL", "ellipse"),
           _BIAS_CACHE.get("src", "config"), timeout_ms))

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
            if lost == 1:
                _why = _detect_meta.get("m", {})
                if rec is not None:
                    rec.snap(img, {"action": "failed_raw", "lost": lost,
                                   "diagnostic": dict(_why)}, "failed_raw")
                log("[H] 本帧未得到可信下弧：%s%s"
                    % (str(_why.get("reason", "?"))[:60],
                       ("（内侧亮度通过率 %s）" % _why.get("inside_ratio"))
                       if _why.get("inside_ratio") is not None else ""))
            img.draw_string(2, 2, "H: no arc %d/%d" % (lost, lost_limit),
                            image.COLOR_RED, 2)
            if rec is not None:
                rec.data({"iter": iters, "lost": lost,
                          "model": getattr(config, "PLATE_ARC_MODEL", "ellipse"),
                          "diagnostic": dict(_detect_meta.get("m", {}))},
                         "no_arc")
            if disp:
                disp.show(img)
            if lost >= lost_limit:
                log("[H] 连续 %d 帧看不到下弧 → NONE" % lost)
                if rec is not None:
                    rec.snap(img, {"action": "final_none", "lost": lost,
                                   "diagnostic": dict(_detect_meta.get("m", {}))},
                             "final_none", force=True)
                return finish(pack_none("H"))
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（看不到下弧）")
                return finish(pack_none("H"))
            continue

        lost = 0
        lx, ly, info = r
        cx, cy, rr = info["c"]
        flx, fly = P.smooth("H", lx, ly)
        dx = int(round(flx - tx + dx_offset))
        dy = int(round(fly - ty + dy_offset))
        ok_xy = abs(dx) <= tol_x and abs(dy) <= tol_y

        # 先存一张「干净帧」（画线之前），供离线复现；再存画了标注的版本
        imeta = info["meta"]
        if imeta.get("low_delta") is not None:
            _le = imeta.get("low_ellipse") or [0, 0]
            _ld = imeta.get("low_direct") or [0, 0]
            _ldx, _ldy = imeta.get("low_delta") or [0, 0]
            log("[H] 底点校验：椭圆=(%.1f,%.1f) 直测=(%.1f,%.1f) Δ=(%+.1f,%+.1f) %s"
                % (_le[0], _le[1], _ld[0], _ld[1], _ldx, _ldy,
                   imeta.get("low_check", "?")))
        if imeta.get("inside_ratio") is not None:
            log("[H] 内侧亮度：通过率 %.2f（阈值 %.2f）→ %s"
                % (imeta["inside_ratio"],
                   float(getattr(config, "PLATE_INSIDE_MIN_RATIO", 0.60)),
                   imeta.get("inside_check", "?")))
        if rec is not None:
            rec.snap(img, dbg_fields(info["c"], flx, fly, dx, dy, imeta,
                                     {"action": "detect_raw"}),
                     "detect_raw")
        # 画出拟合形状（椭圆/圆）+ 最低点 + 目标十字（调试抓图/预览都看得到）
        is_ell = str(imeta.get("model", "circle")).lower() == "ellipse"
        try:
            if is_ell:
                ia = float(imeta.get("a", rr))
                ib = float(imeta.get("b", rr))
                img.draw_ellipse(int(round(cx)), int(round(cy)),
                                 int(round(ia)), int(round(ib)),
                                 0.0, 0.0, 360.0, image.COLOR_GREEN, 2)
            else:
                img.draw_circle(int(round(cx)), int(round(cy)), int(round(rr)),
                                image.COLOR_GREEN, 2)
            img.draw_cross(int(round(cx)), int(round(cy + rr)), image.COLOR_RED, 6, 2)
            img.draw_line(0, int(round(ty)), img.width() - 1, int(round(ty)),
                          image.COLOR_YELLOW, 1)
            img.draw_line(int(round(tx)), 0, int(round(tx)), img.height() - 1,
                          image.COLOR_YELLOW, 1)
        except Exception:
            try:
                img.draw_circle(int(round(cx)), int(round(cy)), int(round(rr)),
                                image.COLOR_GREEN, 2)
            except Exception:
                pass
        img.draw_string(2, 2, "dx=%+d dy=%+d %s %s k=%.2f rms=%.1f n=%d" %
                        (dx, dy, "OK" if ok_xy else "MOVE",
                         "ell" if is_ell else "cir",
                         float(imeta.get("k", 1.0)), float(imeta.get("rms", 0.0)),
                         imeta.get("npts", 0)),
                        image.COLOR_GREEN if ok_xy else image.COLOR_YELLOW, 2)
        if rec is not None:
            rec.snap(img, dbg_fields(info["c"], flx, fly, dx, dy, imeta,
                                     {"action": "detect", "ok": 1 if ok_xy else 0,
                                      "low_conf": 1 if imeta.get("low_conf") else 0}),
                     "detect")
        if imeta.get("low_conf"):
            log("[H] 低置信拟合（%s）→ 仍按最佳猜测执行：low=(%d,%d) dx=%+d dy=%+d"
                % (imeta.get("reject_reason", ""), flx, fly, dx, dy))

        # ---- 容差内：连续 N 帧稳定才回 ok=1 ----
        if ok_xy:
            reset_guard()
            stable += 1
            if stable >= need_stable:
                log("[H] 物料盘最低点 (%d,%d) dx=%+d dy=%+d 已对齐（%d 次微调）| %s"
                    % (flx, fly, dx, dy, iters, imeta.get("reason", "")))
                if disp:
                    disp.show(img)
                if rec is not None:
                    rec.snap(img, dbg_fields(info["c"], flx, fly, dx, dy, imeta,
                                             {"action": "final_ok"}), "final_ok", force=True)
                return finish(pack_disc(flx, fly, dx, 1))
            if disp:
                disp.show(img)
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（等待稳定），带偏差回复")
                return finish(pack_disc(flx, fly, dx, 1))
            continue

        stable = 0

        # ---- 超差：发 MOVEXY（并行）或 MOVE / MOVEY（拆分）----
        if align_on and iters < max_iter:
            x_out = abs(dx) > tol_x
            y_out = abs(dy) > tol_y
            sx_step = _axis_step(dx, gain, step_min, step_max, 1.0, inv_x) if x_out else 0
            sy_step = _axis_step(dy, gain, step_min, step_max, 1.0, inv_y) if y_out else 0
            _cap = max(0, int(getattr(config, "PLATE_LOWCONF_STEP_MAX", 0)))
            if _cap and imeta.get("low_conf"):
                sx_step = max(-_cap, min(_cap, sx_step))
                sy_step = max(-_cap, min(_cap, sy_step))
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
            if rec is not None:
                rec.snap(img, dbg_fields(info["c"], flx, fly, dx, dy, imeta,
                                         {"action": "move_xy" if _align_movexy_mode()
                                          else "move_split", "sent": " ".join(parts)}),
                         "iter%d_before_move" % (iters - 1))
            if disp:
                disp.show(img)
            if oneshot:
                rx, ry, rdx = flx, fly, dx
                post_meta = imeta
                log("[H] 去回读：直接回动作前偏差 dx=%+d" % dx)
                if rec is not None:
                    rec.snap(img, dbg_fields(info["c"], rx, ry, rdx, dy, post_meta,
                                             {"action": "final_oneshot"}),
                             "final_oneshot", force=True)
                return finish(pack_disc(rx, ry, rdx, 1))
            if settle:
                sleep_ms(settle)
            if ticks_ms() - t0 > timeout_ms:
                log("[H] 超时（微调中），带偏差回复")
                return finish(pack_disc(flx, fly, dx, 1))
            continue

        if disp:
            disp.show(img)
        if ticks_ms() - t0 > timeout_ms:
            log("[H] 超时")
            if rec is not None:
                rec.snap(img, dbg_fields(info["c"], flx, fly, dx, dy, imeta,
                                         {"action": "final_timeout"}),
                         "final_timeout", force=True)
            return finish(pack_disc(flx, fly, dx, 1))
    return finish(pack_none("H"))


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
                                    prefer=last, meta=meta,
                                    template_paths=vision.disc_template_paths(tag))
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
    bias = _plate_bias()
    dx_offset = int(bias.get("STACK_ALIGN_DX_OFFSET", getattr(config, "STACK_ALIGN_DX_OFFSET", 0)))
    dy_offset = int(bias.get("STACK_ALIGN_DY_OFFSET", getattr(config, "STACK_ALIGN_DY_OFFSET", 0)))
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
                log("[F] 一次性模式：已发 %s，去回读：直接回动作前偏差 dx=%+d"
                    % (" ".join(parts), dx))
                if disp:
                    disp.show(img)

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
            log("[G] 超时，无法确认前方畅通")
            return pack_none("G")
    return pack_none("G")
