# -*- coding: utf-8 -*-
"""
main.py —— 新流程视觉主程序（上位机服务端）

角色：MaixCAM = 上位机，被动应答下位机(STM32)的命令。
流程编排在 STM32 侧（它发命令、做运动），本程序只负责"看"和"答"。

命令分发（见 protocol.py 顶部命令字表）：
  A              → 启停区
  T,<任务码>      → 解析任务码、确定颜色
  C,<颜色码>      → 原料区三选一
  D / E          → 粗加工区 / 暂存区 圆盘中心
  F              → 码垛中心 + 物料高度
  R              → 复位（回 READY）
  Z              → 结束（回 OK）
"""

from maix import camera, display, app
from timing import ticks_ms, sleep_ms

import config
import vision
import ai_model
import precision
import protocol
import tasks
from protocol import (VisionLink, preflight, parse_command, pack_ready, pack_ok,
                      pack_error, pack_none)

# ------------------------------ 初始化 ------------------------------
def _open_camera():
    """打开相机（带重试）：失败时抛最后一次异常"""
    retries = max(1, int(getattr(config, "STARTUP_RETRY", 3)))
    delay = max(0, int(getattr(config, "STARTUP_RETRY_DELAY_MS", 1000)))
    last = None
    for attempt in range(1, retries + 1):
        try:
            c = camera.Camera(config.DET_W, config.DET_H, fps=config.FPS)
            c.skip_frames(20)
            return c
        except Exception as e:
            last = e
            print("[!] 相机打开失败(%d/%d): %s" % (attempt, retries, e))
            if attempt < retries and delay:
                sleep_ms(delay)
    raise last


# 先开串口：这样相机失败时还能回一帧 ERR,camera 给下位机
link = VisionLink(config.UART_DEV, config.UART_BAUDRATE)
try:
    cam = _open_camera()
except Exception as e:
    print("[!] 相机不可用，退出")
    try:
        link.send_payload(pack_error("camera"))
    except Exception:
        pass
    raise

disp = display.Display() if config.SHOW_PREVIEW else None

# 精度流水线：检测域 → 对外口径（缩放 / 反畸变 / 多帧平均）
pline = precision.build_pipeline()

# AI 判色（可选）：USE_AI=False 时 ai 仍会创建但内部不加载模型，ColorClassifier 退回纯 CV
ai = ai_model.AI()
clf = vision.ColorClassifier(config.THRESHOLDS, config.COLOR_NAMES, ai=ai)

# 任务执行期间的串口看门：R 中断，其它命令回 BUSY
ctl = tasks.TaskCtl(link)

# 保存最近一次收到的任务码（供 T/C 之间共享，也便于排查）
TASK = None

# ---- 触屏调试入口（默认关，见 config.TOUCH_UI）----
ui = None
ui_rects = {}
if config.TOUCH_UI:
    if disp is None:
        print("[触屏] SHOW_PREVIEW=False，跳过触屏入口")
    else:
        import touch_ui
        ui = touch_ui.TouchUI(config.W, config.H, touch_ui.DEFAULT_BUTTONS)
        ui_rects = ui.layout()
        ui.status = "TOUCH MODE - pick a task"


def log(*a):
    if config.VERBOSE:
        print(*a)


# ======================================================================
# 命令分发
# ======================================================================
def reply(payload):
    """统一发回复：任务若被 R 打断，顺手清掉任务码状态"""
    global TASK
    if ctl.reset_flag:
        ctl.reset_flag = False
        TASK = None
    link.send_payload(payload)


def dispatch(cmd, arg):
    global TASK

    if cmd == protocol.CMD_START:
        reply(tasks.task_start(cam, disp, p=pline, ctl=ctl, clf=clf))

    elif cmd == protocol.CMD_TASK:
        TASK = protocol.parse_task_code(arg)
        reply(tasks.task_task(arg))

    elif cmd == protocol.CMD_PICK:
        if not protocol.is_color_code(arg):
            log("[C] 颜色码非法: %r（只接受 1/2/3/4）" % arg)
            reply(pack_none("C"))
        else:
            reply(tasks.task_pick(cam, clf, disp, arg, p=pline, ctl=ctl))

    elif cmd == protocol.CMD_DISC_ROUGH:
        reply(tasks.task_disc(cam, disp, config.ROI_DISC_ROUGH, "D", p=pline, ctl=ctl))

    elif cmd == protocol.CMD_DISC_TEMP:
        reply(tasks.task_disc(cam, disp, config.ROI_DISC_TEMP, "E", p=pline, ctl=ctl))

    elif cmd == protocol.CMD_STACK:
        if arg and not protocol.is_color_code(arg):
            log("[F] 颜色码非法: %r（只接受 1/2/3/4）" % arg)
            reply(pack_none("F"))
        else:
            reply(tasks.task_stack(cam, clf, disp, p=pline, ctl=ctl,
                                   color_code=(arg or None)))

    elif cmd == protocol.CMD_FORWARD:
        reply(tasks.task_forward(cam, disp, p=pline, ctl=ctl))

    elif cmd == protocol.CMD_RESET:
        log("[R] 复位/握手")
        TASK = None
        reply(pack_ready())

    elif cmd == protocol.CMD_END:
        log("[Z] 结束")
        reply(pack_ok())

    else:
        # 未知命令静默忽略（只记日志，不回帧，避免干扰下位机解析）
        log("[!] 未知命令 %r 参数 %r，已忽略" % (cmd, arg))


# ======================================================================
# 触屏调试循环（临时，默认关）
# ======================================================================
def ui_loop():
    while not app.need_exit():
        code = ui_wait_click()
        if code is None:
            continue
        while code is not None and not app.need_exit():
            code = ui_run_job(code)


def ui_wait_click():
    while not app.need_exit():
        img = cam.read()
        ui.draw(img, ui_rects)
        disp.show(img)
        _x, _y, clicked = ui.poll(disp.width(), disp.height(), ui_rects)
        if clicked == "__exit__":
            app.set_exit_flag(True)
            return None
        if clicked is not None and clicked in touch_ui.JOBS:
            return clicked
        sleep_ms(5)
    return None


def ui_run_job(code):
    name, payload = touch_ui.JOBS[code]
    log("[触屏] 任务 %s" % name)
    while not app.need_exit():
        img = cam.read()
        clf.detect(img, pline.roi(config.ROI_MATERIAL))
        ui.status = name
        ui.draw(img, ui_rects, active=code)
        disp.show(img)
        _x, _y, clicked = ui.poll(disp.width(), disp.height(), ui_rects)
        if clicked is not None:
            if clicked == code or clicked == "__exit__":
                ui.status = "TOUCH MODE - pick a task"
                return None
            if clicked in touch_ui.JOBS:
                return clicked
        sleep_ms(5)
    return None


def main():
    preflight()
    log("=" * 56)
    log("MaixCAM 视觉端（新流程·上位机服务）")
    log("串口: %s @ %d | 分辨率: %dx%d @%dfps"
        % (config.UART_DEV, config.UART_BAUDRATE, config.DET_W, config.DET_H, config.FPS))
    log("精度: %s" % pline.describe())
    log("命令: A=启停区 T=任务码 C=原料区 D/E=圆盘 F=码垛 G=障碍物 R=复位 Z=结束")
    log("握手: 下位机发 R → READY；任务中收到 R 立即中断，收到其它命令回 BUSY")
    log("=" * 56)

    link.send_payload(pack_ready())
    log("已发送 READY")

    if ui is not None:
        log("[触屏] 触屏调试模式（不读串口命令）")
        ui_loop()
    else:
        while not app.need_exit():
            raw = link.poll_command()
            if raw is None:
                sleep_ms(5)
                continue
            cmd, arg = parse_command(raw)
            log("[命令] %r" % raw)
            dispatch(cmd, arg)

    link.close()
    log("退出")


if __name__ == "__main__":
    main()