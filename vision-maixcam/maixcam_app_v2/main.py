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

import sys
import os
import time
# launcher 把 stdout 重定向到 /maixapp/tmp/last_run.log，默认是块缓冲；
# 改成行缓冲后现场日志/调试信息能实时看到（Python 3.7+）。
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

import config
import vision
import ai_model
import precision
import protocol
import tasks
import screen_echo
import serial_log
from protocol import (VisionLink, preflight, parse_command, pack_ready, pack_ok,
                      pack_error, pack_none, pack_busy)

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

# ---- 屏幕命令回显条：把 display 包一层，任务里原有的 disp.show() 自动带上回显 ----
# 触屏模式不读串口命令，且屏幕下方是按钮，不加（避免压住按钮）
screen_echo.set_state("IDLE")
if disp is not None and getattr(config, "SCREEN_ECHO", False) and ui is None:
    disp = screen_echo.wrap(disp)


def echo_bad_frames():
    """把链路层的坏帧数同步到回显条（CRC/帧尾/半帧都算）"""
    try:
        screen_echo.set_bad(getattr(link.parser, "bad", 0))
    except Exception:
        pass


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
    screen_echo.set_tx(payload)


_last_disc_ms = 0


def _disc_gate():
    """D/E 最快每 DISC_MIN_INTERVAL_MS 处理一次；太快的重复 D 回 BUSY。"""
    global _last_disc_ms
    now = ticks_ms()
    gap = now - _last_disc_ms
    min_gap = max(0, int(getattr(config, "DISC_MIN_INTERVAL_MS", 100)))
    if _last_disc_ms and gap < min_gap:
        log("[D/E] 距上次检测 %dms < %dms，回 BUSY（等机构停稳/下位机降频）"
            % (gap, min_gap))
        reply(pack_busy())
        return False
    return True


def _disc_done():
    global _last_disc_ms
    _last_disc_ms = ticks_ms()


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
        if _disc_gate():
            reply(tasks.task_disc(cam, disp, config.ROI_DISC_ROUGH, "D", p=pline, ctl=ctl))
            _disc_done()

    elif cmd == protocol.CMD_DISC_TEMP:
        if _disc_gate():
            reply(tasks.task_disc(cam, disp, config.ROI_DISC_TEMP, "E", p=pline, ctl=ctl))
            _disc_done()

    elif cmd == protocol.CMD_STACK:
        if arg and not protocol.is_color_code(arg):
            log("[F] 颜色码非法: %r（只接受 1/2/3/4）" % arg)
            reply(pack_none("F"))
        else:
            reply(tasks.task_stack(cam, clf, disp, p=pline, ctl=ctl,
                                   color_code=(arg or None)))

    elif cmd == protocol.CMD_FORWARD:
        reply(tasks.task_forward(cam, disp, p=pline, ctl=ctl))

    elif cmd == protocol.CMD_PLATE:
        reply(tasks.task_plate(cam, disp, p=pline, ctl=ctl))

    elif cmd == protocol.CMD_RESET:
        log("[R] 复位/握手")
        TASK = None
        tasks.reset_guard()
        reply(pack_ready())

    elif cmd == protocol.CMD_END:
        log("[Z] 结束")
        tasks.reset_guard()
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


def camera_light(on=None):
    """摄像头补光灯（MaixCAM2 板载照明 LED，B25/GPIOB25，高电平点亮）。

    注意：官方文档说照明 LED 也能用 PWM6 调亮度，但 PWM6 与 A30(UART1) 二选一，
    会和 Maix 通信协议抢引脚，所以这里只用普通 GPIO 开关。
    失败只打印告警，不影响主流程。
    """
    if on is None:
        on = bool(getattr(config, "CAM_LIGHT_ON", False))
    if not on:
        return False
    pin = str(getattr(config, "CAM_LIGHT_PIN", "B25"))
    func = str(getattr(config, "CAM_LIGHT_FUNC", "GPIOB25"))
    try:
        from maix import gpio, pinmap
        try:
            pinmap.set_pin_function(pin, func)
        except Exception:
            pass
        led = gpio.GPIO(func, gpio.Mode.OUT)
        led.value(1)
        log("[灯] 补光灯已打开（%s→%s=1）" % (pin, func))
        return True
    except Exception as e:
        log("[灯] 补光灯打开失败: %s" % e)
        return False


def main():
    preflight()
    log("=" * 56)
    log("MaixCAM 视觉端（新流程·上位机服务）")
    log("版本: %s | 启动时间: %s" % (serial_log.app_version(), time.strftime("%Y-%m-%d %H:%M:%S")))
    log("串口: %s @ %d | 分辨率: %dx%d @%dfps"
        % (config.UART_DEV, config.UART_BAUDRATE, config.DET_W, config.DET_H, config.FPS))
    log("精度: %s" % pline.describe())
    log("命令: A=启停区 T=任务码 C=原料区 D/E=圆盘 F=码垛 G=障碍物 H=物料盘 R=复位 Z=结束")
    log("握手: 下位机发 R → READY；任务中收到 R 立即中断，收到其它命令回 BUSY")
    log("=" * 56)

    camera_light()

    reply(pack_ready())
    log("已发送 READY")

    if ui is not None:
        log("[触屏] 触屏调试模式（不读串口命令）")
        ui_loop()
    else:
        # 空闲时定期刷一帧预览 + 回显条：屏幕不再"冻住"，并显示 rx/tx/坏帧计数
        # SCREEN_ECHO=False 时完全退回旧行为（空闲不刷屏）
        idle_ms = max(0, int(getattr(config, "SCREEN_ECHO_IDLE_MS", 100)))
        idle_poll = max(1, int(getattr(config, "IDLE_POLL_MS", 2)))
        show_idle = (disp is not None and idle_ms > 0
                     and getattr(config, "SCREEN_ECHO", False))
        last_echo = 0
        while not app.need_exit():
            raw = link.poll_command()
            if raw is None:
                # 调试触发：SSH 上 touch /tmp/run_stack_move → 直接跑一次真实 F
                # （会发 MOVE/MOVEY + STACK，车会动，确保周围留空间）
                stack_trig = getattr(config, "STACK_DEBUG_TRIGGER_REAL_FILE", "")
                if stack_trig and os.path.exists(stack_trig):
                    try:
                        os.remove(stack_trig)
                    except Exception:
                        pass
                    log("[调试] 直接触发一次 F（真实 MOVE/MOVEY + STACK）")
                    tasks.reset_guard()
                    color = getattr(config, "STACK_DEBUG_TRIGGER_COLOR", "") or None
                    reply(tasks.task_stack(cam, clf, disp, p=pline, ctl=ctl,
                                           color_code=color))
                    screen_echo.set_state("IDLE")
                    continue
                # 调试触发：SSH 上 touch /tmp/run_disc_move → 直接跑一次真实 D
                # （会发 MOVE + DISC，车会动，确保周围留空间）
                real_trig = getattr(config, "DISC_DEBUG_TRIGGER_REAL_FILE", "")
                if real_trig and os.path.exists(real_trig):
                    try:
                        os.remove(real_trig)
                    except Exception:
                        pass
                    log("[调试] 直接触发一次 D（真实 MOVE + DISC）")
                    tasks.reset_guard()
                    reply(tasks.task_disc(cam, disp, config.ROI_DISC_ROUGH, "D",
                                          p=pline, ctl=ctl))
                    screen_echo.set_state("IDLE")
                    _disc_done()
                    continue
                # 调试触发：SSH 上 touch /tmp/run_disc_once → 抓一段 D/E 画面
                # 只记录，不发 MOVE，不影响正常串口协议。
                trig = getattr(config, "DISC_DEBUG_TRIGGER_FILE", "")
                if trig and os.path.exists(trig):
                    try:
                        os.remove(trig)
                    except Exception:
                        pass
                    tcmd = getattr(config, "DISC_DEBUG_TRIGGER_CMD", "D")
                    log("[调试] 触发一次 %s 抓图（只记录，不发 MOVE）" % tcmd)
                    if tcmd == protocol.CMD_DISC_TEMP:
                        tasks.task_snapshot(cam, disp, config.ROI_DISC_TEMP, "E",
                                            p=pline)
                    else:
                        tasks.task_snapshot(cam, disp, config.ROI_DISC_ROUGH, "D",
                                            p=pline)
                    screen_echo.set_state("IDLE")
                    continue
                if show_idle:
                    now = ticks_ms()
                    if (now - last_echo) >= idle_ms:
                        last_echo = now
                        # 刷预览前再看一眼命令：有命令就先处理，不让屏幕刷新挡住响应
                        raw = link.poll_command()
                        if raw is None:
                            echo_bad_frames()
                            screen_echo.set_state("IDLE")
                            try:
                                disp.show(cam.read())
                            except Exception as e:
                                log("[回显] 空闲预览失败，已关闭: %s" % e)
                                show_idle = False
                if raw is None:
                    sleep_ms(idle_poll)
                    continue
            cmd, arg = parse_command(raw)
            screen_echo.set_rx(raw)
            echo_bad_frames()
            screen_echo.set_state("RUNNING")
            log("[命令] %r" % raw)
            dispatch(cmd, arg)
            screen_echo.set_state("IDLE")

    link.close()
    log("退出")


if __name__ == "__main__":
    main()