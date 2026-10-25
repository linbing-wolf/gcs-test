# -*- coding: utf-8 -*-
"""cam_light.py —— 板载照明 LED（补光灯）控制，供任务按需开关。

MaixCAM2：照明 LED = **B25 / GPIOB25**，高电平点亮（官方 GPIO 文档）。
⚠️ 不要用 PWM6 调亮度：PWM6 与 A30(UART1) 二选一，会和 Maix 通信协议抢引脚。

设计：GPIO 对象常驻模块级（被回收灯可能会灭）；初始化失败只记一次，不反复重试。
现状：只在任务 H（物料盘对齐）期间点亮，前后自动开关（见 tasks.task_plate）。
"""
import config

_STATE = {"obj": None, "ready": False, "on": False, "err": ""}


def _pin():
    return str(getattr(config, "CAM_LIGHT_PIN", "B25"))


def _func():
    return str(getattr(config, "CAM_LIGHT_FUNC", "GPIOB25"))


def init():
    """初始化引脚并保持 GPIO 常驻；成功返回 True。"""
    if _STATE["ready"]:
        return _STATE["obj"] is not None
    _STATE["ready"] = True
    try:
        from maix import gpio, pinmap
        try:
            pinmap.set_pin_function(_pin(), _func())
        except Exception:
            pass
        _STATE["obj"] = gpio.GPIO(_func(), gpio.Mode.OUT)
        _STATE["obj"].value(0)
        print("[灯] 就绪（%s→%s）" % (_pin(), _func()))
        return True
    except Exception as e:
        _STATE["err"] = str(e)
        print("[灯] 不可用: %s" % e)
        return False


def set_on(on):
    """开/关补光灯；返回实际是否已点亮。"""
    if not init():
        return False
    want = bool(on)
    try:
        _STATE["obj"].value(1 if want else 0)
        _STATE["on"] = want
        return want
    except Exception as e:
        _STATE["err"] = str(e)
        return False


def on():
    return set_on(True)


def off():
    set_on(False)
    return not _STATE["on"]


def is_on():
    return bool(_STATE["on"])
