# -*- coding: utf-8 -*-
"""light.py —— 板载照明 LED（补光灯）开关，工具集共用。

MaixCAM2：照明 LED 接在 **B25 / GPIOB25**，高电平点亮（官方 GPIO 文档）。
⚠️ 不要用 PWM6 调亮度：PWM6 与 A30(UART1) 二选一，会和 Maix 通信协议抢引脚。

用法：
    import light
    light.init()            # 进 APP 时调一次（保持 GPIO 对象常驻）
    light.toggle()          # 开关切换
    light.set_on(True/False)
    light.is_on() / light.label()
"""
PIN = "B25"
FUNC = "GPIOB25"

STATE = {"on": False, "obj": None, "ready": False, "err": ""}


def _apply(v):
    led = STATE["obj"]
    if led is None:
        return
    try:
        led.value(int(v))
    except Exception as e:
        STATE["err"] = str(e)


def init(prefer_on=False):
    """初始化引脚；GPIO 对象保存在模块里（被回收的话灯可能会灭）。"""
    if STATE["ready"]:
        return STATE["obj"] is not None
    STATE["ready"] = True
    try:
        from maix import gpio, pinmap
        try:
            pinmap.set_pin_function(PIN, FUNC)
        except Exception:
            pass
        STATE["obj"] = gpio.GPIO(FUNC, gpio.Mode.OUT)
        STATE["on"] = bool(prefer_on)
        _apply(1 if prefer_on else 0)
        print("[灯] 初始化成功（%s→%s），当前 %s"
              % (PIN, FUNC, "ON" if prefer_on else "OFF"))
        return True
    except Exception as e:
        STATE["err"] = str(e)
        print("[灯] 初始化失败:", e)
        return False


def set_on(on):
    init()
    STATE["on"] = bool(on)
    _apply(1 if on else 0)
    print("[灯] %s" % ("ON" if on else "OFF"))
    return STATE["on"]


def toggle():
    return set_on(not STATE["on"])


def is_on():
    return bool(STATE["on"])


def label():
    if STATE["obj"] is None:
        return "LIGHT N/A"
    return "LIGHT ON" if STATE["on"] else "LIGHT OFF"
