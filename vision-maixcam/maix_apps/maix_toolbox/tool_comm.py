# -*- coding: utf-8 -*-
"""tool_comm.py —— 关闭系统 Maix 通信协议(UART)，让出 UART4/A21A22（工具集内嵌）"""
from maix import app, image

from ui import hit, topbar


def _disable():
    msgs = []
    try:
        now = app.get_sys_config_kv("comm", "method", value="<none>", from_cache=False)
        msgs.append("now method = %r" % now)
    except Exception as e:
        msgs.append("read fail: %s" % e)
    ok = False
    for v in ("none", "off", "tcp"):
        try:
            app.set_sys_config_kv("comm", "method", v)
            msgs.append("set method = %r  OK" % v)
            ok = True
            break
        except Exception as e:
            msgs.append("set %r fail: %s" % (v, e))
    if ok:
        msgs.append("重启板子生效:  reboot")
        msgs.append("(之后 MaixVision UART 连接失效，用 网络/SSH)")
    return ok, msgs


def run(cam, disp, touch, w, h):
    apply_r = [w // 2 + 16, h - 90, 200, 60]
    back_r = [w // 2 - 216, h - 90, 200, 60]
    result = None
    while not app.need_exit():
        img = cam.read()
        topbar(img, w, "Disable Maix Comm  (free UART4)")
        img.draw_string(16, 56, "关闭系统 Maix 通信协议(UART)，让出 UART4 (A21/A22)",
                        image.COLOR_WHITE, 1)
        img.draw_string(16, 76, "注意：之后 MaixVision 的 UART 连接会失效，改用网络(TCP)/SSH",
                        image.COLOR_YELLOW, 1)

        if result is None:
            img.draw_rect(apply_r[0], apply_r[1], apply_r[2], apply_r[3], image.COLOR_GREEN, 2)
            img.draw_string(apply_r[0] + 40, apply_r[1] + 18, "APPLY", image.COLOR_GREEN, 2)
        else:
            ok, msgs = result
            img.draw_string(16, 110, "结果: %s" % ("成功" if ok else "失败"),
                            image.COLOR_GREEN if ok else image.COLOR_RED, 2)
            y = 150
            for m in msgs:
                img.draw_string(16, y, m[:64], image.COLOR_WHITE, 1)
                y += 18
        img.draw_rect(back_r[0], back_r[1], back_r[2], back_r[3], image.COLOR_RED, 2)
        img.draw_string(back_r[0] + 40, back_r[1] + 18, "BACK", image.COLOR_RED, 2)

        ix, iy, clicked = touch.poll(disp)
        if clicked:
            if hit(ix, iy, back_r):
                return
            if result is None and hit(ix, iy, apply_r):
                result = _disable()
                for m in result[1]:
                    print(m)
        disp.show(img)