# -*- coding: utf-8 -*-
"""tool_diag.py —— 设备自检（工具集内嵌）：MaixPy 版本 / 原生算子 / 串口 / 引脚（工具集内嵌）"""
from maix import app, image

from ui import hit, topbar


def collect():
    L = []
    try:
        import maix
        L.append("maix %s" % getattr(maix, "__version__", "?"))
    except Exception as e:
        L.append("import maix: %s" % e)
    try:
        from maix import image as im
        for n in ("find_blobs", "find_circles", "find_qrcodes", "find_lines", "find_apriltags"):
            L.append("%s:%s" % (n, "Y" if hasattr(im.Image, n) else "N"))
    except Exception as e:
        L.append("image: %s" % e)
    try:
        from maix import uart
        L.append("uart: " + ",".join(uart.list_devices()))
    except Exception as e:
        L.append("uart: %s" % e)
    try:
        from maix import pinmap
        for p in ("A21", "A22", "B0", "B1"):
            try:
                L.append("%s: %s" % (p, pinmap.get_pin_functions(p)))
            except Exception as e:
                L.append("%s: %s" % (p, e))
    except Exception as e:
        L.append("pinmap: %s" % e)
    try:
        from maix import uart
        for d in ("/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4"):
            try:
                u = uart.UART(d, 115200)
                u.close()
                L.append("%s OK" % d)
            except Exception:
                L.append("%s FAIL" % d)
    except Exception:
        pass
    return L


def run(cam, disp, touch, w, h):
    lines = collect()
    for s in lines:
        print(s)
    back_r = [w - 120, 44, 108, 36]
    while not app.need_exit():
        img = cam.read()
        topbar(img, w, "Diagnostics")
        img.draw_rect(back_r[0], back_r[1], back_r[2], back_r[3], image.COLOR_RED, 2)
        img.draw_string(back_r[0] + 20, back_r[1] + 8, "BACK", image.COLOR_RED, 2)
        y = 52
        for s in lines[:20]:
            img.draw_string(8, y, str(s)[:62], image.COLOR_WHITE, 1)
            y += 16
        ix, iy, clicked = touch.poll(disp)
        if clicked and hit(ix, iy, back_r):
            return
        disp.show(img)