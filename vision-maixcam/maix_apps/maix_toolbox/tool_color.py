# -*- coding: utf-8 -*-
"""tool_color.py —— 色块取样 (Color Tune)（工具集内嵌）

点画面任意一点 → 显示该点 RGB 与 LAB 值；把 LAB 抄进 config 的 TH_* 即可。
用途：现场给「找色块」标 LAB 阈值，不用来回改代码。

LAB 为 CIELAB（MaixPy find_blobs 阈值口径）：L∈[0,100], A/B∈[-128,127]。
"""
from maix import image, app

from ui import hit, topbar


def _s2l(c):
    c /= 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def rgb_to_lab(r, g, b):
    r, g, b = _s2l(r), _s2l(g), _s2l(b)
    X = r * 0.4124 + g * 0.3576 + b * 0.1805
    Y = r * 0.2126 + g * 0.7152 + b * 0.0722
    Z = r * 0.0193 + g * 0.1192 + b * 0.9505
    Xn, Yn, Zn = 0.95047, 1.0, 1.08883

    def f(t):
        return t ** (1.0 / 3.0) if t > 0.008856 else (7.787 * t + 16.0 / 116.0)

    fx, fy, fz = f(X / Xn), f(Y / Yn), f(Z / Zn)
    return (round(116 * fy - 16), round(500 * (fx - fy)), round(200 * (fy - fz)))


def _pixel_rgb(img, x, y):
    try:
        p = img.get_pixel(x, y)
    except Exception as e:
        print("get_pixel 失败:", e)
        return None
    # 兼容三种返回：[r,g,b] / [[r,g,b]] / Pixel对象
    try:
        if hasattr(p, "r") and hasattr(p, "g") and hasattr(p, "b"):
            return int(p.r), int(p.g), int(p.b)
        v = p
        for _ in range(3):                     # 逐层脱壳
            if isinstance(v, (list, tuple)) and len(v) == 1:
                v = v[0]
            else:
                break
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            return int(v[0]), int(v[1]), int(v[2])
    except Exception as e:
        print("解析像素失败:", e, "raw=", repr(p))
    else:
        print("get_pixel 返回格式未知 raw=", repr(p))
    return None


def run(cam, disp, touch, w, h):
    exit_r = [w - 120, 44, 108, 36]
    sample = None            # (x, y, r, g, b, L, A, B)
    print("Color Tune：点画面上任意一点取 RGB/LAB")
    while not app.need_exit():
        img = cam.read()
        topbar(img, w, "Color Tune (tap to sample)")
        img.draw_rect(exit_r[0], exit_r[1], exit_r[2], exit_r[3], image.COLOR_RED, 2)
        img.draw_string(exit_r[0] + 20, exit_r[1] + 8, "BACK", image.COLOR_RED, 2)

        if sample:
            x, y, r, g, b, L, A, B = sample
            img.draw_cross(x, y, image.COLOR_RED, 10, 2)
            img.draw_string(8, h - 84, "px (%d,%d)" % (x, y), image.COLOR_YELLOW, 2)
            img.draw_string(8, h - 60, "RGB %d,%d,%d" % (r, g, b), image.COLOR_WHITE, 2)
            img.draw_string(8, h - 34, "LAB %d,%d,%d" % (L, A, B), image.COLOR_GREEN, 2)
            img.draw_string(8, h - 8, "-> TH_? = [Lmin,Lmax, Amin,Amax, Bmin,Bmax]",
                            image.COLOR_GRAY, 1)
        else:
            img.draw_string(8, h - 40, "tap a pixel to read RGB / LAB",
                            image.COLOR_GRAY, 2)

        ix, iy, clicked = touch.poll(disp)
        if clicked:
            if hit(ix, iy, exit_r):
                return
            if 0 <= ix < w and 0 <= iy < h:
                rgb = _pixel_rgb(img, ix, iy)
                if rgb:
                    r, g, b = rgb
                    L, A, B = rgb_to_lab(r, g, b)
                    sample = (ix, iy, r, g, b, L, A, B)
                    print("px(%d,%d) RGB=(%d,%d,%d) LAB=(%d,%d,%d)" % (ix, iy, r, g, b, L, A, B))

        disp.show(img)