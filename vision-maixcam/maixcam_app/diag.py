# -*- coding: utf-8 -*-
"""diag.py —— 在 MaixCAM2 上运行：检查 MaixPy 原生能力 + 项目文件是否到位"""
import os, sys

print("=" * 56)
print("[1] MaixPy 版本")
try:
    import maix
    print("    maix __version__ =", getattr(maix, "__version__", "unknown"))
except Exception as e:
    print("    import maix 失败:", e)

print("[2] image 模块的原生算子（MaixPy 自带）")
try:
    from maix import image
    for name in ("find_blobs", "find_lines", "find_circles", "find_rects", "find_qrcodes"):
        print("    img.%-14s %s" % (name, "有" if hasattr(image.Image, name) else "缺失"))
    for cls in ("Blob", "Line", "Circle", "QRCode"):
        print("    image.%-10s %s" % (cls, "有" if hasattr(image, cls) else "缺失"))
except Exception as e:
    print("    import image 失败:", e)

print("[3] Blob 上要用到的方法")
try:
    from maix import image
    need = ("cx", "cy", "cxf", "cyf", "rect", "pixels", "w", "h", "code", "roundness")
    miss = [n for n in need if not hasattr(image.Blob, n)]
    print("    缺失:", miss if miss else "无（全部可用）")
except Exception as e:
    print("    失败:", e)

print("[4] uart / pinmap")
try:
    from maix import uart, pinmap
    print("    uart.list_devices() =", uart.list_devices())
    for pin in ("A21", "A22", "B0", "B1"):
        try:
            print("    %-4s 可用功能 = %s" % (pin, pinmap.get_pin_functions(pin)))
        except Exception as e:
            print("    %-4s 查询失败: %s" % (pin, e))
except Exception as e:
    print("    失败:", e)

print("[4a] 关键：A21/A22 能不能抢过来（Maix 通信协议默认占着 UART4）")
try:
    from maix import pinmap
    for pin, func in (("A21", "UART4_TX"), ("A22", "UART4_RX")):
        r = pinmap.set_pin_function(pin, func)
        if isinstance(r, bool):
            ok, msg = r, ""
        else:
            ok = (getattr(r, 'code', lambda: -1)() == 0)
            try:
                msg = str(r.msg())
            except Exception:
                msg = str(r)
        print("    %s -> %-9s %s %s" % (pin, func, "成功" if ok else "失败:", msg))
except Exception as e:
    print("    失败:", e)
    print("    → 如果失败，请在设备上：设置 → 通信协议 → 改成 TCP，重启后再试")

print("[4b] 时间 API（MaixCAM2 的关键差异点）")
import time as _t
print("    标准 time 有 ticks_ms ?", hasattr(_t, "ticks_ms"))
try:
    from maix import time as _mt
    print("    maix.time 有 ticks_ms ?", hasattr(_mt, "ticks_ms"))
except Exception as e:
    print("    import maix.time 失败:", e)
try:
    from timing import ticks_ms, sleep_ms
    a = ticks_ms(); sleep_ms(5); b = ticks_ms()
    print("    timing 兼容层 OK, 5ms 实测 =", b - a, "ms")
except Exception as e:
    print("    timing 兼容层失败:", e)

print("[4c] 逐个串口试打开（这才是真正的判据）")
try:
    from maix import uart
    for _d in ("/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4"):
        try:
            _u = uart.UART(_d, 115200)
            _u.close()
            print("    %-12s 可以打开" % _d)
        except Exception as _e:
            print("    %-12s 打不开: %s" % (_d, _e))
except Exception as e:
    print("    失败:", e)

print("[5] 项目文件是否在设备上")
here = os.path.dirname(os.path.abspath(__file__))
print("    当前目录:", here)
for f in ("config.py", "protocol.py", "vision.py", "timing.py", "main.py"):
    p = os.path.join(here, f)
    print("    %-14s %s" % (f, "在" if os.path.exists(p) else "不在"))

print("[6] 试着导入我的模块")
sys.path.insert(0, here)
for m in ("config", "protocol", "vision", "timing"):
    try:
        __import__(m)
        print("    import %-10s OK" % m)
    except Exception as e:
        print("    import %-10s 失败: %s" % (m, e))
print("=" * 56)
