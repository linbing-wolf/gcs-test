# -*- coding: utf-8 -*-
"""
timing.py —— 时间函数兼容层（MaixCAM2 必须用这个，不要直接用 time.ticks_ms）

MaixCAM2 跑完整 Linux + CPython3，标准库 time 只有 time()/monotonic()/sleep()，
没有 ticks_ms / sleep_ms（那是 MicroPython API）。本模块自动适配：
  - MaixCAM2（CPython）：用 time.monotonic()
  - 一代 MaixCAM（MicroPython）：用 ticks_ms
  - PC 上跑测试：也能用
"""

import time as _std_time

ticks_ms = None
try:
    from maix import time as _maix_time
    if hasattr(_maix_time, 'ticks_ms'):
        ticks_ms = _maix_time.ticks_ms
except Exception:
    pass

if ticks_ms is None:
    if hasattr(_std_time, 'ticks_ms'):          # MicroPython 风格
        ticks_ms = _std_time.ticks_ms
    elif hasattr(_std_time, 'monotonic'):       # CPython：单调时钟
        ticks_ms = lambda: int(_std_time.monotonic() * 1000)
    else:
        ticks_ms = lambda: int(_std_time.time() * 1000)

if hasattr(_std_time, 'sleep_ms'):
    sleep_ms = _std_time.sleep_ms
else:
    def sleep_ms(ms):
        _std_time.sleep(ms / 1000.0)