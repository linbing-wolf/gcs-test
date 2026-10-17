# -*- coding: utf-8 -*-
"""
timing.py —— 时间函数兼容层（MaixCAM2 必须用这个，不要直接用 time.ticks_ms）

为什么需要它：
  MaixCAM2 跑的是完整 Linux + CPython 3，它上面的 `time` 是**标准库 time**，
  只有 time()/monotonic()/sleep()，**没有 ticks_ms / sleep_ms**（那是 MicroPython API）。
  一代 MaixCAM 的某些固件里 `time` 恰好是 maix.time，所以网上抄来的代码在二代会直接报：
      AttributeError: module 'time' has no attribute 'ticks_ms'

  正确做法：时间相关的调用一律走本模块：
      from timing import ticks_ms, sleep_ms
      t0 = ticks_ms()
      if ticks_ms() - t0 > 3000: ...
      sleep_ms(5)

本模块自动适配三种环境（MaixCAM2 / 一代 MaixCAM / PC 上跑测试），无需改调用方。
"""

import time as _std_time

# ---- 取毫秒时基 ----
ticks_ms = None
try:
    # MaixPy 的 maix.time，官方推荐用它算时间间隔（不受 NTP 校时影响）
    from maix import time as _maix_time
    if hasattr(_maix_time, 'ticks_ms'):
        ticks_ms = _maix_time.ticks_ms
except Exception:
    pass

if ticks_ms is None:
    if hasattr(_std_time, 'ticks_ms'):          # MicroPython 风格
        ticks_ms = _std_time.ticks_ms
    elif hasattr(_std_time, 'monotonic'):       # CPython：单调时钟，最适合算间隔
        ticks_ms = lambda: int(_std_time.monotonic() * 1000)
    else:
        ticks_ms = lambda: int(_std_time.time() * 1000)

# ---- 毫秒延时 ----
if hasattr(_std_time, 'sleep_ms'):
    sleep_ms = _std_time.sleep_ms
else:
    def sleep_ms(ms):
        _std_time.sleep(ms / 1000.0)
