# -*- coding: utf-8 -*-
"""
probe_comm.py —— 只读探测：搞清 Maix 通信协议占了什么、怎么让出来

**本脚本不做任何修改**，只打印信息。确认清楚再决定方案。
"""
import os

print("=" * 60)

# ---------- 1) 通信协议当前配置 ----------
print("[1] Maix 通信协议当前配置")
try:
    from maix import app
    for item, keys in (("comm", ("method", "uart_port", "port", "baudrate", "enable")),):
        for k in keys:
            try:
                v = app.get_sys_config_kv(item, k, value="<未设置>", from_cache=False)
                print("    %s.%s = %r" % (item, k, v))
            except Exception as e:
                print("    %s.%s 读取失败: %s" % (item, k, e))
except Exception as e:
    print("    import maix.app 失败:", e)

# ---------- 2) comm 模块自己怎么说 ----------
print("[2] maix.comm 的视角")
try:
    from maix import comm
    for fn in ("get_method", "get_uart_port", "get_uart_ports"):
        try:
            f = getattr(comm, fn, None)
            if f is None:
                print("    %s: 不存在" % fn)
            else:
                print("    %s() = %r" % (fn, f()))
        except Exception as e:
            print("    %s() 失败: %s" % (fn, e))
except Exception as e:
    print("    import maix.comm 失败:", e)

# ---------- 3) 系统里有哪些串口设备 ----------
print("[3] 串口设备（/dev/ttyS*）")
try:
    devs = sorted(d for d in os.listdir("/dev") if d.startswith("ttyS"))
    print("    存在:", devs)
except Exception as e:
    print("    列出失败:", e)

# ---------- 4) 逐个试着打开，看哪个被占了 ----------
print("[4] 逐个尝试打开（被系统服务占着的会失败）")
try:
    from maix import uart
    for d in ("/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4"):
        try:
            u = uart.UART(d, 115200)
            u.close()
            print("    %-12s 可用" % d)
        except Exception as e:
            print("    %-12s 打不开: %s" % (d, e))
except Exception as e:
    print("    失败:", e)

# ---------- 5) 网络情况（关了 UART 通信后 MaixVision 得走它） ----------
print("[5] 网络（决定关掉 UART 通信协议后还能不能用 MaixVision）")
try:
    from maix import network
    try:
        print("    已连接:", network.get_ip())
    except Exception as e:
        print("    get_ip 失败:", e)
except Exception as e:
    print("    import maix.network 失败:", e)
try:
    from maix import app
    for k in ("ssid", "ip"):
        try:
            print("    wifi.%s = %r" % (k, app.get_sys_config_kv("wifi", k, value="<未设置>")))
        except Exception as e:
            print("    wifi.%s 读取失败: %s" % (k, e))
except Exception:
    pass

# 用 shell 兜底看 IP（MaixPy 的 network API 各版本差异较大）
try:
    print("    ip addr 摘要:")
    for line in os.popen("ip -4 addr show 2>/dev/null").read().splitlines():
        line = line.strip()
        if line.startswith("inet "):
            print("      " + line)
except Exception as e:
    print("    shell 取 IP 失败:", e)

print("=" * 60)
print("把以上 [1]~[5] 全部输出发回来，据此定方案。")
