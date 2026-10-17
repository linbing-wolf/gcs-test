# -*- coding: utf-8 -*-
"""
disable_comm.py —— 让出 UART4：把系统『Maix 通信协议』的 UART 方式关掉

背景：
  MaixCAM2 开机时系统服务会抢占 UART4（A21/A22）用于和 MaixVision 通信：
      [Maix Comm Protocol] auto set pinmap A21 to UART4_TX for uart4
      [Maix Comm Protocol] listening on uart port: uart4(/dev/ttyS4)
  只要它还开着，我们就无法用 A21/A22（引脚抢不到，/dev/ttyS4 也被独占）。

⚠️⚠️ 重要后果：关掉 UART 通信方式后，MaixVision 就不能再用 UART 连接板子跑代码了！
  之后请用下面任一方式继续开发：
    - MaixVision 通过网络(TCP)连接板子（需要板子连上 WiFi）
    - SSH：ssh root@<板子IP>，然后 python3 /root/xxx/main.py
    - 用 maixcam-dev/tools/put.ps1 传代码 + connect.ps1 跑
  想恢复：把本脚本最后的 RESTORE 段取消注释跑一次，或去 设置→通信协议 改回 UART。

用法（在 MaixCAM2 上运行）：
    python3 disable_comm.py            # 只查看当前配置，不改
    python3 disable_comm.py --apply    # 真正关闭 UART 通信方式
"""
import sys

APPLY = "--apply" in sys.argv

from maix import app

# 可能相关的配置项名（不同固件版本命名可能不同，逐个试）
CANDIDATES = [
    ("comm", "method"),
    ("comm", "uart"),
    ("comm", "enable"),
    ("comm", "port"),
    ("comm", "uart_port"),
    ("comm", "baudrate"),
]

print("=" * 60)
print("[1] 当前通信协议配置")
found = {}
for item, key in CANDIDATES:
    try:
        v = app.get_sys_config_kv(item, key, value="<不存在>", from_cache=False)
        found[(item, key)] = v
        print("    %s.%s = %r" % (item, key, v))
    except Exception as e:
        print("    %s.%s 读取异常: %s" % (item, key, e))

print("")
print("[2] maix.comm 现在认为走什么方式")
try:
    from maix import comm
    print("    get_method() =", comm.get_method())
    print("    get_uart_port() =", comm.get_uart_port())
except Exception as e:
    print("    查询失败:", e)

m = found.get(("comm", "method"), None)
print("")
print("[3] 判断")
method_now = found.get(("comm", "method"), None)
method_str = ("" if method_now is None else str(method_now)).strip().lower()
already_off = method_str in ("", "none", "off", "disable", "disabled", "tcp")

if already_off:
    print("    当前 method=%r → UART 方式看起来已经关了，A21/A22 应该可用" % method_now)
    print("    下一步：重启板子后跑 main.py；若仍报 pinmap 失败，说明还有别的服务占着")
else:
    print("    当前 method=%r → UART 方式还开着，需要关闭" % method_now)
    if not APPLY:
        print("    这是查看模式，不做修改。确认要关就加 --apply：")
        print("        python3 disable_comm.py --apply")
    else:
        print("")
        print("[4] 正在关闭 UART 通信方式 ...")
        ok = False
        for value in ("none", "off", "tcp"):
            try:
                app.set_sys_config_kv("comm", "method", value)
                print("    已设置 comm.method = %r" % value)
                ok = True
                break
            except Exception as e:
                print("    设置 %r 失败: %s" % (value, e))
        if ok:
            print("")
            print("    [OK] 已写入。请重启板子让它生效：  reboot")
            print("    重启后 A21/A22 就归我们了，再跑 main.py")
            print("")
            print("    [!] MaixVision 的 UART 连接会失效，改用网络(TCP)或 SSH 连板子")
        else:
            print("    [X] 自动设置失败 —— 这个固件不允许这样改。")
            print("        请手动：设备上 设置 -> 通信协议 -> 改成 TCP 或关闭，然后重启")

# ---------- 想恢复 UART 通信时，取消下面两行注释跑一次 ----------
# app.set_sys_config_kv("comm", "method", "uart")
# print("已恢复 UART 通信，重启板子生效")

print("=" * 60)
