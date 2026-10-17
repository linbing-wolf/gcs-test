# -*- coding: utf-8 -*-
"""
protocol.py —— 与原树莓派视觉侧 100% 一致的串口协议（MaixPy 实现）

为什么要有这个文件？
  原系统的 STM32 端 (yyb_stm32/SYSTEM/Serial.c) 用的是状态机解析：
      0xFF + 9 字节数据 + 0xFE   →  Serial_RxPacket[0..8]
  只要 MaixCAM 发出的帧和树莓派一模一样，STM32 端**一行都不用改**。
  这就是整个移植里最省事、最不容易出错的一环。

原程序 send_data() 的行为（已逐行核对 cv2_python/zongchengxu20250724chusai.py:52-61）：
  array = [ord(c) for c in data]
  while len(array) < 9: array.append(0)      # 不足 9 字节补 0
  packet = bytes([0xFF] + array[:9] + [0xFE]) # 帧头 + 9 数据 + 帧尾
注意：补的是 0x00，不是 ASCII '0'。STM32 端若拿这些位当数字用（-0x30）会得到负数，
      但原程序一直这么跑，说明用到的位都在前 7 位以内，属于既有行为，保持一致即可。

发送函数保持同步阻塞（和 pyserial 的 ser.write 语义一致），收到 STM32 命令字后再发结果，
这样不会出现主从双方同时发、互相覆盖的时序问题。
"""

from maix import uart, pinmap, err

# ⚠️ MaixCAM2 上是 CPython，标准 time 没有 ticks_ms/sleep_ms，统一走兼容层
from timing import ticks_ms, sleep_ms

# ============================== 硬件配置（MaixCAM2）==============================
# 设备型号：MaixCAM2（AX630C），不是一代 MaixCAM —— 两者引脚编号完全不同！
#
# 官方 MaixCAM2 Pin v1.0 引脚图上的串口：
#   UART0: A28=TX / A29=RX  → /dev/ttyS0   系统日志口，禁用
#   UART1: A30=TX / A31=RX  → /dev/ttyS1
#   UART2: B0=TX  / B1=RX   → /dev/ttyS2   左侧 2x6 座      ← 当前采用
#   UART3: B2=TX  / B3=RX   → /dev/ttyS3   （同脚复用 SPI2_MOSI/MISO）
#   UART4: A21=TX / A22=RX  → /dev/ttyS4   底部 6P 座
#          ⚠️ UART4 被系统『Maix 通信协议』占用，见 config.py 里的说明
#
# ⭐ 串口参数统一从 config.py 读（那里是唯一来源），本文件不再重复定义。
#    要切换 UART2 / UART4，只改 config.py 那 5 行即可。
# 接线（当前 UART2）：
#   MaixCAM2 B0 (UART2_TX) → STM32 PD2 (UART5_RX)
#   MaixCAM2 B1 (UART2_RX) ← STM32 PC12(UART5_TX)
#   GND 对 GND；两边都是 3.3V TTL，直连，不要电平转换板
try:
    import config as _cfg
    UART_DEV = _cfg.UART_DEV
    UART_BAUDRATE = _cfg.UART_BAUDRATE
    PIN_TX = _cfg.PIN_TX
    PIN_RX = _cfg.PIN_RX
    UART_FUNC_TX = _cfg.UART_FUNC_TX
    UART_FUNC_RX = _cfg.UART_FUNC_RX
except Exception as _e:
    print("[!] 读 config.py 失败(%s)，使用内置默认值 UART2(B0/B1)" % _e)
    UART_DEV = "/dev/ttyS2"
    UART_BAUDRATE = 115200
    PIN_TX = "B0"
    PIN_RX = "B1"
    UART_FUNC_TX = "UART2_TX"
    UART_FUNC_RX = "UART2_RX"
# =================================================================================


def preflight():
    """启动自检：列出可用串口 + 检查引脚功能名，出问题早发现、报得清楚"""
    print("-" * 60)
    print("[自检] 目标串口: %s @ %d" % (UART_DEV, UART_BAUDRATE))
    print("[自检] 引脚: %s=%s, %s=%s" % (PIN_TX, UART_FUNC_TX, PIN_RX, UART_FUNC_RX))
    # 1) 系统里有哪些串口
    try:
        from maix import uart
        devs = uart.list_devices()
        print("[自检] 可用串口设备: %s" % (devs,))
        if UART_DEV not in devs:
            print("[!] 警告: %s 不在可用列表里！" % UART_DEV)
    except Exception as e:
        print("[自检] 列串口失败: %s" % e)
    # 2) 引脚功能名对不对
    try:
        from maix import pinmap
        for pin, func in ((PIN_TX, UART_FUNC_TX), (PIN_RX, UART_FUNC_RX)):
            try:
                funcs = pinmap.get_pin_functions(pin)
                mark = "OK" if func in funcs else "注意: 该功能名不在列表里"
                print("[自检] %s 支持: %s   (%s)" % (pin, funcs, mark))
            except Exception as e:
                print("[自检] 查询 %s 失败: %s" % (pin, e))
    except Exception as e:
        print("[自检] import pinmap 失败: %s" % e)
    print("-" * 60)

FRAME_HEAD = 0xFF
FRAME_TAIL = 0xFE
DATA_LEN = 9


class VisionLink:
    """与 STM32 的串口链路：发送 10 字节帧 + 接收单字节命令字"""

    def __init__(self, dev=UART_DEV, baud=UART_BAUDRATE, use_pinmap=True):
        if use_pinmap:
            # 尝试声明引脚复用功能。
            # 注意：某些固件版本里 set_pin_function 即使引脚本来就正确，也会返回非 0，
            #       所以这里**不把它当致命错误** —— 失败只警告，接着照样尝试打开串口。
            #       真正的判据是后面 uart.UART() 能不能打开。
            for pin, func in ((PIN_TX, UART_FUNC_TX), (PIN_RX, UART_FUNC_RX)):
                try:
                    ret = pinmap.set_pin_function(pin, func)
                    if isinstance(ret, bool):
                        ok, code, msg = ret, (0 if ret else -1), ""
                    else:
                        try:
                            code = ret.code()
                        except Exception:
                            code = -1
                        try:
                            msg = str(ret.msg())
                        except Exception:
                            msg = str(ret)
                        ok = (code == 0)
                    if ok:
                        print("[自检] pinmap %s -> %s 成功" % (pin, func))
                    else:
                        print("[自检] pinmap %s -> %s 返回非 0 (code=%s, msg=%r)，继续尝试打开串口"
                              % (pin, func, code, msg))
                except Exception as e:
                    print("[自检] pinmap %s -> %s 抛异常: %s（继续尝试打开串口）" % (pin, func, e))
        # 真正的判据：能不能打开这个串口设备
        try:
            self.ser = uart.UART(dev, baud)
        except Exception as e:
            print("[!] 打开串口失败: %s @ %d -> %s" % (dev, baud, e))
            try:
                from maix import uart as _u
                print("[!] 系统里的串口设备: %s" % (_u.list_devices(),))
            except Exception:
                pass
            import os as _os
            for _d in ("/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4"):
                try:
                    _t = uart.UART(_d, baud)
                    _t.close()
                    print("[!]   %-12s 可以打开" % _d)
                except Exception as _e2:
                    print("[!]   %-12s 打不开: %s" % (_d, _e2))
            print("[!] 处理建议：")
            print("    1) 若 /dev/ttyS2 能打开 → 把 config.py 的 UART_DEV 改成它")
            print("    2) 若只有 /dev/ttyS4 能打开 → 说明系统占的是别的口，改配置即可")
            print("    3) 若全都打不开 → 该 UART 被系统服务占用，需关掉（见 disable_comm.py）")
            raise
        self.dev = dev
        self.baud = baud
        self.rx_buf = b""          # 命令字节的累积缓冲
        self.last_send_ms = 0

    # ------------------------------------------------------------------
    # 发送：完全复刻原 send_data()
    # ------------------------------------------------------------------
    def send_data(self, data):
        """发一帧 10 字节：0xFF + 9 字节 ASCII + 0xFE

        data: 字符串，长度 ≤ 9（超长会被截断，和原程序 array[:9] 一致）
        """
        arr = [ord(c) for c in str(data)]
        while len(arr) < DATA_LEN:
            arr.append(0)
        packet = bytes([FRAME_HEAD] + arr[:DATA_LEN] + [FRAME_TAIL])
        self.ser.write(packet)
        self.last_send_ms = ticks_ms()
        print("[发送] %s  <- %r" % (packet.hex(" "), str(data)))
        return packet

    # ------------------------------------------------------------------
    # 接收：STM32 每发一个命令字（0x31/0x32/...）就返回一个 ASCII 字符
    # ------------------------------------------------------------------
    def poll_command(self):
        """非阻塞取一个命令字符；没有则返回 None

        STM32 发的是 0x31~0x39 / 0x61~0x63 / 0x72 这些单字节命令，
        对应 ASCII 的 '1'~'9' / 'a'~'c' / 'r'，所以直接按字符处理。
        'end' 这个命令原程序用字符串匹配，这里用 '!' 代替（见迁移报告 §5.5 待确认项）。
        """
        data = self.ser.read()
        if not data:
            return None
        self.rx_buf += data
        # 只保留最近的 16 字节，防止异常数据堆积
        if len(self.rx_buf) > 16:
            self.rx_buf = self.rx_buf[-16:]
        # 取一个有效命令字符（可打印 ASCII）
        while self.rx_buf:
            ch = self.rx_buf[0:1]
            self.rx_buf = self.rx_buf[1:]
            c = ch.decode("utf-8", "ignore")
            if c and 32 <= ord(c) < 127:
                return c
        return None

    def wait_command(self, timeout_ms=30000):
        """阻塞等一个命令字，超时返回 None（原程序是死等，这里加超时更安全）"""
        t0 = ticks_ms()
        while not app_need_exit():
            c = self.poll_command()
            if c is not None:
                return c
            if ticks_ms() - t0 > timeout_ms:
                print("[!] 等命令超时 %dms" % timeout_ms)
                return None
            sleep_ms(2)
        return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def app_need_exit():
    """把 app.need_exit() 包一层，方便单独测试 protocol.py"""
    try:
        from maix import app
        return app.need_exit()
    except Exception:
        return False


# ----------------------------------------------------------------------
# 数据帧构造器：把各识别任务的结果打包成原来的 10 位字符串
# ----------------------------------------------------------------------
def pack_wukuai(x, y, color_code, radius):
    """物块帧：4 位 x + 4 位 y + 1 位颜色 + 1 位半径，然后截 9 字节

    原程序（run_wukuaiyuanxin_1，第 326 行）：
        formatted_data = f"{center[0]:04}{center[1]:04}{final_color}{last_radius:01}"
    注意这是 10 个字符，而 send_data 只发前 9 个 → 半径的最后一位其实被丢掉了！
    STM32 端 transfer_position_wukuai 只解析 [0..7]（x/y），第 9 位 [8] 是颜色码，
    所以 radius 本来就只用来『占位』，这里保持一致。
    """
    return "%04d%04d%s%01d" % (int(x), int(y), str(color_code), int(radius))


def pack_color_only(color_code):
    """只回颜色码（STM32 用 [8] 判断看到的是什么颜色 / 0x34 表示没看到）"""
    return "00000000%s" % str(color_code)


def pack_not_found():
    """没识别到目标：和原程序一致，回 000000004（[8]='4' → STM32 判定 0x34=无目标）"""
    return "000000004"


def pack_task_code(t1, t2, t3, t4, t5, t6):
    """任务码帧：3 位 + 分隔符 + 3 位，共 7 位（STM32 取 [0][1][2] 和 [4][5][6]）

    与 main_logic_loop1 的 data=f"{INDEX}{INDEX1}{INDEX2}456789" 一致：
    第 4 位（索引 3）是分隔符，STM32 端明确跳过了 [3]。
    """
    return "%s%s%s0%s%s%s" % (t1, t2, t3, t4, t5, t6)


def pack_angle(final_angle):
    """边界线角度帧：6 个 0 + 1 位符号 + 1 位整数 + 1 位小数

    对应 detect_boundary_line() 第 963-967 行：
        sign = 1 if final_angle >= 0 else 0
        data_str = f"000000{sign}{integer_part}{decimal_part}"
    即 [6]=符号, [7]=整数部分, [8]=小数部分（STM32 端按 [8] 的值分支）。
    """
    sign = 1 if final_angle >= 0 else 0
    a = abs(final_angle)
    integer_part = int(a)
    decimal_part = int(round(a * 10)) % 10
    return "000000%d%d%d" % (sign, integer_part, decimal_part)


def pack_two_digits(v1, v2):
    """两位数字帧（用于圆台抓取顺序：ypcode[0..2]）"""
    return "%d%d%s" % (int(v1), int(v2), "0000000")
