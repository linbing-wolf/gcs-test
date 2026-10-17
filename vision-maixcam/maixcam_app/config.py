# -*- coding: utf-8 -*-
"""
config.py —— 所有可调参数集中在这里（原程序这些参数散落在 1892 行代码各处）

最重要的一组：LAB 颜色阈值。原程序的 HSV 阈值在这里必须换成 LAB，
用板子自带的『找色块』APP 点两下就能拿到（方法见同目录 README.md）。

原 HSV 阈值（对照用，标定时心里有个数）：
    红: H 0-10  & 160-180, S 50-255, V 50-255   （红色跨 0 度，需要两组）
    绿: H 30-89,  S 40-255, V 40-255
    蓝: H 90-140, S 50-255, V 50-255
"""

# ============================== 分辨率 ==============================
# 原程序用 640x480（USB 摄像头）。MaixCAM 先保持同样口径，跑通后再考虑降分辨率提帧率。
# ⚠️ 改分辨率 = 所有 ROI 和 STM32 端的标定系数都要按比例重算！
W, H = 640, 480
FPS = 60

# ============================== LAB 阈值 ==============================
# 格式 [L_min, L_max, A_min, A_max, B_min, B_max]
# 下面三组是官方起手值，**必须用你自己的场地/灯光重新标定**！
TH_RED = [0, 80, 40, 80, 10, 80]        # 红
TH_GREEN = [0, 80, -120, -10, 0, 30]    # 绿
TH_BLUE = [0, 80, 30, 100, -120, -60]   # 蓝

# ============================== 检测区域 ROI ==============================
# 直接沿用原程序像素坐标（同为 640x480 口径），格式 [x, y, w, h]
ROI_DINGXIN = [160, 140, 320, 210]      # 定心区域
ROI_A3 = [220, 120, 200, 140]           # 正下方 A3
ROI_A1 = [150, 300, 160, 160]           # 左下角 A1
ROI_A2 = [330, 300, 160, 160]           # 右下角 A2
ROI_ZHUAZI = [210, 300, 220, 160]       # 爪子识别区域（原 i=0）
ROI_ERWEIMA = []                        # 二维码：全画面（原程序也是全图）
ROI_SEHUAN = [220, 80, 200, 200]        # 色环检测
ROI_MADUO = [160, 80, 320, 240]         # 码垛检测

# ============================== 稳定性判定 ==============================
# 原程序『连续 N 帧中心不动 + 颜色投票置信度』机制的参数
STABLE_FRAMES_1 = 30        # run_wukuaiyuanxin_1 的 threshold
STABLE_FRAMES_2 = 15        # run_wukuaiyuanxin_2
STABLE_FRAMES_SELECT = 20   # run_wukuaiyuanxin_xuanzequyu
MAX_PIXEL_MOVE = 10         # 相邻帧中心位移超过它 → 稳定计数清零
COLOR_STABLE_FRAMES = 15    # 颜色投票窗口长度
COLOR_CONFIDENCE = 0.7      # 颜色置信度阈值

# ============================== 物块 / 圆检测 ==============================
MIN_PIXELS = 300            # 色块最小像素数（抗噪点）
CIRCLE_R_MIN = 35           # 原 HoughCircles minRadius
CIRCLE_R_MAX = 45           # 原 HoughCircles maxRadius
CIRCLE_THRESHOLD = 2000     # find_circles 累加器阈值

# ============================== 边界线角度 ==============================
LINE_THRESHOLD = 1500       # find_lines 阈值
LINE_MIN_LEN = 100          # 最小线段长度（原 minLineLength=140 @1920x1080）
LINE_SAMPLES = 3            # 需要连续稳定的次数（原程序 3）
LINE_VARIANCE_MAX = 1.0     # 方差上限（原程序 1.0）
LINE_CALIB_OFFSET = -2.0    # 角度校准偏置（原程序 -2.0）

# ============================== 串口（MaixCAM2）==============================
# 这里是串口配置的唯一来源：protocol.py 会优先读本文件的这几项。
#
# 两个可选方案（官方 MaixCAM2 Pin v1.0 引脚图）：
#
#   方案 A（当前采用）UART2 —— 左侧 2x6 2.54mm 座
#       B0 = UART2_TX   → STM32 PD2 (UART5_RX)
#       B1 = UART2_RX   ← STM32 PC12(UART5_TX)
#       设备路径 /dev/ttyS2
#       优点：系统没占用，零配置，MaixVision 照常用
#
#   方案 B UART4 —— 底部 6P 1.25mm 座
#       A21 = UART4_TX  → STM32 PD2 (UART5_RX)
#       A22 = UART4_RX  ← STM32 PC12(UART5_TX)
#       设备路径 /dev/ttyS4
#       代价：UART4 被系统『Maix 通信协议』默认占用（它负责和 MaixVision 通信），
#             必须先跑 disable_comm.py --apply 关掉它并重启板子，
#             之后 MaixVision 的 UART 连接就没了，只能走网络或 SSH。
#       要切到方案 B：把下面 6 行换成注释里那组值即可（protocol.py 会自动跟随）
UART_DEV = "/dev/ttyS2"
PIN_TX = "B0"
PIN_RX = "B1"
UART_FUNC_TX = "UART2_TX"
UART_FUNC_RX = "UART2_RX"
# 方案 B 的对应值（切换时替换上面 5 行）：
# UART_DEV = "/dev/ttyS4"
# PIN_TX = "A21"
# PIN_RX = "A22"
# UART_FUNC_TX = "UART4_TX"
# UART_FUNC_RX = "UART4_RX"

UART_BAUDRATE = 115200      # ⚠️ STM32 端 UART5 要同步设成 115200
# ============================== 触屏调试入口 ==============================
# 临时加的功能：在屏幕上点按钮直接触发任务，不用等 STM32 发命令。
# 【调试完记得把 TOUCH_UI 改成 False】，恢复正常（只认串口命令）的行为。
TOUCH_UI = True             # True = 屏幕上显示任务按钮面板，可触摸触发
TOUCH_UI_SEND_UART = True   # 触屏触发的任务，结果是否也发给 STM32
                            #   False = 只显示不发（纯调阈值时用这个，避免 STM32 收到野数据）

# ============================== 调试开关 ==============================
SHOW_PREVIEW = True         # 屏幕显示 + MaixVision 预览
VERBOSE = True              # 打印每帧识别结果
