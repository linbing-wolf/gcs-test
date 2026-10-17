# -*- coding: utf-8 -*-
"""
config.py —— 新流程（智能搬运赛项）视觉端配置（唯一参数来源）

与旧工程的关键差异（务必注意）：
  1) 颜色码换了！官方新定义：红1 黄2 蓝3 绿4（旧工程是 红1 绿2 蓝3）。
  2) 任务码变成 4 组三位数："G1+G2+G3+G4"，共 15 字符（旧的是 2 组 7 字符）。
  3) 新增区域：启停区、原料区(转盘)、粗加工区、暂存区。

LAB 阈值下面几组是**起手值**，必须在比赛灯光下用板子自带的「找色块」APP 重新标定！
"""

# ============================== 分辨率 ==============================
W, H = 640, 480
FPS = 60

# ============================== 颜色码（官方定义）==============================
# 顺序即阈值数组顺序，find_blobs 的 code 位序依赖它，别乱动。
COLOR_NAMES = ["red", "yellow", "blue", "green"]
COLOR_CODES = {"red": "1", "yellow": "2", "blue": "3", "green": "4"}

# LAB 阈值 [L_min, L_max, A_min, A_max, B_min, B_max]
# 由 870 张样本集统计得出（tools_analyze_lab.py：取最饱和 8% 像素 + 剔除离群图 + p2/p98）
# 口径：MaixPy/OpenMV  L 0~100, A/B -128~127
TH_RED    = [15, 60,  35,  75,  18,  60]     # 红：A 明显偏正
TH_YELLOW = [45, 90, -16,   5,  45,  78]     # 黄：A 近 0、B 明显偏正
TH_BLUE   = [15, 60,   6,  38, -75, -35]     # 蓝：B 明显偏负
TH_GREEN  = [30, 78, -70, -32,  26,  64]     # 绿：A 明显偏负

THRESHOLDS = [TH_RED, TH_YELLOW, TH_BLUE, TH_GREEN]   # 顺序 = COLOR_NAMES

# ============================== 检测区域 ROI ==============================
# 格式 [x, y, w, h]（640x480 口径）。跑通后按现场收紧。
ROI_START      = []                      # 启停区蓝色块：全画面找最大蓝块
# 启停区编号判定：画面右侧有没有「灰色道路」→ 有=1号 无=2号
ROI_ROAD       = [384, 120, 256, 300]    # 右侧检测区（640x480 口径）
# ⚠️ 样本集里「灰色路面」和「白桌面」都是低饱和（路面 L≈63，桌面 L≈70），
#    LAB 只能靠亮度差分离，现场必须用真实画面复核这条阈值。
TH_ROAD        = [35, 66, -12, 12, -12, 12]  # LAB 灰色路面（低饱和 + 偏暗）
ROAD_MIN_RATIO = 0.35                    # 灰色像素占 ROI ≥ 35% → 判定「有道路」
ROAD_STABLE_FRAMES = 5                   # 连续 N 帧结论一致才确认
START_ZONE_HAS_ROAD    = 1               # 右侧有道路 → 几号启停区
START_ZONE_NO_ROAD     = 2               # 右侧没道路 → 几号启停区
ROAD_AI_LABEL          = "other"         # AI 复确认：判成这个类才算「有道路」
# AI 判成 non-color（background/other）且置信度够时，是否直接丢弃该候选
#   True  = 严格：不把非物料当物料（可能漏，AI 判错时）
#   False = 宽松：退回阈值判色（不漏，但可能误检）
AI_VETO_OTHER = True
ROI_QRCODE     = []                      # 二维码：全画面
ROI_MATERIAL   = [140, 100, 360, 300]    # 原料区转盘（放 3 个物料）
ROI_DISC_ROUGH = [160, 120, 320, 240]    # 粗加工区圆盘
ROI_DISC_TEMP  = [160, 120, 320, 240]    # 暂存区圆盘
ROI_STACK      = [160, 120, 320, 240]    # 码垛区（暂存区同一块）

# ============================== 稳定性判定 ==============================
STABLE_FRAMES    = 15     # 连续多少帧中心不动才确认
MAX_PIXEL_MOVE   = 10     # 相邻帧中心位移超过它就清零
COLOR_WINDOW     = 15     # 颜色投票窗口长度
COLOR_CONFIDENCE = 0.7    # 颜色置信度阈值

# ============================== 识别阈值 ==============================
MIN_PIXELS       = 300      # 色块最小像素数
STACK_COLOR_VOTE = False    # F 码垛时是否也做颜色投票（False=只判中心稳定，更快更稳）
MIN_PIXELS_START = 5000     # 启停区是大面积蓝，用大阈值避免和蓝色物料混淆
QR_REPEAT        = 3        # 二维码连续 N 次一致才确认
QR_REGEX         = r"^\d{3}\+\d{3}\+\d{3}\+\d{3}$"   # 15 字符任务码

# ============================== 圆盘 / 圆环 ==============================
DISC_R_MIN     = 60         # 圆盘半径范围（find_circles）
DISC_R_MAX     = 200
DISC_THRESHOLD = 2000       # find_circles 累加器阈值
# 环距（像素，640x480 口径）。视觉只给圆盘中心，各环位置由 STM32 按此环距
# + 平行关系推算（官方：环为同心/等距，视觉看不到所有环）。
RING_SPACING_PX = 22

# ============================== 圆盘对齐（P6：环2 到水平正中心）==============================
# 视觉端主动发 MOVE,<axis>,<dx> 驱动下位机微调，直到 |x-W/2| <= ALIGN_TOL
ALIGN_ON          = True    # False = 只报告，不主动发 MOVE（退回旧行为）
ALIGN_TOL         = 5       # 对齐容差（px）
ALIGN_STABLE_FRAMES = 5     # 进入容差后还要连续 N 帧稳定才回 ok=1
ALIGN_STEP_MAX    = 40      # 单次 MOVE 最大步长（px），防止一大步冲过头
ALIGN_SETTLE_MS   = 300     # 发完 MOVE 等多久再重新测量（给机构运动时间）
ALIGN_MAX_ITER    = 20      # 最多微调次数，超过就回 DISC,...,0（不再死等）
ALIGN_INVERT      = False   # True = 把 MOVE 的 dx 取负（现场发现越调越远就打开）

# ============================== 码垛 ==============================
MATERIAL_HEIGHT_MM = 40     # 固定物料高度常量，随 STACK 帧下发（STM32 用于减去高度）

# ============================== 超时 ==============================
TASK_TIMEOUT_MS = 20000     # 每个任务的兜底超时（原程序是死等）

# ============================== 串口（MaixCAM2）==============================
# 唯一来源：protocol.py 会读这里。
#
#   ★ 默认采用 UART4（底部 6P 1.25mm 座）：
#       A21 = UART4_TX → STM32 PD2 (UART5_RX)
#       A22 = UART4_RX ← STM32 PC12(UART5_TX)
#       设备路径 /dev/ttyS4
#   ⚠️ UART4 被系统「Maix 通信协议」默认占用（负责和 MaixVision 通信）：
#       必须先跑 maix_toolkit/board/disable_comm.py --apply 关掉它并重启板子；
#       之后 MaixVision 的 UART 连接会失效，改用 网络(TCP) / SSH 连板子。
#
#   备选 UART2（左侧 2x6 座）：B0=TX / B1=RX / /dev/ttyS2（见文件末尾注释）
UART_DEV = "/dev/ttyS4"
PIN_TX = "A21"
PIN_RX = "A22"
UART_FUNC_TX = "UART4_TX"
UART_FUNC_RX = "UART4_RX"
UART_BAUDRATE = 115200      # STM32 端必须同步 115200

# ============================== 启动重试 ==============================
STARTUP_RETRY = 3           # 相机/串口 打开失败时重试次数
STARTUP_RETRY_DELAY_MS = 1000   # 每次重试间隔

# ============================== 串口拆帧 ==============================
# 半帧超过这个时间没有新字节，就丢弃残留（防止断线后吞掉后续正常帧）
FRAME_TIMEOUT_MS = 200

# 备选方案：UART2（左侧 2x6 座）—— 需要时把下面 5 行替换上面的 5 行
# UART_DEV = "/dev/ttyS2"
# PIN_TX = "B0"
# PIN_RX = "B1"
# UART_FUNC_TX = "UART2_TX"
# UART_FUNC_RX = "UART2_RX"

# ============================== 调试开关 ==============================
SHOW_PREVIEW = True         # 屏幕显示 + MaixVision 预览
VERBOSE = True              # 打印识别过程
TOUCH_UI = False            # 【默认关】True=触屏调试模式（会旁路串口命令！）
TOUCH_UI_SEND_UART = False  # 触屏触发的任务是否也发给 STM32（默认否，避免野数据）

# ============================== 精度：总开关 ==============================
# False = 完全退回旧行为（不缩放 / 不去畸变 / 不多帧平均），方便现场 A/B 对比
PRECISION_ON = True

# ============================== 精度：坐标系（高分辨率检测）==============================
# 检测分辨率可与对外口径(W,H)不同；由 precision.Pipeline 自动缩放 ROI 与回传坐标。
# 注意：改 DET 后 CAM_K 会按比例自动缩放，不需要重标。
DET_W, DET_H = 640, 480     # 想提精度就调大（如 1280, 960），记得关掉 MaixVision 预览

# ============================== 精度：镜头畸变（坐标层反畸变）==============================
# 由 maix_toolkit/pc/calib_distort.py 生成后粘过来（2026-10-17 实测，RMS 0.35px）
UNDISTORT_ON = True
CAM_K = [1233.82373356, 0.0, 329.05893682, 0.0, 1251.76780944, 260.49139204, 0.0, 0.0, 1.0]
CAM_DIST = [-1.62119437, 5.17504868, 0.00591734, 0.00596065, -13.65384756]  # k1,k2,p1,p2,k3
# 反畸变保护：位移超过该值(px)或跑出画面 → 该点退回原坐标（四角标定不可信时防呆）
UNDISTORT_MAX_SHIFT = 48

# ============================== 精度：手眼标定（像素→毫米）==============================
# 由 maix_toolkit/pc/calib_homography.py 生成后粘过来
HOMOGRAPHY_ON = False
HOMOGRAPHY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]     # 3x3 行优先

# ============================== 精度：多帧平均 ==============================
CENTER_FILTER_N = 9         # 坐标多帧平均窗口（σ 降为 1/sqrt(N)）；0/1 = 关

# ============================== AI（可选，见 maixcam_app_v2/ai_model.py）==============================
# 融合方式：经典 CV 负责「定位」（LAB 阈值找色块），AI 负责「判色」（抗现场色差）。
# AI 不可用/低置信度时自动退回阈值判色，所以开了不会比纯 CV 更差。
USE_AI = True               # 最终版默认开；模型已实测 val 99.38%，失效会自动退回阈值判色
AI_KIND = "classify"        # classify=分类(抗色差) | detect=检测(形状/码垛)
AI_MODEL = "/root/models/cls11n.mud"   # yolo11n-cls 转换产物（见 maixcam-migration/_converted）
AI_CONF = 0.5
AI_IOU = 0.45
# 分类类别顺序（ultralytics 按文件夹字母序，必须与训练一致）+ 类别→颜色码
AI_LABELS = ["background", "blue", "green", "other", "red", "yellow"]
AI_CLASS_TO_COLOR = {"red": "1", "yellow": "2", "blue": "3", "green": "4"}
AI_MIN_PROB = 0.60          # AI 判色置信度下限；低于它退回阈值判色
AI_LOCATE_MARGIN = 8        # 送进分类器前把色块外扩的像素（给模型一点上下文）
AI_PREFER_CV = False        # True=强制以阈值判色为准，AI 只记录（A/B 对比排查用）
AI_DEBUG_PROB = False       # True=每次 AI 判色都打印 (label, prob)，用来定 AI_MIN_PROB

# ============================== 障碍物（前进状态，命令字 'G'）==============================
# 官方赛道障碍物：黑色，φ50×100mm。黑色 = L 很低。
TH_BLACK = [0, 35, -25, 25, -25, 25]   # LAB 黑色阈值（起手值，现场重标）
ROI_FORWARD = []                       # 前方视野；可收紧成下方条带如 [0,180,640,300]
MIN_PIXELS_OBST = 400                  # 障碍物最小像素数（抗噪点/阴影）
OBST_MERGE_MARGIN = 8                  # 合并碎块
OBST_WIDTH_MM = 50.0                   # 障碍物物理宽度(mm)，用于估距
OBST_CLEAR_FRAMES = 3                  # 连续 N 帧没看到 → 判定"前方无障碍"(CLEAR)
OBST_STABLE_FRAMES = 5                 # 障碍物中心稳定帧数（行进中连续问，要快）
OBST_TIMEOUT_MS = 3000                 # 兜底超时，超时按无障碍回