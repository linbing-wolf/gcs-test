# 快速介绍 · YYB 智能搬运视觉端（MaixCAM2）

> 一句话：**MaixCAM2 当视觉上位机，被动回答 STM32 的问题**。STM32 发命令，视觉回坐标。
> 版本：**3.0.0** ｜ 设备：MaixCAM2（AX630C）｜ 串口：UART4 / 115200

---

## 1. 它能干什么

| 命令 | 场景 | 回什么 |
|---|---|---|
| `A` | 启停区 | `START,<区号>,x,y,w,h`（右侧有灰色道路=1 号，没有=2 号）|
| `T,<任务码>` | 任务码 | `TASK,<第一批颜色>,<第二批颜色>` |
| `C,<1~4>` | 原料区找指定颜色 | `PICK,颜色,x,y` |
| `D` / `E` | 粗加工区 / 暂存区圆盘 | `DISC,x,y,dx,ok`（`ok=1` 表示环已对中）|
| `F,<颜色码>` | 码垛：按颜色找底座 | `STACK,x,y`（**不含高度**；无同色底座时回空环中心）|
| `G` | 前方障碍物 | `OBST,x,y,w,h,mm` 或 `CLEAR` |
| `R` | 复位/握手 | `READY` |
| `Z` | 结束 | `OK` |

失败统一回 `NONE[,标签]`。任务执行中再发命令会收到 `BUSY`；发 `R` 会立即中断。

---

## 2. 它是怎么"看"的

```text
经典 CV 负责定位（LAB 阈值找色块、霍夫找圆）
AI 负责判色（yolo11n-cls，6 类；置信度不够自动退回阈值）
精度链：坐标反畸变（RMS 0.35px 标定）→ 缩回 640×480 → 9 帧平均
```

- AI 模型：`/root/models/cls11n.mud`（+ 两个 `.axmodel`）
- 量化后精度：验证集 **99.38%**（与 float 完全一致）
- AI 缺失/报错时**自动退回纯阈值**，不会导致程序不可用

---

## 3. 装到板子上

```bash
# ① （不需要了）旧文档说要先跑 Disable Maix Comm 让出 UART4；
#    2026-10-18 真机验证：Maix 通信协议占的是 UART1(A30/B3)，UART4 本来就是空的。
#    如果哪天 A21/A22 真的被占，再跑： python3 /root/disable_comm.py --apply

# ② 传模型（三个文件放同一个目录）
scp _converted/cls11n.mud _converted/cls11n_npu.axmodel _converted/cls11n_vnpu.axmodel \
    root@<板子IP>:/root/models/

# ③ 装 APP
scp dist/maix-yyb_mover-v3.0.0.zip root@<板子IP>:/root/
app_store_cli install /root/maix-yyb_mover-v3.0.0.zip
```

> ⚠️ 关掉 Maix 通信后，MaixVision 的 **UART 连接**就不能用了，改用**网络(TCP)/SSH**。

---

## 4. 接线

```text
MaixCAM2 A21 (UART4_TX) → STM32 UART 的 RX
MaixCAM2 A22 (UART4_RX) → STM32 UART 的 TX
GND 必须共地，115200 8N1
```

---

## 5. 常用参数（`maixcam_app_v2/config.py`）

| 参数 | 默认 | 什么时候改 |
|---|---|---|
| `USE_AI` | True | 模型没装想跑纯 CV 时设 False |
| `AI_MIN_PROB` | 0.60 | AI 判色太保守/太激进（先开 `AI_DEBUG_PROB` 看分布）|
| `AI_VETO_OTHER` | True | AI 判成 other 时是否丢弃候选 |
| `UNDISTORT_ON` | True | 换镜头/重标后 |
| `UNDISTORT_MAX_SHIFT` | 48 | 反畸变"防呆"上限（px）|
| `CENTER_FILTER_N` | 9 | 坐标抖动大就加大，反应慢就减小 |
| `ALIGN_ON` | True | 圆盘是否主动发 MOVE 微调 |
| `ALIGN_TOL` | 5 | 环 2 对中容差（px）|
| `STACK_COLOR_VOTE` | False | F 是否也做颜色投票 |
| `PICK_STOP_ONLY` | True | 原料区必须等转盘/物料停稳才回复 |
| `PICK_STOP_SPEED_PX_S` | 8.0 | 判定停稳的残余速度（px/s）|
| `PICK_STOP_HOLD_MS` | 300 | 需持续静止多久 |
| `OBST_STABLE_FRAMES` | 5 | G 的反应速度 |
| `TH_RED/TH_YELLOW/TH_BLUE/TH_GREEN` | 样本集统计值 | 换灯光/换场地后跑 `tools_analyze_lab.py` 重算 |
| `ROI_MATERIAL` 等 | 起手值 | 现场按机位调整 |

---

## 6. 上板自检（照着打勾）

```text
[ ] 启动日志：DET 640x480 → 640x480 | 去畸变 开 | 多帧平均 9
[ ] 启动日志：[AI] 已加载: /root/models/cls11n.mud (labels=...)
     另注意一行：[自检] pinmap A21 -> UART4_TX OK（当前=UART4_TX）
[ ] STM32 发 R → 收到 READY
[ ] A：有路/无路 → 区号 1/2，屏幕上 ratio 数值合理
[ ] C：红黄蓝绿都能命中，日志里 来源=ai / cv
[ ] D/E：会连续发 MOVE 并收敛到 |dx|≤5，ok=1
[ ] F,<颜色>：STACK,x,y（颜色一致 / 无底座回环心）
[ ] G：OBST/CLEAR 不来回抖
[ ] 断开相机 → 收到 ERR,camera（可选验证）
```

---

## 7. 常见问题

| 现象 | 先查什么 |
|---|---|
| 串口打不开 | 先 `ls /dev/ttyS4`；正常情况 UART4 是空的（Maix Comm 在 UART1）|
| 能开但收不到命令 | TX/RX 是否交叉；GND；波特率 115200 |
| AI 没生效 | 日志有没有 `[AI] 已加载`；`/root/models/cls11n.mud` 是否存在 |
| 颜色老判错 | 现场重标 LAB 阈值（`TH_*`），或调 `AI_MIN_PROB` |
| 坐标整体偏 | 是否开了去畸变但 STM32 的像素→mm 仍按原始像素标定 |
| 圆盘一直不等 | 看 `MOVE` 方向是否反了（dx>0 期望向右）；或 `ALIGN_TOL` 太小 |
| 启停区老判 2 号 | 看屏幕 `ROAD YES/no` 和 ratio，调 `ROI_ROAD`/`TH_ROAD` |
| 码垛超时 | 确认 `STACK_COLOR_VOTE=False`；ROI 是否对准 |

---

## 8. 文件地图

```text
maixcam_app_v2/
  main.py        命令分发 + 主循环 + 启动重试
  protocol.py    帧格式 / CRC / 拆帧 / pack_*
  tasks.py       各任务（A/C/D/E/F/G）
  vision.py      阈值找色块 / 找圆 / 找路 / AI 判色融合
  ai_model.py    AI 加载与分类
  precision.py   坐标流水线（缩放/反畸变/多帧平均）
  config.py      唯一参数源
  selftest*.py   三个本地自检（不上板也能跑）
_converted/      模型产物（mud + 2 个 axmodel）
dist/            可安装的 APP 包
```
