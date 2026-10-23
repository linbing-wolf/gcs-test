# YYB Mover —— MaixCAM2 智能搬运视觉端（标准 MaixCAM APP）

> id: `yyb_mover` · version: `3.9.45` · 入口：`main.py`  
> 配套：STM32 智能搬运车（UART5 / 115200），协议见 `protocol.py`  
> 最新交接说明：`HANDOVER_20261023.md`

## 这是什么

MaixCAM2 作为**视觉上位机**，被动应答 STM32 的查询：
启停区、任务码解析、原料区三选一、圆盘中心、码垛中心。

| 命令 | 含义 | 回复 |
| --- | --- | --- |
| `A` | 识别启停区 | `START,<区号>,x,y,w,h` |
| `T,<任务码>` | 下发任务码，确定颜色 | `TASK,<G1>,<G3>` |
| `C,<颜色码>` | 原料区三选一 | `PICK,color,x,y` |
| `D` / `E` | 粗加工区 / 暂存区 圆盘中心 + 双轴对齐 | `DISC,x,y,dx,ok` |
| `F` | 码垛中心 + 双轴对齐 | `STACK,x,y,dx,ok` |
| `G` | 前进状态：检测前方黑色障碍物 | 有：`OBST,x,y,w,h,dist`／无：`CLEAR` |
| `H` | 物料盘下弧对齐（以盘最低点为基准） | `DISC,x,y,dx,ok` |
| `R` / `Z` | 复位 / 结束 | `READY` / `OK` |

帧格式（双向）：`AA 55 LEN payload CRC8 0D`

### D/E 圆盘识别与对齐（加工区 / 暂存区）

- **识别**：`find_circles` 粗定位 → 径向边缘点 + 最小二乘拟合（亚像素）→ 找不到大圆再用同心圆环兜底。
- **对齐**：视觉端发 `MOVE,<step>`（水平）和 `MOVEY,<step>`（垂直）闭环微调，
  `step = dx/dy × ALIGN_GAIN`，限幅 `±ALIGN_STEP_MAX`。
- **闭环模式**（`ALIGN_ONE_SHOT=False`，当前默认）：最多 `ALIGN_MAX_ITER=3` 轮，
  每轮只发超差轴的指令（两轴都超差时先 `MOVE` 再 `MOVEY`），
  每次等 `ALIGN_SETTLE_MS` 后重新测量；对齐回 `DISC,...,1`，
  3 轮仍未对齐回 `DISC,...,1`（`ok` 按要求固定 1）。
- **一次性模式**（`ALIGN_ONE_SHOT=True`）：只发一轮 `MOVE` / `MOVEY`，随后立即回
  `DISC,x,y,dx,1`，不再闭环等待。
- **完成**：`|dx| <= ALIGN_TOL` 且 `|dy| <= ALIGN_DY_TOL` 连续
  `ALIGN_STABLE_FRAMES` 帧 → 回 `DISC,x,y,dx,1`。
- **未完成**：微调超过 `ALIGN_MAX_ITER` 轮 / 超时 → `DISC,x,y,dx,1`。
- **D/E 独立偏置**：`ALIGN_DX_OFFSET_D=-10`（D 实测稳定左偏修正），
  `ALIGN_DX_OFFSET_E=None`（回退到全局 `ALIGN_DX_OFFSET=+5`）。
- **看不到圆**：连续 `DISC_LOST_FRAMES` 帧 → 立即回 `NONE,D` / `NONE,E`。
- **内容校验**：候选圆周围必须在 `assets/target2.png` 模板中匹配到数字“2”，
  否则不画绿圆、直接回 `NONE`（解决“没圆也产生绿圆”）。
  当前 `DISC_TEMPLATE_THRESHOLD=0.55`，尺度 `[0.9, 1.0]`；实测物料挡住数字下半部时
  2 号环约 0.59，1/3 号环 <0.30。
- **启动延时**：收到 D/E 后先等 `DISC_START_DELAY_MS=500ms` 再读第一帧。
- **超时**：`DISC_TIMEOUT_MS = 8000`（D/E 专用，比通用 `TASK_TIMEOUT_MS` 短）。
- 例：`DISC,321,238,2,1` 表示圆心 (321,238)，水平偏差 +2px，已对齐。

### F 码垛双轴对齐

`STACK_ALIGN_ON=True` 时，F 与 D/E 一样发 `MOVE,<dx>` / `MOVEY,<dy>` 闭环：

- 目标：底座/空环中心到画面中心，容差 `STACK_ALIGN_TOL` / `STACK_ALIGN_DY_TOL`
- 偏移：`STACK_ALIGN_DX_OFFSET` / `STACK_ALIGN_DY_OFFSET`
- 方向：`STACK_INVERT_X` / `STACK_INVERT_Y`
- 复用 `ALIGN_GAIN`、`MOVE_SCALE`、`MOVEY_SCALE`、`ALIGN_STEP_MAX`、`ALIGN_SETTLE_MS`、`ALIGN_MAX_ITER`
- 回复：`STACK,x,y,dx,1`（高度仍由 STM32 自己算；`ok` 与 D/E 一样固定为 1）

调试触发：

```bash
touch /tmp/run_stack_move
```

### H 物料盘下弧对齐

物料盘（白色带孔圆盘）常常**只拍得到下半弧**（圆心落在画面外），`find_circles` 找不到整圆。
`H` 用**粗圆心先验 + 沿下弧射线找边缘 + 最小二乘拟合圆**外推圆心，再取**最低点** (cx, cy+r)
作为基准，与目标最低点相减得到 dx/dy，和 D/E 一样发 `MOVEXY,dx,dy` 微调：

- 识别：`vision.find_disc_arc()`（纯 `get_pixel` + `precision.fit_circle`，无 cv2 依赖）
- 参数：`PLATE_INIT_CX/CY/R`（粗圆心，cy 可为负）、`PLATE_SEARCH_BANDS`（迭代搜索带，大→小）、
  `PLATE_TARGET_X/Y`（目标最低点，现场标定）、`PLATE_TOL_X/Y`；一次/闭环跟随 `ALIGN_ONE_SHOT`
- 回复：`DISC,x,y,dx,1`（x,y = 盘最低点，dx = 水平偏差；dy 随 `MOVEXY` 下发）
- 看不到下弧连续 `PLATE_LOST_FRAMES` 帧 → `NONE,H`
- 方向反了改 `PLATE_INVERT_X/Y`，不要改 STM32 符号
- 自检：`python selftest_plate.py`（合成下弧图 + 对齐闭环）

> ⚠️ 目标最低点 `PLATE_TARGET_X/Y` 要现场标定：把盘摆到希望对齐的位置，看日志里
> `[H] 物料盘最低点 (x,y)`，把 x/y 填进 config。

### 启动延时：等画面稳定（替代固定等待）

收到 D/E、F、H 后**不再死等固定毫秒**，而是等画面连续稳定后再读第一帧：

- 机构（机械臂/转盘/车体）还在动 → 画面变化大 → 继续等；停下来 → 立即开始
- `DISC_/STACK_/PLATE_START_DELAY_MS` 变成**最长等待上限**（判稳会提前返回，通常 100~300ms 就开始了）
- 参数：`SCENE_SETTLE_ON`(True) / `SCENE_SETTLE_FRAMES`(3) / `SCENE_SETTLE_DIFF`(3.0) / `SCENE_SETTLE_MIN_MS`(100)
- 判定用画面的 32×24 降采样灰度签名，纯 `get_pixel`，无 cv2 依赖
- 想回到旧行为（固定等待）：`SCENE_SETTLE_ON = False`

### 对齐回复模式：回读 / 去回读

D/E、F、H 发完 `MOVEXY` 之后怎么回复，由 `ALIGN_REPLY_MODE` 决定：

- `"direct"`（默认，**去回读**）：发完 `MOVEXY` 立即返回**动作前**的 `x/y/dx`，
  省掉 0.6~2.5s 回读；STM32 自己知道移动了多少时用这个最快
- `"verify"`：回读一次，用**调整后**的 `x/y/dx` 回复（更准但慢）

```python
ALIGN_REPLY_MODE = "direct"   # "direct"=去回读 | "verify"=回读后再回
```

### 串口日志与运行信息

每次 APP 启动都会在 `/root/yyb_mover_serial/` 新建：

```text
serial_YYYYmmdd-HHMMSS.log
sessions.jsonl
```

- 每行格式：`时间 [+运行秒数] [RX/TX/EVENT] 内容 | 原始帧 hex`
- 启动行包含版本号：`APP START version=3.9.1 pid=...`
- 退出行包含总运行时长：`APP EXIT uptime=... version=...`
- `sessions.jsonl` 每次启动/退出追加一行，记录版本、PID、启动时间、运行时长
- 日志目录在 `/root` 下，不随 APP 安装/升级被覆盖

### D/E 调试抓图（3.6.0）

`config.DISC_DEBUG_SAVE = True` 时，每次 D/E 会在
`/root/yyb_mover_data/disc_debug/<时间戳>-<D|E>/` 下保存（APP 目录外，升级不丢）：

- `000_start.jpg`、`001_iter0_before.jpg`、`final_ok.jpg` … 关键帧
- `meta.jsonl`：每帧的 `dx/cx/cy/r/src/circles/refined/refine_reason/...`
- `summary.json`：本次任务结果、存图数量、时长

画面标注：**蓝色细圆 = 所有候选圆，绿色 = 采用的圆，红十字 = 最终圆心，黄线 = 画面中心**。
可以直接看出是不是 `find_circles` 选了假圆/别的环。

拉回电脑分析：

```bash
scp -r root@<板子IP>:/root/yyb_mover_data/disc_debug ./disc_debug
python3 maix_toolkit/pc/analyze_disc_debug.py ./disc_debug
```

正式比赛可把 `DISC_DEBUG_SAVE` 改回 `False`，避免占存储。

## 文件

| 文件 | 作用 |
| --- | --- |
| `main.py` | 入口：命令分发 + 主循环 |
| `protocol.py` | 变长帧编解码 + 命令解析 + 回复构造 |
| `vision.py` | 4 色 / 启停区 / 圆盘 / 码垛 识别 |
| `tasks.py` | 各视觉任务（带超时 + 稳定性判定）|
| `disc_debug.py` | D/E 调试抓图：关键帧 JPEG + 逐帧检测数据 |
| `config.py` | 唯一参数来源（颜色 / ROI / 串口 / 精度 / AI 开关）|
| `precision.py` | 精度库：缩放 / 去畸变 / 手眼 / 圆拟合 / 多帧平均 |
| `ai_model.py` | 可选 AI 层（分类 / 检测，CV 兜底）|
| `timing.py` | 时间兼容层 |
| `touch_ui.py` | 触屏调试入口（默认关）|
| `run.sh` | 脱机运行脚本 |
| `assets/yyb_mover.png` | 应用图标 |

## 打包（标准 APP）

```bash
# 方式一（推荐）：官方工具
pip install maixtool
maixtool release            # 产出 dist/yyb_mover_v2.0.0.zip

# 方式二：本仓库自带的兜底打包（不装 maixtool 也能出包）
pwsh -File pack.ps1
```

## 安装到设备

```bash
# ① 上传 zip 到设备后，命令行安装
app_store_cli install /root/yyb_mover_v2.0.0.zip

# ② 安装后务必清掉旧字节码，否则可能继续跑上一版
#    （旧 pyc 的时间戳比新源码新时，Python 会优先加载 pyc）
rm -rf /maixapp/apps/yyb_mover/__pycache__

# ③ 重启应用
```

装好后回到 launcher，列表里会出现 **YYB Mover**。

## 直接运行（开发/调试）

```bash
# 拷贝到板子任意目录，前台跑
sh run.sh                   # 日志写 finder.log
tail -f finder.log
```

## 配置要点

- **串口（默认 UART4）**：`config.UART_DEV = /dev/ttyS4`（A21=TX / A22=RX），115200。
  ⚠️ UART4 被系统「Maix 通信协议」占用，**先跑 `maix_toolkit/board/disable_comm.py --apply` 关掉并重启**；
  之后 MaixVision 的 UART 连接失效，改用网络(TCP)/SSH。备选 UART2 见 `config.py` 注释。
- **颜色码**：红1 黄2 蓝3 绿4（`config.COLOR_CODES`）。
- **LAB 阈值**：`config.TH_*` 是起手值，**必须现场用「找色块」APP 重标**。
- **精度**：`UNDISTORT_ON` / `HOMOGRAPHY_ON` / `DET_W,DET_H`（见 `maix_toolkit` 标定流程）。
- **AI**：`USE_AI=False` 默认关；开启前先放好 `/root/models/*.mud`。
- **触屏调试**：`TOUCH_UI=True` 会**旁路串口命令**，仅调试用，**出厂务必关**。

## 退出

设备 **FUNC 键** 退出；或代码里 `app.set_exit_flag(True)`。

## 自检

```bash
python3 selftest.py         # 板上有 maix 环境即可跑；PC 上也能跑（内置假 maix）
```