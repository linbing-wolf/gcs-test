# YYB Mover —— MaixCAM2 智能搬运视觉端（标准 MaixCAM APP）

> id: `yyb_mover` · version: `2.0.0` · 入口：`main.py`
> 配套：STM32 智能搬运车（UART5 / 115200），协议见 `protocol.py`

## 这是什么

MaixCAM2 作为**视觉上位机**，被动应答 STM32 的查询：
启停区、任务码解析、原料区三选一、圆盘中心、码垛中心。

| 命令 | 含义 | 回复 |
| --- | --- | --- |
| `A` | 识别启停区 | `START,x,y,w,h` |
| `T,<任务码>` | 下发任务码，确定颜色 | `TASK,<G1>,<G3>` |
| `C,<颜色码>` | 原料区三选一 | `PICK,color,x,y` |
| `D` / `E` | 粗加工区 / 暂存区 圆盘中心 | `DISC,x,y` |
| `F` | 码垛中心 + 固定物料高 | `STACK,x,y,h` |
| `G` | 前进状态：检测前方黑色障碍物 | 有：`OBST,x,y,w,h,dist`／无：`CLEAR` |
| `R` / `Z` | 复位 / 结束 | `READY` / `OK` |

帧格式（双向）：`AA 55 LEN payload CRC8 0D`

## 文件

| 文件 | 作用 |
| --- | --- |
| `main.py` | 入口：命令分发 + 主循环 |
| `protocol.py` | 变长帧编解码 + 命令解析 + 回复构造 |
| `vision.py` | 4 色 / 启停区 / 圆盘 / 码垛 识别 |
| `tasks.py` | 各视觉任务（带超时 + 稳定性判定）|
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

# ② 本地二维码安装（设备与电脑同一局域网）
maixtool deploy             # 显示二维码，用设备 App Store 扫
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