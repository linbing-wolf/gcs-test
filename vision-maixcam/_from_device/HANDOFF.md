# 交接文档 · MaixCAM 视觉迁移（最新）

> 用途：把 Windows 上做完的一切带到 **Linux** 继续（Linux 上跑 Docker/Pulsar2 完成 MaixCAM2 模型转换 + 写最终程序）。
> 工作区根：`maixcam-migration/` 　最后更新：2026-10-21

---

## 0. 现状一览

| 模块 | 状态 |
| --- | --- |
| 视觉端 **`maixcam_app_v2`（yyb_mover）** | ✅ 新流程上位机，标准 MaixCAM APP，可打包 |
| 工具集 **`maix_apps/maix_toolbox`** | ✅ 标定/取样/采集/关通信/自检；**已装到设备**（launcher 可见）|
| 精度库 `precision.py` + 标定工具 `maix_toolkit` | ✅ |
| STM32 侧改动说明 `stm32_patch/` | ✅ |
| 数据集（合格 870）→ 划分 | ✅ train648 / val162 / calib60 |
| 模型训练（本地 GPU）| ✅ **yolo11n-cls, val top-1 99.4%**，已导出 `best.onnx` |
| **镜头畸变标定** | ✅ **RMS 0.3525px**，参数已写入 config |
| **手眼标定（像素→mm）** | ⏸ **跳过**（待补）|
| **模型转换到 MaixCAM2** | ✅ **NPU2 + NPU1 两份 `.axmodel` + `cls11n.mud`**，量化后仿真 **99.38%** |
| **AI 判色接入** | ✅ CV 定位 + AI 判色，`USE_AI=True` 默认开（失效自动退回阈值）|
| **精度链接线** | ✅ `PRECISION_ON`：ROI 缩放 / 坐标反畸变 / 9 帧平均（v2.1.0）|

---

## 1. 环境

**Windows 训练机（已完成）**：Python 3.13 · torch **2.11.0+cu128** · torchvision 0.26+cu128 · ultralytics 8.4.178 · onnx 1.23.2 · onnxslim 0.1.98 · GPU RTX 5060 Laptop。

**Linux（待做转换）**：需 **Docker** + **Pulsar2 镜像 `pulsar2:6.0`**（MaixCAM2 = AX620E 平台）。

---

## 2. 关键参数速查

| 项 | 值 |
| --- | --- |
| 分辨率口径 | 640×480（对外）/ 检测可调 `DET_W,DET_H` |
| 串口 | 默认 **UART4**：`A21=TX / A22=RX · /dev/ttyS4 · 115200` |
| 串口前提 | **实测无需处理**：板端 Maix 通信协议占的是 **UART1（A30/B3）**，UART4 空着。
（旧文档说"UART4 被占用"是历史结论，2026-10-18 在真机上验证过 A21/A22 本来就是 `UART4_TX/RX`）|
| 帧格式 | `AA 55 LEN payload CRC8 0D` |
| 命令字 | `A`启停区 `T`任务码 `C`原料区三选一 `D/E`圆盘 `F`码垛 `G`障碍物 `R`复位 `Z`结束 |
| 颜色码 | **红1 黄2 蓝3 绿4** |
| 分类类别顺序（字母序）| `background, blue, green, other, red, yellow` |
| 设备 | MaixCAM2 · 局域网 `192.168.3.51` · **Tailscale `100.118.92.10`（maixcam2-9b80）** · `root/sipeed` |
| 设备上 APP 目录 | `/maixapp/apps/yyb_mover/`（用 `app_store_cli install <zip>` 安装）|
| 模型目录 | `/root/models/cls11n.mud` + `cls11n_npu.axmodel` + `cls11n_vnpu.axmodel`（三个必须同目录同名）|

---

## 3. 数据集（`_from_device/`）

```
dataset/              合格 870（红136 黄136 蓝161 绿163 背景143 其它131）
                      + _reject/(65 近重复) + _metrics.csv + _report.md
dataset_split/
  train/<6类>/        648
  val/<6类>/          162
  calib/              60   ← MaixCAM2 INT8 量化校准集
```

---

## 4. 模型（`_from_device/runs/cls11n/`）

- 权重：`weights/best.pt`（3.0MB）、`weights/best.onnx`（5.9MB）
- 验证：**top-1 99.4% / top-5 100%**；唯一瑕疵 yellow 4%→green
- **ONNX 接口（转换必读）**：
  ```
  INPUT : images   [1,3,224,224]  NCHW RGB 归一化 /255
  末节点: Flatten → Gemm(/model.10/linear/Gemm_output_0) → Softmax(output0)
  ⚠️ 分类用 Softmax 前一层的输出节点:  /model.10/linear/Gemm_output_0  (1,6)
  ```

---

## 5. 标定（`maix_toolkit/pc/`）

### 5.1 畸变标定 ✅ 已完成
- 数据：200 张（`/maixapp/share/picture/calib` → 本地 `maix_toolkit/pc/calib`）
- 有效 24 张，**重投影误差 RMS = 0.3525 px**
- 产物：`maix_toolkit/pc/cam_K_dist.py`，**已写入 `maixcam_app_v2/config.py`**：
  ```python
  UNDISTORT_ON = True
  CAM_K    = [1233.82373356,0,329.05893682, 0,1251.76780944,260.49139204, 0,0,1]
  CAM_DIST = [-1.62119437,5.17504868,0.00591734,0.00596065,-13.65384756]
  ```
- ⚠️ 系数偏大（边缘覆盖不足）；若要更稳可**重拍**：整块板入镜 + 移到中心/四角 + 多角度。

### 5.2 手眼标定 ⏸ 跳过
- `HOMOGRAPHY_ON = False`（默认）
- 补做时：板平放**工作平面**、正对拍 1 张 → `python maix_toolkit/pc/calib_homography.py`（填 PX/MM）→ 出 `HOMOGRAPHY`，粘进 config 并置 `HOMOGRAPHY_ON=True`。

---

## 6. 模型转换到 MaixCAM2（Linux 上做）

### 6.1 准备校准集
```bash
tar -cf calib.tar -C _from_device/dataset_split/calib .
```

### 6.2 `cls11n.npu.json`（分类）
```json
{
  "model_type": "ONNX",
  "npu_mode": "NPU2",
  "quant": {
    "input_configs": [{
      "tensor_name": "images",
      "calibration_dataset": "calib.tar",
      "calibration_size": 60,
      "calibration_mean": [0, 0, 0],
      "calibration_std": [255, 255, 255]
    }],
    "calibration_method": "MinMax",
    "precision_analysis": true
  },
  "input_processors": [{
    "tensor_name": "images", "tensor_format": "RGB", "tensor_layout": "NCHW",
    "src_format": "RGB", "src_dtype": "U8", "src_layout": "NHWC", "csc_mode": "NoCSC"
  }],
  "output_processors": [
    { "tensor_name": "/model.10/linear/Gemm_output_0", "dst_perm": [0, 1] }
  ],
  "compiler": { "check": 3, "check_mode": "CheckOutput", "check_cosine_simularity": 0.9 }
}
```

### 6.3 转换（NPU2 + NPU1 各一份）
```bash
pulsar2 build --target_hardware AX620E --input best.onnx --output_dir tmp --config cls11n.npu.json
cp tmp/compiled.axmodel out/cls11n_npu.axmodel
sed 's/"NPU2"/"NPU1"/' cls11n.npu.json > cls11n.vnpu.json
pulsar2 build --target_hardware AX620E --input best.onnx --output_dir tmp --config cls11n.vnpu.json
cp tmp/compiled.axmodel out/cls11n_vnpu.axmodel
```

### 6.4 `cls11n.mud`
```ini
[basic]
type = axmodel
model_npu = cls11n_npu.axmodel
model_vnpu = cls11n_vnpu.axmodel

[extra]
model_type = classifier
input_type = rgb
labels = background, blue, green, other, red, yellow
input_cache = true
output_cache = true
input_cache_flush = true
output_cache_inval = true
mean = 0,0,0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098
```

### 6.5 上板 + 接入
```bash
scp out/cls11n.mud out/cls11n_*.axmodel root@192.168.3.51:/root/models/
```
`maixcam_app_v2/config.py`：
```python
USE_AI = True; AI_KIND = "classify"; AI_MODEL = "/root/models/cls11n.mud"
```
板端：`nn.Classifier("/root/models/cls11n.mud")` → 类别索引 → `ai_model.color_code(idx)` → 颜色码。

---

## 7. 串口协议 & 视觉端结构（写最终程序参考）

`maixcam_app_v2/`：
| 文件 | 作用 |
| --- | --- |
| `main.py` | 命令分发 + 主循环 |
| `protocol.py` | 变长帧编解码 + 命令解析 + 回复构造（`pack_*`）|
| `vision.py` | 4 色 / 启停区 / 圆盘 / 码垛 / 障碍物 识别 |
| `tasks.py` | 各任务（超时 + 稳定性；`task_forward` 障碍物）|
| `config.py` | 唯一参数源（颜色/ROI/串口/精度/AI）|
| `precision.py` | 缩放 / 去畸变 / 手眼 / 圆拟合 / 多帧平均 |
| `ai_model.py` | 可选 AI 层（分类/检测，CV 兜底）|
| `app.yaml` `assets/` `run.sh` `pack.ps1` `selftest.py` | 打包/资源/自检 |

回复帧：`START,<区号>,x,y,w,h` · `TASK,<G1>,<G3>` · `PICK,color,x,y` · `DISC,x,y,dx,ok` · `STACK,x,y,dx,ok` · `OBST,x,y,w,h,dist`/`CLEAR` · `READY`/`OK` · `BUSY` · `NONE[,tag]`/`ERR,msg`

### 7.1 状态机（v3.3.0）

```text
[上电] 自检 → 打开 UART4 → 发一次 READY（提示）→ IDLE
IDLE   : 每 5ms poll 一次命令
RUNNING: 阻塞跑一个任务（最长 TASK_TIMEOUT_MS=20s；G 为 OBST_TIMEOUT_MS=3s）
         每帧「看一眼」串口：
           收到 R      → 立刻中断，回 READY
           收到其它命令 → 回 BUSY（任务继续跑）
         正常结束 → 回结果帧 → IDLE
```

**握手约定**：正式握手由 STM32 发 `R` 触发（视觉端可能先启动，上电那次 READY 会丢）。

**任务 A（启停区）特殊约定**：主输出是**区号**——
画面右侧有灰色道路 → `START_ZONE_HAS_ROAD`(1)，没有 → `START_ZONE_NO_ROAD`(2)；
后面 4 个数是蓝色启停区的中心与尺寸，**看不到时全 0，但区号仍然有效**。
道路判据（需现场标定）：`ROI_ROAD=[384,120,256,300]`，`TH_ROAD` 灰色 LAB，
`ROAD_MIN_RATIO=0.30`，连续 `ROAD_STABLE_FRAMES=5` 帧一致才确认。

**任务 D/E（圆盘）对齐约定（v3.9.1，2026-10-21）**：
视觉端要保证「环 2（同心环组中心）」落在画面中心：`|x-W/2| <= ALIGN_TOL(5px)` 且
`|y-H/2| <= ALIGN_DY_TOL(5px)`。
`ALIGN_ON=True` 时视觉端会**主动发运动指令**：

```
视觉端 → 下位机：MOVE,<dx>      水平偏差；执行后不用回帧
视觉端 → 下位机：MOVEY,<dy>     垂直偏差；执行后不用回帧
```

- 收到 `D/E` 后先等 `DISC_START_DELAY_MS(500ms)` 再读第一帧；
- 两轴都超差时，一轮会先发 `MOVE` 再发 `MOVEY`；
- 每轮等 `ALIGN_SETTLE_MS(800ms)` 后重新测量，最多 `ALIGN_MAX_ITER(2)` 轮；
- 方向反了改视觉端的 `ALIGN_INVERT_D/E` 或 `ALIGN_INVERT_DY_D/E`，不要改 STM32 符号；
- `DISC` 的 `ok` 当前固定为 `1`，即使超差也会回 1，不能只靠 `ok` 判断是否对准；
- D 水平偏置单独用 `ALIGN_DX_OFFSET_D=-10`，E 用全局 `ALIGN_DX_OFFSET=+5`；
- 忽略 `MOVEY` → 垂直方向无法闭环修正。

**任务 F（码垛）对齐约定（v3.9.1）**：
与 D/E 一样使用 `MOVE,<dx>` / `MOVEY,<dy>` 双轴闭环，目标为底座/空环中心。
回复 `STACK,x,y,dx,1`，`ok` 当前固定为 1；`STACK_ALIGN_ON` 可关闭对齐。
方向反了改 `STACK_INVERT_X/Y`。

**STM32 侧必须配合的五点**：
1. 一问一答：收到结果帧再发下一条（任务没结束前发会收到 `BUSY`）。
2. 超时要 > 20s（`TASK_TIMEOUT_MS`），且把 `BUSY` 当「还在算」，不要当失败。
3. `STACK,x,y,dx,ok` 带水平偏差；**没有高度字段**（旧文档里的 `h` 已移除）。
4. `C,<颜色码>` 只接受 `1/2/3/4`，其它值立即回 `NONE,C`。
5. 所有 `x,y,w,h` 都是 **640×480 口径、已去畸变**；`DISC` 只给中心不给半径。

---

## 8. 本次已修复的坑（别再踩）

1. **拍图叠加层进图**：之前保存的图带 UI（蓝九宫格线穿棋盘）→ 已改为**先存干净帧再画预览**（toolbox + CLI 采集脚本）。
2. **app.yaml YAML 报错**：`desc` 里半角冒号导致设备装不上 → 已**加引号**（yyb_mover 同修）。
3. **selftest 断言**：标定后 `estimate_distance_mm` 不再为 0 → 已改。
4. **板端不要"边跑 APP 边安装"**：2026-10-18 同时后台跑 main.py + `app_store_cli install`，
   设备直接失联（需断电重启）。正确顺序：先 `pkill -f "[m]ain\.py"`，再安装，再启动。
5. **`pkill -f "main.py"` 会杀掉自己的 SSH 会话**（命令串里含 main.py）→ 用 `pkill -f "[m]ain\.py"`。
6. **设备上没有 `setsid`** → 后台启动只能用 `nohup ... &`。

---

## 9. 待办清单（Linux 上）

```text
[x] 整包拷到 Linux（含 _from_device/runs/cls11n/weights/best.onnx）
[x] 装 Docker + pulsar2:6.0            → 镜像从 ModelScope 官方包 docker load（29MB/s）
[x] tar 校准集 dataset_split/calib → calib.tar
[x] pulsar2 转换（输出节点 /model.10/linear/Gemm_output_0；NPU2 + NPU1）
[x] 写 cls11n.mud（model_type=classifier；input_cache_flush=false）
[x] scp 到设备 /root/models/（2026-10-18 完成，md5 一致，板端 AI 加载成功：labels=6）
[x] config.USE_AI=True + AI_MODEL      → 已默认开，且做了「AI 判色 + CV 兜底」
[x] precision 接进 vision/tasks        → PRECISION_ON / DET_W,DET_H / CENTER_FILTER_N 生效
[ ] （可选）补手眼标定 → HOMOGRAPHY
[ ] （可选）重拍畸变图（整块入镜 + 覆盖四角）→ 更新 CAM_K/CAM_DIST
[ ] STM32 侧接入视觉协议（**当前完全没接**，且 vision_uart.c 还是旧的 9 字节 0xFF/0xFE 帧）
```

---

## 9.1 Linux 侧续做记录（2026-10-17）

### 模型转换（已完成）
- 镜像：ModelScope `AXERA-TECH/Pulsar2` 的 `ax_pulsar2_6.0.tar.gz` → `docker load` 得 `pulsar2:6.0`
  （Docker Hub 直连只有 0.14MB/s，不要用；ModelScope 29MB/s）
- **必须先裁剪 ONNX** 到输出节点再 `onnxsim`（不是只在 json 里写节点名）：
  `onnx.utils.extract_model(..., ['images'], ['/model.10/linear/Gemm_output_0'])`
- 产物：`_converted/cls11n_npu.axmodel` / `cls11n_vnpu.axmodel` / `cls11n.mud`
- 精度：float ONNX val 99.38% → **INT8 仿真 val 99.38%（argmax 与 float 100% 一致）**
- 全记录见 `_converted/CONVERT_MAIXCAM2.md`

### 精度链接线（v2.1.0）
- `precision.Pipeline`：`ROI(对外口径) → 检测域`、`坐标 → 反畸变 → 缩放`、`多帧平均`
- `main.py` 相机改用 `DET_W/DET_H`；`PRECISION_ON=False` 时完全退回旧行为（可 A/B）
- **CAM_K 会随 DET 自动缩放**（改 1280×960 不用重标）
- 反畸变保护 `UNDISTORT_MAX_SHIFT=48px`：跑出画面或位移超限的点退回原坐标
  （当前标定四角位移 60~124px，会跑出画面；工作 ROI 内 13~19px，全部有效）

### AI 判色（v2.1.0 默认开）
- 修掉 `ai_model.py` 两处 bug：`classify()` 传了 `Classifier` 没有的 `roi` 参数；
  把返回的 `[(idx,prob),...]` 当索引用 → 现在 `classify_top1()` 取 `res[0]`
- 融合：CV 定位 + AI 判色；低置信度/background/异常 → 退回阈值判色
- 回归测试：`selftest.py`（含 Pipeline）与 `selftest_ai.py` 均 ALL PASS

### 打包
- `app.yaml` → **2.2.0**；`dist/maix-yyb_mover-v2.2.0.zip`（按 files 白名单，36KB）

### 仍待做
- 上板实测 FPS / 准确率 / `AI_MIN_PROB` 现场调参
- 手眼标定（可选）
- ⚠️ 本次 Linux 侧写回 Windows 时 `/mnt/win` 是只读挂载，需先 `sudo mount -o remount,rw /mnt/win`

---

## 10. 关键路径总表

```
maixcam-migration/
├─ maixcam_app_v2/                        比赛主程序（yyb_mover）+ dist\maix-yyb_mover-v2.0.0.zip
├─ maix_apps/maix_toolbox/                工具集 APP + dist\maix-maix_toolbox-v1.0.0.zip
├─ maix_toolkit/{pc,board}/               标定脚本 + 板端采集；pc\cam_K_dist.py（畸变结果）
├─ stm32_patch/                           STM32 侧改动
├─ PROJECT_MAP.html / CALIBRATION_GUIDE.html / PROJECT_FULL.html
└─ _from_device/
    ├─ dataset/  dataset_split/           数据
    ├─ yolo11n-cls.pt
    ├─ runs/cls11n/weights/best.{pt,onnx} ← 转换输入
    └─ TRAINING.md / HANDOFF.md（本文件）
```

---

## 11. 上板实测与版本历程（2026-10-18）

### 11.1 设备访问

```text
Tailscale : 100.118.92.10 （maixcam2-9b80）；局域网 192.168.3.51
SSH       : root / sipeed
APP 目录  : /maixapp/apps/yyb_mover/     安装：app_store_cli install <zip>
模型目录  : /root/models/{cls11n.mud, cls11n_npu.axmodel, cls11n_vnpu.axmodel}
            ↑ 三个文件必须同目录同名，否则 MUD 找不到权重
```

### 11.2 真机实测确认

| 项 | 结果 |
| --- | --- |
| pinmap A21/A22 | 本来就是 `UART4_TX/RX`；旧日志"返回非0"是 `code == 0` 比较枚举的**误报**，v3.2.1 已改为看实际复用功能 |
| Maix 通信协议 | 占 **UART1（A30/B3）**，与 UART4 不冲突 → **不需要** Disable Maix Comm |
| AI 模型加载 | ✅ `load model from: /root/models/cls11n_vnpu.axmodel`；`labels num: 6` |
| input_cache_flush | 板端打印 `flush input cache to ddr before run model: 0`，与 MUD 的 `false` 一致 |
| output_cache_inval | 板端打印 `1`，与 MUD 一致 |
| 画面 | 640×480 预览正常（面板 480×640） |
| 串口回环 | READY 帧已发出，**STM32 侧尚未接入**（协议未实现） |

### 11.3 版本历程

| 版本 | 内容 |
| --- | --- |
| **3.1.0** | 阶段交接定稿：BUSY/R 中断、任务码语义校验、启停区区号（右侧道路+AI 双确认）、圆盘 `MOVE` 对齐闭环、G 快响应、STACK 去掉高度 |
| **3.2.0** | 团队新增：`F,<颜色码>` 按颜色选底座；无同色 → 霍夫找空环中心 |
| **3.2.1** | pinmap 诊断日志修正（区分真失败/已生效）；AI 模型缺失时打印排查提示 |
| **3.3.0** | 原料区（转盘）**停稳才回复**：`StopDetector` + `PICK_STOP_ONLY/SPEED_PX_S/HOLD_MS` |
| **3.4.0** | **屏幕命令回显条**（`screen_echo.py`）：画面底部常驻 RX/TX/状态+收发计数+坏帧数；空闲时每 100ms 刷预览（不再冻屏）；`R/T/Z` 也有痕迹。开关 `SCREEN_ECHO / SCREEN_ECHO_IDLE_MS / SCREEN_ECHO_LINES` |
| **3.5.x ~ 3.7.x** | 圆盘识别改为「最内圈白块 / 黑色环质心 / 霍夫细化 / 圆环兜底」；增加数字“2”模板校验；D/E 启动延时、跑飞保护、串口全量日志、真实 D 调试触发；MOVE 系数可调 |
| **3.8.0** | **新增 `MOVEY,<dy>` 垂直微调**；D/E 同时校水平和垂直，两轴独立跑飞保护；`DISC` 回复格式不变、`ok` 固定 1 |
| **3.8.1** | 模板阈值 `0.65 → 0.55`、尺度增加 `0.9`，解决物料挡住数字下半部时 2 号环被误判 `NONE`；上板实测可识别并发出 `MOVEY` |
| **3.9.0** | **F 码垛改为 MOVE/MOVEY 双轴闭环**，回复改为 `STACK,x,y,dx,ok`；串口日志增加运行时间/版本，调试数据迁到 `/root` 持久化 |
| **3.9.1** | D 单独增加 `ALIGN_DX_OFFSET_D=-10` 修正稳定左偏；E 保持全局 `+5`；交接文档同步 D/E 专用偏置 |

### 11.4 新增文档

| 文档 | 给谁 |
| --- | --- |
| `QUICKSTART.md` / `.html` | 人看的快速上手（安装、接线、参数、排障）|
| `AI_HANDOFF_STM32.md` | STM32 侧 AI 的精确协议交接（帧格式、CRC 向量、状态机）|
| `COLOR_GUIDE.html` | 颜色码可视化 + 任务码解析器 + 自测 |

### 11.5 仍然没做的（按优先级）

```text
1. STM32 侧接入视觉协议（当前完全没接；vision_uart.c 还是旧的 9 字节 0xFF/0xFE 帧）
2. 上板跑通 A/T/C/D/E/F/G 全流程实测
3. 手眼标定（HOMOGRAPHY）——可选
4. 现场标定 LAB_TH_*（数据集统计值已有）/ TH_ROAD（需要真实前视画面）
5. 全线路偏移：PPM_Y 未按实测修正 + 前进左搓（详见队友的路径诊断文档）
```
