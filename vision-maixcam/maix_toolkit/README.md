# maix_toolkit —— 通用视觉工具集（与比赛内容无关）

> 独立项目：这里的工具**不依赖任何比赛逻辑**，可复用到任意 MaixCAM / MaixCAM2 项目。
> 比赛应用在 `../maixcam_app_v2/`。

## 目录

```
maix_toolkit/
├── pc/                       # 在电脑上跑
│   ├── calib_distort.py      # 镜头畸变标定 → cam_K_dist.py
│   ├── calib_homography.py   # 手眼标定(像素→mm) → homography.py
│   └── analyze_disc_debug.py # 分析 D/E 对齐抓图（meta.jsonl + JPEG）
├── board/                    # 在 MaixCAM2 上跑
│   └── capture_dataset.py    # 采集 AI 训练数据集
└── README.md
```

## 依赖

- PC 端：`pip install opencv-python numpy`
- 板端：MaixPy（板子自带）

## 用法

### 1. 畸变标定（提高几何精度，收益最大）

```bash
# 拍 15~20 张棋盘格，放到 pc/calib/
python pc/calib_distort.py --dir pc/calib --cols 9 --rows 6 --square 25
```

产出 `pc/cam_K_dist.py`，把里面两行（`CAM_K` / `CAM_DIST` + `UNDISTORT_ON=True`）
粘进 `../maixcam_app_v2/config.py`。

- **验收**：重投影误差 **< 0.5 px**
- **前提**：标定与运行**同一分辨率**；棋盘格要平、覆盖面广
- 官方大底镜头（SC850SL / OS04A10）标 **-25.6% 畸变，必做**；OS04D10 无畸变可跳过

### 2. 手眼标定（像素 → 毫米）

```bash
# 先用尺子量出若干标记点的 mm 坐标，读出其像素坐标，填进脚本里的 PX / MM
python pc/calib_homography.py
```

产出 `pc/homography.py`，粘进 `../maixcam_app_v2/config.py`。

- **验收**：反算误差 **< 1~3 mm**
- **前提**：坐标原点/轴向**与 STM32 一致**；标定面与工作台面共面

### 3. 采集 AI 数据集

```bash
# 在 MaixCAM2 上
python3 board/capture_dataset.py --out /root/dataset
# 按提示输入类别名，连拍
```

产出 `<out>/<类别>/*.jpg`，用于 MaixHub / 本地训练。

### 4. 分析 D/E 圆盘对齐抓图

应用里 `DISC_DEBUG_SAVE=True` 时，每次 D/E 会把关键帧和 `meta.jsonl` 存到
`/root/yyb_mover_data/disc_debug/`。拉回电脑后：

```bash
scp -r root@<板子IP>:/root/yyb_mover_data/disc_debug ./disc_debug
python3 pc/analyze_disc_debug.py ./disc_debug
```

会输出每次任务的 `dx` 统计、检测来源（refine/circle/ring）、候选圆数量、
拟合失败原因和逐帧时间线，配合 JPEG 可直接看出是否选错圆。

## 在比赛应用里怎么生效

`../maixcam_app_v2/config.py` 里开关：

| 开关 | 作用 |
| --- | --- |
| `UNDISTORT_ON` + `CAM_K/CAM_DIST` | 坐标层反畸变 |
| `HOMOGRAPHY_ON` + `HOMOGRAPHY` | 像素→毫米 |
| `DET_W/DET_H` | 高分辨率检测（发送前缩回 640×480）|
| `USE_AI` + `AI_MODEL` | 启用 AI 分类/检测（CV 兜底）|

由 `../maixcam_app_v2/precision.py` 与 `ai_model.py` 使用（纯 Python，板上可直接跑）。