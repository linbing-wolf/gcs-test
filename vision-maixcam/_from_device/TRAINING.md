# 本地训练 + 转换到 MaixCAM2（分类模型）

> 与比赛无关的通用流程。数据在 `dataset_split/`，模型产出在 `runs/cls11n/`。

## 0. 环境（本机实测）

| 项 | 版本 |
| --- | --- |
| Python | 3.13 |
| torch | **2.11.0+cu128**（`cuda True`，RTX 5060 Laptop 可用）|
| torchvision | 0.26.0+cu128 |
| ultralytics | 8.4.178 |

安装（如需重来）：
```powershell
python -m pip install --upgrade torch torchvision --index-url https://download.pytorch.org/whl/cu128
python -m pip install ultralytics
```

## 1. 数据

```
dataset_split/
├── train/<类别>/*.jpg   648
├── val/<类别>/*.jpg     162
└── calib/*.jpg          60   ← 量化校准集
```
ultralytics 分类直接认这个结构（`data=dataset_split`）。

## 2. 训练

```powershell
python -c "from ultralytics import YOLO; YOLO('yolo11n-cls.pt').train(data=r'<abs>\dataset_split', epochs=80, imgsz=224, batch=32, device=0, name='cls11n')"
```
产出：`runs/cls11n/weights/best.pt`。

## 3. 类别顺序（**关键**）

ultralytics 按**文件夹名字母序**取类别，本数据集为：

```
0 background
1 blue
2 green
3 other
4 red
5 yellow
```

→ 运行时映射到颜色码：**red=1, yellow=2, blue=3, green=4**；`background/other` = 无颜色。
（已写进 `maixcam_app_v2/config.py` 的 `AI_LABELS` / `AI_CLASS_TO_COLOR`。）

## 4. 导出 ONNX

```powershell
python -c "from ultralytics import YOLO; YOLO(r'<abs>\runs\cls11n\weights\best.pt').export(format='onnx', imgsz=224, opset=12, simplify=True)"
```

## 5. 转换到 MaixCAM2（**注意：分类不能走在线平台**）

- **在线转换平台只支持「目标检测 Detect」**，不支持分类。
- 分类模型转到 MaixCAM2 需要**手动 Pulsar2**（Docker）：
  - 平台 = `AX620E`
  - MUD 里 `model_type = classifier`
  - 输出节点：**取 softmax 前的分类输出**（若末尾有 softmax）
  - INT8 量化校准集：用 `dataset_split/calib`（打包 zip/tar）
- 产出：`xxx.mud` + `xxx_npu.axmodel` + `xxx_vnpu.axmodel` → 上传到设备 `/root/models/`。

## 6. 运行时接入（比赛主程序）

`maixcam_app_v2/config.py`：
```python
USE_AI   = True
AI_KIND  = "classify"
AI_MODEL = "/root/models/wys_cls.mud"
```
`ai_model.AI.classify()` 返回类别索引；用 `ai_model.color_code(idx)` 得到颜色码，喂给原 CV 兜底逻辑。

## 7. 注意

- **取景要一致**：训练用的是整帧（640×480），运行时也要喂同口径的画面，别只喂小裁剪。
- INT8 量化后**必须在板端实测**准确率（可能掉几个点）。