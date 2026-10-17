# Dataset Capture · 数据集采集助手

MaixCAM 标准 APP（`id: dataset_capture`）。用于在设备上**采 AI 训练数据集**。

## 用法

1. 装好 APP，launcher 里点开。
2. 底部一排类别按钮：`red / yellow / blue / green / background / other`，**点一下选中**当前类别。
3. 对准物料，**点画面区域保存一帧**到当前类别；按钮上显示该类别已采张数。
   顶栏 **`AUTO`** 键切换：`AUTO ON` 时自动连拍到当前类别（**`-`/`+` 调间隔 0.1~2.0s，默认 1.0s**；换物料前先退出或切类别）。
4. 右上角 **EXIT** 退出。
5. 数据存到设备 `/root/dataset/<类别>/<类别>_NNNN.jpg`。

## 采集要点

- **取景 = 运行时取景**（同一相机、同一距离/背景比例）。
- 每类 **200~500 张**；跨**光照**（开/关/侧光）、**角度**、**色差**、**背景**。
- 负样本（`background` / `other`）能显著降低误检。
- 类别名 / 顺序要和训练时、MUD 的 `labels` 一致。

## 打包 / 安装

```bash
pip install maixtool
maixtool release            # → dist/dataset_capture_v1.0.0.zip
app_store_cli install /root/dataset_capture_v1.0.0.zip
```

或使用同目录 `../../maix_apps/pack.ps1`。

## 采完

把 `/root/dataset` 拷到 PC，按 `maix_toolkit/README.md` 的流程去训练
（MaixHub 在线训练 / ultralytics 本地训练 → 转 MaixCAM2 模型 → 放 `/root/models`）。

（无触屏设备时，APP 会**每秒自动存一张**到当前类别作兜底。）