# Linux 侧转换记录 · yolo11n-cls → MaixCAM2 axmodel/mud

> 对应 `_from_device/HANDOFF.md` 第 4 节「用 Pulsar2 转换」。
> 工作目录：`/home/linbing/maixcam2_convert/`

---

## 0. 结论速览

| 项 | 值 |
| --- | --- |
| 转换环境 | 官方 Pulsar2 **6.0** docker 镜像（`ax_pulsar2_6.0.tar.gz` → `docker load`） |
| 平台 | `--target_hardware AX620E`（MaixCAM2 = AX630C，对应 Pulsar2 的 AX620E 平台） |
| 输入 | `images` [1,3,224,224]，U8 NHWC（预处理已集成进模型） |
| 输出 | `/model.10/linear/Gemm_output_0` [1,6]（**softmax 前**的 logits） |
| 校准集 | `dataset_split/calib` 60 张（已打 `datasets/train.tar`） |
| float ONNX val top-1 | **99.38 %**（161/162，与训练报告的 99.4 % 一致） |
| INT8 axmodel val top-1 | **99.38 %**（161/162，与 float 完全一致，argmax 一致率 100 %） |

产物：

| 文件 | 大小 | md5 |
| --- | --- | --- |
| `out/cls11n_npu.axmodel` | 1.6 MB | `a5d81280bd886e3b9a40475ca3e4b1bc` |
| `out/cls11n_vnpu.axmodel` | 1.6 MB | `654697ea8b3b361a86eda40b6cbb76a1` |
| `out/cls11n.mud` | 363 B | — |

编译期指标：
- 输入 `images` UINT8 `[1,224,224,3]`，输出 `/model.10/linear/Gemm_output_0` FLOAT `[1,6]`
- 单个 NPU 节点 `subgraph_npu_0`，MACs ≈ 194.6 M，mcode ≈ 57 KiB
- `check` 阶段输出余弦相似度 `0.9999999952`
- 量化后与 float logits 的 cosine 均值 0.977（最低 0.84），但 **top-1 完全没变**

---

## 1. 与 HANDOFF 文档不同的三处（已按官方口径修正）

1. **必须先裁剪 ONNX**，不是只在配置里写输出节点名。
   官方 [MaixCAM2 转换文档](https://wiki.sipeed.com/maixpy/doc/zh/ai_model_converter/maixcam2.html)要求
   `onnx.utils.extract_model` 把图截断到目标输出节点，再 `onnxsim` 简化。
   → 已生成 `export.onnx`（143 节点，末尾就是 `Gemm`，不含 `Softmax`）。
2. **镜像名不是 `pulsar2:6.0` 也能用**，官方文档自己写了 `docker tag pulsar2:3.3 pulsar2:6.0`。
   本机最终用的是 ModelScope 上的官方 `ax_pulsar2_6.0.tar.gz`。
3. **`input_cache_flush = false`**（HANDOFF 里写的是 `true`）。
   依据：官方开源转码平台 `maix_converter_platform/converter/yolo/mud.py` 的 MUD 模板
   —— 输入预处理集成在模型里（首层是 CPU 侧的 normalize/transpose），所以不用 flush；
   末层是 NPU 算子，`output_cache_inval = true`。
   *若上板后输入数据疑似不对，第一件事就是把它改回 `true` 再试。*

---

## 2. 目录结构

```
/home/linbing/maixcam2_convert/
├─ model.onnx                 # 原样拷贝的 best.onnx
├─ export.onnx                # 裁剪+简化后（Pulsar2 的输入）
├─ datasets/train.tar         # 60 张量化校准图（扁平打包）
├─ dataset_split/val/         # 162 张验证图（给仿真评估用）
├─ config/
│  ├─ cls11n.npu.json         # npu_mode = NPU2（满算力）
│  └─ cls11n.vnpu.json        # npu_mode = NPU1（给 AI-ISP 留算力）
├─ build.sh                   # 容器内一键转换脚本
├─ eval_acc.py                # PC 上跑 float ONNX 的 val 精度
├─ eval_int8_sim.py           # 容器内 pulsar2 run 仿真 INT8 精度
└─ out/
   ├─ cls11n_npu.axmodel
   ├─ cls11n_vnpu.axmodel
   └─ cls11n.mud
```

---

## 3. 复现步骤

```bash
# 1) 准备 ONNX（裁剪到 Gemm 输出 + 简化）
cd /home/linbing/maixcam2_convert
python -c "import onnx,onnxsim;\
onnx.utils.extract_model('model.onnx','tmp.onnx',['images'],['/model.10/linear/Gemm_output_0']);\
m,_=onnxsim.simplify(onnx.load('tmp.onnx'));onnx.save(m,'export.onnx')"

# 2) 校准集
tar -cf datasets/train.tar -C <dataset_split>/calib .

# 3) 转换（两个都出）
docker run --rm -v "$PWD":/data -w /data pulsar2:6.0 bash build.sh

# 4) 仿真验证 INT8 精度
docker run --rm -v "$PWD":/data -w /data pulsar2:6.0 \
    python3 eval_int8_sim.py out/cls11n_npu.axmodel
```

---

## 4. 模型输入输出（转换必读）

```
INPUT  : images                    [1, 3, 224, 224]   RGB /255，训练侧口径
OUTPUT : /model.10/linear/Gemm_output_0   [1, 6]       logits（Softmax 前）
```

类别顺序 = 文件夹字母序 = `background, blue, green, other, red, yellow`
（MUD 的 `labels` 必须与之一致；MaixPy 的 `Classifier.classify(softmax=True)` 会自己补 softmax）

---

## 5. 上板

```bash
scp out/cls11n.mud out/cls11n_npu.axmodel out/cls11n_vnpu.axmodel \
    root@<设备IP>:/root/models/
```

`maixcam_app_v2/config.py`：
```python
USE_AI   = True
AI_KIND  = "classify"
AI_MODEL = "/root/models/cls11n.mud"
```

上板后建议先跑一段最小自测（不要一上来就开比赛主程序）：

```python
from maix import camera, display, image, nn
c = nn.Classifier("/root/models/cls11n.mud")          # dual_buff 默认 True
print("labels:", c.labels, "input:", c.input_size(), c.input_format())
cam = camera.Camera(c.input_width(), c.input_height(), c.input_format())
disp = display.Display()
while not __import__("maix").app.need_exit():
    img = cam.read()
    res = c.classify(img)
    idx, prob = res[0]                                 # 注意：返回 [(idx,prob),...] 降序
    img.draw_string(2, 2, "%s %.2f" % (c.labels[idx], prob), image.COLOR_RED)
    disp.show(img)
```

- 期望：正对物料时 `prob` 接近 1.0，`labels[idx]` 与颜色一致。
- **若颜色明显串/置信度低**，第一个要试的开关是 MUD 里的 `input_cache_flush` 改成 `true`。
- 本机 PC 上的 MaixPy（`~/.venvs/maixcam`）**不能**跑 axmodel（没有 libaxengine），
  所以「上板实测」这一步在 Linux 侧无法替代。

---

## 6. AI 层接入（已实现，`USE_AI=True` 才生效）

### 6.1 修掉的两个 bug（原来会让 AI 静默失效）

原 `ai_model.py` 的 `classify()` 直接把 MaixPy 的返回值往外抛：

```python
return self.net.classify(img, roi=roi)   # ① Classifier 没有 roi 参数 → 抛异常
return self.net.classify(img)            # ② 返回的是 list，不是 index
```

MaixPy 的签名是 `classify(img, softmax=True, fit=...)`，**返回 `[(idx, prob), ...]`（按概率降序）**，
官方示例写法 `max_idx, max_prob = res[0]`。原来的 `color_code(idx)` 里 `labels[int(idx)]`
对 list 取 `int()` 会 `TypeError` 并被 `except` 吞掉 → 永远返回 `None`。

现在：

```python
idx, prob = ai.classify_top1(img, roi)     # 内部 img.crop(roi) 后 classify，取 res[0]
code = ai.color_code_of(img, roi)          # idx → 颜色码；background/other/低置信度 → None
ai.classify(img, roi)                      # 兼容旧接口，返回 int
```

### 6.2 真正接进流水线：CV 定位 + AI 判色

分工（在 `vision.ColorClassifier` 里）：

```
经典 CV  → 定位：LAB 阈值 find_blobs 找候选色块（位置/大小/中心，快且稳）
AI       → 判色：把色块框外扩 AI_LOCATE_MARGIN 像素后丢给分类模型
```

`ColorClassifier.find_color()` 现在按 `resolve_code()` 决定颜色码，顺序是
**AI 判色 → 不行就退回阈值判色**，退回条件（任一）：
模型没加载成功 / 不是 classify 模式 / 置信度 < `AI_MIN_PROB` /
判成 `background`·`other` / 抠图或推理抛异常。

所以 **`USE_AI=False` 时行为与之前完全一致**；`True` 时也不可能比纯 CV 更差。
返回的 item 多了两个键，便于现场排查：

| 键 | 含义 |
| --- | --- |
| `src` | `"ai"` / `"cv"`，这一帧的颜色是谁给的 |
| `ai_code` | AI 判出的码（没采用时为 `None`） |

接线：`main.py` 里 `ai = ai_model.AI()` → `vision.ColorClassifier(..., ai=ai)`；
`task_pick` 的日志/画面会显示颜色来源。

### 6.3 新增/改动的配置（`config.py`）

```python
USE_AI = False              # 默认关
AI_MODEL = "/root/models/cls11n.mud"
AI_MIN_PROB = 0.60          # 置信度下限，低于它退回阈值判色
AI_LOCATE_MARGIN = 8        # 送进分类器前把色块外扩的像素
AI_PREFER_CV = False        # True=强制以阈值判色为准（A/B 排查用）
```

### 6.4 回归测试（PC 上跑，无需真机）

```bash
python selftest.py       # 原有自检（协议等）
python selftest_ai.py    # 新增：AI 融合逻辑（AI 覆盖阈值 / 退化回退 / top1 取值）
```

`selftest_ai.py` 覆盖 5 组共 20+ 条断言，两个自检均 `ALL PASS`。
（`app.yaml` 是白名单打包，两个 selftest 都不会进发布包。）

### 6.5 还没做的（需要真机）

- 上板实测：`nn.Classifier` 加载 `cls11n.mud`、`classifier.labels` / `input_size()` 是否正常；
- 现场灯光下**重新标定 LAB 阈值**（AI 只在定位到的候选块上判色，定位仍依赖阈值）；
- `AI_MIN_PROB` 现场调参（建议先用日志把 `prob` 打出来看分布再定）。