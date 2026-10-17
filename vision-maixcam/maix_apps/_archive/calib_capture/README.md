# Calib Capture · 标定拍图助手

MaixCAM 标准 APP（`id: calib_capture`）。用于**棋盘格标定拍图**。

## 用法

1. 装好 APP，launcher 里点开。
2. 相机对准棋盘格，**整块可见**；屏幕有**九宫格参考线**帮你判断是否覆盖到边缘。
3. **点屏幕保存一帧**，左上角显示已保存张数；右上角 **EXIT** 退出。
   顶栏 **`AUTO`** 键切换：`MANUAL`=点屏保存；`AUTO ON`=自动连拍（**`-`/`+` 调间隔 0.1~2.0s，默认 1.0s**，不费手）。
4. 图片存到设备 `/root/calib/calib_0001.jpg ...`。

## 拍多少、怎么拍

| 标定 | 张数 | 要点 |
| --- | --- | --- |
| 畸变 | 15~20 | 正面 + 四角贴边 + 绕 X/Y 倾 ±20~30° + 绕 Z 转 45/90° + 有远有近 |
| 手眼 | 1（可多拍取平均）| 板平放在工作平面、正对、不倾斜 |

**铁律**：整块可见、清晰、不反光、不出框、覆盖边缘。

## 打包 / 安装

```bash
pip install maixtool
maixtool release            # → dist/calib_capture_v1.0.0.zip
app_store_cli install /root/calib_capture_v1.0.0.zip
```

或使用同目录 `../../maix_apps/pack.ps1`。

## 拍完

把 `/root/calib` 拷到 PC 的 `maix_toolkit/pc/calib/`，跑：

```
python maix_toolkit/pc/calib_distort.py --dir maix_toolkit/pc/calib --cols 9 --rows 6 --square 20
```

（无触屏设备时，APP 会**每秒自动存一张**作兜底。）