# Maix Toolbox · Maix 工具集

MaixCAM 标准 APP（`id: maix_toolbox`）——**一个 APP 干四件事**，不用回电脑。

## 菜单

| 功能 | 说明 | 数据目录 |
| --- | --- | --- |
| **Calib Capture** | 标定拍图（棋盘格）：九宫格参考线 + 点屏保存 | `/maixapp/share/picture/calib/` |
| **Color Tune** | 色块取样：点画面任意一点，显示该点 **RGB / LAB**（拿去填 LAB 阈值）| — |
| **Dataset Capture** | 数据集采集：选类别 + 点屏存帧 | `/maixapp/share/picture/dataset/<类别>/` |
| **Disable Maix Comm** | 关闭系统「Maix 通信协议」，让出 **UART4(A21/A22)** | — |
| **Diagnostics** | 设备自检：MaixPy 版本 / 原生算子 / 串口 / 引脚 | — |

点每个工具里的 **BACK** 回菜单；按设备 **FUNC** 键退出整个 APP。

## 补光灯（闪光灯）

主菜单右上角有 **`LIGHT ON / LIGHT OFF`** 按钮，**Dataset Capture** 顶栏左侧也有一个
（采集时随手开关，状态全局共享、切页面不丢）。

- MaixCAM2 板载照明 LED = **B25 / GPIOB25**，高电平点亮。
- 采集颜色样本时**建议打开**，保证与比赛现场光照一致（主程序 `yyb_mover` 启动时也会自动点亮）。
- 没用 PWM6 调亮度：官方文档说明 PWM6 与 A30(UART1) 二选一，会和 Maix 通信协议抢引脚。

## 每个工具怎么用

### Calib Capture
整块棋盘格可见；屏幕有九宫格线帮你覆盖边缘。点屏存一帧，计数在顶栏。
**存 `/maixapp/share/picture/calib/`**（不可写时自动退回 `/root/calib/`）。
**顶栏 `AUTO` 键切换**：`MANUAL`=点屏保存；`AUTO ON`=自动连拍（**`-`/`+` 调间隔 0.1~2.0s，默认 1.0s**，不费手）。
（拍法见 `maix_toolkit` 标定流程：畸变 15~20 张、手眼 1 张。）

### Color Tune
点画面任意一点 → 屏幕显示该点 **RGB** 与 **LAB**（CIELAB，与 `find_blobs` 阈值同口径）。
把 LAB 抄进 `maixcam_app_v2/config.py` 的 `TH_RED / TH_YELLOW / TH_BLUE / TH_GREEN` 即可。

### Dataset Capture
底部类别按钮：`red / yellow / blue / green / background / other`，点选当前类别；
点画面区域存一帧到该类别。**存 `/maixapp/share/picture/dataset/<类别>/`**（不可写退回 `/root/dataset/<类别>/`）。
**顶栏 `AUTO` 键切换**：`AUTO ON` 时自动连拍到当前类别（**`-`/`+` 调间隔 0.1~2.0s，默认 1.0s**；换物料前先 BACK 或切类别）。
**顶栏 `LIGHT` 键**：开/关补光灯（重新采集颜色样本时建议打开，见上一节）。

### Disable Maix Comm
点 **APPLY** 关闭系统通信协议、让出 UART4。
⚠️ 之后 **MaixVision 的 UART 连接会失效**，改用 **网络(TCP)/SSH**；**重启板子生效**。
（等价于 `disable_comm.py --apply`。）

### Diagnostics
一次性打印：MaixPy 版本、`find_blobs/circles/qrcodes/lines/apriltags` 是否有、
串口设备列表、`A21/A22/B0/B1` 可用功能、逐个 `/dev/ttyS*` 试开结果。

## 打包 / 安装

```bash
pip install maixtool
maixtool release            # → dist/maix_toolbox_v1.0.0.zip
app_store_cli install /root/maix_toolbox_v1.0.0.zip
# 或（本地二维码）maixtool deploy → 设备 App Store 扫码
```

或使用 `maix_apps/pack.ps1`（`pwsh -File pack.ps1 -App maix_toolbox`）。

## 数据导出

```bash
scp -r root@<板子IP>:/maixapp/share/picture/calib   ./maix_toolkit/pc/calib
scp -r root@<板子IP>:/maixapp/share/picture/dataset ./maix_toolkit/dataset
```

## 说明

- 本 APP **自包含**（不依赖比赛主程序 `yyb_mover`）。
- 无触屏设备时，拍图/采集工具不做兜底（本工具集按有触屏设计；比赛主程序才有自动兜底）。