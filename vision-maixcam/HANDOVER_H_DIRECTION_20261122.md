# H 自动顺逆时针格式交接

更新日期为 2026 年 11 月 22 日。

## 交付状态

已随 `3.9.97-i2c-gpio` 部署并启用。板端 2026 年 11 月 22 日 22:27 启动，READY 通过 GPIO I²C 写入成功，启动后收发错误计数为 0。实际运行文件位于 `vision-maixcam/releases/20261122-j-h-direction/`，包含保留现场 GPIO I²C 的 main.py 和 protocol.py。实验目录保留为开发参考，默认开关状态以各目录配置为准。

当前板端 `H_DIRECTION_PACKET_ENABLED=True`、`H_DIRECTION_FALLBACK=1`，H 目标 Y=318、预算 1 次、步长上限 0。已进行本地专项和回归测试、板端导入与协议函数检查。尚未观察到现场 STM32 发起 H 的顺逆时针实测任务，STM32 新格式接收处理需在联调中核对。

## 文件与运行依赖

- `tasks.py` 为实验任务入口，包含当前新 H 的完整任务逻辑及方向扩展。
- `h_direction.py` 为纯 Python 运动判向、双色排序和新包生成模块。
- `config.py` 为本地默认配置快照，目标 Y 为 318，方向格式开关关闭。
- `protocol.py` 为现有 AA55 帧封装与传输模块快照。
- `h_material_config.py` 为现有新 H 持久化参数解析模块。
- `selftest_tasks.py` 和 `test_h_direction.py` 用于 PC 验证。

上述实验目录作为覆盖层配合现有 `vision-maixcam/maixcam_app_v2` 运行，依赖原应用的 `main.py`、`vision.py`、`precision.py`、`disc_debug.py`、`timing.py`、`cam_light.py`、`screen_echo.py`、模型和 MaixPy 运行时。此目录本身不是完整安装包。

## 新格式

外层继续使用 `AA 55 LEN payload CRC8 0D`。CRC8 对 LEN 与 payload 计算，poly 为 0x07。GPIO I²C 继续由现有传输层分片发送。STM32 应先完成外层组帧，再解析 ASCII payload。

| 作用 | payload 示例 | 字段解释 |
| --- | --- | --- |
| 两色已确认的中间通知 | `HCOLORS,3,2,0` | 先到蓝、后到黄、顺时针 |
| 终包 | `HDISC,383,151,63,3,2,0` | x、y、dx、先到颜色、后到颜色、方向 |
| 判向不足采用预设的终包 | `HDISC,320,240,0,3,1,1` | 先蓝后红，采用预设逆时针 |

方向字段固定为单个 ASCII 数字，`0` 表示顺时针，`1` 表示逆时针。包内只发送 0 或 1。颜色码沿用 1 红、2 黄、3 蓝、4 绿。`dx=x−320`，x/y 使用现有对外坐标。终包含最后一次纠偏前的坐标。方向字段属于物料盘转动方向。

`MOVEXY,xstep,ystep`、`READY`、`BUSY` 和 `NONE,H` 沿用现有格式。D/E/F 走现有协议入口。开关关闭时新 H 继续发 `COLORS,c1,c2` 与 `DISC,x,y,dx,c1,c2`。

STM32 新入口需要精确识别 HCOLORS 的 4 段和 HDISC 的 7 段，校验两色在 1～4 且不同，方向严格为单字符 0 或 1，x/y/dx 可解析为有符号整数。HCOLORS 为中间通知，HDISC 为任务结束。自动判向证据不足时采用配置预设，接收端收到的方向始终为 0 或 1。坐标与方向的接收代码尚需 STM32 方接入，本次没有部署 STM32。

## 判向与颜色先后

按现场已经给出的镜头约定，原始画面物料向右运动对应逆时针 1，向左对应顺时针 0。判向使用原始图像坐标，双色排序也使用同一坐标系。

1. 只跟踪完整进入画面、离边缘至少 8 px 的有效颜色候选，同色取面积最大者。
2. 同色中心相隔至少 80 ms 后比较位移。窗口超过 400 ms、横向跳变超过 80 px、纵向跳变超过 60 px 时丢弃该窗口。
3. 横向位移至少 6 px 才产生一票。一个画面最多一票，多色方向冲突时清空投票。三次同向有效投票后锁存方向。
4. 方向确定后，全图恰好两种有效颜色且满足现有颜色稳定帧数，才能走运动双色捷径。逆时针 1 右侧先到、左侧后到，顺时针 0 左侧先到、右侧后到。两色横向中心相距小于 6 px 时继续等待。
5. 方向未知时关闭运动双色捷径，继续走原有停稳事件流程。两次不同颜色的停稳事件按观测先后记录。若同帧存在多个停稳候选，按自动确认方向或配置兜底方向排序，不能据此证明真实时间先后。
6. 一旦发送车辆纠偏指令，本轮方向判定冻结，以免相机随车移动被计为物料盘旋转。此前未确认方向则采用配置预设。

方向只在每条 H 开始时重新判断，本轮锁存后不追踪中途反转。算法基于当前拍摄区域的运动方向约定，摄像头翻转、拍摄区域改变、同色目标切换或现场外部移动相机均需重新验证。

首次发送 HCOLORS 时锁定本轮输出方向，自动判向成功则用检测方向，否则用配置预设。随后冻结判向，HDISC 保持相同方向。日志中 direction_assumed 为 true 表示用了预设，false 表示由运动证据确认。

## 纠偏与结束条件

继续使用当前新 H 的 AI 判色、物料中心定位、双轴 MOVEXY、无任务总超时、预算耗尽立即回终包。首件最多使用总预算减一，为后到件留最后一轮。预算为 1 时只对后到件执行一次纠偏。发完最后一轮即返回动作前坐标，实际移动后的误差需要另外观察。

本地快照默认 Y=318、增益 0.65、步长限制为 0。实际板端 `h_material.json` 可覆盖目标、预算等参数，部署时必须保留现场参数。方向格式开关和兜底方向独立位于 config.py，当前调试台 JSON 不包含这两个字段。

```python
H_DIRECTION_PACKET_ENABLED = False
H_DIRECTION_FALLBACK = 1  # 0=顺时针，1=逆时针，自动判向不足时采用
```

默认兜底为逆时针，沿用此前现场约定。兜底值只接受整数 0/1，非法值直接报错。静止画面无法通过本算法证明真实转向，因此预设方向需要现场人员确认。

方向为单个 ASCII 数字，占 1 字节。对比旧实验版 CW、CCW、UNKNOWN，分别减少 1、2、6 字节，LEN 与 CRC8 随内容重新计算。相对现场旧包，新前缀多 1 字节，末尾逗号和方向多 2 字节，总计增加 3 字节。例如 `DISC,383,151,63,3,2` 为 19 字节 payload，新 `HDISC,383,151,63,3,2,0` 为 22 字节，完整 AA55 帧由 24 变为 27 字节。现有 GPIO I²C 每片 31 字节有效数据，较长坐标组合需由现有传输层完整分片和重组。

## 后续启用步骤

1. STM32 接入并验证 HCOLORS、HDISC 新字段以及 0/1 的业务处理。
2. 备份 MaixCAM 现有应用及 `/root/yyb_mover_data/h_material.json`。
3. 将上述实验目录的 `tasks.py`、`h_direction.py` 覆盖到应用。protocol.py 与 h_material_config.py 为配套基线参考，按目标版本核对后再覆盖。config.py 只合并 `H_DIRECTION_PACKET_ENABLED` 开关，保留现场 GPIO I²C、镜头、偏移和日志参数。
4. 确认 `H_DIRECTION_FALLBACK` 为现场失败假设方向，再将 `H_DIRECTION_PACKET_ENABLED` 改为 True，重启 MaixCAM 视觉应用。
5. 验证顺时针、逆时针、已静止起步、物料贴边、无颜色、纠偏导致画面移动、R 中断和 I²C 完整组帧。只观察测试可将现场 H 的 max_moves 临时设为 0。
6. 回退时将开关恢复 False 并重启，随后核对恢复原有两色包格式。

上述部署已按用户本次授权完成。J 停稳检测模式已同期部署，详见 `vision-maixcam/HANDOVER_J_20261122.md`。

## 本地验证命令

在仓库根目录执行。

```powershell
python -X utf8 vision-maixcam/experiments/h_direction_20261122/test_h_direction.py
python -X utf8 -c "import sys,runpy; sys.path.append('vision-maixcam/maixcam_app_v2'); runpy.run_path('vision-maixcam/experiments/h_direction_20261122/selftest_tasks.py',run_name='__main__')"
```

自动测试覆盖顺逆判向、静止抖动、贴边、异常跳变、多色运动冲突、车辆纠偏时禁用判向、新格式字段、启用模式顺逆与两种兜底方向的任务终包，以及开关关闭后的原有任务回归。板端实物方向判定和接收端联调仍待执行。

## 本次发布与备份

发布采用直接文件覆盖层，详见 `vision-maixcam/releases/20261122-j-h-direction/manifest.json` 与 `deployment.json`。板端备份为 `/root/yyb-before-3.9.97-20261122-222610`。当前 E dx=0，D dy=40。
