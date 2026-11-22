# 新 H 自动顺逆时针格式实验版交接

更新日期为 2026 年 11 月 22 日。

## 交付状态

本目录保存基于现场 v3.9.96 新 H 的独立覆盖文件。`H_DIRECTION_PACKET_ENABLED = False` 为出厂状态。此次只发布 GitHub，车辆板端继续运行现有程序。启用实验格式前，需要接收端支持下述 HCOLORS 与 HDISC。

## 文件与运行依赖

- `tasks.py` 为实验任务入口，包含当前新 H 的完整任务逻辑及方向扩展。
- `h_direction.py` 为纯 Python 运动判向、双色排序和新包生成模块。
- `config.py` 为本地默认配置快照，目标 Y 为 271，方向格式开关关闭。
- `protocol.py` 为现有 AA55 帧封装与传输模块快照。
- `h_material_config.py` 为现有新 H 持久化参数解析模块。
- `selftest_tasks.py` 和 `test_h_direction.py` 用于 PC 验证。

本目录作为覆盖层配合现有 `vision-maixcam/maixcam_app_v2` 运行，依赖原应用的 `main.py`、`vision.py`、`precision.py`、`disc_debug.py`、`timing.py`、`cam_light.py`、`screen_echo.py`、模型和 MaixPy 运行时。此目录本身不是完整安装包。

## 新格式

外层继续使用 `AA 55 LEN payload CRC8 0D`。CRC8 对 LEN 与 payload 计算，poly 为 0x07。GPIO I²C 继续由现有传输层分片发送。STM32 应先完成外层组帧，再解析 ASCII payload。

| 作用 | payload 示例 | 字段解释 |
| --- | --- | --- |
| 两色已确认的中间通知 | `HCOLORS,3,2,CW` | 先到蓝、后到黄、顺时针 |
| 终包 | `HDISC,383,151,63,3,2,CW` | x、y、dx、先到颜色、后到颜色、方向 |
| 方向证据不足的终包 | `HDISC,320,240,0,3,1,UNKNOWN` | 先蓝后红，方向未知 |

方向字符串固定为 `CW`、`CCW`、`UNKNOWN`。颜色码沿用 1 红、2 黄、3 蓝、4 绿。`dx=x−320`，x/y 使用现有对外坐标。终包含最后一次纠偏前的坐标。方向字段属于物料盘转动方向。

`MOVEXY,xstep,ystep`、`READY`、`BUSY` 和 `NONE,H` 沿用现有格式。D/E/F 走现有协议入口。开关关闭时新 H 继续发 `COLORS,c1,c2` 与 `DISC,x,y,dx,c1,c2`。

STM32 新入口需要精确识别 HCOLORS 的 4 段和 HDISC 的 7 段，校验两色在 1～4 且不同，方向属于上述三值，x/y/dx 可解析为有符号整数。HCOLORS 为中间通知，HDISC 为任务结束。方向未知应显式交给上层处理。坐标与方向的接收代码尚需 STM32 方接入，本次没有部署 STM32。

## 判向与颜色先后

按现场已经给出的镜头约定，原始画面物料向右运动对应逆时针 CCW，向左对应顺时针 CW。判向使用原始图像坐标，双色排序也使用同一坐标系。

1. 只跟踪完整进入画面、离边缘至少 8 px 的有效颜色候选，同色取面积最大者。
2. 同色中心相隔至少 80 ms 后比较位移。窗口超过 400 ms、横向跳变超过 80 px、纵向跳变超过 60 px 时丢弃该窗口。
3. 横向位移至少 6 px 才产生一票。一个画面最多一票，多色方向冲突时清空投票。三次同向有效投票后锁存方向。
4. 方向确定后，全图恰好两种有效颜色且满足现有颜色稳定帧数，才能走运动双色捷径。CCW 右侧先到、左侧后到，CW 左侧先到、右侧后到。两色横向中心相距小于 6 px 时继续等待。
5. 方向未知时关闭运动双色捷径，继续走原有停稳事件流程。两次不同颜色的停稳事件按观测先后记录。若同帧存在多个停稳候选，沿用原有空间排序选择，不能据此证明真实时间先后。
6. 一旦发送车辆纠偏指令，本轮方向判定冻结，以免相机随车移动被计为物料盘旋转。此前未确认方向则继续报告 UNKNOWN。

方向只在每条 H 开始时重新判断，本轮锁存后不追踪中途反转。算法基于当前拍摄区域的运动方向约定，摄像头翻转、拍摄区域改变、同色目标切换或现场外部移动相机均需重新验证。

HCOLORS 发出时方向可能为 UNKNOWN，后续在首次车辆纠偏前如果得到足够运动证据，HDISC 可以给出已确认方向。接收端以终包方向为本轮最终值。

## 纠偏与结束条件

继续使用当前新 H 的 AI 判色、物料中心定位、双轴 MOVEXY、无任务总超时、预算耗尽立即回终包。首件最多使用总预算减一，为后到件留最后一轮。预算为 1 时只对后到件执行一次纠偏。发完最后一轮即返回动作前坐标，实际移动后的误差需要另外观察。

本地快照默认 Y=271、增益 0.65、步长限制为 0。实际板端 `h_material.json` 可覆盖目标、预算等参数，部署时必须保留现场参数。方向格式开关独立位于 config.py，当前调试台 JSON 不包含该开关。

## 后续启用步骤

1. STM32 接入并验证 HCOLORS、HDISC 新字段以及 UNKNOWN 的业务处理。
2. 备份 MaixCAM 现有应用及 `/root/yyb_mover_data/h_material.json`。
3. 将本目录的 `tasks.py`、`h_direction.py` 覆盖到应用。protocol.py 与 h_material_config.py 为配套基线参考，按目标版本核对后再覆盖。config.py 只合并 `H_DIRECTION_PACKET_ENABLED` 开关，保留现场 GPIO I²C、镜头、偏移和日志参数。
4. 将 `H_DIRECTION_PACKET_ENABLED` 改为 True，重启 MaixCAM 视觉应用。
5. 验证顺时针、逆时针、已静止起步、物料贴边、无颜色、纠偏导致画面移动、R 中断和 I²C 完整组帧。只观察测试可将现场 H 的 max_moves 临时设为 0。
6. 回退时将开关恢复 False 并重启，随后核对恢复原有两色包格式。

本次没有执行上述部署和启用步骤。

## 本地验证命令

在仓库根目录执行。

```powershell
python -X utf8 vision-maixcam/experiments/h_direction_20261122/test_h_direction.py
python -X utf8 -c "import sys,runpy; sys.path.append('vision-maixcam/maixcam_app_v2'); runpy.run_path('vision-maixcam/experiments/h_direction_20261122/selftest_tasks.py',run_name='__main__')"
```

自动测试覆盖顺逆判向、静止抖动、贴边、异常跳变、多色运动冲突、车辆纠偏时禁用判向、新格式字段、启用模式顺逆与 UNKNOWN 任务终包，以及开关关闭后的原有任务回归。板端实物方向判定和接收端联调仍待执行。
