# J 物料停稳检测指令交接

日期为 2026 年 11 月 22 日。本功能已随 3.9.97-i2c-gpio 上传板端，2026 年 11 月 22 日 22:27 启动验证通过。J 已接入正式命令入口，尚未观察到现场 STM32 发起的 J 实测任务。

## 指令与任务生命周期

发送 `J` 开始全图物料检测，使用现有颜色定位和 AI 判色能力。每种颜色跟踪一个候选，检测范围为当前整个画面。

任务没有观察总时长上限。每 500 ms 左右回报一次当前状态。画面没有有效物料时继续等待，有物料运动或证据不足时继续等待，当前可见物料全部满足停稳条件时返回终包并退出。

收到 `R` 立即中断任务并回复 `READY`，任务期间收到其他指令按现有 TaskCtl 回复 `BUSY`。应用退出时返回 `NONE,J`。J 不接受参数，`J,1` 等带参数请求回复 `ERR,j_arg`。

J 只执行图像采样、显示、日志和状态回包。该任务内没有车辆移动指令。

## 数据帧

外层继续使用现有 `AA 55 LEN payload CRC8 0D`，CRC8 对 LEN 和 payload 计算，poly 为 0x07。GPIO I²C 的 32 字节传输单元由现有传输层分片，接收方需完整组帧后解析 ASCII payload。

payload 结构如下。

```text
MSTATE,done,n,color1,x1,y1,state1,color2,x2,y2,state2,...
```

| 字段 | 取值和含义 |
| --- | --- |
| done | 0 为过程包，1 为终包 |
| n | 该帧报告的颜色数量，0～4 |
| color | 1 红、2 黄、3 蓝、4 绿，按颜色码递增排列 |
| x、y | 物料中心，对外图像坐标口径，正常配置为 640×480 |
| state | 0 运动中、1 已停稳、2 证据不足 |

总字段数为 `3 + 4*n`。done 为 1 时 n 必须大于 0 且每个 state 都为 1。同一包每种颜色最多出现一次。最多报告四种颜色，n 表示颜色数量。

没有有效物料的过程包如下，任务继续等待。

```text
MSTATE,0,0
```

红色物料正在运动、蓝色物料已经停稳的过程包如下。

```text
MSTATE,0,2,1,320,240,0,3,450,210,1
```

新出现的红色物料还没有足够连续观察记录。

```text
MSTATE,0,1,1,320,240,2
```

两种物料全部停稳的终包如下，发送后本次 J 退出。

```text
MSTATE,1,2,1,320,240,1,3,450,210,1
```

## 停稳判定

使用最近至少 300 ms 的连续采样，至少需要 3 帧。将中心坐标换算成现有对外坐标后，计算最近窗口的 X、Y 极差所形成的对角线长度，长度不超过 3 px 则判为停稳，超过则判为运动中。

采用窗口内的运动范围，物料运动一圈后回到原点仍会被识别为运动。判定针对物料相对图像的位置变化，测量时摄像头应保持固定。

以下情况报告证据不足，并清除或重新积累对应颜色的运动窗口。

- 首次出现，连续观察时长或帧数不足。
- 相邻有效帧间隔超过 150 ms。
- 检测框距任意画面边缘不足 8 px，物料可能未完整入镜。
- 同一帧出现多个相同颜色候选，无法确定是否一直在跟踪同一个物料。包中展示面积最大者的中心，state 为 2。

某颜色从当前帧消失时，清除该颜色的跟踪记录。终包代表结束当帧可见的全部有效颜色都已停稳。已经离开画面的物料不计入该帧的数量。

识别的颜色采用现有 ColorClassifier 的最终结果，包括现有 AI 阈值和 CV 回退策略。没有被现有分类器认可的对象不会计入颜色数量。

## 配置

```python
MATERIAL_STATE_REPORT_MS = 500
MATERIAL_STATE_HOLD_MS = 300
MATERIAL_STATE_TOL_PX = 3.0
MATERIAL_STATE_MIN_FRAMES = 3
MATERIAL_STATE_MAX_GAP_MS = 150
```

REPORT_MS 是过程包间隔，HOLD_MS 是停稳判断窗口。这些配置均不设置任务总超时。如果有效识别帧率低于约 7 fps，需要结合现场测量调整 MAX_GAP_MS。

## 文件和验证

- `maixcam_app_v2/material_state.py` 实现按颜色使用的连续运动窗口。
- `maixcam_app_v2/tasks.py` 的 `task_material_state` 实现持续采样、过程包、终包和屏幕显示。
- `maixcam_app_v2/protocol.py` 新增 J 命令常量和 MSTATE 打包函数。
- `maixcam_app_v2/main.py` 新增 J 命令分发入口与启动提示。
- `maixcam_app_v2/config.py` 增加五个 J 配置项。
- `maixcam_app_v2/test_material_state.py` 为无硬件测试。

在仓库根目录运行。

```powershell
python -X utf8 vision-maixcam/maixcam_app_v2/test_material_state.py
python -X utf8 vision-maixcam/maixcam_app_v2/selftest_tasks.py
```

专项测试覆盖静止、运动、往返运动、采样断档、同色歧义、包字段与 CRC8、超过 8 秒继续观察、空画面等待、贴边等待、R 中断和 main.dispatch 的正常 J 入口及非法参数入口。

后续如部署，应备份当前应用，将上述运行文件更新到 MaixCAM 后重启视觉应用，再通过真正的 STM32 J 请求验证完整组帧、过程包和终包。当前保留的 H 目标 Y 为 318。顺逆时针版已同期部署并启用，方向不足时兜底为逆时针 1。

## 本次部署记录

直接文件发布目录为 `vision-maixcam/releases/20261122-j-h-direction/`。已核对 7 个运行文件哈希，并确认启动 READY 的 I²C write_ok。备份位于板端 `/root/yyb-before-3.9.97-20261122-222610`。此次保留现场 GPIO I²C，H 目标 Y=318，D dy=40，E dx=0。
