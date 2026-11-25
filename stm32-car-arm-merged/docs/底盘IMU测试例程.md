# 底盘 IMU 测试例程

实验动作由 `CHASSIS_IMU_EXPERIMENT=1` 启用。普通构建收到 `imutest` 会提示加载实验固件。实验需要手动下发动作，沿用底盘互锁及 `!` 急停。试跑前退出机械臂会话，记录测试方向和地面起止点。

## 本地构建

从合并工程目录运行以下命令。需要已授权的 ARM Compiler 5，`CompilerBin` 指向其 `bin` 目录。

```powershell
.\tools\verify-imu-experiment.ps1
.\tools\build-isolated.ps1 -ChassisImuExperiment -CompilerBin 'C:\Users\linbing\AppData\Local\Keil_v5\ARM\ARM_Compiler_5.06u7\bin'
```

实验构建输出到 `evidence/merged-keil-imu-experiment-*`。构建脚本不连接或烧录设备。

`D:\keil5\ARM\ARMCC\bin` 已失效，直接用上面的 `ARM_Compiler_5.06u7` 路径，否则 armcc 版本不一致。

## 串口命令

本地调试当前使用软件串口：PB0 接 USB 转串口 RX 作 TX，PB1 接 USB 转串口 TX 作 RX，共地，3.3 V 逻辑。终端使用 115200、8N1、无流控。`src/car_config.h` 的 `CAR_DEBUG_UART_SOFT_PB0_PB1=1` 启用该映射；设为 0 恢复 USART3 的 PB10/PB11。先打开终端再复位，预期依次出现 `[imutest v2 boot]` 和 `[imutest v2 ready]`。终端晚打开时，发送 `imutest` 并附 CR 或 LF，预期收到 `[imutest v2]` 及传感器状态。若 `info` 和 `imutest` 均无回包，检查转接器对应端口、接线和复位后固件运行状态。

| 命令 | 动作 |
|---|---|
| `imutest` | 显示档位、平移距离和限速、角速度、加速度和数据年龄 |
| `imutest dist 250` | 设置下次平移距离，允许 50–2000 mm，默认 1000 mm |
| `imutest rpm 15` | 设置下次平移主轴限速，允许 10–200 RPM，默认 40 RPM |
| `imutest pd` | 基线档。航向角差分 PD，原地转向启用取整修正 |
| `imutest gyro` | `0x52` Z 轴角速度低通后作阻尼。航向误差仍来自 `0x53` |
| `imutest pid` | 继承 `gyro` 档，前后及左右平移时加泄漏积分。原地转向维持 PD |
| `imutest kp 120` / `imutest ki 25` / `imutest kd 30` | 停车时设置 PID 平移增益，输入为实际增益乘 100，复位恢复默认值 |
| `imutest f` / `imutest b` | 按当前距离和限速前进或后退 |
| `imutest l` / `imutest r` | 按当前距离和限速向车身左侧或右侧横移 |
| `imutest turnl` / `imutest turnr` | 用当前档位左转或右转 90°，转速上限 30 RPM |
| `imutest trim 8 27` | 注入世界系偏流补偿，参数为前后/横移分量，单位 0.1 RPM，范围 ±10 RPM；下一段平移生效，`trim 0 0` 关闭 |
| `imutest extra 0` / `imutest extra 1` | 关闭/开启加速度缺口追加距离，默认开启；标定世界系补偿时关掉以免互相污染 |
| `imutest clear` | 静止时清除四轮堵转保护并重新使能；不代替排除机械卡滞 |
| `imu rate 50/100/200` | 临时写 JY901B 输出率，不发 SAVE；50 Hz 恢复，断电也恢复 |
| `imu baud 115200/230400/460800/921600` | 临时切换 JY901B 与 USART2 波特率，不发 SAVE；失败自动尝试回退，断电恢复 115200 |
| `!` | 急停 |

停车后发送距离和限速设置，复位恢复默认值。限速是平移主轴速度上限。实验平移按剩余距离、指令减速度和控制延迟计算制动速度，再叠加速度斜坡和航向修正；速度斜坡同时限制起步与换向。

IMU 发送 `0x53` 航向角、`0x52` 角速度及可选的 `0x51` 加速度。用 `imutest` 检查年龄与方向，验证车身左转时 Z 轴角速度和航向角变化同为正。`gyro` 和 `pid` 要求角速度年龄不超过 100 ms。试跑中丢失角速度时，停车并退出本次试验，恢复数据后重新发送动作命令。

JY901B 的传感器口为 USART2，当前默认使用 115200。模块设为 50 Hz 时，连续两次查询的 `gyro_ms` 差除以 `gyro_seq` 差，应约为 20 ms。六轴模式的航向基准取每趟起步时的读数。115200 下 `imu rate 200` 实测只有约 70 Hz，并触发过调试串口 RX 溢出；先发 `imu baud 460800` 再发 `imu rate 200`，静止实测约 200.6 Hz、校验错误 0。调试串口仍保持 115200，只有 JY901B 的 USART2 改变。测完依次发 `imu rate 50`、`imu baud 115200`，不要发保存命令。原始记录见 `evidence/imu-460800-200hz-20261125T013100Z.log`。

200 Hz 下还必须保证 PB1 软件串口采样不被 USART2 抢占：PB1 的 `EXTI1` 抢占优先级为 0，USART2 调为 1。修复前 200 Hz 会在 `imutest kp` 等命令处触发 `[RX overflow: stopped]`；修复后连续 40 条查询全部回包、无 RX overflow，实测 200.5 Hz。前进 2 m 重复性对比：50 Hz 为 2000 mm / 峰值 0.26° / 末端 0.14°，200 Hz 为 1998 mm / 峰值 0.27° / 末端 0°。原始记录见 `evidence/imu-200hz-irqfix-stress-20261125T015700Z.log` 和 `evidence/imutest-forward-2m-200hz-irqfix-20261125T015900Z.log`。

## 参数与记录

初始参数位于 `src/car_config.h` 中的 `IMUTEST_` 宏。Z 轴按 `CAR_IMU_YAW_SIGN` 修正安装方向，静止零偏初始值为 0，可按静止记录调整。角速度低通时间常数为 50 ms。泄漏积分 `Ki` 为 0.25 RPM/(°·s)，泄漏时间常数为 2 s，贡献上限为 ±3 RPM。编码器方向确认且当前平移主轴速度达到 10 RPM 才累计。横移时，限幅和抗饱和使用左右轴指令 \(v_L\)。

$$
u_{\max}=\min\bigl(20,\max(0,|v_L|-1)\bigr)\ \mathrm{RPM},\qquad
I_k^{*}=\operatorname{clip}_{[-3,3]}\bigl(\alpha I_{k-1}
+(1-\alpha)K_i\tau_I e_k\bigr),\quad \alpha=e^{-\Delta t/\tau_I}.
$$

若积分候选值使输出继续向同方向饱和，只保留 \(\alpha I_{k-1}\)。停止横移后清除积分，末端使用停车对齐逻辑。

## 运动中纠偏闭环

横移和前后平移在 `velocity_phase == VELOCITY_FORWARD` 时叠加两层不停车补偿：

| 字段 | 公式 | 上限 |
|---|---|---:|
| `course_rpm` | 横移 `-v·sin(Δψ)`、前后 `+v·sin(Δψ)`，`Δψ=clip(e_θ,±15°)` | ±20 RPM |
| `slip_rpm` | `e=v_obs-v_des`，`s_k=αs_{k-1}+(1-α)e`，`u=clip(-K·s,±12)`，`α=e^{-Δt/80ms}`，`K=1.0` | ±12 RPM |

四轮合成：`v_f=(-v1+v2+v3-v4)/4`，`v_l=(-v1-v2+v3+v4)/4`。反馈失效或过期（`CAR_VELOCITY_FEEDBACK_MS=300 ms`）时本次不修正；四轮最大幅值 <10 RPM 也不修正。

叠加方式：横移 `u_f += u_course + u_slip`；前后 `u_l += u_course + u_slip`。指令归零或离开前向阶段时清零。

牵引同步：横移中 `max|v_i|≥60` 且 `min|v_i|<0.60·max|v_i|` 时，整体速度压到 `max(20, min|v_i|+8)` RPM，只降速不刹车。

末端阻尼：横移指令归零但残余轮速 `r>5 RPM` 时，航向修正限到 `min(8, r/2)` RPM；低于 5 RPM 立即释放，避免重新驱动。

这些闭环修的是航向一致、轮速一致和轨迹斜率，**不能**从绝对位置把已经斜滑出去的车拉回原点。后者需要光流、下视相机或 UWB 等非轮式位置观测。

## 日志字段

运动日志中的 `gyro_raw_cdeg_s` 是转换单位和方向后的未滤波值，`rate_cdeg_s` 是控制器使用的角速度，`i_mrpm` 是积分贡献，`accel_mg` 是原始传感器轴上的三轴加速度。`accel_dyn_mg = |√(ax²+ay²+az²)-1000|` 是去掉重力模长后的动态分量，只用于观察起步、制动、冲击和打滑。

加速度保留重力分量与安装倾角投影，二次积分会随 \(t^2\) 发散；实测 2 m 右移积出约 \(-2913\) mm / \(+2573\) mm，不可用。里程和位姿不使用加速度积分。`course_rpm`、`slip_rpm`、`wheel_vf`、`wheel_vl` 分别记录本次交叉轴前馈、反馈修正和四轮合成速度，用于判断闭环是否及时响应。

同一地面、同一起点和同一方向各跑几次 `pd`、`gyro`、`pid`。左右横移分别比较实测横向距离、前后窜动、航向误差和积分贡献。PID 可以补偿持续偏航，横移距离还受 X 轴标定和轮滑影响，前后窜动还受经验补偿影响。用地面标记测终点，编码器距离与实测距离分别记录。

## 四向闭环测试脚本

以下脚本都通过 PB0/PB1 调试串口驱动实验固件，失败即发 `!` 急停，结束时自动把 JY901B 恢复到 `115200`、`50 Hz`。`--port` 默认 `COM9`。

| 脚本 | 用途 |
|---|---|
| `tools/run-imutest-cross.py` | 四向"横移 + 原地左转 90°"，走完应回到起点附近；`--distance` 设边长，`--trim` 注入世界系补偿，`--no-extra` 关闭加速度追加距离 |
| `tools/run-imutest-turns.py` | 只做 N 次原地 90°，用来分离"转向带偏"和"横移滑移" |
| `tools/run-imutest-square.py` | 传统前后左右 f/r/b/l 方形标定 |
| `tools/analyze-cross-log.py` | 解析运动日志，按航向和编码器里程积分出每段位移与理论闭合误差 |

典型命令：

```powershell
python .\tools\run-imutest-cross.py --distance 2000 --no-extra `
  --trim "8 27 27 -8 -8 -27 -27 8" --log evidence\imutest-cross-2m-<时间戳>.log
python .\tools\analyze-cross-log.py evidence\imutest-cross-2m-<时间戳>.log
```

## 末端收停与世界系补偿

末端曾出现"差几毫米原地不动"：1 mm 完成容差把速度压到 3 RPM，克服不了静摩擦。现参数：

| 宏 | 值 | 作用 |
|---|---:|---|
| `CAR_DISTANCE_TOLERANCE_MM` | 6 | 完成容差，避免末端无限爬行 |
| `CAR_DISTANCE_MIN_RPM` | 8 | 末端最低平移速度，能真正推动车 |
| `CAR_HEADING_TOLERANCE_CDEG` | 40 | 转向完成容差 0.4°；0.8° 时每角稳定少转约 0.5°，四个角叠加出几十毫米闭合误差 |

地面在固定世界方向存在约 1.7% 的偏流（同向横移 4 次会累积 60~70 mm，原地转 4 圈则不产生位移）。闭环本身无法消除：任何"每段都在车体系里相同"的角度或距离修正，绕一圈都会相互抵消。做法是按段注入一个车体系的小速度分量，使它在世界系里指向同一个方向：

$$
\mathbf{u}^{trim}_k=\frac{d_{trim}}{L}\cdot v\cdot
\bigl(\cos(\psi_k-\psi_{trim}),\ \sin(\psi_k-\psi_{trim})\bigr)
$$

其中 \(\psi_k\) 是第 \(k\) 段的航向，\(\psi_{trim}\) 是偏流的反向世界角，`imutest trim` 直接给前后/横移两个分量的十分之一 RPM。补偿量随时间线性累积，因此 1 m 和 2 m 可以共用同一组数值。

加速度缺口追加距离（`CAR_ACCEL_DISTANCE_*`）另有一层保护：偏移小于 `CAR_ACCEL_DISTANCE_DEADBAND_MM=80` 不追加，且一进入停车段就冻结目标，避免"追加目标→多跑→积分再涨→再追加"的正反馈。它仍受积分漂移影响，标定世界系补偿时用 `imutest extra 0` 关闭。
