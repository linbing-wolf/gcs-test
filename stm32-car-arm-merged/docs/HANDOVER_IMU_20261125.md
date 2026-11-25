# 底盘 IMU 闭环交接（2026-11-25）

## 一句话结论

四向"横移 + 原地转 90°"的闭环已经可用：转向角稳定在 90°±0.2°，1 m 与 2 m 边长都能按编码器收停，剩下的偏差是地面固定方向的偏流，已用按段注入的世界系补偿压住。加速度仍只用于"缺口"辅助，不作为里程或位姿来源。

## 当前固件与构建

- 工程：`stm32-car-arm-merged`，实验功能由 `-DCHASSIS_IMU_EXPERIMENT=1` 打开。
- 最近构建：`evidence/merged-keil-imu-experiment-20261125T073450223Z`（已烧录、已实跑）。
- 编译：

```powershell
.\tools\build-isolated.ps1 -ChassisImuExperiment `
  -CompilerBin 'C:\Users\linbing\AppData\Local\Keil_v5\ARM\ARM_Compiler_5.06u7\bin'
```

`D:\keil5\ARM\ARMCC\bin` 已失效；Keil 现在装在 `C:\Users\linbing\AppData\Local\Keil_v5`，AC5 在 `ARM_Compiler_5.06u7` 下。

- 烧录（CMSIS-DAP）：

```powershell
Copy-Item evidence\<构建目录>\table_navigation.hex tmp\table_navigation.hex -Force
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
  -c "adapter speed 100; program tmp/table_navigation.hex verify reset exit"
```

长路径或中文路径下直接 `program` 偶发失败，先复制到 `tmp/` 再烧；传输报错时把 `adapter speed` 降到 100。

## 硬件与串口

| 项目 | 配置 |
|---|---|
| 调试串口 | PB0=TX、PB1=RX 软件串口，115200 8N1，主机 `COM9` |
| IMU | JY901B，USART2：PD5=TX、PD6=RX，默认 115200 / 50 Hz |
| 六轴模式 | 航向基准取每段起步读数；`0x53` 航向、`0x52` 角速度、`0x51` 加速度 |
| 临时高速 | `imu baud 460800` 后 `imu rate 200`；不保存，测完 `imu rate 50`、`imu baud 115200` |

PB1 的 EXTI1 抢占优先级必须高于 USART2，否则 200 Hz 会打断软件串口采样并触发 `[RX overflow: stopped]`。

## 本轮修掉的问题

### 1. 转向"报完成但不转"（根因）

`run_velocity_update()` 里横移补偿的非横移分支每个控制周期都执行：

```c
run.heading_control.target_cdeg = run.lateral_heading_base_cdeg;  // 转向时该值为 0
```

转向例程的基准是 0，90° 目标一启动就被抹成 0，误差瞬间消失，车只抖几度就报 `DONE_BY_IMU`。现在两处赋值都加了 `if (run.translation_only)`，只允许平移例程锁定基准。

### 2. 每个角稳定少转约 0.5°

`CAR_HEADING_TOLERANCE_CDEG` 80 → 40。四角少转 0.5° 会让闭合回路整体偏 20~30 mm，收紧后实测 90.02°~90.18°。

### 3. 末端差几毫米原地不动

1 mm 完成容差把末端速度压到 3 RPM，克服不了静摩擦，编码器停在 993~997 mm 后永远不动，最后超时。现在 `CAR_DISTANCE_TOLERANCE_MM=6`、`CAR_DISTANCE_MIN_RPM=8`。

### 4. 加速度追加距离的正反馈

`extra_target_mm` 曾一路涨到上限 300 mm，目标从 1000 变成 1300，车越跑越长且末端一直爬。现在：

- 偏移小于 `CAR_ACCEL_DISTANCE_DEADBAND_MM=80` 视为积分漂移，不追加；
- 进入停车段立即冻结 `lateral_extra_frozen`，不再更新；
- `imutest extra 0/1` 可在标定时整体关闭。

### 5. 起步误判驱动故障

上一段转向的迟到状态帧会被新一段当成"驱动未使能/堵转"。现在起步 500 ms 内忽略该状态帧，真实堵转仍由轮速跟随判据拦截。

## 地面偏流的处理

判定实验：原地只转 4×90°，车基本没有位移；四向横移一圈则稳定偏向同一世界方向（连续两轮实测"往右"从 70 mm 只降到 67 mm，"往后"随转角修正从 40 mm 降到 20 mm）。说明偏流来自横移段的地面/轮胎侧滑，不是转向。

闭合回路有个数学约束：每段都在车体系里做相同的角度或距离修正，绕一圈会相互抵消，所以改航向偏置或统一加长都压不掉它。必须注入一个在世界系里方向固定的分量，并在每段按 \(90°\) 轮换到车体系。实现为 `imutest trim <前后> <横移>`（单位 0.1 RPM），按段注入：

| 段 | 前后 | 横移 |
|---|---:|---:|
| 1 | +0.8 | +2.7 |
| 2 | +2.7 | -0.8 |
| 3 | -0.8 | -2.7 |
| 4 | -2.7 | +0.8 |

补偿量随时间线性累积，与距离成比例，所以 1 m 和 2 m 共用同一组数值。这组数按当前场地标定，换场地要重标。

## 实测结果

1 m 四向（`evidence/imutest-cross-trim-20261125T074140.log`）：

| 段 | 编码器 | 转角 | 角误差 |
|---|---:|---:|---:|
| 1 | 1001 mm | 90.04° | +0.04° |
| 2 | 998 mm | 89.96° | -0.04° |
| 3 | 997 mm | 90.11° | +0.11° |
| 4 | 1000 mm | 90.11° | +0.11° |

编码器+陀螺仪推算闭合误差 18 mm。

2 m 四向（`evidence/imutest-cross-2m-20261125T074555.log`）：

| 段 | 编码器 | 转角 | 角误差 |
|---|---:|---:|---:|
| 1 | 1999 mm | 90.05° | +0.05° |
| 2 | 1999 mm | 90.02° | +0.02° |
| 3 | 1997 mm | 90.18° | +0.18° |
| 4 | 2000 mm | 90.08° | +0.08° |

推算闭合误差 4 mm。地面实测由人工卷尺记录：加世界系补偿前为"往后 20 mm、往右 67 mm"，补偿后肉眼观察已可接受，未再逐点复测。

## 调试脚本

| 脚本 | 作用 |
|---|---|
| `tools/run-imutest-cross.py` | 四向横移 + 转 90°，支持 `--distance`、`--trim`、`--no-extra` |
| `tools/run-imutest-turns.py` | 只原地转 N 次，用于分离转向带偏 |
| `tools/run-imutest-square.py` | 前后左右 f/r/b/l 方形 |
| `tools/analyze-cross-log.py` | 从日志积分每段位移与闭合误差 |

脚本失败即发 `!` 急停，正常结束会恢复 IMU 到 115200 / 50 Hz。日志写入 `evidence/`（仓库只保留 `*.log`）。

## 已知限制

- 加速度二次积分仍会随 \(t^2\) 发散，只用于"缺口"和动态观察，不参与里程或位姿。
- 世界系补偿是场地标定值，换地面、换地毯方向要重新标定；标定时先用 `imutest extra 0` 排除追加距离干扰。
- 四轮全向底盘横移的侧滑无法被轮速闭环观测，只能靠外部位置观测（光流/下视相机/UWB）真正闭环到绝对点。
- 航向只用六轴，长时间运行会有零偏漂移；目前每段起步重取基准，未做在线零偏估计。

## 后续建议

1. 换场地时跑一次 `run-imutest-turns.py` + `run-imutest-cross.py --no-extra --trim 0 0`，先确认偏流方向和量级，再按比例给 trim。
2. 若要把 2 m 的闭合进一步压到 10 mm 内，优先标定四轮 X/Y 的 `CALIBRATED_PPM_*` 与轮距，而不是继续加补偿。
3. 需要绝对定位时再引入下视相机或光流，轮式闭环到此为止。

## 回滚

本次改动集中在 `src/main_table_nav.c`、`src/car_config.h`、`tools/`、`docs/`。回滚到上一版行为只需要：`CAR_HEADING_TOLERANCE_CDEG=80`、`CAR_DISTANCE_TOLERANCE_MM=1`、`CAR_DISTANCE_MIN_RPM=3`、`imutest trim 0 0`、`imutest extra 1`。
