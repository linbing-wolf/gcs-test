> 这是旧入口清理记录。20261018-merge2-r1 已接入自动工位；默认 `runimu` 需要保留收拢的机构会话，`station auto off` 才是只跑底盘。当前操作见 [自动作业说明](merged2功能合并与自动作业.md)。

# runimu全程入口与旧代码清理（20261018-imuonly1）

整圈路线只保留 `runimu A` / `runimu C`。发车前把车放在对应启停区，朝扫码区摆正，确认IMU新鲜、四轮反馈正常且机构会话已经退出。串口115200、8N1、无流控，文本命令加换行；`!` 无需换行。

## 已移除

- 旧 `run A` / `run C` 命令及其脉冲直行、固定脉冲转向、位置ACK/到位推进状态机。
- 原始 `tl` / `tr` 转向、`pulses N`、`ccw90 N` / `cw90 N` 命令。
- 上述路径专用的提交/转向函数、状态字段、超时与标定参数、测试。`motion_start_rotation` 专用接口及调用方一并移除。

这些旧命令在导航、平移测试、转向测试模式下均拒绝，不会自动替换成 `runimu` 或触发运动。`help` 不再列出旧入口。历史文档、证据和清理前备份仍保留，不把旧操作指南当作当前说明。

## 保留的操作

| 用途 | 命令与行为 |
|---|---|
| 完整路线 | `runimu A` / `runimu C`；保留工位朝向、同向直行过点、编码器距离和四轮反馈检查 |
| 障碍退回 | 运行中发送 `obstacle`，先停稳，再沿编码器记录反向退回上一已记录节点并停等；当前没有自动检测信号 |
| 定角调试 | `turn` → 可选 `rpm 30` → `left90` / `right90` / `left180` / `right180`；需要新鲜IMU，每次只执行一个闭环动作 |
| 平移调试 | `test`、`dist N`、`rpm N`、`f/b/l/r`；左右横移为IMU＋编码器闭环，前后单段位置测试仍保留 |
| 停止与诊断 | `!`、`info`、`imu`、`feedback`、扫码查询及按钮提示 |
| 机构 | `arm ...` / `claw ...` 和底盘互锁保持不变，仍不自动接入取放任务 |
| 人工导航 | `nav/pose/heading/go` 作为显式人工入口保留；共用地图、查表和停车代码没有整文件删除 |

`CAR_TURN_CCW_DIR` 仍供IMU四轮速度混控使用，属于有效方向映射。保留 `CAR_RUN_RPM=3000` 及既有控制参数；`rpm` 命令只改当前单步测试或人工导航速度，不改变 `runimu`。实际轮速仍受混控300RPM上限、斜坡和制动限制。

为了兼容现有日志读取方式，`[run]`、`[route segment]`、`DONE_BY_MOTOR` 等日志名称保留；这些名称并不表示旧 `run` 命令还存在。`imu_mode` 仅标记已建立闭环会话，供诊断和IMU故障检查使用，不再选择旧路线执行分支。

障碍退回结束进入 `WAIT_OBSTACLE`，不自动续跑。现场检查实际位置后，可以显式切到 `nav` 再人工处理；重跑整圈前必须摆回对应启停区。完整边界见 [连续直行与障碍退回](连续直行与障碍退回.md)。

## 验证与备份

清理前快照在 `evidence/remove-legacy-before-20261018T080925207915Z`，包含修改前源码、测试、说明、Keil配置和固件。当前结果及源码哈希见 [当前验证](../evidence/current-validation.json)。

```powershell
.\tools\verify-heading.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-continuous.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-drive.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-arm-integration.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\build-isolated.ps1 -CompilerBin D:\keil5\ARM\ARMCC\bin
.\tools\check-package.ps1
```

`verify-heading.ps1` 默认包含所有保留的入口测试，不再提供 `-SkipLegacy`。新测试还检查已删除命令在各模式下均拒绝且不发运动指令。

本次没有烧录、发送车辆命令或上传GitHub。模拟和编译不能证明实车精度，之前报告的IMU与真实车头5～10°差异仍未解决。
