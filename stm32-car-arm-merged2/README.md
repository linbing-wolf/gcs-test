# 底盘＋机械机构独立合并工程

**当前版本：20261019-merge2-r10。新增粗加工D的150mm逐环索引、按任务码物块顺序放下并抓回、最后回2号环基准；保留当前底盘闭环、路线、原料区、暂存区及机械参数。粗加工姿态和平台取料仍未标定，READY=0，到D会保护停车；[粗加工配置与操作](docs/粗加工环位作业合并.md)。r8增加起步编码器方向确认，r9使任务码只对一整轮两批作业有效，返程过B不重扫，整轮完成或取消后需新任务。机械臂两轴300RPM、夹爪0.5秒、云台/平台2秒插值，英文串口及原抓放参数保持。[完整操作与标定要求](docs/merged2功能合并与自动作业.md)。全程只使用 `runimu A/C`；同向直行节点不停，串口 `obstacle` 可在普通道路退回上一已记录节点并停等，工位作业/索引中遇障碍直接取消停车。尚无自动障碍输入，保留 `CAR_RUN_RPM=3000`。[连续直行与退回边界](docs/连续直行与障碍退回.md)。构建和电脑回归通过不代表烧录或实车验收。**

本目录是2026-10-17新建的完整工程，含底盘、机械臂、夹爪、云台及载物台。原 `stm32-car-test` 的应用文件保持任务开始时的状态。本地工作目录仍为 `stm32-car-arm-merged`；本次按操作者指定上传至GitHub的 `stm32-car-arm-merged2/`，原底盘位于 `stm32-car-test/`，各项独立测试位于 `test-code/`。

**打开 [keil/TableNavigation.uvprojx](keil/TableNavigation.uvprojx)，目标 `TableNavigation`。现场操作先看本文，再查 [串口命令速查与故障处理](docs/串口命令速查.md)；完整命令、互锁和参数来源见 [合并与串口操作](docs/合并与串口操作.md)。** SDK、离线查表和驱动均已包含，可单独复制本文件夹使用。工具链沿用ARM Compiler 5.06 update 5、STM32F4xx_DFP 3.1.1；不要切换到Compiler 6。

- 保留底盘原有IMU闭环、A/C全程路线、地图/查表、扫码和PC9按钮行为。
- 新增 `arm ...` / `claw ...` 串口命令。底盘运动与机械机构会话互斥，四轮反馈确认停稳后才启用机械机构。
- `!`始终提出全系统停止请求，不沿用独立测试程序的夹取快捷键。上电不启动路线、不使能5/7号机械电机、不输出三路舵机PWM。
- `runimu A/C` 默认自动到站作业；粗加工/暂存/平台取料姿态尚未标定，目前会拒绝对应动作。`station auto off` 可仅跑底盘，串口屏未在本次接入。

## 1. 打开、编译和下载

1. 克隆或下载整个仓库，进入本文件夹，用Keil打开 `keil/TableNavigation.uvprojx`，选择 `TableNavigation` 目标。
2. 使用ARM Compiler 5，执行 **Rebuild all target files**。当前工程为70个编译单元，构建结果见 [当前验证](evidence/current-validation.json)。
3. 下载本目录 `keil/OBJ/table_navigation.hex`，或下载自己刚编译的固件。源码改过就重新编译，不使用旁边原底盘工程的固件。
4. 下载后复位并运行芯片。串口接收编码选择ASCII或UTF-8，应出现 `merged chassis + arm serial control; ! always emergency stop`，并看到 `[drive] version=20261019-merge2-r10`。只显示发送记录不代表板子已回复。

工程预设CMSIS-DAP/SWD、STM32F407ZG，按实际下载器选择设置。随包固件通过软件验证，本次未由代理执行烧录或实车动作。

## 2. 接线和串口设置

| 功能 | 接线/地址 |
|---|---|
| 电脑调试USART3 | MCU PB10 TX→USB转串口RX；MCU PB11 RX←USB转串口TX；GND共地，3.3V逻辑 |
| CAN1 | PA12 TX / PA11 RX接CAN收发器；500kbit/s，扩展帧 |
| 电机地址 | 底盘1～4；升降5；平推7 |
| 舵机信号 | PA1夹爪、PA2载物台、PA3云台；电源按本车已验证接线，信号共地 |
| HWT101航向 | USART2，PD5 TX / PD6 RX，115200 |
| XM1601扫码 | USART1，PB6 TX / PB7 RX，暂定9600；PC0低有效触发 |
| 开始按钮 | PC9，按下高电平；只打印提示，不发车 |
| 视觉UART5 | PD2 RX←上位机TX；PC12 TX→上位机RX；115200、AA55+CRC8；替换原9字节接收协议 |

串口助手设置为 **115200、8数据位、无校验、1停止位、无流控、文本/ASCII发送**，关闭循环发送；普通命令以换行结束，LF或CRLF均可。`!`无需换行。

**下文代码框每一行都是一次单独发送。不要把多行动作一次粘贴发送；先等当前动作的完成反馈、检查实物，再发下一条。**

## 3. 上电先查询

```text
info
imu
feedback
qr
```

`imu`应显示 `uart_enabled=1 fresh=1`，采样数增加，转动车身时角度变化。`qr`返回 `[scanner]` 状态，配置应答通过后才是READY；扫码文本打印 `[scan] ...`，现在可保存为任务码并让B点扫码等待继续，但扫码本身不会启动整圈路线。上电不会自动发车，机械5/7号不使能，三路舵机PWM关闭。

## 4. 底盘怎么操作

手动底盘测试仍要求退出机构会话。自动 `runimu` 则要求机构会话有效、真实姿态已声明、已收拢且平台在1号位；具体准备步骤见 [自动作业说明](docs/merged2功能合并与自动作业.md)。

| 目的 | 逐条发送 | 完成/前置条件 |
|---|---|---|
| 检查60mm横移 | `test` → `dist 60` → `rpm 30` → `l`或`r` | 新鲜IMU；等 `DONE_BY_IMU_ENCODER`，再检查真实距离 |
| 前后位置单段 | `test` → `dist 60` → `rpm 30` → `f`或`b` | `f/b`仍是位置模式；估时提示不能当作实际停稳或到位 |
| 原地左转90° | `turn` → `rpm 60` → `left90` | 等 `DONE_BY_IMU`；其他角度见速查表 |
| 右上启停区1整圈 | `runimu A` | 初始车头向下、朝扫码区；起步右移60mm |
| 右下启停区2整圈 | `runimu C` | 初始车头向上、朝扫码区；起步左移60mm |
| 停止 | `!` | 不等换行，提出底盘和机构停止请求 |

A/C固定任务路线为扫码B→原料F→粗加工D→暂存H→F→D→H→起点。**默认到站执行工位作业；行驶和机构动作互斥。只测底盘时先退出机构会话并发送 `station auto off`，再发送 `runimu A/C`。** 全程或单动作结束后检查实物；归位偏差、真实角度和间隙仍待现场验收。旧 `run A/C`、`tl/tr`、`pulses N`、`ccw90 N/cw90 N` 已移除；定角调试保留 `turn` 后的 `left90/right90/left180/right180`。清理范围见 [runimu全程入口与旧代码清理](docs/runimu全程入口与旧代码清理.md)。

## 5. 机械机构首次小步测试

先停住底盘，在串口发送：

```text
arm on
```

等到 `[arm interlock acquired: ...]`，再发送：

```text
arm status
```

应看到 `[arm] enabled=1` 和 `[arm interlock] state=2`。`arm on`只启用5/7号并建立独占，舵机仍不输出PWM。它要求底盘四轮全部在线、使能正常且反馈零速；任何一轮失联都不会放行。

随后声明机构**当前实际位置**：`arm pose 平推毫米 升降毫米 云台角度 夹爪角度`。这不会移动两轴或清编码器，但会直接写云台/夹爪PWM；不是回零命令。只有实际处于收回0、最高135、云台0、夹爪45时，才发送这个例子：

```text
arm pose 0 135 0 45
```

回复 `arm pose declared by operator; NOT automatic homing`。当前云台0度对应约1975us，与旧独立测试工程的1586us不同。实际姿态不明时先检查和建立基准，不直接套用示例。

按上述基准，依次做小步检查：

| 发送一条 | 应观察到什么 |
|---|---|
| `arm slide 1` | 平推向外1mm，等 `[arm done: ...]` |
| `arm slide 0` | 平推回到收回0，等完成 |
| `arm lift 130` | 从135下降5mm，等完成 |
| `arm lift 135` | 回最高135，等完成 |
| `arm close` | 夹爪合到0度，等完成并观察 |
| `arm open` | 夹爪张开45度，等完成 |

所有正常机构动作均要求5/7号反馈有效。`arm done`表示两轴反馈条件和舵机插值结束，没有夹持或舵机角度传感器，仍须看实物。动作中其他机构命令会报busy。

## 6. 夹取、放平台和平台换位

小步动作确认正确，机构在收回0、最高135、云台0后：

1. 发 `arm pick`。程序开爪→云台131°→高位伸55mm→降至105mm→合爪15°。等 `arm done`，确认物块夹稳；此时不会自动回收或松爪。
2. 放料前声明载物台实际角度。**只有平台确实在20°初始档时**，发 `arm platform pose 20`；它会立即写PA2 PWM。其他姿态按真实角度填写。
3. 发 `arm place`。程序保持夹持，先升135→收0→云台256°→伸35→降110→开爪45°。等完成并检查物块已放稳。
4. 发 `arm home`，等完成、确认收拢后，可用 `arm platform 2`切到第二档；另外两档为 `arm platform 1`、`arm platform 3`。平台档位角度为20/139/256°，沿用原驱动，实际落位仍需测量。

`arm cycle`把回收拢位、夹取、放平台连成一套，会开爪；手里已有物块时用单独的 `arm place`。平台不会在取放流程中自动轮换，也没有物块占位检测。

## 7. 回到底盘、急停与恢复

正常退出时，先处理好物块，再逐条发：

```text
arm home
arm off
```

必须等 `arm home`完成且检查机构收拢，才发 `arm off`；否则会提示 `park with arm home`。`arm off`关闭三路舵机PWM并释放底盘互锁，夹爪可能失去保持力，5/7号步进仍使能。然后可用 `test`、`turn` 或 `station auto off` 下的 `runimu A/C`；自动作业的 `runimu` 需要保留已声明、已收拢的机构会话。

`!`随时提出底盘和机构停止请求；`arm stop`在手动会话仅停止机构；自动行驶/工位期间会同时取消整圈作业并停车，保留故障锁。停止时保持最后PWM及步进使能，舵机仍可能朝最后目标运动。停止后的机械坐标基准失效，不能盲目重发整套动作。

全系统急停/底盘反馈故障后：检查实物→`arm on`重新等四轮停稳→按真实姿态重新 `arm pose ...`；平台也重新声明实际姿态。故障不会自动续跑。具体反馈含义和恢复方法见 [串口命令速查与故障处理](docs/串口命令速查.md)。

## 文件与验证

- [src/arm_config.h](src/arm_config.h)：最新机械参数；云台0/131/256度分别约1975/1247/553us；夹爪开45°/合15°约826/604us。
- [src/arm_control.c](src/arm_control.c)：非阻塞取放、两轴反馈和三路PWM插值。
- [src/main_table_nav.c](src/main_table_nav.c)：共享串口/CAN与底盘互锁。
- [evidence/current-validation.json](evidence/current-validation.json)：本次源码、测试、构建和固件一致性记录。
- `keil/OBJ/table_navigation.axf`、`.hex`、`.map`：本次完整构建的对应产物。
- `docs`其他说明和 `evidence/chassis-history`：继承的底盘说明及历史证据；涉及“当前”的历史表述以本README及合并说明为准。

电脑验证：

```powershell
.\tools\verify-heading.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-arm-integration.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-continuous.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-drive.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\verify-station-vision.ps1 -Gcc C:\msys64\ucrt64\bin\gcc.exe
.\tools\build-isolated.ps1 -CompilerBin D:\keil5\ARM\ARMCC\bin
.\tools\check-package.ps1
```

本次未烧录、未驱动车辆或机械机构。电脑模拟、ARM编译及AXF/HEX一致不代表夹牢、真实角度、道路位置或碰撞间隙已验收。
