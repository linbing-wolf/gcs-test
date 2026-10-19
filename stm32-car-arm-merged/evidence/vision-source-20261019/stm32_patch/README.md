# STM32 侧改动清单（视觉口 → UART5）

> ## ⚠️ 本文已过时（协议变了）
> 本目录的 `vision_uart.c/h` 实现的是**旧协议**（`0xFF` + 9 字节 + `0xFE`，单字节命令字），
> 对应的是树莓派时代的视觉端。当前视觉端是 **v3.3.0**，用**变长帧**协议：
> `AA 55 | LEN | payload | CRC8 | 0x0D`，命令是 ASCII 文本（`R` / `A` / `T,134+123+314+231` / `C,1` / `D` …）。
>
> **请改用 `test-code/stm32/vision_proto.c/h`**（已按 v3.3.0 实现并在电脑端自检通过，
> 见 `test-code/stm32/README_视觉联调.md`）。
> 接线结论仍然有效：**MaixCAM2 ↔ STM32 UART5（PC12=TX / PD2=RX）**。
> 精确协议以 `AI_HANDOFF_STM32.md` 为准。

> 配套：MaixCAM2 视觉端已完成（`maixcam_app/`）。
> 本文档是 **STM32 固件** 需要做的全部改动。
> 目标：让 STM32 通过 **UART5（PC12=TX / PD2=RX）** 和 MaixCAM2 通信，协议与原树莓派版本完全一致。

---

## 0. 一句话总结

**加两个文件 + 改两行初始化 + 全局替换 97 处名字。** 协议和坐标换算逻辑一个字都不用改。

| 类别 | 数量 | 说明 |
| --- | --- | --- |
| 新增文件 | 2 个 | `vision_uart.c/.h`（UART5 驱动 + 帧状态机） |
| 从工程移除 | 1 个 | `lanya/usart5.c` —— **必须移除**，否则中断函数重复定义 |
| 初始化 | 2 行 | 删 `uart5_init(9600)`，加 `Vision_Uart_Init(115200)` |
| 全局替换 | 97 处 | 三个变量/函数改名（见 §3.3） |
| 建议顺带修 | 4 处 | 工程自带的严重缺陷（见 §5） |

---

## 1. 硬件前提

### 1.1 接线（MaixCAM2 UART4 → STM32 UART5）

| MaixCAM2（底部 6P 1.25mm 座 · UART4） | 方向 | STM32 | 板上位置 |
| --- | --- | --- | --- |
| **A21**（UART4_TX） | → | **PD2**（UART5_RX） | 集成板 U8 座 |
| **A22**（UART4_RX） | ← | **PC12**（UART5_TX） | 集成板 U8 座 |
| **GND** | — | GND | — |
| 供电 | — | MaixCAM2 用 Type-C 独立供电 | 左侧座只有 3V3，**别接 STM32 的 5V1** |

- 两边都是 **3.3V TTL 电平，直连**，不要加电平转换板
- **GND 必须共地**，否则偶发乱码
- 波特率两端都必须是 **115200**（MaixPy 官方只保证 115200 可靠）

### 1.2 为什么用 UART5 而不是 USART1

| 串口 | 引脚 | 原用途 | 现在给谁 |
| --- | --- | --- | --- |
| USART1 | PB6/PB7（板上接线） | 原树莓派视觉 | **扫码模块** |
| USART2 | PD5/PD6 | IMU（115200） | 不动 |
| **UART5** | **PC12/PD2** | 蓝牙 HC-05（死代码，从未被主流程调用） | **MaixCAM2 视觉** |
| USART6 | PC6/PC7 | 串口屏 TJC | 不动 |

选 UART5 的理由：板上已引到 U8 座、蓝牙本来就没接入主流程、不用新占引脚、不影响 IMU 和串口屏。

---

## 2. 第 1 步：新增 UART5 视觉驱动

### 2.1 ⚠️ 先移除 `lanya/usart5.c`（否则编译报重复定义）

工程里 `lanya/usart5.c` 已经有一份 UART5 驱动，其中第 **70 行定义了 `UART5_IRQHandler`**：

```c
// lanya/usart5.c:17   已有 UART5 初始化（引脚和我们完全一样）
void uart5_init(u32 bound) {
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);   // PC12 = UART5_TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);   // PD2  = UART5_RX
    ...
}
// lanya/usart5.c:70   已有中断服务函数 —— 和我们的会冲突
void UART5_IRQHandler(void) { ... }
```

**必须处理它，二选一：**

| 做法 | 操作 | 评价 |
| --- | --- | --- |
| **推荐：从工程移除** | Keil 左侧工程树 → 展开 `lanya` 分组 → 右键 `usart5.c` → **Remove File from Group**（文件仍在磁盘上，只是不参与编译） | 干净；蓝牙本来就没用 |
| 备选：改它 | 直接把它改成我们要的样子（见 §2.3） | 不推荐，会在 `lanya/` 目录留一个视觉驱动，混乱 |

**移除后要同步删掉 `main.c:73` 的调用**：

```c
// main.c:73  删掉这一行
uart5_init(9600);//蓝牙
```

> 检查有没有别处引用 `lanya/usart5.h` 的 `UART5_SendString` / `UART5_SendArray` / `fasong_task_code`：
> 经核对 `main.c` 里这三个函数**出现 0 次**，只有 `uart5_init` 被调用过 1 次，所以移除是安全的。
> 如果编译提示别的文件引用了 `usart5.h`，把那些 `#include` 也删掉。

### 2.2 新增 `vision_uart.h`

放到 `SYSTEM/` 目录下（和 `Serial.c` 同级）：

```c
/**
  ******************************************************************************
  * @file    vision_uart.h
  * @brief   视觉串口驱动（MaixCAM2 → STM32 UART5）
  *
  * 为什么需要这个文件：
  *   原工程把"视觉数据"接在 USART1 上，而这块自制集成板（集成板3.1）把
  *   MaixCAM2 的串口接到了 UART5：PC12=TX, PD2=RX（原理图上的 U8 座）。
  *   原 usart5.c 是给蓝牙 HC-05 用的死代码（find_congji/fasong_task_code 从未被调用），
  *   所以这里直接把 UART5 改造成视觉口。
  *
  * 协议：与原树莓派版本 100% 一致，STM32 侧解析逻辑不用改语义
  *       接收帧 = 0xFF + 9 字节数据 + 0xFE
  *       发送   = 单字节命令字（'1'~'9' / 'a'~'c' / 'r'）
  *
  * 接线：MaixCAM2 A21(UART4_TX) → PD2(UART5_RX)
  *       MaixCAM2 A22(UART4_RX) ← PC12(UART5_TX)
  *       共地，3.3V TTL 直连
  ******************************************************************************
  */

#ifndef __VISION_UART_H
#define __VISION_UART_H

#include "stm32f4xx.h"

#define VISION_BAUDRATE   115200   /* 注意：MaixPy 官方只保证 115200 可靠 */

/* 收到的 9 字节数据，下标含义与原 Serial_RxPacket 完全一致：
 *   [0..3] 坐标 x 的 4 位十进制 ASCII
 *   [4..7] 坐标 y 的 4 位十进制 ASCII
 *   [8]    颜色码：'1'红 '2'绿 '3'蓝 '4'未识别（ASCII，不是数字）
 *   任务码帧时：[0..2] 第一组顺序，[3] 分隔符，[4..6] 第二组顺序
 */
extern uint8_t Vision_RxPacket[9];

void Vision_Uart_Init(uint32_t baudrate);   /* 初始化 UART5 + 中断 */
uint8_t Vision_GetRxFlag(void);             /* 取标志位，读到后自动清零（同原 Serial_GetRxFlag） */
void Vision_SendByte(uint8_t byte);         /* 发一个命令字给 MaixCAM2 */
void Vision_ClearRxFlag(void);              /* 主动清标志（等价原 Serial_GetRxFlag() 的丢弃调用） */

#endif
```
### 2.3 新增 `vision_uart.c`

```c
/**
  ******************************************************************************
  * @file    vision_uart.c
  * @brief   视觉串口驱动实现（UART5: PC12=TX, PD2=RX）
  ******************************************************************************
  */

#include "vision_uart.h"

uint8_t Vision_RxPacket[9];
static uint8_t Vision_RxFlag = 0;
static uint8_t RxState = 0;
static uint8_t pRxPacket = 0;

/* --------------------------------------------------------------------------
 * 初始化 UART5 @ PC12(TX) / PD2(RX)
 *
 * 时钟对照（STM32F407，务必别抄错）：
 *   PC12 → GPIOC → AHB1
 *   PD2  → GPIOD → AHB1
 *   UART5 → APB1（不是 APB2！USART1/6 才是 APB2）
 * -------------------------------------------------------------------------- */
void Vision_Uart_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* PC12 复用为 UART5_TX */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    /* PD2  复用为 UART5_RX */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;          /* TX */
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           /* RX */
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    /* 接收中断 */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    /* 用子优先级 3：低于 PID(TIM4, 抢占1) 和 CAN 接收，避免打断运动控制 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(UART5, ENABLE);
}

/* --------------------------------------------------------------------------
 * UART5 中断服务函数
 * 状态机与原 Serial.c:157 的 USART1_IRQHandler 完全一致：
 *   RxState=0 等 0xFF → =1 连收 9 字节 → =2 等 0xFE → 置标志
 * -------------------------------------------------------------------------- */
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        uint8_t RxData = (uint8_t)USART_ReceiveData(UART5);

        if (RxState == 0)
        {
            if (RxData == 0xFF)          /* 等帧头 */
            {
                RxState   = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            Vision_RxPacket[pRxPacket++] = RxData;
            if (pRxPacket >= 9)          /* 收满 9 字节 */
            {
                RxState = 2;
            }
        }
        else if (RxState == 2)
        {
            if (RxData == 0xFE)          /* 等帧尾 */
            {
                Vision_RxFlag = 1;       /* 一帧完整数据到手 */
            }
            RxState = 0;                 /* 无论对不对都回到等帧头 */
        }
        else
        {
            RxState = 0;
        }
    }
}

/* 取标志位，读到后自动清零（与原 Serial_GetRxFlag 行为一致） */
uint8_t Vision_GetRxFlag(void)
{
    if (Vision_RxFlag == 1)
    {
        Vision_RxFlag = 0;
        return 1;
    }
    return 0;
}

void Vision_ClearRxFlag(void)
{
    Vision_RxFlag = 0;
}

/* 发一个命令字给 MaixCAM2（阻塞等 TXE，与原 Serial_SendByte 一致） */
void Vision_SendByte(uint8_t byte)
{
    USART_SendData(UART5, byte);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
}
```
### 2.4 加进 Keil 工程

1. 把两个文件复制到 `SYSTEM/` 目录
2. Keil 工程树 → 右键 `SYSTEM` 分组 → Add Existing Files to Group → 选 `vision_uart.c`
3. Options for Target → C/C++ → Include Paths 确认 `SYSTEM` 目录在内（原 `Serial.c` 就在这组，通常已经有）

---

## 3. 第 2 步：修改 `main.c`

### 3.1 加头文件（顶部 `#include` 区，约 1~17 行）

```c
#include "vision_uart.h"
```

### 3.2 改初始化（`main.c:73-75`）

```c
// 改前
uart5_init(9600);
uart6_init(9600);//串口屏用的UART6
Serial_Init();   //通讯串口1初始化，用UART1

// 改后
//uart5_init(9600);                      // 删掉（蓝牙，UART5 已由 vision_uart 接管）
uart6_init(9600);                        // 串口屏不动
Serial_Init();                           // 扫码模块用（注意 §5.1 的引脚问题）
Vision_Uart_Init(VISION_BAUDRATE);       // 新增：视觉口 UART5 @115200
```

### 3.3 全局替换 97 处（用 Keil 的在文件中替换，不要手工点）

在 `main.c` 里依次做三次替换（**区分大小写**）：

| # | 查找 | 替换为 | 数量 | 触发位置 |
| --- | --- | --- | --- | --- |
| 1 | `Serial_GetRxFlag` | `Vision_GetRxFlag` | 8 处 | `107, 114, 136, 140, 338, 343, 823, 826` |
| 2 | `Serial_RxPacket` | `Vision_RxPacket` | 85 处引用 / 52 行 | `96, 116, 117, 123, 124, 147, 155, 166, 169, 178, 181, 209, 220, 278, 289, 349, 357, 369, 372, 380, 383, 411, 422, 478, 485, 699, 700, 715, 716, 718, 722, 723, 725, 729, 730, 732, 736, 750, 751, 761, 763, 765, 776, 777, 779, 783, 784, 786, 790, 791, 793, 797` |
| 3 | `Serial_SendByte` | `Vision_SendByte` | 4 处 | `107, 136, 338, 824` |

替换完搜一遍确认：搜 `Serial_RxPacket` 应该是 **0 结果**。

> ⚠️ **只在 `main.c` 里替换，不要整个工程替换。**
> `SYSTEM/Serial.c` 里那些同名变量是它自己的定义和状态机，动了会出问题。
> 另外注意别误伤 `Serial_TxPacket`（发送用的，原工程有定义但没人调用，保持原样）。

---

## 4. 第 3 步：验证（按顺序，别跳）

| 步骤 | 操作 | 期望结果 |
| --- | --- | --- |
| 1 | 只上电 MaixCAM2，用 USB 转 TTL 并在它的 TX 上看 | 开机发一帧 `FF 39 38 37 36 35 34 33 32 31 FE`（就是 987654321） |
| 2 | 接上 STM32，在 `UART5_IRQHandler` 里下断点 | 收到帧尾 0xFE 时 `Vision_RxFlag` 置 1 |
| 3 | STM32 发 0x32（字符 2），MaixCAM2 对着色块 | MaixCAM2 打印 `[命令] '2'`，随后回一帧物块坐标 |
| 4 | 看 `Vision_RxPacket` | [0..3] 是 x、[4..7] 是 y、[8] 是颜色码 |
| 5 | 全流程空跑 | 不卡死（需先修 §5.2） |

**第 3 步的检查技巧**：`Vision_RxPacket[8] - 0x30` 应该是 1/2/3；
如果是 4，说明 MaixCAM2 没识别到（它回的是 `000000004`）。

---

## 5. 建议顺带修的 4 个严重缺陷

这 4 条不改，MaixCAM2 再稳也没用。全部来自工程自带的分析报告（`程序分析报告.md` §8）。

### 5.1 扫码模块的引脚对不上（必须确认）

原理图上扫码模块走 **USART1 = PB6/PB7**，但固件配的是 **PA9/PA10**：

```c
// SYSTEM/Serial.c:27-41  现状（对不上板子）
GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;

// 应改成
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);      // 时钟从 GPIOA 换成 GPIOB
GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);  // PB6 = USART1_TX
GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);  // PB7 = USART1_RX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_Init(GPIOB, &GPIO_InitStructure);
```

另外原理图上扫码模块有 **PC0 低电平使能**，确认这个信号现在怎么给的（硬件接地？软件拉低？）。

### 5.2 `main.c:83-91` 疑似调试残留（**最高优先级**）

```c
// main.c:83-91  这段让第 92 行起的整个比赛流程不可达
while(swith_out() == 1){          // 等开关
    LED_Toggle2();
    delay_ms1(50);
}
move_all(-120.0f,1000,200, 0,2000,200, 278.0f,13);
set_zhuashou_kai();
jixie_reset();                     // main.c:814 内部是 while(1){} ，永不返回
while(1){}                         // 又一层死循环
```

**处理**：不用于调试就删掉 83-91 这段（或整体注释掉）。
不修的话 MaixCAM2 永远等不到命令 —— 流程根本走不到发包的地方。

### 5.3 握手加超时（否则视觉丢一帧就永久卡死）

```c
// main.c:821-830  现状：time 参数声明了但从不使用
void wait_send_shumeipai(uint8_t message,uint16_t time)
{
    Serial_GetRxFlag();       // -> Vision_GetRxFlag()
    Serial_SendByte(message); // -> Vision_SendByte()
    while(1){
        if(Serial_GetRxFlag() == 1) break;   // 永远出不来
    }
}

// 建议改成（返回 1=成功 0=超时）
uint8_t wait_send_shumeipai_timeout(uint8_t message, uint16_t timeout_ms)
{
    uint16_t waited = 0;
    Vision_ClearRxFlag();
    Vision_SendByte(message);
    while (waited < timeout_ms) {
        if (Vision_GetRxFlag() == 1) return 1;
        delay_ms1(1);
        waited++;
    }
    return 0;   // 超时，让上层决定重试还是跳过
}
```

> MaixCAM2 侧**已经给每个任务加了超时**，超时会回 `000000004`。
> 但 STM32 侧自己不设超时的话，收不到字节它还是出不来。

### 5.4 USART6（串口屏）有中断使能但没有 ISR

```c
// ckp/uart6_ckp.c:52  使能了接收中断
USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
```

但工程里没有 `USART6_IRQHandler`，会落到启动文件的 `Default_Handler: B .`，
**串口屏只要回传一个字节，MCU 就死循环**。二选一：

```c
// 方案一：补一个 ISR，把数据读掉（推荐）
void USART6_IRQHandler(void)
{
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        volatile uint8_t dummy = (uint8_t)USART_ReceiveData(USART6);  // 读走就清标志
        (void)dummy;
    }
}

// 方案二：只有单向显示需求时，直接关掉接收中断
// 把 ckp/uart6_ckp.c:52 那行注释掉
```

### 5.5 其他已知问题（来自分析报告）

| 位置 | 问题 | 影响 |
| --- | --- | --- |
| `yyb_move.c:184, 217` | 升降/平推越界直接 while(1){} | 坐标异常就锁死 |
| `main.c:833` | TIM5 中断里整屏刷 OLED（软件 I2C，>30ms） | 影响实时性 |
| `can.c:12, 33` | `can_GetRxFlag()` 判断的变量和中断置位的不同，恒返回 0 | CAN 应答无法判断 |
| 全局 | fAngle / sReg / Serial_RxPacket 被多中断裸读写 | 数据撕裂风险 |

---

## 6. 改动前后对照

| 项目 | 改动前 | 改动后 |
| --- | --- | --- |
| 视觉口 | USART1（PA9/PA10，9600） | **UART5（PC12/PD2，115200）** |
| 收包变量 | `Serial_RxPacket[9]` | `Vision_RxPacket[9]`（下标含义不变） |
| 收包标志 | `Serial_GetRxFlag()` | `Vision_GetRxFlag()` |
| 发命令字 | `Serial_SendByte()` | `Vision_SendByte()` |
| 帧格式 | 0xFF + 9 字节 + 0xFE | **完全不变** |
| 颜色码 | 1红 2绿 3蓝 4未识别 | **完全不变** |
| 坐标换算 | `transfer_position_wukuai/sehuan` 的标定系数 | **完全不变**（摄像头仍装机构上俯视） |
| 蓝牙 | `lanya/usart5.c` 占着 UART5（死代码） | 从工程移除，UART5 让给视觉 |
| 扫码模块 | USART1 配错引脚（PA9/PA10） | 改为 PB6/PB7 对上硬件 |

---

## 7. 编译/调试报错速查

| 报错 | 原因 | 处理 |
| --- | --- | --- |
| `Symbol UART5_IRQHandler multiply defined` | `lanya/usart5.c` 还在工程里 | 执行 §2.1 的移除 |
| `Vision_RxPacket undefined` | 没包含头文件 | main.c 加 `#include "vision_uart.h"` |
| `undefined symbol uart5_init` | 移除了 usart5.c 但 main.c:73 还在调用 | 删掉那行 |
| `cannot open source input file usart5.h` | 别的文件还在 include 它 | 删掉那些 include（主流程不需要） |
| 编译过但收不到数据 | 引脚接反 / 没共地 / 波特率不一致 | 查 §1.1；先用 USB 转 TTL 验 MaixCAM2 的 TX 有输出 |
| `Vision_RxPacket` 一直是旧数据 | 标志位没清 | 处理完调 `Vision_GetRxFlag()`（读后自动清零） |

---

## 8. 最小改动清单（照着勾）

```text
[ ]  1. 从 Keil 工程移除 lanya/usart5.c
[ ]  2. 新增 SYSTEM/vision_uart.h
[ ]  3. 新增 SYSTEM/vision_uart.c，并 Add 进 Keil 的 SYSTEM 分组
[ ]  4. main.c 加 #include vision_uart.h
[ ]  5. main.c:73 删掉 uart5_init(9600);
[ ]  6. main.c 初始化段加 Vision_Uart_Init(VISION_BAUDRATE);
[ ]  7. main.c 替换 Serial_GetRxFlag -> Vision_GetRxFlag  (8 处)
[ ]  8. main.c 替换 Serial_RxPacket -> Vision_RxPacket  (85 处 / 52 行)
[ ]  9. main.c 替换 Serial_SendByte -> Vision_SendByte  (4 处)
[ ] 10. 编译通过
[ ] 11. 修 main.c:83-91 调试残留死循环（§5.2）  <-- 最高优先级
[ ] 12. 改 SYSTEM/Serial.c 的 USART1 引脚到 PB6/PB7（§5.1）
[ ] 13. 给 wait_send_shumeipai 加超时（§5.3）
[ ] 14. 补 USART6_IRQHandler 或关掉它的接收中断（§5.4）
[ ] 15. 联调：MaixCAM2 发就绪帧 -> STM32 收到 -> 发 2 -> 收到坐标
```

---

## 9. 参考

- MaixCAM2 侧代码：`../maixcam_app/`；迁移分析总报告：`../README.md`
- 工程自带 `yyb_stm32/程序分析报告.md` —— 协议表、硬件资源、风险清单，强烈建议通读
- STM32F407 参考手册：UART5 在 **APB1** 总线（USART1/6 在 APB2）；引脚复用见 datasheet Table 9
