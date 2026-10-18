# 上位机（MaixCAM2 视觉端）联调测试台

适用视觉端版本：**v3.3.0**；协议见 `vision-maixcam/AI_HANDOFF_STM32.md`。

这套代码的作用：**在 STM32 上模拟"整车"去和视觉端对话**，把视觉端的每一条命令、
每一个回复、每一次超时都摆出来，先把"上位机功能"单独测通，再谈接整车。
不走路线、不动机械臂。

## 0. 文件清单

| 文件 | 作用 |
| --- | --- |
| `vision_proto.h/.c` | **协议驱动**（帧/CRC/拆帧/解析/一问一答会话）。不依赖具体 main，可直接搬进整车工程 |
| `main_vision.c` | 测试台主程序（菜单、自动判定、统计、自检、回环自检、全流程脚本） |
| `vision_host_test.c` | 电脑端自检（**不需要硬件**，gcc 直接跑，153 条断言） |
| `README_视觉联调.md` | 本文件 |

## 1. 接线

```text
MaixCAM2 TX (A21, UART4_TX) ──► PD2  STM32 UART5_RX
MaixCAM2 RX (A22, UART4_RX) ◄── PC12 STM32 UART5_TX
GND ── GND（必须共地）
115200 8N1，无流控，3.3V 电平直连（不要接 5V）
```

调试打印 + 按键走 USART3（接集成板的 U4 座）：

```text
PB10 = USART3_TX（MCU 发）  PB11 = USART3_RX（MCU 收）  115200
```

> 为什么是 UART5：集成板 3.1 把 MaixCAM2 的串口就接到 UART5 的 PC12/PD2（原理图 U8 座）。
> 视觉端自己那边是 `/dev/ttyS4`（UART4，A21/A22）——两边都 3.3V，直连即可。
> 想换口只改 `vision_proto.c` 顶部那组宏。

## 2. 中断函数冲突（**必读，否则收不到数据**）

Keil 工程里已经有 `vendor/stm32/lanya/usart5.c`，它定义了 `UART5_IRQHandler`
（蓝牙 HC-05 的死代码）。同一个工程里不能有两个同名中断，二选一：

- **方案 ②（默认，改动最小）**：保留 `usart5.c`，在它的中断函数里加一行：
  ```c
  void UART5_IRQHandler(void)
  {
      if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
          Vision_IsrByte((uint8_t)USART_ReceiveData(UART5));   /* ← 加这一行 */
  }
  ```
- **方案 ①**：把 `usart5.c` 从工程里移除，并在工程预定义宏里加
  `VISION_PROVIDE_UART5_IRQ`，`vision_proto.c` 会自己提供中断。

两种方案都编译过一次（语法检查 + 分支检查都过）。

## 3. 怎么编译

### 3.1 板上（Keil）

1. 把 `main_vision.c` 当 `main.c` 用：根目录执行 `.\tools\switch-main.ps1 vision -Build -Download`
   （或手工把本文件拷成 Keil 工程的 `main.c`，并把 `vision_proto.c` 加进工程）。
2. 工程需要同时编译 `vision_proto.c`。

### 3.2 电脑端自检（**推荐先跑这个**）

```bash
cd test-code/stm32
gcc -DVISION_HOST_TEST -std=c99 -Wall -Wextra -Werror \
    vision_host_test.c vision_proto.c -o vision_host_test && ./vision_host_test
```

输出 `[ALL PASS]` 表示协议层（CRC/拆帧/解析/会话/全流程脚本）都对。
它覆盖了：交接文档 §2.1 的 5 条 CRC 实测向量、前置噪声、粘连帧、逐字节分片、
坏 CRC/坏帧尾/LEN 超限/半帧 200ms 丢弃、13 种回复解析（含负数 dx）、
BUSY 不算失败、MOVE 不当终态、超时重发 2 次、全流程 11 步。

## 4. 按键表与预期输出

| 键 | 动作 | 正常应该看到 |
| --- | --- | --- |
| `v` | 自检（**不用接视觉端**） | `[SELFTEST PASS]` |
| `l` | 回环自检（PC12↔PD2 短接） | `[LOOPBACK PASS]` |
| `r` | 发 `R` | 1s 内 `[<-] READY PASS` |
| `a` | 发 `A` | `START,区号,x,y,w,h PASS` |
| `t` | 发 `T,134+123+314+231` | `TASK,134,314 PASS` |
| `c` | 发 `C,<颜色码>`（颜色码由 `1..4` 设） | `PICK,色,x,y PASS` |
| `d` | 发 `D` | 先若干 `[move] dx=..`，最后 `DISC,x,y,dx,1 PASS` |
| `e` | 发 `E` | 同上 |
| `f` | 发 `F,<颜色码>` | `STACK,x,y PASS`（**只有两个数，没有高度**）|
| `g` | 发 `G` | `OBST,x,y,w,h,mm` 或 `CLEAR`，都算 PASS |
| `z` | 发 `Z` | `OK PASS` |
| `p` | 自动跑全流程（§7 的 11 步） | 逐步 `PASS`，最后 `[PLAN PASS]` |
| `s` | 把上一条命令连测 5 次 | `通过 n/5`，看成功率与平均耗时 |
| `m` | 连续 20 次 `G` | `OBST×n CLEAR×m` + 平均/最慢耗时 |
| `k` | 统计 | 帧/坏CRC/坏帧尾/噪声/半帧丢弃/BUSY/MOVE/超时/重发 |
| `o` | 上次的帧 | 原文 + 整帧 hex |
| `x` | 中止会话 | 发 `R` 打断视觉端，回到空闲 |
| `b` | 帧 hex 打印开关 | 开/关 |
| `1..4` | 设颜色码（红1 黄2 蓝3 绿4） | — |
| `0` | MOVE 换算演示 | 打印 dx→毫米（不真动电机）|

打印约定：

```text
[->] 握手指令  发: R                      ← 你要发的
      帧: AA 55 01 52 AC 0D               ← 实际线上字节（自检用）
[<-] READY   12ms  PASS                  ← 视觉端回的（含耗时与判定）
[move] dx=35 → 应前进 18 mm              ← 圆盘对齐的微调请求（不驱动，只打印）
```

## 5. 验收清单（按顺序做）

- [ ] `v` 自检 `[SELFTEST PASS]`
- [ ] `l` 短接 PC12-PD2，`[LOOPBACK PASS]`
- [ ] 接上视觉端（视觉端那侧先看它自己的日志：`[AI] 已加载 /load model from:...`）
- [ ] `r` → `READY`（连发 3 次内成功）
- [ ] `k` → `坏CRC=0 坏帧尾=0`（只要不为 0 就是线上/波特率问题，别往下做）
- [ ] `a/t/c/f` 各来一次，回复字段含义对得上实物（颜色码 1红2黄3蓝4绿）
- [ ] `d` → 出现 `[move]` 且最后 `DISC,...,1`；**ok=0 也能接受但要记录**
- [ ] `p` 全流程 `[PLAN PASS]`
- [ ] `m` 压力测：20 次 `G` 平均耗时 < 3s，`CLEAR` 居多
- [ ] `s` 连测：`C,<色>` 5 次成功率 ≥4（转盘停稳判定会拉长耗时，正常）

## 6. `d`/`e` 的 MOVE 闭环怎么标定（重要）

视觉端在圆盘对中时**自己**会发 `MOVE,<dx>`（`dx` = 目标在画面上的水平偏差像素，
本车把它映射成前进/后退）。收到 `MOVE` 后**不要**再发新的 `D`/`E`，等最终 `DISC`。
视觉端等 300ms 后自己重新测量，最多 20 次。

本测试台默认**只打印**换算结果：

```text
[move] dx=35 → 应前进 18 mm（只打印不驱动；毫米数是占位值，须现场标定）
```

标定方法：
1. 按 `d`，把每次 `[move] dx=` 和车**实际**往前后走的毫米数记下来（先在实车上把
   `motion` 走位接上，或人工推车量位移）。
2. `毫米/像素 = 实际位移 / |dx|`，取几次平均。
3. 改 `vision_proto.h` 里的 `VISION_MOVE_MM_PER_PX`。
4. **方向反了**（越调越远）：不要去翻 STM32 的符号，改**视觉端**的
   `config.ALIGN_INVERT = True`，让它发相反符号。

## 7. 常见故障对照

| 现象 | 先查什么 |
| --- | --- |
| 什么都收不到 | ① 中断没接（见第 2 节）② PC12/PD2 接反 ③ 没共地 ④ `l` 回环也不通 |
| `坏CRC` 一直涨 | 波特率不是 115200 / 两边时钟不对 / 线长或干扰；**不要**靠重发掩盖 |
| `噪声字节` 涨 | 波特率轻微不匹配或线上有别的设备在发 |
| 每次都超时 | 视觉端没在跑 / 它的 UART 口被占了（视觉端启动日志要看到 `pinmap ... UART4_TX OK`）|
| 收到 `BUSY` 刷屏 | 你在一条命令没结束时又发了下一条；等回复或用 `p` 自动跑 |
| 收到 `NONE,A` | 视觉端右侧没看到灰色道路且未稳定（要真实前视画面才判）|
| 收到 `NONE,C` | 转盘一直没停稳（`PICK_STOP_HOLD_MS` 判据），或颜色码不对（只能 1~4）|
| 收到 `ERR,camera` | 视觉端相机不可用，它会退出，检查它的摄像头连接 |
| `DISC` 的 `ok=0` | 20 次 MOVE 还没对中：多半是 MOVE 方向反了或 mm/px 差太多（第 6 节）|

## 8. 接进整车工程（只有三处改动）

`vision_proto.c/h` 是刻意写成独立的，整车工程接入只需要：

```c
/* ① 上电初始化（和 uart_init/clock_init_ms 放一起）*/
Vision_Init(115200);
Vision_ParserReset(&g_parser, clock_ms);

/* ② 主循环里收（放在读调试串口的同一轮，别用阻塞等）*/
(void)Vision_Pump(&g_parser, clock_ms);

/* ③ 用会话引擎发命令、等回复（一次只发一条）*/
Vision_SessionStart(&g_vsess, "C,1", clock_ms, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
switch (Vision_SessionPoll(&g_vsess, clock_ms, &msg)) {
case 1:  /* 终态：msg 是 PICK/DISC/START/...；NONE/ERR 也是终态，要按失败处理 */
         break;
case 2:  /* MOVE：这里做前后微调，别回帧，继续等下一个 MOVE/DISC */
         break;
case 0:  /* 还在等 */
         break;
default: /* -1：超时且重发用尽 → 走你的失败分支 */
         break;
}
```

要注意的三条语义（都在 `AI_HANDOFF_STM32.md`）：
1. **一问一答**：一条没回完不要发下一条，否则拿到的是上一个任务的结果。
2. **`BUSY` 不是失败**：会话引擎已经帮你重置计时了。
3. **`G` 可能出现多次 `OBST` 再 `CLEAR`**：整车侧要按"障碍物消失=可以走"处理。

## 9. 已知边界（这套测试台没做的）

- **不驱动底盘/机械臂**：`MOVE` 默认只打印（`VISION_BENCH_DRIVE_MOVE=0`）。
  真接整车时在 `Move_Handle()` 里调走位函数，或直接用第 8 节的接入方式。
- `t` 用的是固定任务码 `DEFAULT_TASK_CODE`（要换就改宏；字符输入在这个串口交互模式下没做）。
- 帧 hex 打印是"收到合法帧后按 payload 重新组帧"，所以**坏帧不会出现在 hex 里**，
  坏帧只会体现在 `k` 的计数器上；要看线上原始字节请用 `pc/serial-hexdump.ps1`。
- 需要 200ms 半帧超时才丢的半帧残留，只在主循环持续调用 `Vision_Pump` 时才生效。