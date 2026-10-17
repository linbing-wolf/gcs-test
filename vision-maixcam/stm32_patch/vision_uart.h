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
  * 接线：MaixCAM2 B0(UART2_TX) → PD2(UART5_RX)
  *       MaixCAM2 B1(UART2_RX) ← PC12(UART5_TX)
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
