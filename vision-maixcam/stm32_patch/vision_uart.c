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
