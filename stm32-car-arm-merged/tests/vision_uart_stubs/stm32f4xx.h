#ifndef VISION_UART_STUB_STM32F4XX_H
#define VISION_UART_STUB_STM32F4XX_H
#include <stdint.h>
typedef struct {volatile uint32_t SR,DR;} USART_TypeDef;
typedef struct {uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_Speed,GPIO_PuPd;} GPIO_InitTypeDef;
typedef struct {uint32_t USART_BaudRate,USART_WordLength,USART_StopBits,USART_Parity,USART_Mode,USART_HardwareFlowControl;} USART_InitTypeDef;
typedef struct {uint8_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
extern USART_TypeDef vision_test_uart4,vision_test_uart5;
#define UART4 (&vision_test_uart4)
#define UART5 (&vision_test_uart5)
#define GPIOC ((void *)3)
#define ENABLE 1
#define RESET 0
#define RCC_AHB1Periph_GPIOC 4u
#define RCC_APB1Periph_UART4 (1u<<19)
#define GPIO_PinSource10 10
#define GPIO_PinSource11 11
#define GPIO_Pin_10 (1u<<10)
#define GPIO_Pin_11 (1u<<11)
#define GPIO_AF_UART4 8
#define GPIO_Mode_AF 2
#define GPIO_OType_PP 0
#define GPIO_Speed_50MHz 2
#define GPIO_PuPd_UP 1
#define USART_Mode_Rx 4
#define USART_Mode_Tx 8
#define USART_WordLength_8b 0
#define USART_StopBits_1 0
#define USART_Parity_No 0
#define USART_HardwareFlowControl_None 0
#define USART_IT_RXNE 0x0525u
#define USART_FLAG_PE 1u
#define USART_FLAG_FE 2u
#define USART_FLAG_NE 4u
#define USART_FLAG_ORE 8u
#define USART_FLAG_RXNE 32u
#define USART_FLAG_TXE 128u
#define UART4_IRQn 52
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
void RCC_AHB1PeriphClockCmd(uint32_t clocks,int enabled);
void RCC_APB1PeriphClockCmd(uint32_t clocks,int enabled);
void GPIO_PinAFConfig(void *port,unsigned pin,unsigned af);
void GPIO_StructInit(GPIO_InitTypeDef *gpio);
void GPIO_Init(void *port,GPIO_InitTypeDef *gpio);
void USART_StructInit(USART_InitTypeDef *uart);
void USART_Init(USART_TypeDef *port,USART_InitTypeDef *uart);
void NVIC_Init(NVIC_InitTypeDef *irq);
void USART_ITConfig(USART_TypeDef *port,unsigned interrupt,int enabled);
void USART_Cmd(USART_TypeDef *port,int enabled);
int USART_GetFlagStatus(USART_TypeDef *port,unsigned flag);
void USART_SendData(USART_TypeDef *port,uint16_t byte);
#endif
