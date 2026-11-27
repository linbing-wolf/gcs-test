#ifndef VISION_I2C_STUBS_H
#define VISION_I2C_STUBS_H
#include <stdint.h>
typedef struct {uint32_t IDR,ODR;} GPIO_TypeDef;
typedef struct {uint32_t PR;} EXTI_TypeDef;
typedef struct {uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_Speed,GPIO_PuPd;} GPIO_InitTypeDef;
typedef struct {uint32_t EXTI_Line,EXTI_Mode,EXTI_Trigger,EXTI_LineCmd;} EXTI_InitTypeDef;
typedef struct {uint8_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
extern GPIO_TypeDef test_gpiog,test_gpiod;
extern EXTI_TypeDef test_exti;
#define GPIOG (&test_gpiog)
#define GPIOD (&test_gpiod)
#define EXTI (&test_exti)
#define ENABLE 1
#define DISABLE 0
#define RCC_AHB1Periph_GPIOG 64u
#define RCC_AHB1Periph_GPIOD 8u
#define RCC_APB2Periph_SYSCFG 0x4000u
#define GPIO_Pin_15 0x8000u
#define GPIO_Pin_7 0x80u
#define GPIO_Mode_OUT 1
#define GPIO_OType_OD 1
#define GPIO_Speed_50MHz 2
#define GPIO_PuPd_NOPULL 0
#define GPIO_PuPd_UP 1
#define EXTI_Line7 0x80u
#define EXTI_Line15 0x8000u
#define EXTI_PortSourceGPIOG 6
#define EXTI_PortSourceGPIOD 3
#define EXTI_PinSource7 7
#define EXTI_PinSource15 15
#define EXTI_Mode_Interrupt 0
#define EXTI_Trigger_Rising_Falling 0x10
#define EXTI9_5_IRQn 23
#define EXTI15_10_IRQn 40
void RCC_AHB1PeriphClockCmd(uint32_t clocks,int enabled);
void RCC_APB2PeriphClockCmd(uint32_t clocks,int enabled);
void GPIO_StructInit(GPIO_InitTypeDef *gpio);
void GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *gpio);
void GPIO_SetBits(GPIO_TypeDef *port,uint16_t pins);
void GPIO_ResetBits(GPIO_TypeDef *port,uint16_t pins);
void SYSCFG_EXTILineConfig(unsigned port,unsigned pin);
void EXTI_StructInit(EXTI_InitTypeDef *edge);
void EXTI_Init(EXTI_InitTypeDef *edge);
void EXTI_ClearITPendingBit(uint32_t lines);
void NVIC_Init(NVIC_InitTypeDef *irq);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t mask);
#endif
