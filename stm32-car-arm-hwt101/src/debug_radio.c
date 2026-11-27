#include "debug_radio.h"
#ifndef CHASSIS_OBSTACLE_TEST
#include "car_config.h"
#include <string.h>
#ifndef DEBUG_RADIO_HOST_TEST
#include "stm32f4xx.h"
#endif

/* 115200每位仅约8.68us，定时器中断使用优化代码。 */
#ifdef __CC_ARM
#pragma O2
#endif

#define RADIO_TX_SIZE 4096u
static struct {
    volatile uint16_t head,tail;
    volatile uint8_t busy,bit,value;
    uint8_t data[RADIO_TX_SIZE];
} tx;
static struct {
    volatile uint8_t busy,stage,value;
} rx;
static volatile DebugRadioStats counters;
static void (*receive_byte)(uint8_t);
static void (*receive_fault)(void);

static uint32_t irq_lock(void);
static void irq_unlock(uint32_t previous);
static void tx_write(uint8_t high);
static uint8_t rx_level(void);
static void tx_timer_enable(uint8_t enabled);
static void rx_timer_start(void);
static void rx_timer_full_bit(void);
static void rx_timer_disable(void);
static void rx_start_enable(uint8_t enabled);
static void hardware_init(void);

static void tx_begin_byte(void)
{
    tx.value=tx.data[tx.tail];
    tx.tail=(uint16_t)((tx.tail+1u)%RADIO_TX_SIZE);
    tx.bit=0;tx.busy=1;tx_write(0);
}

void debug_radio_send(uint8_t byte)
{
    uint16_t next=(uint16_t)((tx.head+1u)%RADIO_TX_SIZE);
    uint32_t previous;
    /* 日志队列满时计数丢弃，前台运动与接收中断仍能继续运行。 */
    if(next==tx.tail) {++counters.tx_dropped;return;}
    tx.data[tx.head]=byte;tx.head=next;
    previous=irq_lock();
    if(!tx.busy) {tx_begin_byte();tx_timer_enable(1);}
    irq_unlock(previous);
}

static void tx_tick(void)
{
    if(!tx.busy)return;
    if(tx.bit<8u) {tx_write((uint8_t)((tx.value>>tx.bit)&1u));++tx.bit;}
    else if(tx.bit==8u) {tx_write(1);++tx.bit;}
    else if(tx.tail!=tx.head)tx_begin_byte();
    else {tx.busy=0;tx_timer_enable(0);}
}

static void rx_end(void)
{
    rx_timer_disable();rx.busy=0;rx_start_enable(1);
}

static void rx_start(void)
{
    if(rx.busy)return;
    rx_start_enable(0);rx.busy=1;rx.stage=rx.value=0;rx_timer_start();
}

static void rx_tick(void)
{
    uint8_t high=rx_level();
    if(!rx.busy)return;
    if(!rx.stage) {
        if(high) {++counters.false_starts;rx_end();return;}
        rx.stage=1;rx_timer_full_bit();
    } else if(rx.stage<=8u) {
        if(high)rx.value|=(uint8_t)(1u<<(rx.stage-1u));
        ++rx.stage;
    } else {
        if(high) {++counters.received;receive_byte(rx.value);}
        else {++counters.framing_errors;receive_fault();}
        rx_end();
    }
}

void debug_radio_init(void (*receive)(uint8_t),void (*receive_error)(void))
{
    memset(&tx,0,sizeof(tx));memset(&rx,0,sizeof(rx));
    memset((void *)&counters,0,sizeof(counters));
    receive_byte=receive;
    receive_fault=receive_error;
    hardware_init();
}

void debug_radio_stats(DebugRadioStats *stats)
{
    uint32_t previous=irq_lock();
    stats->received=counters.received;
    stats->framing_errors=counters.framing_errors;
    stats->false_starts=counters.false_starts;
    stats->tx_dropped=counters.tx_dropped;
    irq_unlock(previous);
}

#ifndef DEBUG_RADIO_HOST_TEST
static uint32_t bit_ticks;
static uint32_t irq_lock(void) {uint32_t previous=__get_PRIMASK();__disable_irq();return previous;}
static void irq_unlock(uint32_t previous) {if(!previous)__enable_irq();}
static void tx_write(uint8_t high) {if(high)GPIOB->BSRRL=GPIO_Pin_0;else GPIOB->BSRRH=GPIO_Pin_0;}
static uint8_t rx_level(void) {return (GPIOB->IDR&GPIO_Pin_1)!=0;}
static void tx_timer_enable(uint8_t enabled)
{
    if(enabled) {TIM7->CNT=0;TIM7->SR=(uint16_t)~TIM_SR_UIF;TIM7->CR1|=TIM_CR1_CEN;}
    else TIM7->CR1&=~TIM_CR1_CEN;
}
static void rx_timer_start(void)
{
    TIM6->CR1&=~TIM_CR1_CEN;TIM6->CNT=0;
    TIM6->ARR=(bit_ticks+1u)/2u-1u;TIM6->SR=(uint16_t)~TIM_SR_UIF;TIM6->CR1|=TIM_CR1_CEN;
}
static void rx_timer_full_bit(void) {TIM6->ARR=bit_ticks-1u;}
static void rx_timer_disable(void) {TIM6->CR1&=~TIM_CR1_CEN;}
static void rx_start_enable(uint8_t enabled)
{
    if(enabled) {EXTI->PR=EXTI_Line1;EXTI->IMR|=EXTI_Line1;}
    else EXTI->IMR&=~EXTI_Line1;
}
static void hardware_init(void)
{
    GPIO_InitTypeDef gpio;TIM_TimeBaseInitTypeDef timer;
    EXTI_InitTypeDef edge;NVIC_InitTypeDef irq;
    RCC_ClocksTypeDef clocks;uint32_t timer_hz;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6|RCC_APB1Periph_TIM7,ENABLE);
    tx_write(1);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=GPIO_Pin_0;gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_1;gpio.GPIO_Mode=GPIO_Mode_IN;gpio.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOB,&gpio);
    RCC_GetClocksFreq(&clocks);
    timer_hz=clocks.PCLK1_Frequency*(clocks.PCLK1_Frequency==clocks.HCLK_Frequency ? 1u : 2u);
    bit_ticks=(timer_hz+CAR_DEBUG_UART_BAUD/2u)/CAR_DEBUG_UART_BAUD;
    TIM_TimeBaseStructInit(&timer);timer.TIM_Period=bit_ticks-1u;
    TIM_TimeBaseInit(TIM6,&timer);TIM_TimeBaseInit(TIM7,&timer);
    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource1);
    EXTI_StructInit(&edge);edge.EXTI_Line=EXTI_Line1;edge.EXTI_Mode=EXTI_Mode_Interrupt;
    edge.EXTI_Trigger=EXTI_Trigger_Falling;edge.EXTI_LineCmd=ENABLE;EXTI_Init(&edge);
    EXTI_ClearITPendingBit(EXTI_Line1);
    irq.NVIC_IRQChannelCmd=ENABLE;irq.NVIC_IRQChannelSubPriority=0;
    irq.NVIC_IRQChannel=EXTI1_IRQn;irq.NVIC_IRQChannelPreemptionPriority=1;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=TIM6_DAC_IRQn;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=TIM7_IRQn;irq.NVIC_IRQChannelPreemptionPriority=2;NVIC_Init(&irq);
}

void EXTI1_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line1) {EXTI->PR=EXTI_Line1;rx_start();}
}
void TIM6_DAC_IRQHandler(void)
{
    if(TIM6->SR&TIM_SR_UIF) {TIM6->SR=(uint16_t)~TIM_SR_UIF;rx_tick();}
}
void TIM7_IRQHandler(void)
{
    if(TIM7->SR&TIM_SR_UIF) {TIM7->SR=(uint16_t)~TIM_SR_UIF;tx_tick();}
}
#endif
#endif
