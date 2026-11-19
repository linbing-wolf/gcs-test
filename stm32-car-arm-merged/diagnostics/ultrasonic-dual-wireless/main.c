#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef CHASSIS_OBSTACLE_TEST
#include "../ultrasonic-chassis/obstacle_io.h"
#endif
#ifndef DUAL_HOST_TEST
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#endif

/* 独立传感器测试：U4不再是调试串口，不链接任何整车/电机/机构代码。 */
#define RADIO_BAUD 115200u
#ifdef CHASSIS_OBSTACLE_TEST
#define TX_SIZE 4096u /* 单次 feedback 包含四轮状态；预留完整快照，发送仍不阻塞控制。 */
#else
#define TX_SIZE 512u
#endif
#define RX_SIZE 128u
#define SHOT_GAP_US 70000u /* 所有触发之间留70ms；双路交替时单探头周期至少140ms。 */
#define ECHO_TIMEOUT_US 40000u
#define TRIGGER_US 12u
#define MIN_ECHO_US 116u
#define MAX_ECHO_US 23200u
enum { FRONT, LEFT, SENSOR_COUNT };
enum { CAP_IDLE, CAP_RISE, CAP_FALL, CAP_DONE };
enum { ECHO_OK, ECHO_PREHIGH, ECHO_NO_RISE, ECHO_HIGH_TIMEOUT, ECHO_RANGE };
static const char *const sensor_names[]={"FRONT","LEFT"};
static const char *const sensor_ports[]={"U5","U4"};
static const char *const echo_names[]={"OK","ECHO_HIGH_BEFORE_TRIGGER","NO_ECHO_RISE","ECHO_HIGH_TIMEOUT","OUT_OF_RANGE"};

static uint32_t clock_us(void);
static uint32_t irq_lock(void);
static void irq_unlock(uint32_t previous);
static void trigger_write(uint8_t channel,uint8_t high);
static uint8_t echo_level(uint8_t channel);
static void echo_route(uint8_t channel);
static void radio_tx_write(uint8_t high);
static uint8_t radio_rx_level(void);
static void tx_timer_enable(uint8_t enabled);
static void rx_timer_start(void);
static void rx_timer_full_bit(void);
static void rx_timer_disable(void);
static void rx_start_enable(uint8_t enabled);
static void put_text(const char *s);

static struct {
    volatile uint8_t busy,bit,value;
    volatile uint16_t head,tail;
    uint8_t data[TX_SIZE];
} tx;
static struct {
    volatile uint8_t busy,stage,value;
    volatile uint32_t frames,framing_errors,false_starts,overflows;
    volatile uint8_t head,tail,overflow,stop;
    uint8_t data[RX_SIZE];
} rx;
static struct {
    volatile uint8_t phase,ready,status,channel;
    volatile uint32_t started,rising,pulse;
} capture;
static struct {
    uint8_t continuous,mask,next,pending,started;
    uint32_t last_start,sequence;
    struct {uint32_t samples,good,failed;int last_mm;} stats[SENSOR_COUNT];
} ranging;
static char line[48],message[224];
static unsigned line_length;

/* 软件UART只在定时器中断移位，前台打印不会屏蔽Echo或无线接收。 */
static void tx_begin_byte(void)
{
    tx.value=tx.data[tx.tail];tx.tail=(uint16_t)((tx.tail+1u)%TX_SIZE);
    tx.bit=0;tx.busy=1;radio_tx_write(0);
}
static void radio_send(uint8_t byte)
{
    uint16_t next=(uint16_t)((tx.head+1u)%TX_SIZE);
    uint32_t previous;
#ifdef CHASSIS_OBSTACLE_TEST
    if(next==tx.tail) return; /* 调试日志不能让CAN闭环或停车等待。 */
#else
    while(next==tx.tail) {} /* 中断持续排空；本函数仅由前台调用。 */
#endif
    tx.data[tx.head]=byte;tx.head=next;
    previous=irq_lock();
    if(!tx.busy) {tx_begin_byte();tx_timer_enable(1);}
    irq_unlock(previous);
}
static void radio_tx_tick(void)
{
    if(!tx.busy)return;
    if(tx.bit<8u) {radio_tx_write((uint8_t)((tx.value>>tx.bit)&1u));++tx.bit;}
    else if(tx.bit==8u) {radio_tx_write(1);++tx.bit;}
    else if(tx.tail!=tx.head)tx_begin_byte();
    else {tx.busy=0;tx_timer_enable(0);}
}
static void receive_byte(uint8_t byte)
{
#ifndef CHASSIS_OBSTACLE_TEST
    uint8_t next;
#endif
    ++rx.frames;
#ifdef CHASSIS_OBSTACLE_TEST
    obstacle_io_receive(byte);
#else
    if(byte=='!') {rx.stop=1;return;}
    next=(uint8_t)((rx.head+1u)%RX_SIZE);
    if(next==rx.tail) {++rx.overflows;rx.overflow=1;return;}
    rx.data[rx.head]=byte;rx.head=next;
#endif
}
static void radio_rx_end(void)
{
    rx_timer_disable();rx.busy=0;rx_start_enable(1);
}
static void radio_rx_start(void)
{
    if(rx.busy)return;
    rx_start_enable(0);rx.busy=1;rx.stage=rx.value=0;rx_timer_start();
}
static void radio_rx_tick(void)
{
    uint8_t high=radio_rx_level();
    if(!rx.busy)return;
    if(!rx.stage) {
        if(high) {++rx.false_starts;radio_rx_end();return;}
        rx.stage=1;rx_timer_full_bit();
    } else if(rx.stage<=8u) {
        if(high)rx.value|=(uint8_t)(1u<<(rx.stage-1u));
        ++rx.stage;
    } else {
        if(high)receive_byte(rx.value);else ++rx.framing_errors;
        radio_rx_end();
    }
}

static void capture_finish(uint8_t status,uint32_t pulse)
{
    capture.pulse=pulse;capture.status=status;capture.phase=CAP_DONE;capture.ready=1;
}
static void capture_edge(uint8_t high,uint32_t now)
{
    if(capture.phase!=CAP_RISE && capture.phase!=CAP_FALL)return;
    if((uint32_t)(now-capture.started)>=ECHO_TIMEOUT_US) {
        capture_finish(capture.phase==CAP_RISE?ECHO_NO_RISE:ECHO_HIGH_TIMEOUT,0);return;
    }
    if(high && capture.phase==CAP_RISE) {capture.rising=now;capture.phase=CAP_FALL;}
    else if(!high && capture.phase==CAP_FALL) {
        uint32_t pulse=now-capture.rising;
        capture_finish(pulse>=MIN_ECHO_US && pulse<=MAX_ECHO_US?ECHO_OK:ECHO_RANGE,pulse);
    }
}
static void capture_cancel(void)
{
    capture.phase=CAP_IDLE;capture.ready=0;
    trigger_write(FRONT,0);trigger_write(LEFT,0);
}
static void capture_begin(uint8_t channel)
{
    uint32_t previous=irq_lock();
    capture.phase=CAP_IDLE;capture.ready=0;
    /* PC10与PB10共用EXTI10，必须在每次触发前切换端口，而不是同时配置两路。 */
    echo_route(channel);capture.channel=channel;
    capture.started=clock_us();
    capture.phase=CAP_RISE;
    irq_unlock(previous);
    ++ranging.sequence;++ranging.stats[channel].samples;
    if(echo_level(channel)) {capture_finish(ECHO_PREHIGH,0);return;}
    trigger_write(channel,1);
    {
        uint32_t start=clock_us();
        while((uint32_t)(clock_us()-start)<TRIGGER_US) {}
    }
    trigger_write(channel,0);
}
static void report_echo(void)
{
    uint8_t channel=capture.channel,status=capture.status;
    uint32_t pulse=capture.pulse;
    int mm=status==ECHO_OK ? (int)((pulse*10u+29u)/58u) : -1;
    capture_cancel();ranging.stats[channel].last_mm=mm;
#ifdef CHASSIS_OBSTACLE_TEST
    obstacle_io_result(channel,mm);
#else
    if(status==ECHO_OK)++ranging.stats[channel].good;else ++ranging.stats[channel].failed;
    snprintf(message,sizeof(message),"[range] sensor=%s port=%s seq=%lu status=%s echo_us=%lu distance_mm=%d\r\n",
        sensor_names[channel],sensor_ports[channel],(unsigned long)ranging.sequence,
        echo_names[status],(unsigned long)pulse,mm);put_text(message);
#endif
}
static void range_status(void)
{
    unsigned i;
    for(i=0;i<SENSOR_COUNT;i++) {
        snprintf(message,sizeof(message),"[range status] sensor=%s port=%s samples=%lu good=%lu failed=%lu last_mm=%d echo=%u\r\n",
            sensor_names[i],sensor_ports[i],(unsigned long)ranging.stats[i].samples,
            (unsigned long)ranging.stats[i].good,(unsigned long)ranging.stats[i].failed,
            ranging.stats[i].last_mm,(unsigned)echo_level((uint8_t)i));put_text(message);
    }
    snprintf(message,sizeof(message),"[radio status] baud=115200 8N1 TX=PB0 RX=PB1 frames=%lu framing_errors=%lu false_starts=%lu overflows=%lu uptime_ms=%lu\r\n",
        (unsigned long)rx.frames,(unsigned long)rx.framing_errors,(unsigned long)rx.false_starts,
        (unsigned long)rx.overflows,(unsigned long)(clock_us()/1000u));put_text(message);
}
static void command(const char *s)
{
    uint8_t mask=3;
    if(!strcmp(s,"status")) {range_status();return;}
    if(!strcmp(s,"stop")) {rx.stop=1;return;}
    if(!strcmp(s,"loop front") || !strcmp(s,"once front"))mask=1;
    else if(!strcmp(s,"loop left") || !strcmp(s,"once left"))mask=2;
    else if(strcmp(s,"loop") && strcmp(s,"once")) {
        put_text("commands: loop [front|left] / once [front|left] / status / !; sensors only, no CAN or motors\r\n");return;
    }
    ranging.mask=mask;ranging.next=(uint8_t)(mask==2);
    ranging.continuous=(uint8_t)(!strncmp(s,"loop",4));ranging.pending=ranging.continuous?0:mask;
}
static void dual_reset(void)
{
    memset(&tx,0,sizeof(tx));memset(&rx,0,sizeof(rx));
    memset(&capture,0,sizeof(capture));memset(&ranging,0,sizeof(ranging));
    ranging.stats[FRONT].last_mm=ranging.stats[LEFT].last_mm=-1;line_length=0;
}
static void range_poll(void)
{
    uint32_t now=clock_us();
    if((capture.phase==CAP_RISE || capture.phase==CAP_FALL)
        && (uint32_t)(now-capture.started)>=ECHO_TIMEOUT_US)
        capture_finish(capture.phase==CAP_RISE?ECHO_NO_RISE:ECHO_HIGH_TIMEOUT,0);
    if(capture.ready) {report_echo();return;}
    if(capture.phase==CAP_IDLE && (ranging.continuous || ranging.pending)
        && (!ranging.started || (uint32_t)(now-ranging.last_start)>=SHOT_GAP_US)) {
        uint8_t channel=ranging.next;
        if(!ranging.continuous)channel=(uint8_t)((ranging.pending&1u)?FRONT:LEFT);
        ranging.pending&=(uint8_t)~(1u<<channel);
        ranging.next=(uint8_t)(ranging.mask==3 ? channel^1u : channel);
        ranging.started=1;ranging.last_start=now;
#ifdef CHASSIS_OBSTACLE_TEST
        obstacle_io_shot(channel);
#endif
        capture_begin(channel);
    }
}

#ifndef CHASSIS_OBSTACLE_TEST
static void dual_step(void)
{
    if(rx.stop) {
        uint32_t previous=irq_lock();
        ranging.continuous=ranging.pending=0;capture_cancel();
        rx.tail=rx.head;rx.stop=rx.overflow=0;line_length=0;
        irq_unlock(previous);
        put_text("[range] cancelled; both triggers LOW; no motor interface\r\n");return;
    }
    if(rx.overflow) {
        uint32_t previous=irq_lock();rx.tail=rx.head;rx.overflow=0;line_length=0;irq_unlock(previous);
        put_text("[radio] input overflow; command discarded; ! still cancels\r\n");
    }
    if(rx.tail!=rx.head) {
        uint8_t byte=rx.data[rx.tail];rx.tail=(uint8_t)((rx.tail+1u)%RX_SIZE);
        if(byte=='\r' || byte=='\n') {
            if(line_length) {line[line_length]=0;command(line);line_length=0;}
        } else if(line_length<sizeof(line)-1u)line[line_length++]=(char)byte;
        else {line_length=0;rx.overflow=1;}
    }
    if(rx.stop)return;
    range_poll();
}
#endif

#ifndef DUAL_HOST_TEST
static uint32_t radio_bit_ticks;
static uint32_t clock_us(void) {return TIM2->CNT;}
static uint32_t irq_lock(void) {uint32_t previous=__get_PRIMASK();__disable_irq();return previous;}
static void irq_unlock(uint32_t previous) {if(!previous)__enable_irq();}
static void trigger_write(uint8_t channel,uint8_t high)
{
    GPIO_TypeDef *gpio=channel==FRONT?GPIOC:GPIOB;
    if(high)gpio->BSRRL=GPIO_Pin_11;else gpio->BSRRH=GPIO_Pin_11;
}
static uint8_t echo_level(uint8_t channel) {return ((channel==FRONT?GPIOC:GPIOB)->IDR&GPIO_Pin_10)!=0;}
static void echo_route(uint8_t channel)
{
    EXTI->IMR&=~EXTI_Line10;
    SYSCFG_EXTILineConfig(channel==FRONT?EXTI_PortSourceGPIOC:EXTI_PortSourceGPIOB,EXTI_PinSource10);
    EXTI->PR=EXTI_Line10;NVIC_ClearPendingIRQ(EXTI15_10_IRQn);EXTI->IMR|=EXTI_Line10;
}
static void radio_tx_write(uint8_t high) {if(high)GPIOB->BSRRL=GPIO_Pin_0;else GPIOB->BSRRH=GPIO_Pin_0;}
static uint8_t radio_rx_level(void) {return (GPIOB->IDR&GPIO_Pin_1)!=0;}
static void tx_timer_enable(uint8_t enabled)
{
    if(enabled) {TIM4->CNT=0;TIM4->SR=(uint16_t)~TIM_SR_UIF;TIM4->CR1|=TIM_CR1_CEN;}
    else TIM4->CR1&=~TIM_CR1_CEN;
}
static void rx_timer_start(void)
{
    TIM3->CR1&=~TIM_CR1_CEN;TIM3->CNT=0;
    TIM3->ARR=(radio_bit_ticks+1u)/2u-1u;TIM3->SR=(uint16_t)~TIM_SR_UIF;TIM3->CR1|=TIM_CR1_CEN;
}
static void rx_timer_full_bit(void) {TIM3->ARR=radio_bit_ticks-1u;}
static void rx_timer_disable(void) {TIM3->CR1&=~TIM_CR1_CEN;}
static void rx_start_enable(uint8_t enabled)
{
    if(enabled) {EXTI->PR=EXTI_Line1;EXTI->IMR|=EXTI_Line1;}
    else EXTI->IMR&=~EXTI_Line1;
}
static void put_text(const char *s) {while(*s)radio_send((uint8_t)*s++);}
static void hardware_init(void)
{
    GPIO_InitTypeDef gpio;TIM_TimeBaseInitTypeDef timer;EXTI_InitTypeDef edge;NVIC_InitTypeDef irq;
    RCC_ClocksTypeDef clocks;uint32_t timer_hz;
    SystemCoreClockUpdate();NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4,ENABLE);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_10;gpio.GPIO_Mode=GPIO_Mode_IN;gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);GPIO_Init(GPIOC,&gpio); /* 两路Echo均为FT输入，不启用内部上下拉。 */
    trigger_write(FRONT,0);trigger_write(LEFT,0);radio_tx_write(1);
    gpio.GPIO_Pin=GPIO_Pin_11;gpio.GPIO_Mode=GPIO_Mode_OUT;gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&gpio);GPIO_Init(GPIOC,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_0;GPIO_Init(GPIOB,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_1;gpio.GPIO_Mode=GPIO_Mode_IN;gpio.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOB,&gpio);
    RCC_GetClocksFreq(&clocks);timer_hz=clocks.PCLK1_Frequency*(clocks.PCLK1_Frequency==clocks.HCLK_Frequency?1u:2u);
    TIM_TimeBaseStructInit(&timer);timer.TIM_Prescaler=(uint16_t)(timer_hz/1000000u-1u);timer.TIM_Period=0xffffffffu;
    TIM_TimeBaseInit(TIM2,&timer);TIM_Cmd(TIM2,ENABLE);
    radio_bit_ticks=(timer_hz+RADIO_BAUD/2u)/RADIO_BAUD;
    TIM_TimeBaseStructInit(&timer);timer.TIM_Period=radio_bit_ticks-1u;
    TIM_TimeBaseInit(TIM3,&timer);TIM_TimeBaseInit(TIM4,&timer);
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource1);
    EXTI_StructInit(&edge);edge.EXTI_Line=EXTI_Line1;edge.EXTI_Mode=EXTI_Mode_Interrupt;
    edge.EXTI_Trigger=EXTI_Trigger_Falling;edge.EXTI_LineCmd=ENABLE;EXTI_Init(&edge);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
    edge.EXTI_Line=EXTI_Line10;edge.EXTI_Trigger=EXTI_Trigger_Rising_Falling;EXTI_Init(&edge);
    EXTI->PR=EXTI_Line1|EXTI_Line10;
    irq.NVIC_IRQChannelCmd=ENABLE;irq.NVIC_IRQChannelSubPriority=0;
    irq.NVIC_IRQChannel=EXTI15_10_IRQn;irq.NVIC_IRQChannelPreemptionPriority=0;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=EXTI1_IRQn;irq.NVIC_IRQChannelPreemptionPriority=1;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=TIM3_IRQn;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=TIM4_IRQn;irq.NVIC_IRQChannelPreemptionPriority=2;NVIC_Init(&irq);
}
void EXTI15_10_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line10) {uint32_t now=clock_us();EXTI->PR=EXTI_Line10;capture_edge(echo_level(capture.channel),now);}
}
void EXTI1_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line1) {EXTI->PR=EXTI_Line1;radio_rx_start();}
}
void TIM3_IRQHandler(void)
{
    if(TIM3->SR&TIM_SR_UIF) {TIM3->SR=(uint16_t)~TIM_SR_UIF;radio_rx_tick();}
}
void TIM4_IRQHandler(void)
{
    if(TIM4->SR&TIM_SR_UIF) {TIM4->SR=(uint16_t)~TIM_SR_UIF;radio_tx_tick();}
}
#ifdef CHASSIS_OBSTACLE_TEST
void obstacle_io_init(void)
{
    dual_reset();hardware_init();ranging.mask=3;ranging.continuous=1;
}
void obstacle_io_send(uint8_t byte) {radio_send(byte);}
void obstacle_io_poll(void) {range_poll();}
#else
int main(void)
{
    dual_reset();hardware_init();
    put_text("[ultrasonic-dual-wireless-v1] NO CAN / NO MOTORS / NO SERVOS\r\n");
    put_text("FRONT U5: PC11=Trig PC10=Echo; LEFT U4: PB11=Trig PB10=Echo\r\n");
    put_text("radio U6: PB0 TX -> module RX; PB1 RX <- module TX; 115200 8N1 SOFTWARE UART\r\n");
    put_text("PB10/PB11 ARE NOT A DEBUG UART; commands: loop / once / status / !\r\n");
    for(;;)dual_step();
}
#endif
#endif
