#include "hmi_status.h"
#include <stdio.h>
#include <string.h>
#ifndef ARM_HOST_TEST
#include "stm32f4xx.h"
#endif

#define HMI_TX_CAPACITY 1024u
static const char *const names[HMI_FIELDS]={
    "taska","taskb","taskok","phase","elapsed","pick","place",
    "lf_t","lf_r","rf_t","rf_r","lr_t","lr_r","rr_t","rr_r",
    "lift","push","grip","tray","pan","mechanism",
    "yaw","yaw_t","color","dx","dy","vision_age","imu_age","voltage","comm"
};
static struct {
    uint8_t bytes[HMI_TX_CAPACITY];
    volatile uint16_t length,offset;
    uint16_t building;
    uint8_t initialized,sent,force,view_pending,page,group;
    uint32_t boot_ms,refresh_ms,resync_ms;
    HmiSnapshot previous;
} screen;

static int append(const char *text)
{
    size_t n=strlen(text);
    if(screen.building+n+3u>sizeof(screen.bytes))return 0;
    memcpy(screen.bytes+screen.building,text,n);screen.building+=(uint16_t)n;
    memset(screen.bytes+screen.building,0xff,3);screen.building+=3u;
    return 1;
}
static int assign(const char *name,int32_t value)
{
    char line[40];
    snprintf(line,sizeof(line),"%s=%ld",name,(long)value);
    return append(line);
}
int hmi_busy(void) {return screen.offset<screen.length;}
int hmi_tx_next(uint8_t *byte)
{
    if(!hmi_busy())return 0;
    *byte=screen.bytes[screen.offset++];return 1;
}
#ifndef ARM_HOST_TEST
void USART6_IRQHandler(void)
{
    uint8_t byte;
    if(USART_GetITStatus(USART6,USART_IT_TXE)!=RESET) {
        if(hmi_tx_next(&byte))USART_SendData(USART6,byte);
        else USART_ITConfig(USART6,USART_IT_TXE,DISABLE);
    }
}
static void uart_start(void) {USART_ITConfig(USART6,USART_IT_TXE,ENABLE);}
static void uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef irq;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_6;gpio.GPIO_Mode=GPIO_Mode_AF;
    gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_Speed=GPIO_Speed_50MHz;gpio.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOC,&gpio);
    USART_StructInit(&uart);uart.USART_BaudRate=9600;uart.USART_Mode=USART_Mode_Tx;
    USART_Init(USART6,&uart);
    USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);
    USART_ITConfig(USART6,USART_IT_TXE,DISABLE);
    USART_Cmd(USART6,ENABLE);
    irq.NVIC_IRQChannel=USART6_IRQn;irq.NVIC_IRQChannelPreemptionPriority=3;
    irq.NVIC_IRQChannelSubPriority=0;irq.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&irq);
}
#endif
void hmi_init(uint32_t now)
{
    memset(&screen,0,sizeof(screen));screen.initialized=1;screen.force=screen.view_pending=1;
    screen.boot_ms=now;
#ifndef ARM_HOST_TEST
    uart_init();
#endif
}
void hmi_select(uint8_t page,uint8_t group)
{
    if(page>1u || group>2u)return;
    screen.page=page;screen.group=group;screen.view_pending=screen.force=1;
}
void hmi_resync(void) {screen.force=1;}
void hmi_update(const HmiSnapshot *snapshot,uint32_t now)
{
    unsigned i;int full,changed=0,task_changed;
    if(!screen.initialized || hmi_busy() || (uint32_t)(now-screen.boot_ms)<HMI_BOOT_MS)return;
    full=!screen.sent || screen.force || (uint32_t)(now-screen.resync_ms)>=HMI_RESYNC_MS;
    if(!full && (uint32_t)(now-screen.refresh_ms)<HMI_REFRESH_MS)return;
    screen.building=0;
    if(!screen.sent) {
        /* 清掉屏幕接收端可能残留的半帧，再设置无应答模式。 */
        memset(screen.bytes,0xff,3);screen.building=3;
        if(!append("bkcmd=0"))return;
    }
    task_changed=full || snapshot->value[HMI_TASKA]!=screen.previous.value[HMI_TASKA]
        || snapshot->value[HMI_TASKB]!=screen.previous.value[HMI_TASKB]
        || snapshot->value[HMI_TASKOK]!=screen.previous.value[HMI_TASKOK];
    /* 两个六位任务码先写全，最后置有效标志，避免半新半旧任务。 */
    if(task_changed) {
        if(!assign("taskok",0) || !assign("taska",snapshot->value[HMI_TASKA])
            || !assign("taskb",snapshot->value[HMI_TASKB])
            || !assign("taskok",snapshot->value[HMI_TASKOK]))return;
        changed=1;
    }
    for(i=HMI_PHASE;i<HMI_FIELDS;i++)
        if(full || snapshot->value[i]!=screen.previous.value[i]) {
            if(!assign(names[i],snapshot->value[i]))return;
            changed=1;
        }
    if(screen.view_pending) {
        if(!assign("group",screen.group) || !append(screen.page ? "page 1" : "page 0"))return;
        changed=1;
    } else if(changed && !append("click b0,1"))return;
    screen.refresh_ms=now;
    if(!changed)return;
    screen.previous=*snapshot;screen.sent=1;screen.force=screen.view_pending=0;
    if(full)screen.resync_ms=now;
    /* 数据全部构造后才发布长度；中断绝不会消费未写完的命令。 */
#ifndef ARM_HOST_TEST
    __disable_irq();
#endif
    screen.offset=0;screen.length=screen.building;
#ifndef ARM_HOST_TEST
    __enable_irq();
#endif
#ifndef ARM_HOST_TEST
    uart_start();
#endif
}
