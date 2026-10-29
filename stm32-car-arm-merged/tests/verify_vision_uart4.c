/* 不定义ARM_HOST_TEST：执行vision_link.c真实硬件分支，寄存器/外设调用用替身记录。 */
#include "vision_uart_stubs/stm32f4xx.h"
#include "vision_link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned assertions,af_mask,gpio_inits,uart_inits,irq_inits,rx_enables,uart_enables,tx_count;
static uint32_t gpio_clocks,uart_clocks;
static uint8_t tx[256];
static char output[4096];
static GPIO_InitTypeDef saved_gpio;
static USART_InitTypeDef saved_uart;
static NVIC_InitTypeDef saved_irq;
USART_TypeDef vision_test_uart4,vision_test_uart5;
#define CHECK(x) do {assertions++;if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}} while(0)
void RCC_AHB1PeriphClockCmd(uint32_t clocks,int enabled) {CHECK(enabled==ENABLE);gpio_clocks|=clocks;}
void RCC_APB1PeriphClockCmd(uint32_t clocks,int enabled) {CHECK(enabled==ENABLE);uart_clocks|=clocks;}
void GPIO_PinAFConfig(void *port,unsigned pin,unsigned af)
{CHECK(port==GPIOC && (pin==10 || pin==11) && af==8);af_mask|=1u<<pin;}
void GPIO_StructInit(GPIO_InitTypeDef *gpio) {memset(gpio,0,sizeof(*gpio));}
void GPIO_Init(void *port,GPIO_InitTypeDef *gpio) {CHECK(port==GPIOC);saved_gpio=*gpio;gpio_inits++;}
void USART_StructInit(USART_InitTypeDef *uart)
{memset(uart,0,sizeof(*uart));uart->USART_WordLength=USART_WordLength_8b;uart->USART_StopBits=USART_StopBits_1;}
void USART_Init(USART_TypeDef *port,USART_InitTypeDef *uart) {CHECK(port==UART4);saved_uart=*uart;uart_inits++;}
void NVIC_Init(NVIC_InitTypeDef *irq) {saved_irq=*irq;irq_inits++;}
void USART_ITConfig(USART_TypeDef *port,unsigned interrupt,int enabled)
{CHECK(port==UART4 && interrupt==USART_IT_RXNE && enabled==ENABLE);rx_enables++;}
void USART_Cmd(USART_TypeDef *port,int enabled) {CHECK(port==UART4 && enabled==ENABLE);uart_enables++;}
int USART_GetFlagStatus(USART_TypeDef *port,unsigned flag) {CHECK(port==UART4);return (port->SR&flag)!=0;}
void USART_SendData(USART_TypeDef *port,uint16_t byte)
{CHECK(port==UART4 && tx_count<sizeof(tx));tx[tx_count++]=(uint8_t)byte;}
void arm_output(const char *message)
{CHECK(strlen(output)+strlen(message)<sizeof(output));strcat(output,message);}
extern void UART4_IRQHandler(void);
static uint8_t crc(const uint8_t *bytes,unsigned n)
{
    uint8_t c=0;unsigned i,b;
    for(i=0;i<n;i++){c^=bytes[i];for(b=0;b<8;b++)c=(uint8_t)((c<<1)^((c&128)?7:0));}
    return c;
}
static void receive(uint8_t byte,uint32_t flags)
{UART4->DR=byte;UART4->SR=USART_FLAG_TXE|flags;UART4_IRQHandler();UART4->SR=USART_FLAG_TXE;}
static void reply(const char *payload)
{
    uint8_t bytes[64];unsigned n=(unsigned)strlen(payload),i;
    CHECK(n<48);bytes[0]=0xaa;bytes[1]=0x55;bytes[2]=(uint8_t)n;memcpy(bytes+3,payload,n);
    bytes[n+3]=crc(bytes+2,n+1);bytes[n+4]=13;
    for(i=0;i<n+5;i++)receive(bytes[i],USART_FLAG_RXNE);
}
int main(void)
{
    unsigned i;int16_t x,y;uint8_t color;
    UART5->SR=0xdeadbeefu;UART5->DR=0x12345678u;vision_init();
    CHECK(gpio_clocks==RCC_AHB1Periph_GPIOC && uart_clocks==RCC_APB1Periph_UART4);
    CHECK(af_mask==(GPIO_Pin_10|GPIO_Pin_11) && gpio_inits==1);
    CHECK(saved_gpio.GPIO_Pin==af_mask && saved_gpio.GPIO_Mode==GPIO_Mode_AF
        && saved_gpio.GPIO_OType==GPIO_OType_PP && saved_gpio.GPIO_PuPd==GPIO_PuPd_UP);
    CHECK(uart_inits==1 && saved_uart.USART_BaudRate==115200 && saved_uart.USART_Mode==(USART_Mode_Rx|USART_Mode_Tx));
    CHECK(saved_uart.USART_WordLength==USART_WordLength_8b && saved_uart.USART_StopBits==USART_StopBits_1
        && saved_uart.USART_Parity==USART_Parity_No && saved_uart.USART_HardwareFlowControl==USART_HardwareFlowControl_None);
    CHECK(irq_inits==1 && saved_irq.NVIC_IRQChannel==UART4_IRQn && saved_irq.NVIC_IRQChannelCmd==ENABLE);
    CHECK(rx_enables==1 && uart_enables==1 && strstr(output,"UART4 115200 PC11=RX PC10=TX"));
    vision_request_pick(1,0);vision_update(1);CHECK(!tx_count);
    UART4->SR=USART_FLAG_TXE;for(i=0;i<10;i++)vision_update(i+2);
    CHECK(tx_count==8 && !memcmp(tx,"\xaa\x55\x03" "C,1",6) && tx[6]==crc(tx+2,4) && tx[7]==13);
    reply("PICK,1,7,-3");vision_update(20);CHECK(vision_pick_state()==VISION_PICK_READY);
    vision_pick_result(&color,&x,&y);CHECK(color==1 && x==7 && y==-3);
    vision_request_align('D',2,21);for(i=0;i<10;i++)vision_update(i+22);
    CHECK(tx_count==14 && tx[11]=='D');
    reply("MOVEXY,12,-9");reply("DISC,320,240,0,1");vision_update(40);
    CHECK(vision_align_state()==VISION_PICK_READY && vision_motion_peek_xy(&x,&y,NULL) && x==12 && y==-9);
    vision_motion_take();CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
    vision_request_pick(2,41);for(i=0;i<10;i++)vision_update(i+42);
    receive(0xaa,USART_FLAG_RXNE);receive(0x55,USART_FLAG_RXNE|USART_FLAG_ORE);vision_update(55);
    CHECK(vision_pick_state()==VISION_PICK_WAITING);
    reply("PICK,2,8,4");vision_update(56);CHECK(vision_pick_state()==VISION_PICK_READY);
    CHECK(UART5->SR==0xdeadbeefu && UART5->DR==0x12345678u);
    printf("PASS UART4 hardware branch: %u assertions; PC10/PC11 AF8,115200 8N1,RX IRQ,nonblocking TX,CRC/FIFO and error recovery; no hardware accessed\n",assertions);
    return 0;
}
