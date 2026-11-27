/* 仅由vision_link.c包含：PG15=SCL、PD7=SDA的开漏I2C从机，无轮询等待。 */
#ifndef VISION_SOFT_I2C_H
#define VISION_SOFT_I2C_H
#define VISION_I2C_ADDRESS 0x42u
#define VISION_I2C_MAILBOX 32u
#define VISION_I2C_TX_RING 128u
enum { SI_IDLE,SI_ADDRESS,SI_ADDRESS_ACK,SI_RECEIVE,SI_RECEIVE_ACK,SI_SEND,SI_SEND_ACK,SI_IGNORE };
static struct {
    volatile uint8_t head,tail;
    volatile uint8_t stream[VISION_I2C_TX_RING];
    uint8_t rx[VISION_I2C_MAILBOX],tx[VISION_I2C_MAILBOX];
    uint8_t active,rx_pos,tx_pos,tx_count,phase,bits,byte,ack_seen,last_scl;
    volatile uint32_t generation;
    uint32_t staged_generation;
    volatile uint32_t reads,writes,invalid,bus_errors;
    volatile uint32_t last_error_ms;
    volatile uint32_t tx_drained_ms;
    volatile uint8_t error_phase,error_bits,error_rx_pos;
} vision_i2c;

static int vision_i2c_tx_ready(void)
{
    return (uint8_t)((vision_i2c.head+1u)%VISION_I2C_TX_RING)!=vision_i2c.tail;
}
static void vision_i2c_send_byte(uint8_t byte)
{
    vision_i2c.stream[vision_i2c.head]=byte;
    vision_i2c.head=(uint8_t)((vision_i2c.head+1u)%VISION_I2C_TX_RING);
}
static void vision_i2c_cancel_tx(void)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();vision_i2c.generation++;vision_i2c.tail=vision_i2c.head;__set_PRIMASK(mask);
}
static void vision_i2c_sda(unsigned high)
{
    if(high)GPIO_SetBits(GPIOD,GPIO_Pin_7);else GPIO_ResetBits(GPIOD,GPIO_Pin_7);
    /* 本机只在SCL低时改变SDA；清掉自己产生的边沿，防稍后误认START/STOP。 */
    EXTI_ClearITPendingBit(EXTI_Line7);
}
static void vision_i2c_prepare_tx(void)
{
    unsigned i;
    uint8_t count=(uint8_t)((vision_i2c.head-vision_i2c.tail+VISION_I2C_TX_RING)%VISION_I2C_TX_RING);
    if(count>31u)count=31u;
    memset(vision_i2c.tx,0,sizeof(vision_i2c.tx));vision_i2c.tx[0]=count;
    for(i=0;i<count;i++)vision_i2c.tx[i+1u]=vision_i2c.stream[(vision_i2c.tail+i)%VISION_I2C_TX_RING];
    vision_i2c.tx_count=count;vision_i2c.tx_pos=0;vision_i2c.staged_generation=vision_i2c.generation;
}
static void vision_i2c_finish_rx(void)
{
    if(vision_i2c.rx_pos==VISION_I2C_MAILBOX && vision_i2c.rx[0]<=31u) {
        vision_receive_mailbox(vision_i2c.rx+1,vision_i2c.rx[0]);
        vision_i2c.writes++;
    } else {
        vision_i2c.invalid++;vision_i2c.last_error_ms=vision_time_ms();
        vision_i2c.error_phase=vision_i2c.phase;vision_i2c.error_bits=vision_i2c.bits;vision_i2c.error_rx_pos=vision_i2c.rx_pos;
        vision_receive_byte(0,2);
    }
    vision_i2c.active=0;
}
static void vision_i2c_boundary(void)
{
    if(vision_i2c.active==1u)vision_i2c_finish_rx();
    else if(vision_i2c.active==2u){vision_i2c.invalid++;vision_i2c.active=0;}
}
static void vision_i2c_load_tx(void)
{
    vision_i2c.byte=vision_i2c.tx_pos<VISION_I2C_MAILBOX ? vision_i2c.tx[vision_i2c.tx_pos] : 0;
    if(vision_i2c.tx_pos<=VISION_I2C_MAILBOX)vision_i2c.tx_pos++;
    vision_i2c.bits=0;vision_i2c.phase=SI_SEND;vision_i2c_sda(vision_i2c.byte&0x80u);
}
static void vision_i2c_rising(unsigned sda)
{
    switch(vision_i2c.phase) {
    case SI_ADDRESS:case SI_RECEIVE:
        vision_i2c.byte=(uint8_t)((vision_i2c.byte<<1)|sda);
        if(++vision_i2c.bits<8u)break;
        vision_i2c.ack_seen=0;
        if(vision_i2c.phase==SI_ADDRESS) {
            if((vision_i2c.byte>>1)!=VISION_I2C_ADDRESS){vision_i2c.phase=SI_IGNORE;break;}
            vision_i2c.active=(vision_i2c.byte&1u) ? 2u : 1u;
            vision_i2c.rx_pos=0;vision_i2c.phase=SI_ADDRESS_ACK;
        } else {
            if(vision_i2c.rx_pos<VISION_I2C_MAILBOX)vision_i2c.rx[vision_i2c.rx_pos]=vision_i2c.byte;
            if(vision_i2c.rx_pos<=VISION_I2C_MAILBOX)vision_i2c.rx_pos++;
            vision_i2c.phase=SI_RECEIVE_ACK;
        }
        break;
    case SI_ADDRESS_ACK:case SI_RECEIVE_ACK:vision_i2c.ack_seen=1;break;
    case SI_SEND:
        if(++vision_i2c.bits==8u){vision_i2c.phase=SI_SEND_ACK;vision_i2c.ack_seen=0;}
        break;
    case SI_SEND_ACK:
        vision_i2c.ack_seen=1;
        if(sda) {
            if(vision_i2c.tx_pos==VISION_I2C_MAILBOX) {
                if(vision_i2c.staged_generation==vision_i2c.generation) {
                    vision_i2c.tail=(uint8_t)((vision_i2c.tail+vision_i2c.tx_count)%VISION_I2C_TX_RING);
                    if(vision_i2c.tx_count && vision_i2c.tail==vision_i2c.head)vision_i2c.tx_drained_ms=vision_time_ms();
                }
                vision_i2c.reads++;
            } else vision_i2c.invalid++;
            vision_i2c.active=0;vision_i2c.phase=SI_IGNORE;
        }
        break;
    default:break;
    }
}
static void vision_i2c_falling(void)
{
    switch(vision_i2c.phase) {
    case SI_ADDRESS_ACK:
        if(!vision_i2c.ack_seen) {
            if(vision_i2c.active==2u)vision_i2c_prepare_tx();
            vision_i2c_sda(0);
        } else if(vision_i2c.active==2u)vision_i2c_load_tx();
        else {vision_i2c.phase=SI_RECEIVE;vision_i2c.bits=vision_i2c.byte=0;vision_i2c_sda(1);}
        break;
    case SI_RECEIVE_ACK:
        if(!vision_i2c.ack_seen)vision_i2c_sda(0);
        else {vision_i2c.phase=SI_RECEIVE;vision_i2c.bits=vision_i2c.byte=0;vision_i2c_sda(1);}
        break;
    case SI_SEND:vision_i2c_sda(vision_i2c.byte&(0x80u>>vision_i2c.bits));break;
    case SI_SEND_ACK:
        if(!vision_i2c.ack_seen)vision_i2c_sda(1);else vision_i2c_load_tx();
        break;
    case SI_IGNORE:vision_i2c_sda(1);break;
    default:break;
    }
}
static void vision_i2c_edges(void)
{
    uint32_t pending=EXTI->PR&(EXTI_Line15|EXTI_Line7);
    unsigned scl=(GPIOG->IDR&GPIO_Pin_15)!=0,sda=(GPIOD->IDR&GPIO_Pin_7)!=0;
    /* 空闲总线的START和首个下降沿可能一起挂起；SDA此时甚至已是地址最高位。
     * 只在空闲且上一SCL为高时恢复START，不能把事务内的数据边沿当作新地址。 */
    if((pending&(EXTI_Line15|EXTI_Line7))==(EXTI_Line15|EXTI_Line7)
        && !scl && vision_i2c.last_scl && vision_i2c.phase==SI_IDLE) {
        vision_i2c.bits=vision_i2c.byte=0;vision_i2c.phase=SI_ADDRESS;
    }
    if(pending&EXTI_Line15) {
        EXTI_ClearITPendingBit(EXTI_Line15);
        if(scl==vision_i2c.last_scl) {
            /* EXTI仅保存一位待处理中断，整脉冲合并后不能伪造已接收字节。 */
            vision_i2c.last_error_ms=vision_time_ms();vision_i2c.error_phase=vision_i2c.phase;
            vision_i2c.error_bits=vision_i2c.bits;vision_i2c.error_rx_pos=vision_i2c.rx_pos;
            vision_i2c.bus_errors++;vision_receive_byte(0,1);
            vision_i2c.active=0;vision_i2c.phase=SI_IGNORE;
        } else {
            vision_i2c.last_scl=(uint8_t)scl;
            if(scl)vision_i2c_rising(sda);
            else {
                GPIO_ResetBits(GPIOG,GPIO_Pin_15); /* 已到低电平才伸钟，不截断高脉冲。 */
                vision_i2c_falling();GPIO_SetBits(GPIOG,GPIO_Pin_15);
            }
        }
    }
    if(pending&EXTI_Line7) {
        EXTI_ClearITPendingBit(EXTI_Line7);
        /* 空闲START已在上面恢复；事务内与时钟一起挂起的SDA边沿仍按数据处理。 */
        if(scl && !(pending&EXTI_Line15)) {
            vision_i2c_boundary();vision_i2c.bits=vision_i2c.byte=0;
            vision_i2c.phase=sda ? SI_IDLE : SI_ADDRESS;
        }
    }
}
#ifndef CHASSIS_OBSTACLE_TEST
void EXTI15_10_IRQHandler(void) {vision_i2c_edges();start_scan_echo_irq();}
void EXTI9_5_IRQHandler(void) {vision_i2c_edges();}
#endif
static void vision_i2c_init(void)
{
    GPIO_InitTypeDef gpio;EXTI_InitTypeDef edge;NVIC_InitTypeDef irq;
    memset(&vision_i2c,0,sizeof(vision_i2c));
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    GPIO_SetBits(GPIOG,GPIO_Pin_15);GPIO_SetBits(GPIOD,GPIO_Pin_7);
    GPIO_StructInit(&gpio);gpio.GPIO_Mode=GPIO_Mode_OUT;gpio.GPIO_OType=GPIO_OType_OD;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;gpio.GPIO_PuPd=GPIO_PuPd_UP;
    gpio.GPIO_Pin=GPIO_Pin_15;GPIO_Init(GPIOG,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_7;GPIO_Init(GPIOD,&gpio);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG,EXTI_PinSource15);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource7);
    EXTI_StructInit(&edge);edge.EXTI_Line=EXTI_Line15|EXTI_Line7;
    edge.EXTI_Mode=EXTI_Mode_Interrupt;edge.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
    edge.EXTI_LineCmd=ENABLE;EXTI_ClearITPendingBit(edge.EXTI_Line);EXTI_Init(&edge);
    vision_i2c.last_scl=(GPIOG->IDR&GPIO_Pin_15)!=0;
    irq.NVIC_IRQChannel=EXTI15_10_IRQn;irq.NVIC_IRQChannelPreemptionPriority=0;
    irq.NVIC_IRQChannelSubPriority=0;irq.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&irq);
    irq.NVIC_IRQChannel=EXTI9_5_IRQn;NVIC_Init(&irq);
}
#endif
