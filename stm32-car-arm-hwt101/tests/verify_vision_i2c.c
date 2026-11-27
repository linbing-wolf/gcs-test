/* 主机逐边沿驱动真实软件从机；开漏线与EXTI待处理位独立建模，无物理时序保证。 */
#include "../src/vision_link.c"
#include <stdlib.h>
static unsigned scan_irq_calls;
void start_scan_echo_irq(void)
{
    if(EXTI->PR&(1u<<10)) {EXTI_ClearITPendingBit(1u<<10);++scan_irq_calls;}
}
static unsigned assertions,primask,irq_mask,gpio_mask,map_mask,master_scl=1,master_sda=1,dispatching;
static unsigned gpio_sda_high_changes,stretch_calls;
static uint32_t ahb_clocks,apb_clocks,edge_mask;
static uint32_t test_clock_ms;
uint32_t vision_time_ms(void) {return test_clock_ms;}
static char output[16384];
GPIO_TypeDef test_gpiog,test_gpiod;EXTI_TypeDef test_exti;
#define CHECK(x) do {assertions++;if(!(x)){fprintf(stderr,"FAIL line %d: %s phase=%u bits=%u rx=%u tx=%u\n",__LINE__,#x,vision_i2c.phase,vision_i2c.bits,vision_i2c.rx_pos,vision_i2c.tx_pos);exit(1);}} while(0)
static void lines(void)
{
    uint32_t scl=master_scl && (GPIOG->ODR&GPIO_Pin_15),sda=master_sda && (GPIOD->ODR&GPIO_Pin_7);
    if(!!scl!=!!(GPIOG->IDR&GPIO_Pin_15))EXTI->PR|=EXTI_Line15;
    if(!!sda!=!!(GPIOD->IDR&GPIO_Pin_7))EXTI->PR|=EXTI_Line7;
    GPIOG->IDR=scl ? GPIO_Pin_15 : 0;GPIOD->IDR=sda ? GPIO_Pin_7 : 0;
}
void RCC_AHB1PeriphClockCmd(uint32_t c,int e) {CHECK(e==ENABLE);ahb_clocks|=c;}
void RCC_APB2PeriphClockCmd(uint32_t c,int e) {CHECK(e==ENABLE);apb_clocks|=c;}
void GPIO_StructInit(GPIO_InitTypeDef *g) {memset(g,0,sizeof(*g));}
void GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g)
{
    CHECK(g->GPIO_Mode==GPIO_Mode_OUT && g->GPIO_OType==GPIO_OType_OD && g->GPIO_PuPd==GPIO_PuPd_UP);
    if(p==GPIOG){CHECK(g->GPIO_Pin==GPIO_Pin_15);gpio_mask|=1;}else {CHECK(p==GPIOD && g->GPIO_Pin==GPIO_Pin_7);gpio_mask|=2;}
}
void GPIO_SetBits(GPIO_TypeDef *p,uint16_t pins)
{
    if(p==GPIOD && !(p->ODR&pins) && (GPIOG->IDR&GPIO_Pin_15))gpio_sda_high_changes++;
    p->ODR|=pins;lines();
}
void GPIO_ResetBits(GPIO_TypeDef *p,uint16_t pins)
{
    if(p==GPIOD && (p->ODR&pins) && (GPIOG->IDR&GPIO_Pin_15))gpio_sda_high_changes++;
    if(p==GPIOG){CHECK(!(p->IDR&pins));stretch_calls++;}
    p->ODR&=~pins;lines();
}
void SYSCFG_EXTILineConfig(unsigned port,unsigned pin)
{if(port==6){CHECK(pin==15);map_mask|=1;}else {CHECK(port==3 && pin==7);map_mask|=2;}}
void EXTI_StructInit(EXTI_InitTypeDef *e) {memset(e,0,sizeof(*e));}
void EXTI_Init(EXTI_InitTypeDef *e)
{CHECK(e->EXTI_Mode==EXTI_Mode_Interrupt && e->EXTI_Trigger==EXTI_Trigger_Rising_Falling && e->EXTI_LineCmd==ENABLE);edge_mask=e->EXTI_Line;}
void EXTI_ClearITPendingBit(uint32_t p) {EXTI->PR&=~p;}
void NVIC_Init(NVIC_InitTypeDef *irq)
{CHECK(irq->NVIC_IRQChannelCmd==ENABLE && !irq->NVIC_IRQChannelPreemptionPriority && !irq->NVIC_IRQChannelSubPriority);if(irq->NVIC_IRQChannel==40)irq_mask|=1;else {CHECK(irq->NVIC_IRQChannel==23);irq_mask|=2;}}
uint32_t __get_PRIMASK(void) {return primask;}
void __disable_irq(void) {primask=1;}
void __enable_irq(void) {primask=0;}
void __set_PRIMASK(uint32_t p) {primask=p;}
void arm_output(const char *s) {CHECK(strlen(output)+strlen(s)<sizeof(output));strcat(output,s);}
static void dispatch(void)
{
    unsigned count=0;CHECK(!dispatching && !primask);dispatching=1;
    while(EXTI->PR&(EXTI_Line7|EXTI_Line15)) {
        CHECK(count++<16);
        if(EXTI->PR&EXTI_Line15) {EXTI->PR|=1u<<10;EXTI15_10_IRQHandler();}
        else EXTI9_5_IRQHandler();
    }
    dispatching=0;
}
static void scl(unsigned high) {master_scl=high;lines();dispatch();}
static void sda(unsigned high) {master_sda=high;lines();dispatch();}
static void reset_test(void)
{
    memset(&test_gpiog,0,sizeof(test_gpiog));memset(&test_gpiod,0,sizeof(test_gpiod));EXTI->PR=0;
    primask=irq_mask=gpio_mask=map_mask=ahb_clocks=apb_clocks=edge_mask=0;master_scl=master_sda=1;output[0]=0;
    test_clock_ms=0;
    GPIOG->ODR=GPIO_Pin_15;GPIOD->ODR=GPIO_Pin_7;lines();EXTI->PR=0;vision_init();
    gpio_sda_high_changes=stretch_calls=0;
    CHECK(ahb_clocks==(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOD) && apb_clocks==RCC_APB2Periph_SYSCFG);
    CHECK(irq_mask==3 && gpio_mask==3 && map_mask==3 && edge_mask==(EXTI_Line7|EXTI_Line15));
    CHECK(vision_i2c.last_scl==1 && !EXTI->PR);
}
static void start(void) {sda(1);scl(1);sda(0);}
static void stop(void) {scl(0);sda(0);scl(1);sda(1);CHECK(vision_i2c.phase==SI_IDLE);}
static unsigned send_byte(uint8_t byte)
{
    unsigned bit,ack;
    for(bit=0;bit<8;bit++){scl(0);sda(!!(byte&(0x80u>>bit)));scl(1);}
    scl(0);sda(1);scl(1);ack=!(GPIOD->IDR&GPIO_Pin_7);scl(0);return ack;
}
static uint8_t read_byte(unsigned ack)
{
    unsigned bit;uint8_t byte=0;
    for(bit=0;bit<8;bit++){scl(1);byte=(uint8_t)((byte<<1)|!!(GPIOD->IDR&GPIO_Pin_7));scl(0);}
    sda(!ack);scl(1);scl(0);sda(1);return byte;
}
static void address(unsigned read) {start();CHECK(send_byte((uint8_t)((0x42u<<1)|read)));}
static void master_read(unsigned n,uint8_t *out)
{unsigned i;CHECK(n>0);address(1);for(i=0;i<n;i++)out[i]=read_byte(i+1<n);stop();}
static void write_body(const uint8_t *data,unsigned n) {unsigned i;for(i=0;i<n;i++)CHECK(send_byte(data[i]));}
static void master_write(const uint8_t *data,unsigned n) {address(0);write_body(data,n);stop();}
static void pump(unsigned ticks) {unsigned i;for(i=0;i<ticks;i++){test_clock_ms++;vision_update(test_clock_ms);}}
static uint8_t crc(const uint8_t *p,unsigned n)
{uint8_t c=0;unsigned i,b;for(i=0;i<n;i++){c^=p[i];for(b=0;b<8;b++)c=(uint8_t)((c<<1)^((c&128)?7:0));}return c;}
static unsigned frame(const char *s,uint8_t *out)
{unsigned n=(unsigned)strlen(s);CHECK(n<=VISION_PAYLOAD_MAX);out[0]=0xaa;out[1]=0x55;out[2]=(uint8_t)n;memcpy(out+3,s,n);out[n+3]=crc(out+2,n+1);out[n+4]=13;return n+5;}
static void reply(const char *s,unsigned split)
{
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32];unsigned n=frame(s,bytes),at=0;uint32_t base=test_clock_ms+1;
    while(at<n){unsigned count=n-at;if(count>split)count=split;memset(box,0xa5,32);box[0]=(uint8_t)count;memcpy(box+1,bytes+at,count);test_clock_ms=base+at;master_write(box,32);vision_update(test_clock_ms);at+=count;}
}
static void mailbox_and_ready_tests(void)
{
    uint8_t box[64],expected[64];unsigned n,before;
    reset_test();master_read(32,box);CHECK(!memcmp(box,(uint8_t[32]){0},32));
    reply("READY",31);CHECK(vision.frames==1 && !strcmp(vision_last_message(),"READY"));
    before=vision.frames;CHECK(vision_command("vision range",0));pump(20);CHECK(!vision_tx_complete());
    n=frame("R",expected);master_read(31,box);CHECK(vision_i2c.invalid==1 && !vision_tx_complete());
    master_read(32,box);CHECK(box[0]==n && !memcmp(box+1,expected,n) && vision_tx_complete());
    for(unsigned i=n+1;i<32;i++)CHECK(!box[i]);
    reply("READY",3);CHECK(vision.frames==before+1 && !strcmp(vision_last_message(),"READY"));
    printf("I2C_READY startup_frames=%u after_R_frames=%lu read_transactions=%lu\n",before,(unsigned long)vision.frames,(unsigned long)vision_i2c.reads);
    vision_request_align('D',2,0);pump(20);master_read(32,box);CHECK(box[0]==6 && box[4]=='D');
    reply("MOVEXY,12,-9",1);reply("DISC,320,240,0,1",31);
    {int16_t x,y;CHECK(vision_align_state()==VISION_PICK_READY && vision_motion_peek_xy(&x,&y,NULL) && x==12 && y==-9);vision_motion_take();}
    CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
    puts("PASS mailbox/READY: startup consumed before manual R, fresh READY, D/MOVEXY/terminal FIFO; padding ignored");
}
static void material_h_mailbox_tests(void)
{
    uint8_t box[32],first,second;int16_t x,y;
    reset_test();CHECK(vision_command("vision h",0));pump(20);
    reply("HCOLORS,3,1,1",3);reply("HDISC,300,240,-20,3,1,1",7);
    CHECK(vision_align_state()==VISION_PICK_WAITING && !vision.material_colors[0]);
    master_read(31,box);CHECK(!vision_tx_complete());
    reply("HDISC,300,240,-20,3,1,1",31);CHECK(vision_align_state()==VISION_PICK_WAITING);
    master_read(32,box);CHECK(box[0]==6 && box[4]=='H' && vision_tx_complete());
    reply("HCOLORS,3,1,1",1);CHECK(!vision_material_colors(NULL,NULL));
    reply("MOVEXY,-12,9",7);reply("HDISC,300,240,-20,1,3,1",5);
    CHECK(vision_align_state()==VISION_PICK_WAITING);
    reply("HDISC,300,240,-20,3,1,1",1);
    CHECK(vision_material_colors(&first,&second) && first==3 && second==1 && vision_material_direction()==1);
    CHECK(vision_motion_peek_xy(&x,&y,NULL) && x==-12 && y==9);vision_motion_take();
    vision_request_align('D',2,0);CHECK(!vision_material_colors(NULL,NULL));pump(20);master_read(32,box);
    reply("DISC,320,240,0,1",3);CHECK(vision_align_state()==VISION_PICK_READY);
    vision_cancel_pick();CHECK(!vision_material_colors(NULL,NULL));
    CHECK(!gpio_sda_high_changes && !vision_i2c.bus_errors);
    puts("PASS H mailbox: actual-delivery/short-read gate, split HCOLORS/HDISC, matching order/direction, retained FIFO and next-session isolation");
}
static void material_hdisc2_mailbox_tests(void)
{
    static const char *const valid[]={
        "HDISC2,1,0",
        "HDISC2,0,1,4,0,479",
        "HDISC2,1,3,1,100,230,2,300,250,3,500,240",
        "HDISC2,0,4,4,100,179,2,300,250,3,500,240,1,639,479"
    };
    static const char *const invalid[]={
        "HDISC20,0,0","HDISC2","HDISC2,2,0","HDISC2,01,0","HDISC2,+1,0","HDISC2,1,5",
        "HDISC2,1,-1","HDISC2,1,0,2,1,2","HDISC2,0,2,1,1,2","HDISC2,0,1,1,1",
        "HDISC2,0,2,2,1,2,2,3,4","HDISC2,0,1,0,1,2","HDISC2,0,1,5,1,2",
        "HDISC2,0,1,1,640,2","HDISC2,0,1,1,3,480","HDISC2,0,1,1,-1,2",
        "HDISC2,0,1,1,30.5,2","HDISC2,0,1,1, 1,2","HDISC2,0,1,1,1,2,"
    };
    uint8_t box[32],bytes[VISION_PAYLOAD_MAX+5u],count;
    VisionMaterial samples[4];
    unsigned i,n;
    int16_t dx,dy;
    reset_test();
    vision_request_align('H',2,0);pump(20);
    master_read(31,box);
    reply(valid[3],31);CHECK(vision_align_state()==VISION_PICK_WAITING);
    master_read(32,box);CHECK(box[0]==6 && box[4]=='H');
    for(i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        reply(invalid[i],31);
        CHECK(vision_align_state()==VISION_PICK_WAITING);
    }
    n=frame(valid[3],bytes);CHECK(n>31u && n<=55u);bytes[n-2u]^=1;
    memset(box,0,sizeof(box));box[0]=31;memcpy(box+1,bytes,31);
    master_write(box,32);pump(1);CHECK(vision_align_state()==VISION_PICK_WAITING);
    memset(box,0,sizeof(box));box[0]=(uint8_t)(n-31);memcpy(box+1,bytes+31,n-31);
    master_write(box,32);pump(1);
    CHECK(vision.crc_errors==1 && vision_align_state()==VISION_PICK_WAITING);
    reply("HCOLORS,3,2,1",31);reply("BUSY",31);reply("READY",31);reply("MOVEXY,-7,11",31);
    CHECK(vision_align_state()==VISION_PICK_WAITING);
    reply(valid[3],31);
    CHECK(vision_align_state()==VISION_PICK_READY && vision_material_h_result(&count,samples)==0);
    CHECK(count==4 && samples[0].color==4 && samples[0].state==1 && samples[3].color==1);
    CHECK(samples[3].x==639 && samples[3].y==479 && !vision_material_state_result(NULL));
    CHECK(!vision_material_colors(NULL,NULL) && vision_material_direction()==-1);
    CHECK(vision_motion_peek_xy(&dx,&dy,NULL) && dx==-7 && dy==11);
    for(i=0;i<3;i++) {
        vision_request_align('H',2,test_clock_ms);pump(20);master_read(32,box);
        if(i==2)CHECK(frame(valid[i],bytes)>31u);
        reply(valid[i],31);
        CHECK(vision_align_state()==VISION_PICK_READY && !vision_motion_peek_xy(NULL,NULL,NULL));
        CHECK(vision_material_h_result(&count,samples)==(i!=1) && count==(i==2 ? 3u : i));
        if(i==1)CHECK(samples[0].color==4 && samples[0].x==0 && samples[0].y==479);
    }
    vision_request_align('H',2,test_clock_ms);pump(20);master_read(32,box);
    vision_cancel_pick();vision_request_align('H',2,test_clock_ms);pump(20);master_read(32,box);
    CHECK(box[0]==6 && box[4]=='R');
    reply(valid[3],31);
    CHECK(vision_align_state()==VISION_PICK_WAITING && vision_material_h_result(NULL,NULL)<0);
    reply("READY",31);pump(20);master_read(32,box);CHECK(box[0]==6 && box[4]=='H');
    reply(valid[1],31);
    CHECK(vision_material_h_result(&count,samples)==0 && count==1);
    puts("PASS HDISC2: 0/1/3/4 colors, strict fields and CRC, split I2C, H delivery, MOVEXY FIFO and R/READY");
}
static void material_j_mailbox_tests(void)
{
    const char *payload="MSTATE,1,4,1,320,240,1,2,320,240,1,3,320,240,1,4,320,240,1";
    uint8_t box[32];VisionMaterial samples[4];uint32_t before;
    reset_test();vision_request_material_state(0);pump(20);
    reply(payload,31);CHECK(vision_material_state()==VISION_PICK_WAITING);
    master_read(31,box);CHECK(!vision_tx_complete());
    reply(payload,1);CHECK(vision_material_state()==VISION_PICK_WAITING);
    master_read(32,box);CHECK(box[0]==6 && box[4]=='J' && vision_tx_complete());
    reply("MSTATE,0,0",31);CHECK(vision_material_state()==VISION_PICK_WAITING);
    before=vision_i2c.writes;reply(payload,31);
    CHECK(vision_i2c.writes-before==3 && vision_material_state()==VISION_PICK_READY);
    CHECK(vision_material_state_result(samples)==4 && samples[3].color==4 && samples[3].state==1);
    vision_request_material_state(100);pump(20);master_read(32,box);
    vision_cancel_pick();vision_cancel_pick();pump(20);master_read(32,box);
    CHECK(box[0]==6 && box[4]=='R' && vision_material_state()==VISION_PICK_IDLE);
    reply(payload,31);CHECK(vision_material_state()==VISION_PICK_IDLE);
    CHECK(!gpio_sda_high_changes && !vision_i2c.bus_errors);
    puts("PASS J mailbox:58-byte payload/63-byte frame across three mailboxes, short-read gate and cancellation R");
}
static void cancellation_ready_tests(void)
{
    uint8_t box[32];VisionMaterial samples[4];
    reset_test();vision_request_align('H',2,0);pump(20);master_read(32,box);
    CHECK(box[0]==6 && box[4]=='H');
    vision_cancel_pick();pump(2);vision_cancel_pick();pump(20);
    vision_request_material_state(100);pump(20);
    master_read(31,box);CHECK(!vision_tx_complete());
    reply("READY",31);reply("HDISC,371,298,51,2,1,0",31);
    CHECK(cancel_pending && vision_material_state()==VISION_PICK_WAITING);
    master_read(32,box);CHECK(box[0]==6 && box[4]=='R' && vision_tx_complete());
    pump(20);master_read(32,box);CHECK(!box[0] && cancel_pending);
    reply("BUSY",31);reply("MSTATE,1,1,1,310,315,1",31);
    CHECK(vision_material_state()==VISION_PICK_WAITING && !vision_material_state_result(samples));
    reply("READY",3);pump(20);
    CHECK(!cancel_pending && !vision_tx_complete());
    reply("MSTATE,1,1,1,310,315,1",31);CHECK(vision_material_state()==VISION_PICK_WAITING);
    master_read(32,box);CHECK(box[0]==6 && box[4]=='J' && vision_tx_complete());
    reply("MSTATE,1,2,3,491,32,1,1,310,315,1",31);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==2);
    CHECK(samples[0].color==3 && samples[1].color==1);
    CHECK(!gpio_sda_high_changes && !vision_i2c.bus_errors);
    puts("PASS R/READY mailbox: partial R survives duplicate cancel, short-read/early READY gates, no new J until READY, stale terminal ignored");
}
static void silent_j_fragment_tests(void)
{
    const char *payload="MSTATE,1,4,3,491,132,1,1,310,315,1,4,136,144,1,2,320,240,1";
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32];unsigned n,at=0,part=0;VisionMaterial samples[4];
    reset_test();vision_request_material_state(0);pump(20);master_read(32,box);
    CHECK(box[0]==6 && box[4]=='J');vision_update(60000);master_read(32,box);
    CHECK(!box[0] && vision_material_state()==VISION_PICK_WAITING);
    n=frame(payload,bytes);CHECK(n==63);
    while(at<n) {
        unsigned count=n-at;if(count>31)count=31;
        memset(box,0,32);box[0]=(uint8_t)count;memcpy(box+1,bytes+at,count);
        test_clock_ms=60010+150*part++;master_write(box,32);vision_update(test_clock_ms);at+=count;
        if(at<n)CHECK(vision_material_state()==VISION_PICK_WAITING);
    }
    CHECK(part==3 && vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==4);
    CHECK(samples[0].color==3 && samples[1].color==1 && samples[2].color==4 && samples[3].color==2);
    puts("PASS silent J: 60s no response/no resend; 63-byte unordered result across three mailboxes over300ms, each gap150ms");
}
static void j_fault_then_complete_test(void)
{
    const char *payload="MSTATE,1,4,1,320,240,1,2,320,240,1,3,320,240,1,4,320,240,1";
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32]={0};unsigned n,at,mode;
    for(mode=0;mode<2;mode++) {
        reset_test();vision_request_material_state(0);pump(20);master_read(32,box);
        /* 无效邮箱或真实丢SCL脉冲后，下一完整结果在前台运行前写完。 */
        if(!mode){memset(box,0,32);master_write(box,31);}
        else {
            address(0);scl(1);master_scl=0;lines();master_scl=1;lines();dispatch();stop();
        }
        n=frame(payload,bytes);at=0;
        while(at<n) {
            unsigned count=n-at;if(count>31)count=31;
            test_clock_ms=30+at;memset(box,0,32);box[0]=(uint8_t)count;memcpy(box+1,bytes+at,count);
            master_write(box,32);at+=count;
        }
        vision_update(100);
        CHECK((!mode ? vision_i2c.invalid==1 : vision_i2c.bus_errors==1)
            && vision.material_state==VISION_PICK_READY && vision.material_count==4);
    }
    puts("PASS J recovery: invalid mailbox/lost SCL pulse do not discard a later complete queued result");
}
static void j_delayed_foreground_test(void)
{
    const char *payload="MSTATE,1,4,1,320,240,1,2,320,240,1,3,320,240,1,4,320,240,1";
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32];unsigned n,at=0,part=0;
    reset_test();vision_request_material_state(0);pump(6);master_read(32,box);n=frame(payload,bytes);
    while(at<n) {
        unsigned count=n-at;if(count>31)count=31;
        test_clock_ms=10+50*part;memset(box,0,32);box[0]=(uint8_t)count;memcpy(box+1,bytes+at,count);
        master_write(box,32);at+=count;
        if(!part)vision_update(test_clock_ms);
        part++;
    }
    /* 实际收片10/60/110ms，主循环400ms才来，不能误判相邻片超时。 */
    vision_update(400);
    CHECK(vision.material_state==VISION_PICK_READY && !vision.bad_frames);
    puts("PASS J timing: queued fragments arriving within200ms survive a delayed foreground update");
}
static void j_arrival_during_update_test(void)
{
    const char *payload="MSTATE,1,2,1,310,315,1,3,491,32,1";
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32]={0};unsigned n;
    reset_test();vision_request_material_state(0);pump(20);master_read(32,box);n=frame(payload,bytes);
    test_clock_ms=110;box[0]=20;memcpy(box+1,bytes,20);master_write(box,32);
    vision_update(100);CHECK(vision.length==20 && !vision.bad_frames);
    test_clock_ms=120;memset(box,0,32);box[0]=(uint8_t)(n-20);memcpy(box+1,bytes+20,n-20);master_write(box,32);
    vision_update(130);CHECK(vision.material_state==VISION_PICK_READY && !vision.bad_frames);
    puts("PASS J timing: an interrupt arriving after the update entry timestamp is not a wrapped timeout");
}
static void j_gap_loss_duplicate_tests(void)
{
    const char *payload="MSTATE,1,4,1,320,240,1,2,320,240,1,3,320,240,1,4,320,240,1";
    uint8_t bytes[VISION_PAYLOAD_MAX+5u],box[32];unsigned n,mode;
    for(mode=0;mode<4;mode++) {
        reset_test();vision_request_material_state(0);pump(6);master_read(32,box);pump(1);
        n=frame(payload,bytes);CHECK(n==63);
        if(mode!=3) {
            test_clock_ms=10;memset(box,0,32);box[0]=31;memcpy(box+1,bytes,31);master_write(box,32);vision_update(10);
        }
        if(mode!=1) {
            test_clock_ms=mode==0 ? 211 : 60;
            box[0]=31;memcpy(box+1,bytes+31,31);if(mode==2)box[5]^=1;
            master_write(box,32);vision_update(test_clock_ms);
        }
        test_clock_ms=mode==0 ? 220 : 110;
        memset(box,0,32);box[0]=1;box[1]=bytes[62];master_write(box,32);vision_update(test_clock_ms);
        CHECK(vision.material_state==VISION_PICK_WAITING);
        vision_update(500);
        CHECK((mode==2 && vision.crc_errors==1 && strstr(output,"reason=crc"))
            || (mode==3 && j_diag.orphans==32 && !j_diag.half_frames)
            || (mode<2 && j_diag.half_frames==1 && strstr(output,"reason=half_frame")));
        CHECK(vision_command("vision status",500));
        CHECK(strstr(output,"[vision J delta]") && strstr(output,"[vision J delivered]") && j_diag.chunks==((mode==1 || mode==3) ? 2u : 3u));
        if(mode==3)CHECK(strstr(output,"orphans=32"));
        /* 仅模拟相机重传同一次完整结果；不再发J，不把它当成已实现的ACK协议。 */
        reply(payload,31);CHECK(vision.material_state==VISION_PICK_READY);
        reply(payload,31);CHECK(vision.material_state==VISION_PICK_READY && j_diag.ignored==1);
        master_read(32,box);CHECK(!box[0]);
    }
    reset_test();vision_request_material_state(0);pump(20);master_read(32,box);pump(1);
    for(mode=0;mode<300;mode++)vision_receive_byte(0xaa,0);
    vision_update(100);CHECK(vision.material_state==VISION_PICK_WAITING && rx_ring_overflows && strstr(output,"reason=rx_ring_full"));
    reply(payload,31);CHECK(vision.material_state==VISION_PICK_READY);
    puts("PASS J receive failures: true201ms gap, missing middle mailbox, CRC corruption and ring full retain wait; exact-result resend recovers, duplicates never complete twice, no extra J");
}
static void legacy_isolation_gap_test(void)
{
    uint8_t box[32];const char *old_result="MSTATE,1,1,1,310,315,1";
    reset_test();vision_request_material_state(0);pump(20);master_read(32,box);
    vision_cancel_pick();pump(20);master_read(32,box);reply("READY",31);
    vision_request_material_state(100);pump(20);master_read(32,box);
    reply(old_result,31);
    /* 独立运行的待实现契约反例：旧协议没有任务号，这条旧终包仍会完成新J。 */
    CHECK(vision.material_state==VISION_PICK_WAITING);
}
static void j_queued_before_delivery_test(void)
{
    uint8_t bytes[64],box[32]={0};unsigned n;
    reset_test();vision_request_material_state(0);pump(20);
    n=frame("MSTATE,1,1,1,310,315,1",bytes);box[0]=(uint8_t)n;memcpy(box+1,bytes,n);
    test_clock_ms=30;master_write(box,32); /* 终包到达时J还没被相机读走。 */
    test_clock_ms=60;master_read(32,box);vision_update(100);
    CHECK(vision.material_state==VISION_PICK_WAITING && strstr(output,"arrived_before_request_delivery"));
    test_clock_ms=120;reply("MSTATE,1,1,1,310,315,1",31);CHECK(vision.material_state==VISION_PICK_READY);
    puts("PASS J delivery: a pre-delivery queued terminal cannot become valid merely because the main loop runs later");
}
static void recorded_j_gap_test(void)
{
    const char *payload="MSTATE,1,3,1,504,32,1,2,193,397,1,3,302,368,1";
    uint8_t bytes[64],box[32];unsigned n,mode;
    for(mode=0;mode<2;mode++) {
        reset_test();vision_request_material_state(0);pump(6);master_read(32,box);
        n=frame(payload,bytes);CHECK(n==50 && bytes[n-2]==0x40);
        test_clock_ms=1000;memset(box,0,32);box[0]=31;memcpy(box+1,bytes,31);
        master_write(box,32);vision_update(test_clock_ms);
        test_clock_ms=mode ? 1114 : 1211;memset(box,0,32);box[0]=19;memcpy(box+1,bytes+31,19);
        master_write(box,32);vision_update(test_clock_ms);
        CHECK(vision.material_state==(mode ? VISION_PICK_READY : VISION_PICK_WAITING));
        CHECK(j_diag.half_frames==(mode ? 0u : 1u));
        master_read(32,box);CHECK(!box[0]);
    }
    puts("PASS recorded J: exact50-byte/CRC0x40 result, observed211ms gap rejects; contiguous114ms gap accepts; no extra J");
}
static void split_and_wrap_tests(void)
{
    uint8_t box[64],bytes[VISION_PAYLOAD_MAX+5u],got[VISION_PAYLOAD_MAX+5u];char payload[VISION_PAYLOAD_MAX+1u];unsigned n,at=0,i,total=0,next=0;
    reset_test();memset(payload,'Q',VISION_PAYLOAD_MAX);payload[VISION_PAYLOAD_MAX]=0;n=frame(payload,bytes);
    CHECK(vision_send(payload));pump(200);
    while(at<n) {unsigned count=n-at;if(count>31)count=31;CHECK(!vision_tx_complete());master_read(32,box);CHECK(box[0]==count);memcpy(got+at,box+1,count);at+=count;}
    CHECK(!memcmp(got,bytes,n) && vision_tx_complete());
    reply(payload,31);CHECK(vision.frames==1 && !strcmp(vision_last_message(),payload));
    for(i=0;i<1000;i++) {
        if(!vision_i2c_tx_ready()) {
            master_read(32,box);for(unsigned j=1;j<=box[0];j++)CHECK(box[j]==(uint8_t)next++);total+=box[0];
        }
        vision_i2c_send_byte((uint8_t)i);
    }
    while(vision_i2c.head!=vision_i2c.tail){master_read(32,box);for(i=1;i<=box[0];i++)CHECK(box[i]==(uint8_t)next++);total+=box[0];}
    CHECK(total==1000 && next==1000);puts("PASS split/wrap: maximum101-byte frame across four mailboxes, exact stream order, backpressure and ring wrap");
}
static void bounds_and_recovery_tests(void)
{
    static const unsigned sizes[]={0,1,31,33,64};uint8_t box[64]={0},out[64];unsigned i,invalid;
    reset_test();
    start();CHECK(!send_byte(0x41u<<1));stop();CHECK(!vision_i2c.active && !vision_i2c.invalid);
    for(i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++){invalid=vision_i2c.invalid;master_write(box,sizes[i]);CHECK(vision_i2c.invalid==invalid+1 && !vision.frames);}
    box[0]=32;invalid=vision_i2c.invalid;master_write(box,32);CHECK(vision_i2c.invalid==invalid+1);
    box[0]=0;master_write(box,32);CHECK(vision_i2c.writes==1);
    for(i=1;i<32;i++){unsigned reads=vision_i2c.reads;vision_i2c_send_byte((uint8_t)i);master_read(i,out);CHECK(vision_i2c.reads==reads);master_read(32,out);CHECK(out[0]==1 && out[1]==i);}
    vision_i2c_send_byte(0x5a);master_read(33,out);CHECK(vision_i2c.head!=vision_i2c.tail);master_read(32,out);CHECK(out[0]==1 && out[1]==0x5a);
    CHECK(!gpio_sda_high_changes && stretch_calls>0 && !vision_i2c.bus_errors);
    /* 遮住一次完整SCL脉冲，EXTI位合并：仅报告传输错误，保留TX，并可在新START恢复。 */
    vision_i2c_send_byte(0xa5);address(0);scl(1);master_scl=0;lines();master_scl=1;lines();dispatch();
    CHECK(vision_i2c.bus_errors==1 && vision_i2c.phase==SI_IGNORE && vision_i2c.head!=vision_i2c.tail);
    stop();master_read(32,out);CHECK(out[0]==1 && out[1]==0xa5);pump(1);
    vision_request_pick(1,0);pump(20);master_read(32,out);reply("PICK,1,7,-3",31);CHECK(vision_pick_state()==VISION_PICK_READY);
    puts("PASS address/bounds: wrong address NACK, RX sizes/count, short1..31 and long33 reads retain TX, missed clock pulse reports error and next START recovers");
}
static void concurrency_tests(void)
{
    uint8_t box[32]={0},bytes[64],out[32];unsigned n,i,old_tail;
    reset_test();vision_i2c_send_byte(0x11);address(1);out[0]=read_byte(1);vision_i2c_send_byte(0x22);
    for(i=1;i<32;i++)out[i]=read_byte(i+1<32);
    stop();CHECK(out[0]==1 && out[1]==0x11);master_read(32,out);CHECK(out[0]==1 && out[1]==0x22);
    vision_request_pick(1,0);pump(20);address(1);for(i=0;i<8;i++)read_byte(1);
    vision_cancel_pick();CHECK(vision_command("vision range",0));pump(20);old_tail=vision_i2c.tail;
    for(i=8;i<32;i++)read_byte(i+1<32);
    stop();CHECK(vision_i2c.tail==old_tail && !vision_tx_complete());
    master_read(32,out);CHECK(out[0]==6 && out[4]=='R' && vision_tx_complete());
    n=frame("READY",bytes);box[0]=(uint8_t)n;memcpy(box+1,bytes,n);
    address(0);write_body(box,32);address(1);for(i=0;i<32;i++)read_byte(i+1<32);stop();pump(1);
    CHECK(vision.frames==1 && !strcmp(vision_last_message(),"READY"));
    vision_request_pick(1,0);pump(20);reply("PICK,1,7,-3",31);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    master_read(32,out);reply("PICK,1,7,-3",31);CHECK(vision_pick_state()==VISION_PICK_READY);
    CHECK(!gpio_sda_high_changes && !vision_i2c.bus_errors);
    reset_test();for(i=0;i<127;i++)vision_i2c_send_byte((uint8_t)i);
    CHECK(!vision_i2c_tx_ready());CHECK(vision_send("D"));pump(20);CHECK(tx_offset==0);
    for(i=0;i<5;i++){master_read(32,out);pump(20);}
    while(!vision_tx_complete()){master_read(32,out);pump(20);}
    puts("PASS concurrency: frozen in-flight mailbox, new TX retained, cancel generation, write/read repeated START, actual-delivery gate, backpressure");
}
static unsigned send_byte_coalesced(uint8_t byte)
{
    unsigned bit,ack;
    for(bit=0;bit<8;bit++) {
        master_scl=0;lines();master_sda=!!(byte&(0x80u>>bit));lines();dispatch();scl(1);
    }
    scl(0);sda(1);scl(1);ack=!(GPIOD->IDR&GPIO_Pin_7);scl(0);return ack;
}
static void coalesced_start_tests(void)
{
    unsigned irq_first,first_bit_ready,direction,i,ack;
    uint8_t box[32]={0},bytes[64],out[32];
    for(irq_first=0;irq_first<2;irq_first++)for(first_bit_ready=0;first_bit_ready<2;first_bit_ready++)
    for(direction=0;direction<3;direction++) {
        uint8_t address_byte=(uint8_t)(direction==2 ? 0x82u : 0x84u+direction);
        reset_test();vision_i2c_send_byte(0x5a);
        /* START和首个下降沿都先挂起，主机可能已将SDA更新成地址最高位。 */
        master_sda=0;lines();master_scl=0;lines();
        if(first_bit_ready){master_sda=address_byte>>7;lines();}
        if(irq_first)EXTI9_5_IRQHandler();else EXTI15_10_IRQHandler();
        dispatch();CHECK(vision_i2c.phase==SI_ADDRESS && !vision_i2c.bits && !vision_i2c.bus_errors);
        ack=send_byte(address_byte);CHECK(ack==(direction!=2));
        if(direction==0) {
            box[0]=(uint8_t)frame("READY",bytes);memcpy(box+1,bytes,box[0]);write_body(box,32);stop();pump(1);
            CHECK(vision_i2c.writes==1 && vision.frames==1 && !strcmp(vision_last_message(),"READY"));
        } else if(direction==1) {
            for(i=0;i<32;i++)out[i]=read_byte(i+1<32);
            stop();CHECK(vision_i2c.reads==1 && out[0]==1 && out[1]==0x5a && vision_i2c.head==vision_i2c.tail);
        } else {stop();CHECK(vision_i2c.head!=vision_i2c.tail && !vision_i2c.reads && !vision_i2c.writes);}
        CHECK(!gpio_sda_high_changes && !vision_i2c.invalid && !vision_i2c.bus_errors);
    }
    /* 不能把事务内普通SDA/SCL合并边沿误当START；覆盖所有字节取值。 */
    for(i=0;i<256;i++) {
        unsigned j;reset_test();start();CHECK(send_byte_coalesced(0x84));
        box[0]=31;for(j=1;j<32;j++)box[j]=(uint8_t)(i+j);
        for(j=0;j<32;j++)CHECK(send_byte_coalesced(box[j]));
        stop();CHECK(vision_i2c.writes==1 && rx_head==32 && !vision_i2c.bus_errors);
        CHECK(rx_ring[0]==(VISION_RX_MAILBOX|31u));
        for(j=0;j<31;j++)CHECK(rx_ring[j+1]==box[j+1]);
    }
    /* 若连首个完整SCL脉冲也丢失，保留原错误语义，不能猜造地址位。 */
    reset_test();vision_i2c_send_byte(0x5a);master_sda=0;lines();master_scl=0;lines();
    master_sda=1;lines();master_scl=1;lines();dispatch();
    CHECK(vision_i2c.bus_errors==1 && vision_i2c.phase==SI_IGNORE && vision_i2c.head!=vision_i2c.tail);stop();
    puts("PASS delayed START: both IRQ entry orders, SDA before/after address MSB setup, write0x84 READY32/read0x85/wrong0x82; all256 data patterns keep normal parsing; lost full clock remains an error");
}
int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"delayed")){j_delayed_foreground_test();return 0;}
    if(argc>1 && !strcmp(argv[1],"legacy-isolation")){legacy_isolation_gap_test();return 0;}
    if(argc>1 && !strcmp(argv[1],"early-delivery")){j_queued_before_delivery_test();return 0;}
    reset_test();EXTI->PR|=1u<<10;EXTI15_10_IRQHandler();
    CHECK(scan_irq_calls==1 && !(EXTI->PR&(1u<<10)) && !vision_i2c.bus_errors);
    j_fault_then_complete_test();j_delayed_foreground_test();j_arrival_during_update_test();j_gap_loss_duplicate_tests();
    j_queued_before_delivery_test();
    recorded_j_gap_test();
    coalesced_start_tests();mailbox_and_ready_tests();material_h_mailbox_tests();material_hdisc2_mailbox_tests();material_j_mailbox_tests();cancellation_ready_tests();silent_j_fragment_tests();split_and_wrap_tests();bounds_and_recovery_tests();concurrency_tests();
    printf("PASS software I2C: %u assertions; edge/open-drain model only, no physical 10kHz timing acceptance\n",assertions);return 0;
}
