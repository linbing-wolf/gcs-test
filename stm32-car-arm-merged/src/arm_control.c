#include "arm_control.h"
#include "arm_config.h"
#include <string.h>
#include <stdio.h>
#ifdef ARM_HOST_TEST
#include "../tests/arm_host_io.h"
#else
#include "stm32f4xx.h"
#include "Emm_V5.h"
#endif

enum { ARM_IDLE, ARM_PREFLIGHT, ARM_SUBMIT, ARM_MOVING, ARM_FAULT, ARM_YAW_TEST };
enum { SLIDE = 1, LIFT = 2, YAW = 4, CLAW = 8, PLATFORM = 16 };
typedef struct { int slide, lift; float yaw, claw, platform; uint8_t mask; } ArmStep;
typedef struct {
    int64_t position, baseline;
    uint32_t pulses, status_ms, speed_ms, position_ms;
    int32_t rpm;
    uint8_t flags, samples, ack, submitted, dir;
} ArmAxis;
typedef struct {
    uint8_t axis, length, data[7];
    uint32_t ms;
} ArmFrame;
#define ARM_RX_SIZE 16u
static volatile ArmFrame arm_rx[ARM_RX_SIZE];
static volatile uint8_t arm_head, arm_tail, arm_rx_fault;
static struct {
    volatile uint8_t enabled;
    uint8_t referenced, state, sequence, step, count, submit;
    int slide, lift;
    float yaw, claw, platform;
    uint8_t platform_known;
    ArmStep target, single;
    ArmAxis axis[2]; /* 0=平推7号，1=升降5号。 */
    uint32_t phase_ms, move_ms, pwm_ms, poll_ms, query_ms, servo_ms;
    uint8_t query_axis, query_kind, waiting, poll;
    float start_yaw, start_claw, start_platform;
    uint16_t claw_us, yaw_us, platform_us; /* 最后实际提交给PWM的脉宽，不是位置反馈。 */
    uint16_t test_start_us, test_target_us;
} arm;

/* 队友Quick_Grab的分步路径：必须从最高且收回、云台初始角开始。
 * 同一步只移动一种机构，不照搬旧参考车的斜向三轴取料路径。 */
static const ArmStep pick_steps[] = {
    {0,0,0,ARM_CLAW_OPEN_DEG,0,CLAW},
    {0,0,ARM_PICK_YAW_DEG,0,0,YAW},
    {ARM_PICK_SLIDE_MM,0,0,0,0,SLIDE},
    {0,ARM_PICK_LIFT_MM,0,0,0,LIFT},
    {0,0,0,ARM_CLAW_CLOSE_DEG,0,CLAW}
};
/* 先抬起，再收回，最后转云台；保留夹爪状态，不自动松开物块。 */
static const ArmStep home_steps[] = {
    {0,ARM_LIFT_HOME_MM,0,0,0,LIFT},
    {ARM_SLIDE_HOME_MM,0,0,0,0,SLIDE},
    {0,0,ARM_YAW_HOME_DEG,0,0,YAW}
};
static const ArmStep place_steps[] = {
    {0,ARM_LIFT_HOME_MM,0,0,0,LIFT},
    {ARM_SLIDE_HOME_MM,0,0,0,0,SLIDE},
    {0,0,ARM_PLACE_YAW_DEG,0,0,YAW},
    {ARM_PLACE_SLIDE_MM,0,0,0,0,SLIDE},
    {0,ARM_PLACE_LIFT_MM,0,0,0,LIFT},
    {0,0,0,ARM_CLAW_OPEN_DEG,0,CLAW}
};
#define PICK_COUNT (sizeof(pick_steps)/sizeof(pick_steps[0]))
#define HOME_COUNT (sizeof(home_steps)/sizeof(home_steps[0]))
#define PLACE_COUNT (sizeof(place_steps)/sizeof(place_steps[0]))
static uint8_t axis_address(unsigned i) { return i ? ARM_LIFT_ADDRESS : ARM_SLIDE_ADDRESS; }
static float axis_ppm(unsigned i) { return i ? ARM_LIFT_PULSES_PER_MM : ARM_SLIDE_PULSES_PER_MM; }

/* 不启用旧PWM_Init：它会同时驱动PA2平台，并在上电时输出旧机构位置。
 * 本模块独占TIM2的CH2/CH3/CH4，使用主循环插值，不开TIM2中断。 */
static void arm_pwm_init(void)
{
#ifdef ARM_HOST_TEST
    entry_host_arm_pwm(0, 0);
#else
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 84 - 1; /* APB1定时器84MHz，1us/计数。 */
    timer.TIM_Period = 20000 - 1;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC3Init(TIM2, &oc);
    TIM_OC4Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, ENABLE);
#endif
}
static void arm_pwm(uint16_t claw, uint16_t yaw)
{
    arm.claw_us=claw;
    arm.yaw_us=yaw;
#ifdef ARM_HOST_TEST
    entry_host_arm_pwm(claw, yaw);
#else
    TIM_SetCompare2(TIM2, claw);
    TIM_SetCompare4(TIM2, yaw);
#endif
}
static void platform_pwm(uint16_t pulse)
{
    arm.platform_us=pulse;
#ifdef ARM_HOST_TEST
    entry_host_arm_platform_pwm(pulse);
#else
    TIM_SetCompare3(TIM2,pulse);
#endif
}
static void platform_pose(float angle)
{
    platform_pwm((uint16_t)(500.0f+angle*2000.0f/270.0f+0.5f));
}
static void servo_pose(float claw, float yaw)
{
    /* 与队友的Claw_PulseOf/Yuntai_PulseOf一致，初始0度对应1975us。
     * 云台不取模：超范围由命令拒绝，避免接缝附近隐式扫过整圈。 */
    float claw_pulse=500.0f+(claw-ARM_CLAW_OFFSET_DEG)*2000.0f/270.0f;
    float yaw_inner=yaw*ARM_YAW_DIRECTION+ARM_YAW_ZERO_DEG-ARM_YAW_OFFSET_DEG;
    if(yaw_inner<0.0f)yaw_inner=0.0f;
    if(yaw_inner>360.0f)yaw_inner=360.0f;
    arm_pwm((uint16_t)(claw_pulse+0.5f),
            (uint16_t)(500.0f+yaw_inner*2000.0f/360.0f+0.5f));
}
static float yaw_of_pulse(uint16_t pulse)
{
    return (((float)pulse-500.0f)*360.0f/2000.0f-ARM_YAW_ZERO_DEG+ARM_YAW_OFFSET_DEG)/ARM_YAW_DIRECTION;
}
void arm_init(void) { memset(&arm,0,sizeof(arm)); arm_head=arm_tail=arm_rx_fault=0; }
int arm_enabled(void) { return arm.enabled; }
int arm_parked(void)
{
    return arm.enabled && arm.referenced && arm.state==ARM_IDLE
        && arm.slide==ARM_SLIDE_HOME_MM && arm.lift==ARM_LIFT_HOME_MM
        && arm.yaw==ARM_YAW_HOME_DEG;
}
static void clear_arm_rx(int clear_fault)
{
    __disable_irq();
    arm_tail=arm_head;
    if(clear_fault)arm_rx_fault=0;
    __enable_irq();
}
void arm_stop(void)
{
    if (!arm.enabled) return;
    Emm_V5_Stop_Now(ARM_SLIDE_ADDRESS, false);
    Emm_V5_Stop_Now(ARM_LIFT_ADDRESS, false);
    arm.referenced=0; arm.platform_known=0; arm.state=ARM_FAULT; arm.sequence=0; arm.waiting=0;
    clear_arm_rx(1);
    /* 保持最后PWM和电机使能，避免立即松爪或升降失去保持力。
     * 舵机没有位置反馈，停止插值并不保证轴已物理停稳。 */
    arm_output("[arm stopped: pose invalid; PWM held; inspect before resuming; arm pose required for normal motion]\r\n");
}
static void arm_fail(const char *reason)
{
    arm_stop(); arm_output("[arm fault] "); arm_output(reason); arm_output("\r\n");
}
void arm_can_error(uint8_t flags) { if (arm.enabled) arm_rx_fault |= flags; }
void arm_can_rx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t length,const uint8_t *data,uint32_t now)
{
    uint8_t next, i, address=(uint8_t)(id>>8);
    if (!arm.enabled || !data || ide!=4 || rtr || (id&255u)
        || (id>>8)!=address || (address!=ARM_SLIDE_ADDRESS && address!=ARM_LIFT_ADDRESS)) return;
    if (!((length==3 && (data[0]==0xfd || data[0]==0x3a || data[0]==0xf3 || data[0]==0xfe))
        || (length==5 && data[0]==0x35 && data[1]<=1)
        || (length==7 && data[0]==0x36 && data[1]<=1)) || data[length-1]!=0x6b) return;
    next=(uint8_t)((arm_head+1u)%ARM_RX_SIZE);
    if (next==arm_tail) {arm_rx_fault=1;return;}
    arm_rx[arm_head].axis=(uint8_t)(address==ARM_LIFT_ADDRESS);
    arm_rx[arm_head].length=length; arm_rx[arm_head].ms=now;
    for (i=0;i<length;i++) arm_rx[arm_head].data[i]=data[i];
    arm_head=next;
}
static void arm_receive(void)
{
    static const uint8_t kinds[3]={0x3a,0x35,0x36};
    ArmFrame f;
    ArmAxis *a;
    uint32_t magnitude;
    uint8_t i;
    while (arm_tail!=arm_head) {
        f.axis=arm_rx[arm_tail].axis; f.length=arm_rx[arm_tail].length; f.ms=arm_rx[arm_tail].ms;
        for(i=0;i<f.length;i++) f.data[i]=arm_rx[arm_tail].data[i];
        arm_tail=(uint8_t)((arm_tail+1u)%ARM_RX_SIZE); a=&arm.axis[f.axis];
        if (f.data[0]==0xfd && a->submitted) {
            if(f.data[1]!=2) {arm_fail("position command rejected");return;} a->ack=1; continue;
        }
        if (!arm.waiting || f.axis!=arm.query_axis || f.data[0]!=kinds[arm.query_kind]
            || (int32_t)(f.ms-arm.query_ms)<0) continue;
        arm.waiting=0; a->samples |= (uint8_t)(1u<<arm.query_kind);
        if (arm.query_kind==0) { a->flags=f.data[1]; a->status_ms=f.ms;
            if (!(a->flags&1u) || (a->flags&12u)) {arm_fail("axis disabled/stalled/protected");return;}
        } else if (arm.query_kind==1) {
            a->rpm=(int32_t)(((uint16_t)f.data[2]<<8)|f.data[3]);
            if(f.data[1])a->rpm=-a->rpm;
            a->speed_ms=f.ms;
        } else {
            magnitude=((uint32_t)f.data[2]<<24)|((uint32_t)f.data[3]<<16)|((uint32_t)f.data[4]<<8)|f.data[5];
            a->position=f.data[1] ? -(int64_t)magnitude : (int64_t)magnitude; a->position_ms=f.ms;
        }
    }
}
static void arm_poll(uint32_t now)
{
    static const SysParams_t params[3]={S_FLAG,S_VEL,S_CPOS};
    if (arm.waiting) {
        if((uint32_t)(now-arm.query_ms)>ARM_FEEDBACK_TIMEOUT_MS) arm_fail("feedback timeout");
        return;
    }
    if((uint32_t)(now-arm.poll_ms)<ARM_POLL_MS)return;
    arm.query_axis=(uint8_t)(arm.poll/3u); arm.query_kind=(uint8_t)(arm.poll%3u);
    arm.poll=(uint8_t)((arm.poll+1u)%6u); arm.query_ms=arm.poll_ms=now; arm.waiting=1;
    Emm_V5_Read_Sys_Params(axis_address(arm.query_axis),params[arm.query_kind]);
}
static int pose_valid(int slide,int lift,float yaw,float claw)
{
    return slide>=ARM_SLIDE_MIN_MM && slide<=ARM_SLIDE_MAX_MM
        && lift>=ARM_LIFT_MIN_MM && lift<=ARM_LIFT_MAX_MM
        && yaw>=ARM_YAW_MIN_DEG && yaw<=ARM_YAW_MAX_DEG
        && claw>=ARM_CLAW_MIN_DEG && claw<=ARM_CLAW_MAX_DEG;
}
/* 毫米参数为整数；角度支持一位小数，返回十分之一度，兼容原整数命令。 */
static int numbers(const char *s,int *values,unsigned count,unsigned angle_mask)
{
    unsigned i;
    for(i=0;i<count;i++) {
        int sign=1,value=0,digits=0;
        while(*s==' ' || *s=='\t')s++;
        if(*s=='-'){sign=-1;s++;}
        while(*s>='0' && *s<='9') {value=value*10+(*s++-'0');if(value>10000)return 0;digits=1;}
        if(angle_mask&(1u<<i)) {
            value*=10;
            if(*s=='.') {
                s++;
                if(*s<'0' || *s>'9')return 0;
                value+=*s++-'0';
            }
        }
        if(!digits || (*s && *s!=' ' && *s!='\t'))return 0;
        values[i]=sign*value;
    }
    while(*s==' ' || *s=='\t')s++;
    return !*s;
}
static const ArmStep *current_step(void)
{
    if(arm.sequence==1)return &pick_steps[arm.step];
    if(arm.sequence==2)return &home_steps[arm.step];
    if(arm.sequence==3)return &place_steps[arm.step];
    if(arm.sequence==4) {
        unsigned step=arm.step;
        if(step<HOME_COUNT)return &home_steps[step];
        step-=HOME_COUNT;
        if(step<PICK_COUNT)return &pick_steps[step];
        return &place_steps[step-PICK_COUNT];
    }
    return &arm.single;
}
static void begin_step(uint32_t now)
{
    const ArmStep *s=current_step();
    arm.target.slide=(s->mask&SLIDE)?s->slide:arm.slide;
    arm.target.lift=(s->mask&LIFT)?s->lift:arm.lift;
    arm.target.yaw=(s->mask&YAW)?s->yaw:arm.yaw;
    arm.target.claw=(s->mask&CLAW)?s->claw:arm.claw;
    arm.target.platform=(s->mask&PLATFORM)?s->platform:arm.platform;
    if(!pose_valid(arm.target.slide,arm.target.lift,arm.target.yaw,arm.target.claw)) {
        arm_fail("recipe target outside configured travel");
        return;
    }
    memset(arm.axis,0,sizeof(arm.axis)); clear_arm_rx(0);
    arm.state=ARM_PREFLIGHT;arm.phase_ms=now;arm.poll_ms=now-ARM_POLL_MS;
    arm.servo_ms=ARM_SERVO_TIME_MS;
    arm.waiting=arm.poll=arm.submit=0;
    arm_output("[arm step: waiting for fresh axis feedback]\r\n");
}
static void start_single(ArmStep s,uint32_t now)
{
    arm.single=s;arm.sequence=arm.step=0;arm.count=1;begin_step(now);
}
static void arm_status(void)
{
    /* 仅主循环调用；固定缓冲避免浮点格式化占满启动文件的1KiB栈。 */
    static char message[260];
    snprintf(message,sizeof(message),"[arm] enabled=%u referenced=%u state=%u step=%u/%u slide_mm=%d lift_mm=%d yaw_deg=%.1f claw_deg=%.1f rpm=%u claw_us=%u yaw_us=%u; servo positions are commanded estimates\r\n",
        arm.enabled,arm.referenced,arm.state,arm.step,arm.count,arm.slide,arm.lift,(double)arm.yaw,(double)arm.claw,
        (unsigned)ARM_AXIS_RPM,(unsigned)arm.claw_us,(unsigned)arm.yaw_us);
    arm_output(message);
    snprintf(message,sizeof(message),"[platform] referenced=%u angle_deg=%.1f pulse_us=%u; no position sensor\r\n",
        arm.platform_known,(double)arm.platform,(unsigned)arm.platform_us);
    arm_output(message);
}
int arm_command(const char *s,uint32_t now)
{
    int v[4]; ArmStep step={0,0,0,0,0,0};
    if(strcmp(s,"arm") && strncmp(s,"arm ",4) && strncmp(s,"claw ",5))return 0;
    if(!strcmp(s,"arm") || !strcmp(s,"arm status") || !strcmp(s,"claw status")){arm_status();return 1;}
    if(!strcmp(s,"arm stop")){arm_stop();return 1;}
    if(!strcmp(s,"arm off")){
        if(arm.enabled){arm_stop();arm_pwm(0,0);platform_pwm(0);} arm.enabled=0;arm_output("[arm off: PWM disabled; stepper holding enabled]\r\n");return 1;
    }
    if(!strcmp(s,"arm on")){
        if(arm.enabled){arm_output("[arm rejected: already on]\r\n");return 1;}
        arm_init();arm.enabled=1;arm_pwm_init();platform_pwm(0);
        Emm_V5_En_Control(ARM_SLIDE_ADDRESS,true,false);Emm_V5_En_Control(ARM_LIFT_ADDRESS,true,false);
        arm_output("[arm on: lift 5 / slide 7; arm pose for normal motion; yawtest init for PWM-only test]\r\n");return 1;
    }
    if(!arm.enabled){arm_output("[arm rejected: send arm on first]\r\n");return 1;}
    if(arm.state!=ARM_IDLE && arm.state!=ARM_FAULT){arm_output("[arm rejected: busy; use ! or arm stop]\r\n");return 1;}
    if(!strncmp(s,"arm pose ",9) && numbers(s+9,v,4,12u) && pose_valid(v[0],v[1],v[2]/10.0f,v[3]/10.0f)){
        arm.slide=v[0];arm.lift=v[1];arm.yaw=v[2]/10.0f;arm.claw=v[3]/10.0f;
        servo_pose((float)arm.claw,(float)arm.yaw);arm.referenced=1;arm.state=ARM_IDLE;clear_arm_rx(1);
        arm_output("[arm pose declared by operator; NOT automatic homing]\r\n");return 1;
    }
    if(!strncmp(s,"arm yawtest",11)) {
        static char message[200];
        if(!strncmp(s,"arm yawtest init ",17) && numbers(s+17,v,1,0) && v[0]>=500 && v[0]<=2500) {
            /* 显式声明云台起始脉宽，只写PA3，不用虚构升降/平推坐标。 */
            arm_pwm(arm.claw_us,(uint16_t)v[0]);arm.yaw=yaw_of_pulse((uint16_t)v[0]);
            arm.referenced=0;arm.state=ARM_IDLE;arm.sequence=arm.step=arm.count=arm.waiting=0;
            clear_arm_rx(0);
            arm_output("[arm yawtest initialized: yaw PWM written directly; claw held; axes pose invalid; CAN independent]\r\n");return 1;
        }
        if(strncmp(s,"arm yawtest ",12) || !numbers(s+12,v,1,1u) || v[0]<0 || v[0]>3600) {
            arm_output("[arm rejected: yawtest 0..360 or yawtest init 500..2500us]\r\n");return 1;
        }
        if(arm.yaw_us<500 || arm.yaw_us>2500) {
            arm_output("[arm rejected: yaw PWM unknown; arm yawtest init ACTUAL_US required]\r\n");return 1;
        }
        /* 诊断从最后提交的脉宽续走；不读取电机反馈，不恢复两轴位置基准。 */
        arm.test_start_us=arm.yaw_us;
        arm.test_target_us=(uint16_t)(2500.0f-v[0]*2000.0f/3600.0f+0.5f);
        arm.target.yaw=ARM_YAW_MIN_DEG+v[0]/10.0f;
        arm.referenced=0;arm.state=ARM_YAW_TEST;arm.sequence=arm.step=arm.waiting=0;arm.count=1;
        arm.move_ms=arm.pwm_ms=now;clear_arm_rx(0);
        snprintf(message,sizeof(message),"[arm yawtest: test_deg=%.1f target_us=%u time_ms=%u; CAN independent; full-stroke scale, not measured angle]\r\n",
            (double)(v[0]/10.0f),(unsigned)arm.test_target_us,(unsigned)ARM_YAW_TEST_TIME_MS);
        arm_output(message);return 1;
    }
    if(!strncmp(s,"arm platform pose ",18) && numbers(s+18,v,1,1u) && v[0]>=0 && v[0]<=2700) {
        arm.platform=v[0]/10.0f;arm.platform_known=1;platform_pose(arm.platform);
        arm_output("[platform pose declared: PWM submitted; actual angle unverified]\r\n");return 1;
    }
    if(!arm.referenced){arm_output("[arm rejected: actual pose unknown; arm pose SLIDE LIFT YAW CLAW required]\r\n");return 1;}
    if(!strcmp(s,"arm pick") || !strcmp(s,"arm pick red") || !strcmp(s,"arm pick green") || !strcmp(s,"arm pick blue")){
        if(arm.slide!=ARM_SLIDE_HOME_MM || arm.lift!=ARM_LIFT_HOME_MM || arm.yaw!=ARM_YAW_HOME_DEG) {
            arm_output("[arm rejected: pick requires slide=0 lift=135 yaw=0; inspect pose or use arm home]\r\n");
            return 1;
        }
        arm_output("[arm pick: shared calibrated pose; no color selection]\r\n");
        arm.sequence=1;arm.count=(uint8_t)(sizeof(pick_steps)/sizeof(pick_steps[0]));
        arm.step=0;begin_step(now);return 1;
    }
    if(!strcmp(s,"arm home")) {
        arm.sequence=2;arm.count=(uint8_t)(sizeof(home_steps)/sizeof(home_steps[0]));
        arm.step=0;begin_step(now);return 1;
    }
    if(!strcmp(s,"arm place") || !strcmp(s,"arm cycle")) {
        if(!arm.platform_known) {
            arm_output("[arm rejected: declare actual platform angle with arm platform pose DEG]\r\n");return 1;
        }
        arm.sequence=(uint8_t)(!strcmp(s,"arm place")?3:4);
        arm.count=(uint8_t)(arm.sequence==3?PLACE_COUNT:HOME_COUNT+PICK_COUNT+PLACE_COUNT);
        arm.step=0;begin_step(now);return 1;
    }
    if(!strncmp(s,"arm platform ",13) && numbers(s+13,v,1,0) && v[0]>=1 && v[0]<=3) {
        static const float angles[3]={ARM_PLATFORM_FIRST_DEG,ARM_PLATFORM_SECOND_DEG,ARM_PLATFORM_THIRD_DEG};
        if(!arm.platform_known || arm.slide!=ARM_SLIDE_HOME_MM || arm.lift!=ARM_LIFT_HOME_MM || arm.yaw!=ARM_YAW_HOME_DEG) {
            arm_output("[arm rejected: platform requires declared angle and arm home pose]\r\n");return 1;
        }
        step.platform=angles[v[0]-1];step.mask=PLATFORM;
    }
    else if(!strcmp(s,"arm open") || !strcmp(s,"claw open")){step.claw=ARM_CLAW_OPEN_DEG;step.mask=CLAW;}
    else if(!strcmp(s,"arm close") || !strcmp(s,"claw close")){step.claw=ARM_CLAW_CLOSE_DEG;step.mask=CLAW;}
    else if(!strncmp(s,"arm move ",9) && numbers(s+9,v,3,4u)){step.slide=v[0];step.lift=v[1];step.yaw=v[2]/10.0f;step.mask=SLIDE|LIFT|YAW;}
    else if(!strncmp(s,"arm slide ",10) && numbers(s+10,v,1,0)){step.slide=v[0];step.mask=SLIDE;}
    else if(!strncmp(s,"arm lift ",9) && numbers(s+9,v,1,0)){step.lift=v[0];step.mask=LIFT;}
    else if(!strncmp(s,"arm yaw ",8) && numbers(s+8,v,1,1u)){step.yaw=v[0]/10.0f;step.mask=YAW;}
    else if(!strncmp(s,"claw angle ",11) && numbers(s+11,v,1,1u)){step.claw=v[0]/10.0f;step.mask=CLAW;}
    if(!step.mask || !pose_valid((step.mask&SLIDE)?step.slide:arm.slide,(step.mask&LIFT)?step.lift:arm.lift,
          (step.mask&YAW)?step.yaw:arm.yaw,(step.mask&CLAW)?step.claw:arm.claw)) {
        arm_output("[arm rejected: command/range; see arm serial guide]\r\n");return 1;
    }
    start_single(step,now);return 1;
}
void arm_update(uint32_t now)
{
    unsigned i; int complete=1;
    if(!arm.enabled || arm.state==ARM_FAULT)return;
    if(arm.state==ARM_YAW_TEST) {
        uint32_t elapsed=now-arm.move_ms;
        clear_arm_rx(0); /* CAN状态不参与本诊断；正常夹取仍使用下面的完整检查。 */
        if(elapsed>=ARM_YAW_TEST_TIME_MS) {
            arm_pwm(arm.claw_us,arm.test_target_us);arm.yaw=arm.target.yaw;
            arm.state=ARM_IDLE;arm.step=1;
            arm_output("[arm yawtest done: PWM target submitted; actual angle unverified; axes pose invalid]\r\n");
        } else if((uint32_t)(now-arm.pwm_ms)>=20u) {
            float t=(float)elapsed/ARM_YAW_TEST_TIME_MS;
            float smooth=t*t*t*(10.0f+t*(-15.0f+6.0f*t));
            float pulse=arm.test_start_us+((float)arm.test_target_us-arm.test_start_us)*smooth;
            arm_pwm(arm.claw_us,(uint16_t)(pulse+0.5f));arm.pwm_ms=now;
        }
        return;
    }
    if(arm.state==ARM_IDLE && !arm.referenced)return;
#ifdef ARM_HOST_TEST
    if(entry_host_can_bus_off())arm_rx_fault=1;
#else
    if(CAN1->ESR & 4u)arm_rx_fault=1;
#endif
    if(arm_rx_fault){arm_fail("CAN error/receive overflow");return;}
    arm_receive();
    if(arm.state==ARM_IDLE || arm.state==ARM_FAULT)return;
    if((uint32_t)(now-arm.phase_ms)>ARM_ACTION_TIMEOUT_MS){arm_fail("action timeout");return;}
    if(arm.state==ARM_PREFLIGHT) {
        for(i=0;i<2;i++)if(arm.axis[i].samples!=7)complete=0;
        if(complete){
            for(i=0;i<2;i++) {
                ArmAxis *a=&arm.axis[i];
                int delta=i ? arm.target.lift-arm.lift : arm.target.slide-arm.slide;
                if(a->rpm){arm_fail("axis moving before action; inspect pose");return;}
                a->baseline=a->position;a->dir=(uint8_t)(delta>0?0:1);
                if(i ? ARM_LIFT_DIR_INVERT : ARM_SLIDE_DIR_INVERT)a->dir^=1u;
                a->pulses=(uint32_t)((delta<0?-delta:delta)*axis_ppm(i)+0.5f);
                a->ack=(uint8_t)(a->pulses==0);a->samples=0;
            }
            arm.waiting=0;arm.poll=0;clear_arm_rx(0);arm.state=ARM_SUBMIT;
            arm.start_yaw=arm.yaw;arm.start_claw=arm.claw;arm.start_platform=arm.platform;arm.move_ms=arm.pwm_ms=now;
        }
    }
    if(arm.state==ARM_SUBMIT) {
        ArmAxis *a=&arm.axis[arm.submit];
        if(a->pulses){
            a->submitted=1;
            Emm_V5_Pos_Control(axis_address(arm.submit),a->dir,ARM_AXIS_RPM,ARM_AXIS_ACCEL,a->pulses,false,false);
        }
        if(++arm.submit==2)arm.state=ARM_MOVING;
        return; /* 每轮最多提交一个电机，下一轮先响应急停。 */
    }
    if(arm.state==ARM_MOVING){
        uint32_t elapsed=now-arm.move_ms;
        if((uint32_t)(now-arm.pwm_ms)>=20u){
            float t=elapsed>=arm.servo_ms?1.0f:(float)elapsed/arm.servo_ms;
            float smooth=t*t*t*(10.0f+t*(-15.0f+6.0f*t));
            servo_pose(arm.start_claw+(arm.target.claw-arm.start_claw)*smooth,
                       arm.start_yaw+(arm.target.yaw-arm.start_yaw)*smooth);
            if(arm.platform_known)platform_pose(arm.start_platform+(arm.target.platform-arm.start_platform)*smooth);
            arm.pwm_ms=now;
        }
        complete=elapsed>=arm.servo_ms;
        for(i=0;i<2;i++){
            ArmAxis *a=&arm.axis[i];
            int64_t expected=(int64_t)a->pulses*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
            int64_t tolerance=(int64_t)(ARM_POSITION_TOLERANCE_MM*axis_ppm(i)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
            int64_t delta=a->position-a->baseline;
            int sign=i?ARM_LIFT_ENCODER_SIGN:ARM_SLIDE_ENCODER_SIGN;
            if(a->dir)expected=-expected;
            expected*=sign;
            if(!a->ack && elapsed>ARM_ACK_TIMEOUT_MS){arm_fail("position ACK timeout");return;}
            if(a->samples!=7 || !a->ack || !(a->flags&2u) || a->rpm
                || (uint32_t)(now-a->status_ms)>ARM_FEEDBACK_TIMEOUT_MS
                || (uint32_t)(now-a->speed_ms)>ARM_FEEDBACK_TIMEOUT_MS
                || (uint32_t)(now-a->position_ms)>ARM_FEEDBACK_TIMEOUT_MS
                || delta<expected-tolerance || delta>expected+tolerance)complete=0;
        }
        if(complete){
            servo_pose((float)arm.target.claw,(float)arm.target.yaw);
            if(arm.platform_known){platform_pose(arm.target.platform);arm.platform=arm.target.platform;}
            arm.slide=arm.target.slide;arm.lift=arm.target.lift;arm.yaw=arm.target.yaw;arm.claw=arm.target.claw;
            arm_output("[arm step complete: axes feedback verified; servo motion estimated]\r\n");
            if(++arm.step<arm.count)begin_step(now);
            else{arm.state=ARM_IDLE;arm.waiting=0;arm_output("[arm done: inspect actual grip; no grip sensor]\r\n");}
            return;
        }
    }
    arm_poll(now);
}
