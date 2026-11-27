#include "start_scan.h"
#include "road_map.h"
#include "car_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef ARM_HOST_TEST
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#endif

#define SCAN_STEP_CDEG 300 /* 每3度停稳采样，0..90度共31个位置。 */
#define SCAN_TURN_RPM 20u
#define SCAN_SHOTS 3u
#define SCAN_SHOT_INTERVAL_MS 100u
#define SCAN_ECHO_TIMEOUT_US 40000u
#define OBSTACLE_SHOT_GAP_MS 70u
#define SCAN_RANGE_SPREAD_MM 40 /* 同角度至少两次距离相差不超过40mm才关联道路；试验参数。 */
#define SCAN_RANGE_UNCERTAINTY_MM 40 /* 未实车标定的保守候选扩展，不是精度承诺。 */
#define SCAN_BEAM_HALF_DEG 15.0 /* 资料测量角15度未作角分辨率使用；候选按左右各15度扩展。 */
#define SCAN_SENSOR_FORWARD_MM 145 /* 用户提供：探头在车头、距旋转中心前约145mm。 */
#define SCAN_SENSOR_LEFT_MM 0
#define SCAN_SENSOR_HEIGHT_MM 70 /* 用户提供离地约70mm；二维道路关联不证明垂直方向无遮挡。 */
#define SCAN_CHASSIS_LENGTH_MM 291 /* 第三版装配底板+四轮外廓290.637mm，按291mm计；模型证据见扫描文档。 */
#define SCAN_CHASSIS_WIDTH_MM 270 /* 同一模型低位外廓270mm，不计已抬高的上部机构。 */
#define SCAN_REAR_OVERHANG_MM 30 /* 用户补充：物块平台超出底盘后缘约30mm，只扩展车尾。 */
#define SCAN_MARGIN_MM 20 /* 候选扩展的试验初值，非已验收的碰撞安全余量。 */
#define SCAN_SUSPECT_POSITIONS 2u /* 至少两个角度只有同一条候选路，才标SUSPECT；仍非确认禁行。 */
#define SCAN_PI 3.14159265358979323846

enum { SCAN_IDLE, SCAN_TURN, SCAN_CAPTURE, SCAN_NEXT, SCAN_DONE, SCAN_CANCELLED };
enum { CAP_OFF, CAP_RISE, CAP_FALL, CAP_DONE };
enum { ECHO_OK, ECHO_PREHIGH, ECHO_NO_RISE, ECHO_HIGH_TIMEOUT, ECHO_OUT_OF_RANGE };
static StartScanIO port;
/* 输出仅由主循环调用，中断只保存回波；复用缓冲避免三层扫描调用叠加占用1KB系统栈。 */
static char scan_line[200];
static struct {
    uint8_t configured, phase, node, index, positions, shots, valid;
    int length_mm, width_mm, margin_mm;
    int16_t initial_yaw;
    int direction;
    uint32_t last_shot_ms, warning_ms, good, failed;
    int16_t ranges[SCAN_SHOTS], angles[SCAN_SHOTS];
    uint8_t candidates[ROAD_COUNT], unique[ROAD_COUNT];
} scan;
static struct {
    volatile uint8_t phase, ready, status;
    volatile uint32_t started, rising, pulse_us;
    uint32_t cycles_per_us;
    uint8_t initialized;
    volatile uint8_t channel;
} capture;
static struct {
    uint32_t shot_ms;
    uint8_t active,started,next;
} obstacle_capture;

#ifdef ARM_HOST_TEST
static uint32_t test_cycles;
static uint8_t test_echo, test_trigger;
void start_scan_test_time(uint32_t us) {test_cycles=us*168u;}
static uint32_t capture_cycles(void) {return test_cycles;}
static uint8_t capture_level(void) {return test_echo;}
static void capture_trigger(uint8_t high) {test_trigger=high;}
static void capture_init(void) {capture.cycles_per_us=168u;capture.initialized=1;}
#else
static uint32_t capture_cycles(void) {return DWT->CYCCNT;}
static GPIO_TypeDef *capture_port(void) {return capture.channel ? GPIOC : GPIOB;}
static uint8_t capture_level(void) {return (capture_port()->IDR & GPIO_Pin_10)!=0;}
static void capture_trigger(uint8_t high)
{
    if(high)GPIO_SetBits(capture_port(),GPIO_Pin_11);else GPIO_ResetBits(capture_port(),GPIO_Pin_11);
}
static void capture_init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef edge;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=GPIO_Pin_10;gpio.GPIO_Mode=GPIO_Mode_IN;
    gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;GPIO_Init(GPIOB,&gpio);
    GPIO_Init(GPIOC,&gpio); /* PB10/PC10为FT输入，接5V Echo时不上下拉。 */
    GPIO_ResetBits(GPIOB,GPIO_Pin_11);GPIO_ResetBits(GPIOC,GPIO_Pin_11);
    gpio.GPIO_Pin=GPIO_Pin_11;gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&gpio);GPIO_Init(GPIOC,&gpio);
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk; /* 不改TIM2舵机或TIM5毫秒时基，不重置已有DWT计数。 */
    capture.cycles_per_us=SystemCoreClock/1000000u;
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
    EXTI_StructInit(&edge);edge.EXTI_Line=EXTI_Line10;
    edge.EXTI_Mode=EXTI_Mode_Interrupt;edge.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
    edge.EXTI_LineCmd=ENABLE;EXTI_ClearITPendingBit(EXTI_Line10);EXTI_Init(&edge);
    NVIC_EnableIRQ(EXTI15_10_IRQn); /* 沿用共享向量现有优先级，IRQ同时分发视觉15和回波10。 */
    capture.initialized=1;
}
#endif

static void capture_finish(uint8_t status, uint32_t pulse_us)
{
    capture.pulse_us=pulse_us;capture.status=status;
    capture.phase=CAP_DONE;capture.ready=1;
}
static void capture_edge(uint8_t high, uint32_t cycles)
{
    if(capture.phase!=CAP_RISE && capture.phase!=CAP_FALL)return;
    if((uint32_t)(cycles-capture.started)/capture.cycles_per_us>=SCAN_ECHO_TIMEOUT_US) {
        capture_finish(capture.phase==CAP_RISE ? ECHO_NO_RISE : ECHO_HIGH_TIMEOUT,0);return;
    }
    if(high && capture.phase==CAP_RISE) {capture.rising=cycles;capture.phase=CAP_FALL;}
    else if(!high && capture.phase==CAP_FALL) {
        uint32_t us=(uint32_t)(cycles-capture.rising)/capture.cycles_per_us;
        capture_finish(us>=116u && us<=23200u ? ECHO_OK : ECHO_OUT_OF_RANGE,us);
    }
}
void start_scan_echo_irq(void)
{
#ifndef ARM_HOST_TEST
    if(EXTI_GetITStatus(EXTI_Line10)!=RESET) {
        uint32_t cycles=capture_cycles();
        uint8_t high=capture_level();
        EXTI_ClearITPendingBit(EXTI_Line10);
        capture_edge(high,cycles);
    }
#endif
}
#ifdef ARM_HOST_TEST
void start_scan_test_edge(uint8_t level, uint32_t us)
{
    start_scan_test_time(us);test_echo=level;capture_edge(level,capture_cycles());
}
#endif
static void capture_begin(uint8_t channel)
{
    capture.phase=CAP_OFF;capture.ready=0;
#ifndef ARM_HOST_TEST
    EXTI->IMR&=~EXTI_Line10;
    SYSCFG_EXTILineConfig(channel ? EXTI_PortSourceGPIOC : EXTI_PortSourceGPIOB,EXTI_PinSource10);
    EXTI_ClearITPendingBit(EXTI_Line10);
#endif
    capture.channel=channel;
#ifndef ARM_HOST_TEST
    EXTI->IMR|=EXTI_Line10;
#endif
    if(capture_level()) {capture_finish(ECHO_PREHIGH,0);return;}
#ifndef ARM_HOST_TEST
    EXTI_ClearITPendingBit(EXTI_Line10);
#endif
    capture.started=capture_cycles();capture.phase=CAP_RISE;
    capture_trigger(1);
#ifndef ARM_HOST_TEST
    while((uint32_t)(capture_cycles()-capture.started)<12u*capture.cycles_per_us) {}
#else
    test_cycles+=12u*capture.cycles_per_us;
#endif
    capture_trigger(0);
}
static void capture_cancel(void)
{
    capture.phase=CAP_OFF;capture.ready=0;
    if(capture.initialized)capture_trigger(0);
}

void start_scan_obstacle_poll(uint32_t now,int enabled,
                              void (*shot)(uint8_t),void (*result)(uint8_t,int))
{
    if(start_scan_active())return;
    if(!enabled) {
        if(obstacle_capture.active){capture_cancel();obstacle_capture.active=0;}
        return;
    }
    if(obstacle_capture.active) {
        if(!capture.ready && now-obstacle_capture.shot_ms<SCAN_ECHO_TIMEOUT_US/1000u)return;
        int mm=capture.ready && capture.status==ECHO_OK ? (int)((capture.pulse_us*10u+29u)/58u) : -1;
        uint8_t channel=capture.channel;
        capture_cancel();obstacle_capture.active=0;
        result(channel,mm);
        return;
    }
    if(obstacle_capture.started && now-obstacle_capture.shot_ms<OBSTACLE_SHOT_GAP_MS)return;
    if(!capture.initialized)capture_init();
    obstacle_capture.started=obstacle_capture.active=1;
    obstacle_capture.shot_ms=now;
    shot(obstacle_capture.next);
    capture_begin(obstacle_capture.next);
    obstacle_capture.next^=1u;
}

static double point_segment(double px,double py,double ax,double ay,double bx,double by)
{
    double dx=bx-ax,dy=by-ay,length=dx*dx+dy*dy;
    double t=length>0 ? ((px-ax)*dx+(py-ay)*dy)/length : 0;
    if(t<0)t=0;
    if(t>1)t=1;
    dx=px-(ax+t*dx);dy=py-(ay+t*dy);return sqrt(dx*dx+dy*dy);
}
static double chassis_radius_mm(void)
{
    double rear=scan.length_mm/2.0+SCAN_REAR_OVERHANG_MM;
    /* 后悬不改变旋转中心：不能直接把增加后的总长除2，否则会低估车尾角的扫掠半径。 */
    return sqrt(rear*rear+scan.width_mm*scan.width_mm/4.0);
}
static uint16_t road_candidates(int16_t relative_yaw, int range_mm)
{
    RoadPoint origin;
    double angle=(scan.node==NODE_A ? SCAN_PI/2.0 : SCAN_PI*1.5)-relative_yaw*SCAN_PI/18000.0;
    double x,y,uncertainty;
    unsigned i,j;
    uint16_t mask=0,seen=0;
    road_point(scan.node,&origin);
    x=origin.x+(SCAN_SENSOR_FORWARD_MM+range_mm)*cos(angle)+SCAN_SENSOR_LEFT_MM*sin(angle);
    y=origin.y+(SCAN_SENSOR_FORWARD_MM+range_mm)*sin(angle)-SCAN_SENSOR_LEFT_MM*cos(angle);
    /* 以包含整段回波圆弧的圆盘扩大候选；宁可有歧义，不把波束轴当精确物体方位。 */
    uncertainty=2.0*range_mm*sin(SCAN_BEAM_HALF_DEG*SCAN_PI/360.0)
        +SCAN_RANGE_UNCERTAINTY_MM+chassis_radius_mm()+scan.margin_mm;
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        const RoadPlan *road=&road_motion_table[i];
        RoadPoint begin;
        double ax,ay;
        if(seen&(1u<<road->road))continue;
        seen|=(uint16_t)(1u<<road->road);road_point(road->from,&begin);ax=begin.x;ay=begin.y;
        for(j=0;j<road->count;j++) {
            double bx=ax,by=ay;
            if(road->segments[j].axis==BODY_LEFT)bx-=road->segments[j].mm;
            else by+=road->segments[j].mm;
            if(point_segment(x,y,ax,ay,bx,by)<=uncertainty)mask|=(uint16_t)(1u<<road->road);
            ax=bx;ay=by;
        }
    }
    return mask;
}
static void scan_summary(void)
{
    unsigned i,j;
    snprintf(scan_line,sizeof(scan_line),"[scan summary] phase=%u origin=%c positions=%u good=%lu failed=%lu; no route changes\r\n",
        scan.phase,scan.node==NODE_A?'A':'C',scan.positions,(unsigned long)scan.good,(unsigned long)scan.failed);port.write(scan_line);
    for(i=0;i<ROAD_COUNT;i++) {
        for(j=0;j<ROAD_DIRECTION_COUNT;j++)if(road_motion_table[j].road==i)break;
        snprintf(scan_line,sizeof(scan_line),"[scan road] id=%u %c-%c state=%s candidate_positions=%u unique_positions=%u\r\n",
            i,'A'+road_motion_table[j].from,'A'+road_motion_table[j].to,
            scan.unique[i]>=SCAN_SUSPECT_POSITIONS ? "SUSPECT" : "UNKNOWN",scan.candidates[i],scan.unique[i]);port.write(scan_line);
    }
}
static void finish_position(void)
{
    int range=-1,relative=0,stable=0;
    unsigned i,j;
    uint16_t mask=0;
    for(i=0;i<scan.valid;i++)for(j=i+1u;j<scan.valid;j++)
        if(scan.ranges[i]>scan.ranges[j]) {int16_t swap=scan.ranges[i];scan.ranges[i]=scan.ranges[j];scan.ranges[j]=swap;}
    if(scan.valid>=2u) {
        range=scan.valid==3u ? scan.ranges[1] : (scan.ranges[0]+scan.ranges[1])/2;
        for(i=1;i<scan.valid;i++)if(scan.ranges[i]-scan.ranges[i-1u]<=SCAN_RANGE_SPREAD_MM)stable=1;
        for(i=0;i<scan.valid;i++)relative+=scan.angles[i];
        relative/=(int)scan.valid;
        if(stable)mask=road_candidates((int16_t)relative,range);
    }
    for(i=0;i<ROAD_COUNT;i++)if(mask&(1u<<i)) {
        ++scan.candidates[i];
        if((mask&(uint16_t)(mask-1u))==0)++scan.unique[i];
    }
    snprintf(scan_line,sizeof(scan_line),"[scan position] index=%u yaw_left_cdeg=%d range_mm=%d stable=%d candidates_mask=%u; no clear-road claim\r\n",
        scan.index,relative,range,stable,(unsigned)mask);port.write(scan_line);
    ++scan.positions;
    if(scan.index==9000/SCAN_STEP_CDEG) {
        scan.phase=SCAN_DONE;capture_cancel();scan_summary();
        port.write("[scan done] heading=-X, tail toward QR +X; check pivot drift before runimu/button; no automatic launch\r\n");
    } else {++scan.index;scan.phase=SCAN_NEXT;}
}

void start_scan_init(const StartScanIO *io)
{
    port=*io;memset(&scan,0,sizeof(scan));memset(&capture,0,sizeof(capture));
    memset(&obstacle_capture,0,sizeof(obstacle_capture));
    scan.length_mm=SCAN_CHASSIS_LENGTH_MM;scan.width_mm=SCAN_CHASSIS_WIDTH_MM;
    scan.margin_mm=SCAN_MARGIN_MM;scan.configured=1;
#ifdef ARM_HOST_TEST
    test_cycles=0;test_echo=test_trigger=0;
#endif
}
int start_scan_active(void) {return scan.phase>=SCAN_TURN && scan.phase<=SCAN_NEXT;}
void start_scan_cancel(void)
{
    capture_cancel();obstacle_capture.active=0;
    if(start_scan_active()) {scan.phase=SCAN_CANCELLED;port.write("[scan cancelled] incomplete; no roads cleared or blocked\r\n");}
}
int start_scan_command(const char *command,uint32_t now)
{
    char trailing;
    int length,w,m;
    HeadingSample sample;
    uint8_t node;
    if(strncmp(command,"scan",4) || (command[4] && command[4]!=' '))return 0;
    if(!strcmp(command,"scan status")) {scan_summary();return 1;}
    if(start_scan_active()) {port.write("[scan rejected: scan active; use ! or scan stop]\r\n");return 1;}
    if(sscanf(command,"scan size %d %d %d %c",&length,&w,&m,&trailing)==3
        && length>0 && length<=3000 && w>0 && w<=3000 && m>=0 && m<=1000) {
        scan.length_mm=length;scan.width_mm=w;scan.margin_mm=m;scan.configured=1;
        snprintf(scan_line,sizeof(scan_line),"[scan size] base_length_mm=%d width_mm=%d rear_extra_mm=%d margin_mm=%d probe_forward_mm=%d left_mm=%d height_mm=%d; start head toward B\r\n",
            length,w,SCAN_REAR_OVERHANG_MM,m,SCAN_SENSOR_FORWARD_MM,SCAN_SENSOR_LEFT_MM,SCAN_SENSOR_HEIGHT_MM);port.write(scan_line);return 1;
    }
    if(strcmp(command,"scan A") && strcmp(command,"scan C")) {
        port.write("scan size BASE_LENGTH_MM WIDTH_MM MARGIN_MM | scan A/C | scan status | scan stop\r\n");return 1;
    }
    if(!scan.configured) {port.write("[scan rejected: declare chassis model length/width with scan size first]\r\n");return 1;}
    sample=port.heading();
    if(!heading_sample_fresh(&sample,now,CAR_IMU_MAX_AGE_MS)) {port.write("[scan rejected: fresh IMU required]\r\n");return 1;}
    node=command[5]=='A'?NODE_A:NODE_C;
    if(!port.turn_start(0,SCAN_TURN_RPM)) {port.write("[scan rejected: chassis/arm/session busy or feedback unavailable]\r\n");return 1;}
    if(!capture.initialized)capture_init();
    capture_cancel();obstacle_capture.active=0;
    scan.node=node;scan.direction=node==NODE_A?-1:1;scan.initial_yaw=sample.yaw_cdeg;
    scan.phase=SCAN_TURN;scan.index=scan.positions=scan.shots=scan.valid=0;scan.good=scan.failed=0;
    memset(scan.candidates,0,sizeof(scan.candidates));memset(scan.unique,0,sizeof(scan.unique));
    scan.warning_ms=now-1000u;
    snprintf(scan_line,sizeof(scan_line),"[scan footprint] base_length_mm=%d width_mm=%d rear_extra_mm=%d total_length_mm=%d margin_mm=%d probe_forward_mm=%d height_mm=%d\r\n",
        scan.length_mm,scan.width_mm,SCAN_REAR_OVERHANG_MM,scan.length_mm+SCAN_REAR_OVERHANG_MM,
        scan.margin_mm,SCAN_SENSOR_FORWARD_MM,SCAN_SENSOR_HEIGHT_MM);port.write(scan_line);
    port.write(node==NODE_A ? "[scan start] A: head toward B (+Y), then right90 to -X; stationary samples, diagnostics only\r\n"
        : "[scan start] C: head toward B (-Y), then left90 to -X; stationary samples, diagnostics only\r\n");
    return 1;
}
void start_scan_update(uint32_t now)
{
    HeadingSample sample;
    int16_t relative,target;
    if(!start_scan_active())return;
    /* 超时判定不依赖IMU恢复；避免失联期间DWT长时间回绕后把旧脉冲配给新朝向。 */
    if((capture.phase==CAP_RISE || capture.phase==CAP_FALL)
        && (uint32_t)(now-scan.last_shot_ms)>=SCAN_ECHO_TIMEOUT_US/1000u)
        capture_finish(capture.phase==CAP_RISE ? ECHO_NO_RISE : ECHO_HIGH_TIMEOUT,0);
    sample=port.heading();
    if(!heading_sample_fresh(&sample,now,CAR_IMU_MAX_AGE_MS))return;
    relative=heading_wrap_cdeg((int32_t)sample.yaw_cdeg-scan.initial_yaw);
    target=(int16_t)(scan.direction*scan.index*SCAN_STEP_CDEG);
    if(scan.phase==SCAN_NEXT) {
        int16_t delta=heading_wrap_cdeg((int32_t)target-relative);
        if(port.turn_start(delta,SCAN_TURN_RPM))scan.phase=SCAN_TURN;
        return;
    }
    if(scan.phase==SCAN_TURN) {
        if(port.turn_state()!=0)return;
        scan.shots=scan.valid=0;scan.last_shot_ms=now-SCAN_SHOT_INTERVAL_MS;scan.phase=SCAN_CAPTURE;
    }
    if(capture.ready) {
        uint8_t status=capture.status;
        uint32_t pulse=capture.pulse_us;
        int aligned=relative-target<=CAR_HEADING_TOLERANCE_CDEG && relative-target>=-CAR_HEADING_TOLERANCE_CDEG
            && (uint32_t)(now-scan.last_shot_ms)<=CAR_IMU_MAX_AGE_MS;
        capture_cancel();++scan.shots;
        if(status==ECHO_OK && aligned) {
            scan.ranges[scan.valid]=(int16_t)((pulse*10u+29u)/58u);
            scan.angles[scan.valid++]=relative;++scan.good;
        } else ++scan.failed;
        snprintf(scan_line,sizeof(scan_line),"[scan ray] index=%u yaw_left_cdeg=%d shot=%u echo_status=%u aligned=%d echo_us=%lu\r\n",
            scan.index,relative,scan.shots,status,aligned,(unsigned long)pulse);port.write(scan_line);
        if(scan.shots==SCAN_SHOTS) {finish_position();return;}
    } else if(capture.phase==CAP_RISE || capture.phase==CAP_FALL) {
        if((uint32_t)(capture_cycles()-capture.started)/capture.cycles_per_us>=SCAN_ECHO_TIMEOUT_US)
            capture_finish(capture.phase==CAP_RISE ? ECHO_NO_RISE : ECHO_HIGH_TIMEOUT,0);
        return;
    }
    if(relative-target>CAR_HEADING_TOLERANCE_CDEG || relative-target< -CAR_HEADING_TOLERANCE_CDEG) {
        if((uint32_t)(now-scan.warning_ms)>=1000u) {scan.warning_ms=now;port.write("[scan waiting] heading moved outside window; no new motion or fake sample\r\n");}
        return;
    }
    if((uint32_t)(now-scan.last_shot_ms)>=SCAN_SHOT_INTERVAL_MS) {scan.last_shot_ms=now;capture_begin(1);}
}
