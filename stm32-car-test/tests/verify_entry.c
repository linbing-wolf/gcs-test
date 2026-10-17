#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "route_table.h"
static uint32_t assertions;
#define CHECK(x) do {assertions++;if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}} while(0)
#define NAV_ENTRY_HOST_TEST
#include "../src/main_table_nav.c"
#define EXPECTED_FULL_ROUTE_MM (15040u - 4u*CAR_MAP_TOP_INSET_MM - 4u*CAR_MAP_BOTTOM_INSET_MM)

typedef struct {
    uint8_t addr,dir,acc;
    uint16_t rpm;
    uint32_t pulses;
    bool absolute,synchronous;
} SentPosition;
static SentPosition sent[256];
static uint32_t position_count,sync_count,stop_count,velocity_count;
static uint32_t interrupt_at_position;
static uint32_t fail_can_at_position;
static char output[32768];
static size_t output_size;
static uint8_t status_auto_reply,position_ack_enabled,bus_off;
static uint8_t simulated_start_button;
uint8_t entry_host_start_button_pressed(void) {return simulated_start_button;}
static uint8_t scanner_sent[256], scanner_can_tx;
static unsigned scanner_sent_count;
static uint32_t scanner_initialized_baud;
void entry_host_scanner_init(uint32_t baud) {scanner_initialized_baud=baud;}
int entry_host_scanner_tx_ready(void) {return scanner_can_tx;}
void entry_host_scanner_send_byte(uint8_t value)
{CHECK(scanner_sent_count<sizeof(scanner_sent));scanner_sent[scanner_sent_count++]=value;}
static uint8_t motor_flags[4];
static uint32_t status_queries;
static uint32_t diagnostic_queries;
static uint8_t diagnostic_auto_reply;
static int16_t simulated_wheel_rpm[4], staged_wheel_rpm[4], reported_wheel_rpm[4];
static int64_t simulated_encoder[4];
static uint8_t velocity_ack_enabled, position_auto_reply, speed_auto_reply;
static uint8_t missing_velocity_ack, missing_feedback_motor, moving_at_stop_motor;
static uint32_t velocity_batches;
static uint32_t interrupt_at_velocity, fail_can_at_velocity;
typedef struct {
    uint8_t rotating,clockwise;
    RoadHeading heading;
    BodyAxis axis;
    int16_t mm;
    uint32_t pulses;
} RunAction;
static RunAction run_actions[64];
static uint8_t run_action_count;

void entry_host_send_byte(uint8_t value)
{
    CHECK(output_size+1<sizeof(output));
    output[output_size++]=(char)value;output[output_size]=0;
}
void delay_ms(uint16_t milliseconds) {clock_ms+=milliseconds;}
void Emm_V5_Pos_Control(uint8_t addr,uint8_t dir,uint16_t rpm,uint8_t acc,
                        uint32_t pulses,bool absolute,bool synchronous)
{
    SentPosition *p;
    CHECK(position_count<256);
    p=&sent[position_count++];p->addr=addr;p->dir=dir;p->rpm=rpm;p->acc=acc;
    p->pulses=pulses;p->absolute=absolute;p->synchronous=synchronous;
    if(interrupt_at_position && position_count==interrupt_at_position) queue_byte('!');
    if(fail_can_at_position && position_count==fail_can_at_position) route_test_can_error(2u);
    if(run_active && position_ack_enabled) {
        const uint8_t reply[3]={0xfd,2,0x6b};
        route_test_can_rx((uint32_t)addr<<8,4,0,3,reply);
    }
}
int entry_host_can_bus_off(void) {return bus_off;}
void Emm_V5_Read_Sys_Params(uint8_t addr,SysParams_t parameter)
{
    uint8_t reply[7]={0x3a,0,0x6b,0,0,0,0x6b};
    CHECK(addr>=1 && addr<=4);
    if(parameter==S_FLAG) {
        status_queries++;reply[1]=motor_flags[addr-1u];
        if(status_auto_reply) route_test_can_rx((uint32_t)addr<<8,4,0,3,reply);
    } else {
        CHECK(parameter==S_PERR || parameter==S_VEL || parameter==S_CPOS);diagnostic_queries++;
        if(run.imu_mode) {
            int64_t value=parameter==S_CPOS ? simulated_encoder[addr-1u] : reported_wheel_rpm[addr-1u];
            uint32_t magnitude=(uint32_t)(value<0?-value:value);
            if(missing_feedback_motor==addr)return;
            if(parameter==S_VEL && moving_at_stop_motor==addr && !value)magnitude=10;
            reply[0]=parameter==S_CPOS ? 0x36 : 0x35;
            reply[1]=value<0;
            if(parameter==S_CPOS) {
                reply[2]=(uint8_t)(magnitude>>24);reply[3]=(uint8_t)(magnitude>>16);
                reply[4]=(uint8_t)(magnitude>>8);reply[5]=(uint8_t)magnitude;
                if(position_auto_reply)route_test_can_rx((uint32_t)addr<<8,4,0,7,reply);
            } else {
                reply[2]=(uint8_t)(magnitude>>8);reply[3]=(uint8_t)magnitude;reply[4]=0x6b;
                if(speed_auto_reply)route_test_can_rx((uint32_t)addr<<8,4,0,5,reply);
            }
            return;
        }
        reply[0]=parameter==S_PERR ? 0x37 : 0x35;
        reply[1]=1;reply[2]=0;
        if(parameter==S_PERR)reply[5]=8;
        else {reply[3]=0;reply[4]=0x6b;}
        if(diagnostic_auto_reply)route_test_can_rx((uint32_t)addr<<8,4,0,parameter==S_PERR ? 7 : 5,reply);
    }
}
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    CHECK(addr==0);sync_count++;
    if(run_active && run.imu_mode) {
        memcpy(simulated_wheel_rpm,staged_wheel_rpm,sizeof(simulated_wheel_rpm));
        velocity_batches++;return;
    }
    if(run_active) {
        RunAction *action;
        CHECK(run_action_count<64);
        action=&run_actions[run_action_count++];
        action->rotating=drive.rotating;action->clockwise=drive.clockwise;
        action->heading=run.heading;action->axis=drive.parameters.axis;
        action->mm=drive.parameters.mm;action->pulses=drive.pulses;
    }
}
void Emm_V5_Stop_Now(uint8_t addr,bool synchronous)
{CHECK(addr>=1 && addr<=4 && !synchronous);stop_count++;simulated_wheel_rpm[addr-1u]=staged_wheel_rpm[addr-1u]=0;}
void Emm_V5_Vel_Control(uint8_t addr,uint8_t dir,uint16_t rpm,uint8_t acc,bool synchronous)
{
    CHECK(addr>=1 && addr<=4 && acc==150);velocity_count++;
    if(synchronous) {
        const uint8_t ack[3]={0xf6,2,0x6b};
        CHECK(run_active && run.imu_mode && dir<=1 && rpm<=300);
        if(interrupt_at_velocity==velocity_count)queue_byte('!');
        if(fail_can_at_velocity==velocity_count)route_test_can_error(2u);
        staged_wheel_rpm[addr-1u]=dir ? (int16_t)rpm : -(int16_t)rpm;
        if(velocity_ack_enabled && missing_velocity_ack!=addr)route_test_can_rx((uint32_t)addr<<8,4,0,3,ack);
    } else {
        CHECK(dir==0 && rpm==0);
        simulated_wheel_rpm[addr-1u]=staged_wheel_rpm[addr-1u]=0;
    }
}

static void reset_fixture(void)
{
    MotionIO io={motor_position,motor_sync,motor_stop,0};
    MotionConfig config={CALIBRATED_PPM_X,CALIBRATED_PPM_Y,1,0,-1,1,
                         LEFT_FORWARD_MM_PER_M,RIGHT_FORWARD_MM_PER_M};
    NavMotionPort port={port_start,port_poll,port_stop,&drive};
    clock_ms=0;rx_head=rx_tail=stop_pending=rx_overflow=0;
    simulated_start_button=0;start_button_init();
    memset(&scanner,0,sizeof(scanner));scanner_head=scanner_tail=scanner_rx_error=scanner_rx_overflow=0;
    scanner_sent_count=0;scanner_can_tx=1;scanner_initialized_baud=0;
    line_length=pose_set=test_mode=0;speed_rpm=0;test_distance=1000;test_rpm=60;
    turn_rpm=CAR_TURN_RPM;turn_pulses=CAR_TURN_TEST_PULSES;
    turn_ccw90=CAR_TURN_CCW_90_PULSES;turn_cw90=CAR_TURN_CW_90_PULSES;
    reported_nav=NAV_FAULT;reported_motion=MOTION_FAULT;
    memset((void *)rx_buffer,0,sizeof(rx_buffer));memset(line,0,sizeof(line));
    memset(sent,0,sizeof(sent));position_count=sync_count=stop_count=velocity_count=0;
    interrupt_at_position=fail_can_at_position=0;output_size=0;output[0]=0;
    memset(&run,0,sizeof(run));run_active=run_rx_head=run_rx_tail=run_rx_overflow=0;
    memset(&turn_postcheck,0,sizeof(turn_postcheck));
    heading_sensor_reset(&heading_sensor);
    status_auto_reply=position_ack_enabled=1;bus_off=0;status_queries=0;
    diagnostic_queries=0;diagnostic_auto_reply=1;
    velocity_ack_enabled=position_auto_reply=speed_auto_reply=1;
    missing_velocity_ack=missing_feedback_motor=moving_at_stop_motor=0;velocity_batches=0;
    interrupt_at_velocity=fail_can_at_velocity=0;
    memset(simulated_wheel_rpm,0,sizeof(simulated_wheel_rpm));
    memset(staged_wheel_rpm,0,sizeof(staged_wheel_rpm));memset(simulated_encoder,0,sizeof(simulated_encoder));
    memset(reported_wheel_rpm,0,sizeof(reported_wheel_rpm));
    memset(motor_flags,3,sizeof(motor_flags));
    run_action_count=0;memset(run_actions,0,sizeof(run_actions));
    CHECK(motion_init(&drive,&io,&config));CHECK(nav_init(&nav,NODE_A,HEAD_Y_POS,&port));
}
static void send_line(const char *s)
{
    output_size=0;output[0]=0;
    while(*s) {queue_byte((uint8_t)*s++);firmware_step();}
    queue_byte('\n');firmware_step();
}
static void stop_request(void) {queue_byte('!');firmware_step();}
static void await_estimate(void)
{clock_ms=drive.started_ms+drive.estimate_ms;firmware_step();}
static void check_positions(uint32_t offset,const uint8_t directions[4],const uint32_t pulses[4],const uint16_t rpm[4])
{
    uint8_t i;
    for(i=0;i<4;i++) {
        SentPosition *p=&sent[offset+i];
        CHECK(p->addr==i+1 && p->dir==directions[i]);CHECK(p->rpm==rpm[i] && p->acc==150);
        CHECK(p->pulses==pulses[i] && !p->absolute && p->synchronous);
    }
}
/* 人工nav仍使用原位置模块；其横移脉冲/预补偿回归直接测试提交层。
 * 串口l/r已改为IMU速度闭环，完整轨迹另在verify_heading.c验证。 */
static void position_lateral_fixture(char direction)
{
    MotionParameters parameters = {BODY_LEFT, direction == 'l' ? (int16_t)test_distance : -(int16_t)test_distance,
                                   test_rpm, CAR_MOTION_ACCELERATION};
    motion_stop(&drive);
    CHECK(port_start(&drive, parameters, clock_ms));
}
static void direction_tests(void)
{
    static const char *keys[]={"f","b","l","r"};
    static const uint8_t directions[4][4]={{0,1,1,0},{1,0,0,1},{0,0,1,1},{1,1,0,0}};
    static const uint16_t speeds[]={30,60,90};
    static const uint32_t pulses[4][4]={{10360,10360,10360,10360},{10360,10360,10360,10360},
                                       {11260,10845,11260,10845},{10970,11136,10970,11136}};
    static const uint16_t low_left[]={29,58,87},low_right[]={30,59,89};
    uint8_t i,j;
    char command_text[20];
    for(j=0;j<3;j++) for(i=0;i<4;i++) {
        uint32_t generation;
        uint16_t expected_rpm[4]={speeds[j],speeds[j],speeds[j],speeds[j]};
        reset_fixture();send_line("test");
        CHECK(test_mode && !pose_set && !nav.position_confirmed && position_count==0);
        send_line("dist 1000");sprintf(command_text,"rpm %u",(unsigned)speeds[j]);send_line(command_text);
        if(i<2) {send_line(keys[i]);CHECK(strstr(output,"[test start]") && strstr(output,"[accepted]"));}
        else position_lateral_fixture(keys[i][0]);
        CHECK(position_count==4 && sync_count==1 && drive.state==MOTION_RUNNING);
        if(i==2) expected_rpm[1]=expected_rpm[3]=low_left[j];
        if(i==3) expected_rpm[0]=expected_rpm[2]=low_right[j];
        check_positions(0,directions[i],pulses[i],expected_rpm);
        CHECK(drive.forward_trim_mm==(i<2?0:i==2?20:8));
        generation=drive.generation;
        send_line(keys[(i+1)%4]);CHECK(strstr(output,"[rejected") && position_count==4);
        send_line("rpm 120");CHECK(test_rpm==speeds[j]);
        send_line("dist 500");CHECK(test_distance==1000);
        await_estimate();CHECK(drive.state==MOTION_WAIT_CONFIRM);
        CHECK(position_count==4 && sync_count==1 && !nav.position_confirmed);
        send_line("confirm");CHECK(strstr(output,"[rejected") && position_count==4);
        if(i<2)send_line(keys[i]);else position_lateral_fixture(keys[i][0]);
        CHECK(position_count==8 && sync_count==2 && drive.generation>generation);
        stop_request();CHECK(drive.state==MOTION_IDLE && test_mode && !nav.position_confirmed);
        CHECK(test_rpm==speeds[j] && test_distance==1000 && position_count==8);
    }
}
static void range_and_mode_tests(void)
{
    uint8_t i;
    static const char *invalid[]={"dist 0","dist 49","dist 2001","dist 65536","dist -1",
                                  "rpm 0","rpm 19","rpm 301","rpm 999999999999999999"};
    reset_fixture();send_line("f");CHECK(position_count==0 && strstr(output,"[rejected"));
    send_line(" \ttest\t ");CHECK(test_mode && strstr(output,"[accepted]"));
    for(i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        send_line(invalid[i]);CHECK(strstr(output,"[rejected") && test_distance==1000 && test_rpm==60);
    }
    send_line(" dist 50 ");CHECK(test_distance==50);
    send_line("rpm 20");CHECK(test_rpm==20);
    send_line("dist 2000");send_line("rpm 300");CHECK(test_distance==2000 && test_rpm==300);
    send_line("pose F");send_line("go E");CHECK(position_count==0 && !pose_set);
    send_line("f 1000");CHECK(position_count==0);
    send_line("l");CHECK(!run_active && position_count==0 && strstr(output,"[rejected"));
    send_line("r");CHECK(!run_active && position_count==0 && strstr(output,"[rejected"));
    send_line("f");CHECK(position_count==4 && drive.pulses==20720);
    send_line("nav");CHECK(!test_mode && drive.state==MOTION_IDLE && !nav.position_confirmed && !pose_set);
    send_line("go B");CHECK(position_count==4 && strstr(output,"[rejected"));
    send_line("pose A");send_line("heading 0");send_line("go B");
    CHECK(position_count==8 && nav.state==NAV_MOVING && drive.parameters.rpm==100);
    send_line("test");CHECK(test_mode && nav.state==NAV_STOPPED && drive.state==MOTION_IDLE);
    CHECK(position_count==8 && !pose_set && !nav.position_confirmed);
}
static void stop_and_timeout_tests(void)
{
    uint16_t i;
    reset_fixture();send_line("test");
    queue_byte('f');queue_byte('\n');queue_byte('!');firmware_step();
    CHECK(position_count==0 && rx_head==rx_tail && line_length==0);
    send_line("f");CHECK(position_count==4);
    clock_ms=drive.started_ms+drive.timeout_ms;firmware_step();
    CHECK(drive.state==MOTION_IDLE && test_mode && !nav.position_confirmed);
    CHECK(strstr(output,"stop requested") && position_count==4 && sync_count==1);
    clock_ms+=60000;firmware_step();CHECK(position_count==4 && sync_count==1);
    send_line("f");CHECK(position_count==8);stop_request();
    for(i=0;i<RX_SIZE;i++) queue_byte('f');
    firmware_step();
    CHECK(rx_head==rx_tail && !rx_overflow && position_count==8);
    for(i=0;i<LINE_SIZE;i++) {queue_byte('x');firmware_step();}firmware_step();
    CHECK(line_length==0 && position_count==8);
    reset_fixture();send_line("test");interrupt_at_position=2;send_line("f");firmware_step();
    CHECK(position_count==2 && sync_count==0 && drive.state==MOTION_IDLE && !stop_pending);
    CHECK(stop_count>=4 && velocity_count>=4);
    reset_fixture();send_line("test");clock_ms=UINT32_MAX-100;send_line("b");await_estimate();
    CHECK(drive.state==MOTION_WAIT_CONFIRM && position_count==4);
}
static void navigation_regression(void)
{
    uint32_t pulses;
    reset_fixture();send_line("go B");CHECK(position_count==0);
    send_line(" pose A\t ");CHECK(pose_set && nav.position_confirmed && strstr(output,"[accepted]"));
    send_line("heading 0");send_line("rpm 30");send_line("go B");
    CHECK(nav.state==NAV_MOVING && position_count==4 && drive.parameters.axis==BODY_LEFT);
    CHECK(drive.parameters.mm==60 && drive.parameters.rpm==30);
    CHECK(drive.pulses==663);
    CHECK(drive.forward_trim_mm==1 && sent[0].pulses==674 && sent[1].pulses==653);
    await_estimate();CHECK(nav.state==NAV_MOVING && drive.state==MOTION_WAIT_CONFIRM);
    send_line("confirm");CHECK(position_count==8 && drive.parameters.axis==BODY_FORWARD && drive.parameters.mm==1030);
    pulses=drive.pulses;CHECK(pulses==10671);
    await_estimate();send_line("confirm");CHECK(nav.state==NAV_WAIT_NODE && position_count==8);
    send_line("node B");CHECK(nav.state==NAV_ARRIVED && nav.position_confirmed && (nav.passed&1));
    reset_fixture();send_line("pose A");send_line("go B");
    clock_ms=drive.started_ms+drive.timeout_ms;firmware_step();
    CHECK(nav.state==NAV_FAULT && nav.error==NAV_MOTION_FAILED);
    stop_request();send_line("pose F");CHECK(nav.state==NAV_IDLE && nav.error==NAV_OK);
}
static void trim_tests(void)
{
    Motion candidate;
    reset_fixture();drive.config.left_forward_mm_per_m=0;drive.config.right_forward_mm_per_m=0;
    send_line("test");position_lateral_fixture('l');
    CHECK(drive.forward_trim_mm==0 && sent[0].pulses==11053 && sent[1].pulses==11053);
    CHECK(sent[0].rpm==60 && sent[1].rpm==60);
    stop_request();position_lateral_fixture('r');CHECK(drive.forward_trim_mm==0 && sent[4].pulses==11053);
    reset_fixture();drive.config.left_forward_mm_per_m=-20;send_line("test");position_lateral_fixture('l');
    CHECK(drive.forward_trim_mm==-20 && sent[0].pulses==10845 && sent[1].pulses==11260);
    CHECK(sent[0].dir==0 && sent[1].dir==0 && sent[0].rpm==58 && sent[1].rpm==60);
    reset_fixture();send_line("test");send_line("dist 50");position_lateral_fixture('l');
    CHECK(drive.forward_trim_mm==1 && sent[0].pulses==563 && sent[1].pulses==542);
    reset_fixture();send_line("test");send_line("dist 50");position_lateral_fixture('r');
    CHECK(drive.forward_trim_mm==0 && sent[0].pulses==553 && sent[1].pulses==553);
    reset_fixture();send_line("test");send_line("dist 2000");position_lateral_fixture('l');
    CHECK(drive.forward_trim_mm==40 && sent[0].pulses==22520 && sent[1].pulses==21691);
    drive.config.left_forward_mm_per_m=101;
    CHECK(!motion_init(&candidate,&drive.io,&drive.config));
    drive.config.left_forward_mm_per_m=20;drive.config.right_forward_mm_per_m=-101;
    CHECK(!motion_init(&candidate,&drive.io,&drive.config));
}
static void run_to_first_segment(void)
{
    unsigned ticks=0;
    while(run.state==RUN_PREFLIGHT && ticks++<100) {clock_ms+=25;firmware_step();}
    firmware_step();
    CHECK(run.state==RUN_MOVING && position_count==4 && run.acks==15);
}
static void calibrate_turn_fixture(void)
{
    /* 模拟值只用于软件验证；绝不写入上车默认配置。 */
    turn_ccw90=1000;turn_cw90=1100;
}
static void rotation_tests(void)
{
    const uint32_t pulses[4]={200,200,200,200};
    const uint16_t rpm[4]={30,30,30,30};
    const uint8_t ccw[4]={CAR_TURN_CCW_DIR,CAR_TURN_CCW_DIR,CAR_TURN_CCW_DIR,CAR_TURN_CCW_DIR};
    const uint8_t cw[4]={!CAR_TURN_CCW_DIR,!CAR_TURN_CCW_DIR,!CAR_TURN_CCW_DIR,!CAR_TURN_CCW_DIR};
    const char *invalid[]={"pulses 0","pulses 20001","pulses -1","pulses 65536",
        "rpm 19","rpm 301","ccw90 20001","cw90 65536","dist 100","f","l","tr extra"};
    unsigned i;
    reset_fixture();CHECK(!turn_ccw90 && !turn_cw90);
    send_line("run A");CHECK(!run_active && !position_count && strstr(output,"calibrate"));
    send_line("turn");CHECK(test_mode==2 && !position_count && !pose_set);
    send_line("left90");CHECK(!position_count && strstr(output,"[rejected"));
    for(i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        send_line(invalid[i]);CHECK(strstr(output,"[rejected") && !position_count);
    }
    send_line("tl");CHECK(position_count==4 && drive.rotating && !drive.clockwise && !drive.forward_trim_mm);
    check_positions(0,ccw,pulses,rpm);
    send_line("tr");send_line("pulses 400");send_line("ccw90 1000");
    CHECK(position_count==4 && turn_pulses==200 && !turn_ccw90);
    await_estimate();CHECK(drive.state==MOTION_WAIT_CONFIRM && !nav.position_confirmed);
    CHECK(strstr(output,"ACTUAL angle"));
    stop_request();send_line("tr");check_positions(4,cw,pulses,rpm);
    stop_request();send_line("ccw90 1000");send_line("run A");
    CHECK(!run_active && position_count==8); /* 只标一侧不能跑全程。 */
    send_line("cw90 1100");send_line("left90");
    CHECK(position_count==8 && !run_active && strstr(output,"[rejected"));
    send_line("pulses 1000");send_line("tl");CHECK(drive.pulses==1000 && !drive.clockwise);
    stop_request();send_line("pulses 2200");send_line("tr");CHECK(drive.pulses==2200 && drive.clockwise);
    clock_ms=drive.started_ms+drive.timeout_ms;firmware_step();
    CHECK(drive.state==MOTION_IDLE && position_count==16 && strstr(output,"stop requested"));
    clock_ms+=60000;firmware_step();CHECK(position_count==16);
    send_line("ccw90 0");send_line("run C");CHECK(!run_active && position_count==16);
    send_line("nav");CHECK(!test_mode && !pose_set);
    reset_fixture();send_line("turn");interrupt_at_position=2;send_line("tl");firmware_step();
    CHECK(position_count==2 && !sync_count && drive.state==MOTION_IDLE);
    reset_fixture();CHECK(!motion_start_rotation(&drive,2,200,30,150,0));
    CHECK(!motion_start_rotation(&drive,0,0,30,150,0));
    CHECK(!motion_start_rotation(&drive,0,40001,30,150,0));
    CHECK(!motion_start_rotation(&drive,0,200,19,150,0));
}
static void full_route_tests(void)
{
    uint8_t plan,job,node,cursor,i;
    uint16_t passed,total;
    RouteTablePath route;
    for(plan=0;plan<2;plan++) {
        /* 与当前查表的各阶段最短路逐节点核对，使用与当前几何配套的查表。 */
        node=full_route_nodes[plan][0];cursor=0;passed=total=0;
        for(job=0;job<FULL_ROUTE_JOBS;job++) {
            CHECK(route_table_lookup(node,full_route_jobs[plan][job],0,passed,&route));
            total+=route.distance;
            for(i=1;i<route.count;i++) {
                int road=road_index(route.nodes[i-1],route.nodes[i]);
                CHECK(road>=0 && ++cursor<FULL_ROUTE_NODES);
                CHECK(full_route_nodes[plan][cursor]==route.nodes[i]);passed|=(uint16_t)(1u<<road);
            }
            node=full_route_jobs[plan][job];
        }
        CHECK(total==EXPECTED_FULL_ROUTE_MM && cursor==16);
        reset_fixture();calibrate_turn_fixture();send_line(plan?"run C":"run A");
        CHECK(run_active && !test_mode && !pose_set && !nav.position_confirmed);
        for(i=0;i<4;i++) CHECK(motor_flags[i]==3);
        {
            unsigned ticks=0;
            while(run_active && ticks++<15000) {clock_ms+=25;firmware_step();}
            CHECK(ticks<15000 && run.state==RUN_DONE && run.job==8);
        }
        CHECK(run.steps>18 && run.distance_mm==EXPECTED_FULL_ROUTE_MM && position_count==4u*run.steps && sync_count==run.steps);
        CHECK(run.heading==HEAD_Y_POS && run.finishing);
        CHECK(!pose_set && !nav.position_confirmed && drive.state==MOTION_IDLE && run.edge==16);
        CHECK(strstr(output,"done by motor feedback") && status_queries>=76);
        {
            /* 独立重放实际提交的旋转/正向直行，验证轨迹与每个几何段、最终朝向。 */
            RoadPoint start,point;
            int x,y,heading=0,straight=0,turns=0;
            unsigned action_index,edge=0,segment=0;
            RoadPlan expected;
            CHECK(road_point(full_route_nodes[plan][0],&start));
            x=start.x;y=start.y;
            CHECK(road_plan(full_route_nodes[plan][0],full_route_nodes[plan][1],&expected));
            for(action_index=0;action_index<run_action_count;action_index++) {
                RunAction *action=&run_actions[action_index];
                CHECK((int)action->heading==heading);
                if(action->rotating) {
                    uint32_t quarter=action->clockwise?1100u:1000u;
                    int quarters=(int)(action->pulses/quarter);
                    CHECK(action->pulses%quarter==0 && (quarters==1 || quarters==2));
                    heading=(heading+(action->clockwise?4-quarters:quarters))%4;
                    turns++;
                } else {
                    MotionParameters geometry=expected.segments[segment];
                    int dx=heading==1?-action->mm:heading==3?action->mm:0;
                    int dy=heading==0?action->mm:heading==2?-action->mm:0;
                    CHECK(action->axis==BODY_FORWARD && action->mm>0);
                    CHECK(dx==(geometry.axis==BODY_LEFT?-geometry.mm:0));
                    CHECK(dy==(geometry.axis==BODY_FORWARD?geometry.mm:0));
                    CHECK(action->pulses==(uint32_t)(action->mm*CALIBRATED_PPM_Y+0.5f));
                    x+=dx;y+=dy;straight++;segment++;
                    if(segment==expected.count) {
                        edge++;segment=0;
                        CHECK(road_point(full_route_nodes[plan][edge],&point));
                        CHECK(x==point.x && y==point.y);
                        if(edge<FULL_ROUTE_NODES-1u)
                            CHECK(road_plan(full_route_nodes[plan][edge],full_route_nodes[plan][edge+1],&expected));
                    }
                }
            }
            CHECK(straight==18 && edge==16 && x==start.x && y==start.y && heading==0);
            CHECK(run_action_count==run.steps && run.steps==straight+turns);
            printf("PASS route %c: 18 forward segments, %d turns, %u mm, return heading +Y (simulated)\n",plan?'C':'A',turns,EXPECTED_FULL_ROUTE_MM);
        }
    }
    /* 估时经过、但任一电机未到位：不能推进下一段。 */
    reset_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();motor_flags[1]=1;
    clock_ms=drive.started_ms+drive.estimate_ms;
    for(i=0;i<12;i++) {clock_ms+=25;firmware_step();}
    CHECK(run_active && position_count==4 && run.ready!=15);
    CHECK(run.turning && run.heading==HEAD_Y_POS && drive.rotating);
    send_line("go B");CHECK(strstr(output,"[rejected") && position_count==4);
    stop_request();clock_ms+=60000;firmware_step();
    CHECK(!run_active && run.state==RUN_STOPPED && position_count==4 && !pose_set);
    reset_fixture();calibrate_turn_fixture();status_auto_reply=0;send_line("run A");clock_ms+=1001;firmware_step();
    CHECK(run.state==RUN_FAULT && position_count==0 && strstr(output,"reply timeout"));
    reset_fixture();calibrate_turn_fixture();position_ack_enabled=0;send_line("run A");
    {unsigned ticks=0;while(run.state==RUN_PREFLIGHT && ticks++<100){clock_ms+=25;firmware_step();}}
    clock_ms=run.segment_ms+1501;firmware_step();
    CHECK(run.state==RUN_FAULT && position_count==4 && strstr(output,"position ACK"));
    reset_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
    {
        const uint8_t rejected[3]={0xfd,0xe2,0x6b};
        route_test_can_rx(0x100,4,0,3,rejected);firmware_step();
        CHECK(run.state==RUN_FAULT && position_count==4 && strstr(output,"command rejected"));
    }
    reset_fixture();calibrate_turn_fixture();motor_flags[2]=7;send_line("run A");
    for(i=0;i<10;i++) {clock_ms+=25;firmware_step();}
    CHECK(run.state==RUN_FAULT && position_count==0);
    reset_fixture();calibrate_turn_fixture();send_line("run A");bus_off=1;firmware_step();
    CHECK(run.state==RUN_FAULT && position_count==0 && strstr(output,"bus off"));
    reset_fixture();calibrate_turn_fixture();status_auto_reply=0;send_line("run A");
    {
        uint8_t wrong[3]={0x3a,3,0};
        route_test_can_rx((uint32_t)run.pending<<8,4,0,3,wrong);firmware_step();
        CHECK(run.ready==0 && run.pending!=0);
        wrong[2]=0x6b;
        route_test_can_rx((uint32_t)(run.pending%4u+1u)<<8,4,0,3,wrong);firmware_step();
        route_test_can_rx((uint32_t)run.pending<<8,0,0,3,wrong);
        route_test_can_rx(((uint32_t)run.pending<<8)|1u,4,0,3,wrong);
        route_test_can_rx((uint32_t)run.pending<<8,4,0,2,wrong);firmware_step();
        CHECK(run.ready==0);
        for(i=0;i<RUN_RX_SIZE;i++) route_test_can_rx(0x100,4,0,3,wrong);
        firmware_step();CHECK(run.state==RUN_FAULT && position_count==0);
    }
    reset_fixture();calibrate_turn_fixture();send_line("run A");send_line("test");
    CHECK(!run_active && run.state==RUN_IDLE && test_mode);
    send_line("f");CHECK(position_count==4 && drive.parameters.mm==1000);
}
static void button_tick(uint8_t pressed,uint32_t elapsed_ms)
{
    simulated_start_button=pressed;clock_ms+=elapsed_ms;firmware_step();
}
static unsigned button_messages(void)
{
    unsigned count=0;
    const char *cursor=output;
    while((cursor=strstr(cursor,"[button] START pressed\r\n"))!=0) {count++;cursor++;}
    return count;
}
static void start_button_tests(void)
{
    unsigned i;
    uint32_t stops,velocities;
    reset_fixture();firmware_step();output_size=0;output[0]=0;
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS-1u);
    CHECK(button_messages()==0);
    button_tick(0,0);button_tick(1,1);
    button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS-1u);CHECK(button_messages()==0);
    button_tick(1,1);CHECK(button_messages()==1);
    for(i=0;i<10;i++)button_tick(1,100);
    CHECK(button_messages()==1);
    button_tick(0,0);button_tick(0,CAR_START_BUTTON_DEBOUNCE_MS-1u);
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==1);
    button_tick(0,0);button_tick(0,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==1);
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==2 && !run_active && nav.state==NAV_IDLE);
    CHECK(position_count==0 && velocity_count==0 && sync_count==0);

    reset_fixture();clock_ms=UINT32_MAX-10u;
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS-1u);
    CHECK(button_messages()==0);
    button_tick(1,1);CHECK(button_messages()==1);

    reset_fixture();simulated_start_button=1;start_button_init();
    button_tick(1,1000);CHECK(button_messages()==0);
    button_tick(0,0);button_tick(0,CAR_START_BUTTON_DEBOUNCE_MS);
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==1);

    reset_fixture();send_line("test");stops=stop_count;velocities=velocity_count;
    button_tick(1,0);button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==1 && test_mode==MODE_TRANSLATION_TEST && drive.state==MOTION_IDLE);
    CHECK(position_count==0 && velocity_count==velocities && sync_count==0 && stop_count==stops);

    reset_fixture();button_tick(1,0);output_size=0;output[0]=0;stops=stop_count;
    queue_byte('!');button_tick(1,CAR_START_BUTTON_DEBOUNCE_MS);
    CHECK(button_messages()==1 && stop_count>stops && !run_active);
    CHECK(strstr(output,"[stopped]") && strstr(output,"[stopped]")<strstr(output,"[button]"));
    puts("PASS start button: press/release debounce, one event per hold, re-press, timestamp wrap, boot-held suppression, test mode and stop priority; no motion triggered");
}
static void scanner_receive(const uint8_t *bytes,unsigned count)
{
    unsigned i;
    for(i=0;i<count;i++)CHECK(route_test_scanner_rx(bytes[i],0));
    firmware_step();
}
static void scanner_finish_setup(void)
{
    /* Expected packets are transcribed independently from the supplied manual image. */
    static const uint8_t expected[18]={0x7e,0,8,1,0,0x60,0x21,0x4b,0xf0,
                                      0x7e,0,8,1,0,0,0xd6,0xdf,0x22};
    static const uint8_t ack[7]={2,0,0,1,0,0x33,0x31};
    unsigned i;
    scanner_init(CAR_SCANNER_UART_BAUD);
    CHECK(scanner_initialized_baud==CAR_SCANNER_UART_BAUD && scanner.state==SCANNER_BOOT);
    clock_ms+=CAR_SCANNER_BOOT_MS-1u;firmware_step();CHECK(!scanner_sent_count);
    clock_ms++;
    for(i=0;i<9;i++)firmware_step();
    CHECK(scanner_sent_count==9 && scanner.state==SCANNER_SET_ENDING);
    CHECK(!memcmp(scanner_sent,expected,9));
    scanner_receive(ack,3);CHECK(scanner.state==SCANNER_SET_ENDING && scanner.ack_offset==3);
    scanner_receive(ack+3,4);CHECK(scanner.state==SCANNER_SET_CONTINUOUS && scanner_sent_count==10);
    for(i=0;i<8;i++)firmware_step();
    CHECK(scanner_sent_count==18 && !memcmp(scanner_sent,expected,18));
    scanner_receive(ack,7);
    CHECK(scanner.state==SCANNER_READY && strstr(output,"READY continuous acknowledged"));
    CHECK(!scanner.results && !run_active && !position_count && !velocity_count && !sync_count);
}
static void scanner_tests(void)
{
    static const uint8_t qr1[]="134+123+314+231\r\n";
    static const uint8_t qr2[]="hello\r\nworld\r\n";
    static const uint8_t bad_ack[7]={2,0,0,1,0,0x33,0x30};
    uint8_t too_long[16];
    unsigned i;
    reset_fixture();scanner_finish_setup();
    output_size=0;output[0]=0;
    scanner_receive(qr1,5);CHECK(!scanner.results && !strstr(output,"[scan]"));
    scanner_receive(qr1+5,sizeof(qr1)-6u);
    CHECK(scanner.results==1 && strstr(output,"[scan] 134+123+314+231\r\n"));
    scanner_receive(qr2,sizeof(qr2)-1u);
    CHECK(scanner.results==3 && strstr(output,"[scan] hello\r\n") && strstr(output,"[scan] world\r\n"));
    scanner_receive((const uint8_t *)"unfinished",10);
    CHECK(scanner.results==3);
    CHECK(route_test_scanner_rx(0,1));firmware_step();
    CHECK(scanner.uart_errors==1 && scanner.state==SCANNER_READY && scanner.discard);
    scanner_receive((const uint8_t *)"tail\r\n",6);
    scanner_receive(qr1,sizeof(qr1)-1u);CHECK(scanner.results==4);
    memset(too_long,'x',sizeof(too_long));
    for(i=0;i<10;i++)scanner_receive(too_long,sizeof(too_long));
    CHECK(scanner.discard && scanner.results==4);
    scanner_receive((const uint8_t *)"\r\n",2);
    scanner_receive(qr1,sizeof(qr1)-1u);CHECK(scanner.results==5);
    for(i=0;i<SCANNER_RX_SIZE;i++)CHECK(route_test_scanner_rx('x',0));
    firmware_step();CHECK(scanner.overflows==1 && scanner.discard && scanner.results==5);
    scanner_receive((const uint8_t *)"\r\n",2);
    scanner_receive(qr1,sizeof(qr1)-1u);CHECK(scanner.results==6);
    output_size=0;output[0]=0;queue_byte('!');
    scanner_receive(qr1,sizeof(qr1)-1u);
    CHECK(!run_active && scanner.results==7 && stop_count);
    CHECK(strstr(output,"[stopped]")<strstr(output,"[scan]"));
    send_line("qr");CHECK(strstr(output,"state=READY") && strstr(output,"results=7"));
    send_line("qr baud 115200");CHECK(scanner_initialized_baud==115200u && scanner.state==SCANNER_BOOT);
    send_line("qr init");CHECK(scanner.baud==115200u && scanner.state==SCANNER_BOOT);
    CHECK(!position_count && !sync_count && !run_active);

    reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;
    for(i=0;i<9;i++)firmware_step();
    scanner_receive(bad_ack,7);CHECK(scanner.state==SCANNER_SET_ENDING);
    clock_ms+=CAR_SCANNER_ACK_MS;firmware_step();
    CHECK(scanner.state==SCANNER_FAULT && !scanner.results && scanner_sent_count==9);
    clock_ms+=10000;firmware_step();CHECK(scanner_sent_count==9);
    send_line("qr init");CHECK(scanner.state==SCANNER_BOOT && !scanner.results);

    reset_fixture();clock_ms=UINT32_MAX-500u;scanner_can_tx=0;scanner_init(9600);
    clock_ms+=CAR_SCANNER_BOOT_MS;firmware_step();CHECK(scanner.state==SCANNER_SET_ENDING && !scanner_sent_count);
    clock_ms+=CAR_SCANNER_ACK_MS;firmware_step();CHECK(scanner.state==SCANNER_FAULT);
    reset_fixture();scanner_init(9600);route_test_scanner_rx(0,1);firmware_step();
    CHECK(scanner.state==SCANNER_FAULT && scanner.uart_errors==1 && !scanner_sent_count);
    puts("PASS XM1601: exact CRLF/continuous packets, fragmented ACK, no premature READY, QR line assembly, burst results, UART/queue/line overflow recovery, invalid ACK timeout, explicit baud/reinit, timer wrap and stop priority; no motion or Flash writes");
}
int main(void)
{
    direction_tests();range_and_mode_tests();stop_and_timeout_tests();navigation_regression();trim_tests();rotation_tests();full_route_tests();
    start_button_tests();
    scanner_tests();
    printf("PASS entry: %lu assertions; 12 direction/speed cases; trim/stop/range/mode/timeout/navigation/full-route feedback checks\n",(unsigned long)assertions);
    return 0;
}
