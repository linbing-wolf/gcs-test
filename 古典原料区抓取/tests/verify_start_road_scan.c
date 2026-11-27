/* 使用实际入口、航向闭环及独立电机模型；超声波按精确边沿注入，不接实车。 */
#define HEADING_TEST_MAIN inherited_heading_tests
#include "verify_heading.c"

static char scan_output[65536];
static uint8_t echo_pending,echo_risen;
static uint32_t echo_rise_us,echo_fall_us,last_capture;
static void scan_write(const char *s)
{
    CHECK(strlen(scan_output)+strlen(s)<sizeof(scan_output));strcat(scan_output,s);
}
static void scan_fixture(unsigned plan,int16_t yaw)
{
    RoadPoint p;
    reset_plant(yaw,1.15);station_init();station_command("station auto off");
    CHECK(road_point(plan?NODE_C:NODE_A,&p));
    planned_x=field_x=p.x;planned_y=field_y=p.y;
    initial_field_heading=plan?HEAD_Y_NEG:HEAD_Y_POS;
    scan_output[0]=0;port.write=scan_write;
    echo_pending=echo_risen=0;last_capture=UINT32_MAX;
}
static void scan_tick(int reply)
{
    uint32_t us=(clock_ms+1u)*1000u;
    if(reply && echo_pending) {
        if(!echo_risen && (int32_t)(us-echo_rise_us)>=0) {
            start_scan_test_edge(1,echo_rise_us);echo_risen=1;
        }
        if(echo_risen && (int32_t)(us-echo_fall_us)>=0) {
            start_scan_test_edge(0,echo_fall_us);echo_pending=echo_risen=0;
        }
    }
    start_scan_test_time(us);plant_step();
    if(capture.phase==CAP_RISE && last_capture!=capture.started) {
        last_capture=capture.started;echo_pending=1;echo_risen=0;
        echo_rise_us=clock_ms*1000u+200u;echo_fall_us=echo_rise_us+2813u; /* 独立设定回波=485mm。 */
    }
}
static void scan_full_test(unsigned plan,int16_t yaw,int reply)
{
    uint32_t started;
    scan_fixture(plan,yaw);send_line("scan size 300 280 20");
    nav.blocked=0xaaa;send_line(plan?"scan C":"scan A");
    CHECK(start_scan_active() && run_active && run.turn_only && run.heading_control.target_cdeg==0);
    started=clock_ms;
    while(start_scan_active() && (uint32_t)(clock_ms-started)<120000u)scan_tick(reply);
    CHECK(!start_scan_active() && scan.phase==SCAN_DONE && scan.index==30 && scan.positions==31);
    CHECK(nav.blocked==0xaaa && run.distance_mm==0 && !forward_actions && !lateral_actions);
    CHECK(abs((int)lround(physical_yaw)-yaw-(plan?9000:-9000))<=CAR_HEADING_TOLERANCE_CDEG+2);
    CHECK(scan.good+scan.failed==93 && !strstr(scan_output,"state=CLEAR") && !strstr(scan_output,"state=BLOCKED"));
    if(reply)CHECK(scan.good==93 && scan.failed==0);
    else CHECK(scan.good==0 && scan.failed==93 && !strstr(scan_output,"state=SUSPECT"));
    CHECK(strstr(scan_output,"[scan done]") && !run_active);
    printf("SCAN_FULL plan=%c reply=%d elapsed_ms=%lu good=%lu failed=%lu actual_turn_cdeg=%ld\n",
        plan?'C':'A',reply,(unsigned long)(clock_ms-started),(unsigned long)scan.good,
        (unsigned long)scan.failed,(long)lround(physical_yaw)-yaw);
    if(reply) {
        uint32_t deadline;
        station_command("station auto on");
        simulated_start_button=1;
        for(unsigned i=0;i<(plan?100u:1000u);i++)scan_tick(1);
        CHECK(!run_active);
        simulated_start_button=0;deadline=clock_ms+10000u;
        while((!run_active || run.velocity_phase!=VELOCITY_FORWARD)
            && (int32_t)(clock_ms-deadline)<0)scan_tick(1);
        CHECK(run_active && run.plan==plan && run.initial_heading==HEAD_X_NEG);
        CHECK(!run.turn_only && !run.turning && run.travel_axis==BODY_LEFT && scanner_init_calls==1);
        CHECK(arm_ready() && arm_parked() && arm_platform_slot()==1);
        printf("SCAN_TO_BUTTON plan=%c initial_heading=-X first_axis=BODY_LEFT scanner_init=1\n",plan?'C':'A');
        stop_request();scan_tick(1);
    }
}
static void capture_tests(void)
{
    scan_fixture(0,0);capture_init();start_scan_test_time(0);capture_begin(1u);
    CHECK(!test_trigger && capture.phase==CAP_RISE);
    start_scan_test_edge(1,200);start_scan_test_edge(0,1360);
    CHECK(capture.ready && capture.status==ECHO_OK && capture.pulse_us==1160);
    /* 主循环可延后读取，中断记录的脉宽仍是1160us；DWT32位回绕也保留差值。 */
    start_scan_test_time(20000);CHECK(capture.pulse_us==1160);
    capture_cancel();test_echo=0;test_cycles=UINT32_MAX-10000u;capture_begin(1u);
    capture_edge(1,capture.started+200u*168u);capture_edge(0,capture.started+1360u*168u);
    CHECK(capture.ready && capture.status==ECHO_OK && capture.pulse_us==1160);
    capture_cancel();test_echo=1;capture_begin(1u);CHECK(capture.status==ECHO_PREHIGH);
    test_echo=0;capture_begin(1u);capture_edge(1,capture.started+200u*168u);
    capture_edge(0,capture.started+50000u*168u);CHECK(capture.status==ECHO_HIGH_TIMEOUT);
    capture_cancel();capture_begin(1u);capture_edge(1,capture.started+200u*168u);
    capture_edge(0,capture.started+250u*168u);CHECK(capture.status==ECHO_OUT_OF_RANGE);
    puts("PASS scan capture: delayed foreground, exact edge widths, DWT wrap, pre-high, late and short pulses");
}
static void geometry_tests(void)
{
    scan_fixture(0,0);scan.node=NODE_A;
    CHECK(fabs(chassis_radius_mm()-sqrt(175.5*175.5+135.0*135.0))<0.001);
    CHECK(road_candidates(0,515)==(3u|(1u<<11))); /* 后悬使B路口的BC/EB也进入候选，不再误报唯一AB。 */
    send_line("scan size 200 200 0"); /* 紧凑替身外廓用于单独验证唯一道路判据，不是实车尺寸。 */
    CHECK(road_candidates(0,425)==1u); /* A 朝 B，(2290,740) 的回波只与 AB 候选区相交。 */
    scan.node=NODE_C;CHECK(road_candidates(0,425)==2u);
    CHECK(point_segment(10,5,0,0,20,0)==5.0);
    CHECK(point_segment(-5,0,0,0,20,0)==5.0);
    scan.node=NODE_A;scan.valid=3;scan.shots=3;scan.index=0;
    scan.ranges[0]=425;scan.ranges[1]=425;scan.ranges[2]=1300;
    scan.angles[0]=scan.angles[1]=scan.angles[2]=0;finish_position();
    CHECK(scan.candidates[0]==1 && scan.unique[0]==1);
    scan.valid=3;scan.ranges[0]=100;scan.ranges[1]=900;scan.ranges[2]=2000;
    finish_position();CHECK(scan.candidates[0]==1);
    puts("PASS road association: A/C directions, 145mm probe offset, asymmetric rear platform +30mm, conservative ambiguity, outlier median and unstable rejection");
}
static void cancellation_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        scan_fixture(0,0);send_line("scan size 300 280 20");send_line("scan A");
        if(variant>=2) {
            while(scan.phase!=SCAN_CAPTURE)scan_tick(1);
        }
        send_line("runimu C");CHECK(start_scan_active() && run.turn_only);
        if(variant&1)send_line("scan stop");else queue_byte('!');
        scan_tick(1);CHECK(!start_scan_active() && !run_active && !test_trigger);
        for(unsigned i=0;i<500;i++)scan_tick(1);
        CHECK(scan.phase==SCAN_CANCELLED && !run_active);
    }
    scan_fixture(0,0);scan.configured=0;send_line("scan A");CHECK(!start_scan_active() && !run_active);
    send_line("scan size 300 280 20");heading_sensor_invalidate(&heading_sensor);
    send_line("scan A");CHECK(!start_scan_active() && !run_active);
    scan_fixture(0,0);CHECK(scan.length_mm==291 && scan.width_mm==270 && scan.margin_mm==20);
    send_line("scan A");while(scan.phase!=SCAN_CAPTURE)scan_tick(0);
    frozen_imu=1;heading_sensor_invalidate(&heading_sensor);
    for(unsigned i=0;i<60;i++)scan_tick(0);
    CHECK(capture.ready && capture.status==ECHO_NO_RISE);
    stop_request();scan_tick(0);CHECK(!start_scan_active());
    puts("PASS scan ownership/cancel: missing config/IMU rejected, model defaults, stale-IMU capture timeout, active scan blocks route, ! and scan stop cancel without restart");
}
int main(void)
{
    capture_tests();geometry_tests();cancellation_tests();
    scan_full_test(0,17900,1);scan_full_test(1,-17900,1);scan_full_test(0,0,0);
    printf("PASS start scan: %lu assertions; software models only, no physical obstacle-road acceptance\n",(unsigned long)assertions);
    return 0;
}
