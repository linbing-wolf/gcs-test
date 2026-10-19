/* 复用真实底盘串口入口，同时模拟1~4轮和5/7两轴；不连接硬件。 */
#define main entry_test_main
#include "verify_entry.c"
#undef main

static void advance_arm(unsigned ms)
{
    unsigned i;
    for(i=0;i<ms;i++){clock_ms++;firmware_step();}
}
static void acquire_arm(void)
{
    send_line("arm on");
    CHECK(!arm_enabled() && !arm_enables && !arm_claw_us && !arm_yaw_us && !arm_platform_us);
    advance_arm(400);
    CHECK(arm_enabled() && arm_lock.state==2 && arm_enables==2 && !arm_positions);
    CHECK(!arm_claw_us && !arm_yaw_us && !arm_platform_us);
}
static void declare_pose(void)
{
    acquire_arm();send_line("arm pose 0 135 0 45");
    CHECK(arm_claw_us==826 && arm_yaw_us==1975);
    CHECK(strstr(output,"NOT automatic homing"));
}
static void expect_pose(int slide,int lift,float yaw,float claw)
{
    char expected[100];send_line("arm status");
    snprintf(expected,sizeof(expected),"slide_mm=%d lift_mm=%d yaw_deg=%.1f claw_deg=%.1f",slide,lift,(double)yaw,(double)claw);
    CHECK(strstr(output,expected) && strstr(output,"referenced=1 state=0"));
}
static void manual_and_recipe_tests(void)
{
    unsigned i;
    reset_fixture();CHECK(!arm_enables && !arm_positions);
    send_line("arm pick");CHECK(strstr(output,"confirmation required") && !arm_positions);
    declare_pose();
    send_line("arm slide 1");advance_arm(2400);expect_pose(1,135,0,45);
    CHECK(arm_positions==1 && arm_trace_address[0]==7 && arm_trace_dir[0]==1 && arm_trace_pulses[0]==25);
    send_line("arm lift 130");advance_arm(2400);expect_pose(1,130,0,45);
    CHECK(arm_trace_address[1]==5 && arm_trace_dir[1]==0 && arm_trace_pulses[1]==400);
    send_line("arm home");advance_arm(7000);expect_pose(0,135,0,45);
    send_line("arm yaw 126");advance_arm(2400);CHECK(arm_yaw_us==1275);
    send_line("arm yaw 254");advance_arm(2400);CHECK(arm_yaw_us==564);
    send_line("arm yaw -94.5");advance_arm(2400);CHECK(arm_yaw_us==2500);
    send_line("arm yaw 265.5");advance_arm(2400);CHECK(arm_yaw_us==500);
    send_line("arm home");advance_arm(7000);
    send_line("arm pick");advance_arm(12000);expect_pose(55,105,131,15);
    CHECK(arm_yaw_us==1247 && arm_claw_us==604);
    send_line("arm place");CHECK(strstr(output,"declare actual platform") && !arm_stops);
    send_line("arm platform pose 20");CHECK(arm_platform_us==648);
    send_line("arm place");advance_arm(14500);expect_pose(35,110,256,45);CHECK(arm_yaw_us==553);
    send_line("arm off");CHECK(arm_enabled() && strstr(output,"park with arm home"));
    send_line("arm home");advance_arm(7000);expect_pose(0,135,0,45);
    for(i=1;i<=3;i++){
        char command_text[32];snprintf(command_text,sizeof(command_text),"arm platform %u",i);
        send_line(command_text);advance_arm(2400);
        CHECK(arm_platform_us==(i==1?648:i==2?1530:2396));
        CHECK(!position_count && !sync_count);
    }
    send_line("arm cycle");advance_arm(33000);expect_pose(35,110,256,45);
    send_line("arm home");advance_arm(7000);send_line("arm off");
    CHECK(!arm_enabled() && !arm_lock.state && !arm_claw_us && !arm_yaw_us && !arm_platform_us);
    CHECK(!position_count && !sync_count);
    send_line("test");send_line("f");CHECK(position_count==4 && sync_count==1);
    puts("PASS merged arm: latest calibrated PWM, signed two-axis movement, home/pick/place/cycle, three platform slots, park-before-release and return to chassis");
}
static void mutual_exclusion_tests(void)
{
    static const char *blocked[]={"runimu A","runimu C","run A","run C","test","turn","nav","go B","f","b","l","r","left90","tl"};
    unsigned i;
    reset_fixture();send_line("test");send_line("f");send_line("arm on");
    CHECK(!arm_enabled() && !arm_lock.state && strstr(output,"chassis active"));
    reset_fixture();send_line("pose A");send_line("go B");send_line("arm on");
    CHECK(!arm_enabled() && strstr(output,"chassis active"));
    reset_fixture();send_line("run A"); /* removed route is rejected */
    declare_pose();
    for(i=0;i<sizeof(blocked)/sizeof(blocked[0]);i++) {
        send_line(blocked[i]);CHECK(strstr(output,"arm session owns motion"));
        CHECK(!run_active && !position_count && !sync_count && !arm_positions);
    }
    send_line("arm slide 10");advance_arm(200);CHECK(arm_positions==1);
    send_line("runimu C");CHECK(strstr(output,"arm session owns motion"));
    send_line("arm off");CHECK(arm_enabled() && strstr(output,"park with arm home"));
    stop_request();CHECK(arm_stops==2 && arm_lock.state==3 && !position_count);
    send_line("arm slide 0");CHECK(strstr(output,"confirmation required"));
    send_line("arm on");advance_arm(400);CHECK(arm_lock.state==2);
    send_line("arm pick");CHECK(strstr(output,"pose unknown"));
    send_line("arm pose 10 135 0 45");send_line("arm home");advance_arm(7000);
    send_line("arm off");CHECK(!arm_lock.state && !arm_enabled());
    puts("PASS mutual exclusion: removed-run/nav/runimu/shift/turn blocked in arm session; chassis rejects arm acquisition while busy; ! stops both and requires explicit recovery");
}
static void wheel_lock_failures(void)
{
    unsigned mode;
    for(mode=0;mode<5;mode++) {
        reset_fixture();
        if(mode==0)diagnostic_auto_reply=0;
        if(mode==1)status_auto_reply=0;
        if(mode==2)motor_flags[3]=0;
        if(mode==3)bus_off=1;
        if(mode==4){run.imu_mode=1;moving_at_stop_motor=4;}
        send_line("arm on");advance_arm(2400);
        CHECK(!arm_enables && !arm_enabled() && arm_lock.state==3);
        CHECK(!arm_positions && strstr(output,"interlock fault"));
    }
    for(mode=0;mode<5;mode++) {
        reset_fixture();declare_pose();send_line("arm slide 30");advance_arm(200);
        if(mode==0)diagnostic_auto_reply=0;
        if(mode==1)status_auto_reply=0;
        if(mode==2)motor_flags[3]=7;
        if(mode==3)bus_off=1;
        if(mode==4){run.imu_mode=1;moving_at_stop_motor=4;}
        advance_arm(650);
        CHECK(arm_stops==2 && arm_lock.state==3 && !position_count && !sync_count);
        send_line("runimu C");CHECK(strstr(output,"arm session owns motion"));
    }
    reset_fixture();declare_pose();
    {uint8_t frame[3]={0x3a,3,0x6b};unsigned i;
        for(i=0;i<40;i++)route_test_can_rx(1u<<8,4,0,3,frame);
        advance_arm(1);CHECK(arm_lock.state==3 && arm_stops==2);
    }
    reset_fixture();clock_ms=UINT32_MAX-100;declare_pose();CHECK(arm_lock.state==2);
    /* 复现现场的1RPM停机条件；这次不放宽底盘静止互锁。 */
    for(mode=0;mode<2;mode++) {
        reset_fixture();declare_pose();send_line("arm yaw 131");advance_arm(200);
        run.imu_mode=1;reported_wheel_rpm[1]=mode ? -1 : 1;
        advance_arm(100);
        CHECK(arm_lock.state==3 && arm_stops==2 && strstr(output,"wheel moving/disabled/stalled"));
    }
    puts("PASS four-wheel interlock: missing status/speed, disabled/stalled wheel 4, moving wheel 4, bus off, queue overflow, active-session watchdog and timestamp wrap");
}
static void arm_faults_and_stop_tests(void)
{
    unsigned mode;
    for(mode=0;mode<9;mode++) {
        reset_fixture();declare_pose();
        if(mode==0)arm_auto_reply=0;
        if(mode==1)arm_missing_motor=5;
        if(mode==2)arm_missing_motor=7;
        if(mode==3)arm_ack=0;
        if(mode==4)arm_freeze=1;
        if(mode==5)arm_bad_direction=1;
        if(mode==6)arm_flags=7;
        if(mode==7)arm_inject_stop=1;
        if(mode==8)arm_inject_can_error=1;
        send_line("arm slide 1");advance_arm(16000);
        CHECK(arm_stops>=2 && !strstr(output,"[arm done") && !position_count && !sync_count);
    }
    reset_fixture();declare_pose();
    send_line("arm platform pose 20");send_line("arm platform 3");advance_arm(800);
    {uint16_t held=arm_platform_us;CHECK(held>648 && held<2396);
        stop_request();advance_arm(3000);CHECK(arm_platform_us==held);
    }
    reset_fixture();declare_pose();send_line("arm yawtest 0");advance_arm(8400);
    CHECK(arm_yaw_us==2500 && arm_claw_us==826 && !arm_positions);
    send_line("arm yawtest 360");advance_arm(8400);CHECK(arm_yaw_us==500);
    puts("PASS arm failure paths: either axis missing, ACK missing, frozen/reversed encoder, stall, stop/CAN fault during submission; platform PWM holds at stop; full-stroke yaw diagnostic");
}
static void bounds_and_shared_service_tests(void)
{
    static const char *bad[]={"arm slide -1","arm slide 188","arm lift 136","arm lift 1.2",
        "arm lift 9999999999999999999999999","arm yaw -94.6","arm yaw 265.6","arm yaw NaN","arm yaw 1.23",
        "claw angle -0.1","claw angle 270.1","arm platform 0","arm platform 4","arm platform pose 270.1"};
    unsigned i;
    reset_fixture();declare_pose();
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++){send_line(bad[i]);CHECK(strstr(output,"rejected") && !arm_positions);}
    send_line("arm platform 2");CHECK(strstr(output,"declared angle"));
    send_line("arm pick");advance_arm(100);
    send_line("arm lift 130");CHECK(strstr(output,"busy"));
    send_line("qr");CHECK(strstr(output,"[scanner]"));
    send_line("imu");CHECK(strstr(output,"[imu]"));
    simulated_start_button=1;advance_arm(40);CHECK(strstr(output,"START pressed"));
    /* 5/7号回包不能污染底盘四轮队列；反向亦然。 */
    {uint8_t frame[3]={0x3a,3,0x6b};uint8_t before=run_rx_head;
        route_test_can_rx(5u<<8,4,0,3,frame);CHECK(run_rx_head==before);
    }
    puts("PASS boundaries/shared services: malformed/out-of-range commands, busy rejection, scanner/IMU/button continue during arm motion, CAN address isolation");
}
static void english_console_tests(void)
{
    reset_fixture();declare_pose();
    send_line("arm lift 120");advance_arm(2400);
    CHECK(strstr(output,"[arm step: waiting for fresh axis feedback]\r\n"));
    CHECK(strstr(output,"[arm step complete: axes feedback verified; servo motion estimated]\r\n"));
    CHECK(strstr(output,"[arm done: inspect actual grip; no grip sensor]\r\n"));
    CHECK(!arm_busy() && !arm_failed() && arm_pose_known() && arm_trace_address[0]==5 && arm_trace_pulses[0]==1200);
    send_line("arm status");
    CHECK(strstr(output,"lift_mm=120") && strstr(output,"; servo positions are commanded estimates\r\n"));
    CHECK(strstr(output,"; no position sensor\r\n") && strstr(output,"[arm]"));
    {size_t i;for(i=0;i<output_size;i++)CHECK((unsigned char)output[i]<128u);}
    puts("PASS English console: lift 120 retains axis command and all three messages; complete arm/platform status is ASCII");
}
static void speed_and_feedback_gates_tests(void)
{
    unsigned missing;
    uint32_t started;
    reset_fixture();declare_pose();
    started=clock_ms;send_line("arm slide 55");advance_arm(350);
    CHECK(!arm_busy() && !arm_failed() && arm_positions==1);
    CHECK(arm_trace_address[0]==7 && arm_trace_pulses[0]==1401 && clock_ms-started<1000u);
    send_line("arm lift 105");advance_arm(350);
    CHECK(!arm_busy() && !arm_failed() && arm_trace_address[1]==5 && arm_trace_pulses[1]==2400);
    started=clock_ms;send_line("arm close");advance_arm(250);CHECK(arm_busy());
    advance_arm(500);CHECK(!arm_busy() && !arm_failed() && arm_claw_us==604);
    CHECK(clock_ms-started>=ARM_CLAW_TIME_MS);
    send_line("arm yaw 256");advance_arm(1000);CHECK(arm_busy() && arm_yaw_us>553);
    advance_arm(1400);CHECK(!arm_busy() && arm_yaw_us==553);

    /* 即使已取消2秒下限，迟到的位置反馈、缺失ACK和非零轴速也不能放行。 */
    reset_fixture();declare_pose();arm_freeze=1;
    send_line("arm slide 55");advance_arm(700);CHECK(arm_busy());
    arm_encoder[0]-=(int64_t)1401*65536u/3200u;
    advance_arm(350);CHECK(!arm_busy() && !arm_failed());
    for(missing=0;missing<3;missing++) {
        reset_fixture();declare_pose();
        if(missing==0)arm_ack=0;
        if(missing==1)arm_missing_motor=5;
        if(missing==2)arm_missing_motor=7;
        send_line("arm slide 55");advance_arm(2000);
        CHECK(arm_failed() && !arm_pose_known() && !strstr(output,"[arm done"));
    }
    reset_fixture();declare_pose();arm_moving_motor=7;
    send_line("arm slide 55");advance_arm(350);
    CHECK(arm_failed() && !arm_positions);
    puts("PASS arm speed: axes finish on fresh verified feedback without 2s floor; claw keeps 500ms interpolation, yaw keeps 2s; delayed position/missing ACK/either axis/nonzero axis speed cannot complete early");
}
int main(void)
{
    manual_and_recipe_tests();mutual_exclusion_tests();wheel_lock_failures();
    arm_faults_and_stop_tests();bounds_and_shared_service_tests();english_console_tests();speed_and_feedback_gates_tests();
    printf("PASS arm integration: %u assertions; host simulation only, no hardware accessed\n",assertions);
    return 0;
}
