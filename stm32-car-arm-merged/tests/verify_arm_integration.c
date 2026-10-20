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
static void station_platform_tests(void)
{
    unsigned variant;
    reset_fixture();declare_pose();send_line("arm platform pose 20");
    send_line("arm pose 0 135 125 45");
    send_line("arm platform 2");CHECK(strstr(output,"arm home pose") && !arm_busy());
    CHECK(arm_platform_at_yaw(2,125.0f,clock_ms));advance_arm(2400);
    CHECK(arm_ready() && arm_platform_slot()==2 && arm_platform_us==1530);
    expect_pose(0,135,125,45);
    CHECK(arm_platform_at_yaw(3,125.0f,clock_ms));advance_arm(2400);
    CHECK(arm_platform_slot()==3 && arm_platform_us==2396);expect_pose(0,135,125,45);
    for(variant=0;variant<8;variant++) {
        unsigned before_pwm;
        reset_fixture();declare_pose();
        if(variant!=4)send_line("arm platform pose 20");
        send_line("arm pose 0 135 125 45");
        if(variant==0)send_line("arm pose 1 135 125 45");
        if(variant==1)send_line("arm pose 0 134 125 45");
        if(variant==2)send_line("arm pose 0 135 124 45");
        if(variant==3)send_line("arm yaw 126");
        if(variant==5)arm_stop();
        before_pwm=arm_platform_us;
        CHECK(!arm_platform_at_yaw((uint8_t)(variant==6 ? 0 : variant==7 ? 4 : 2),125.0f,clock_ms));
        CHECK(arm_platform_us==before_pwm);
    }
    for(variant=0;variant<2;variant++) {
        uint16_t held;
        reset_fixture();declare_pose();send_line("arm platform pose 20");
        send_line("arm pose 0 135 125 45");held=arm_platform_us;
        arm_missing_motor=(uint8_t)(variant ? 7 : 5);
        CHECK(arm_platform_at_yaw(2,125.0f,clock_ms));advance_arm(600);
        CHECK(!arm_failed() && arm_pose_known() && arm_busy() && arm_platform_us==held);
        arm_missing_motor=0;advance_arm(1500);
        CHECK(!arm_busy() && arm_ready() && arm_platform_slot()==2);
    }
    puts("PASS station platform: high/retracted125 internal selection holds yaw; manual command still requires home0; extended/low/wrong-angle/busy/unknown/fault/invalid-slot rejected");
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
    unsigned mode,wheel;
    for(mode=0;mode<2;mode++) {
        reset_fixture();
        if(mode)status_auto_reply=0;else diagnostic_auto_reply=0;
        send_line("arm on");advance_arm(2600);
        CHECK(arm_lock.state==1 && !arm_enabled() && !arm_positions);
        status_auto_reply=diagnostic_auto_reply=1;advance_arm(500);
        CHECK(arm_lock.state==2 && arm_enabled());
    }
    for(wheel=0;wheel<4;wheel++)for(mode=0;mode<2;mode++) {
        reset_fixture();declare_pose();run.imu_mode=1;
        reported_wheel_rpm[wheel]=(int16_t)(mode ? -40 : 40);
        send_line("arm yaw 125");advance_arm(4200);
        CHECK(arm_lock.state==2 && !arm_stops && !arm_busy() && arm_pose_known());
    }
    for(mode=0;mode<2;mode++) {
        reset_fixture();declare_pose();
        if(mode)status_auto_reply=0;else diagnostic_auto_reply=0;
        advance_arm(1200);CHECK(arm_lock.state==2 && !arm_stops && arm_pose_known());
        status_auto_reply=diagnostic_auto_reply=1;advance_arm(500);
        CHECK(arm_lock.state==2 && !arm_stops);
    }
    for(mode=0;mode<2;mode++) {
        reset_fixture();declare_pose();
        if(mode)bus_off=1;else motor_flags[3]=7;
        advance_arm(650);CHECK(arm_stops==2 && arm_lock.state==3);
    }
    reset_fixture();declare_pose();
    {uint8_t frame[3]={0x3a,3,0x6b};unsigned i;
        for(i=0;i<40;i++)route_test_can_rx(1u<<8,4,0,3,frame);
        advance_arm(500);CHECK(arm_lock.state==2 && !arm_stops);
    }
    reset_fixture();declare_pose();send_line("arm slide 55");advance_arm(20);
    {uint8_t frame[3]={0x3a,3,0x6b};unsigned i;
        for(i=0;i<40;i++)route_test_can_rx(5u<<8,4,0,3,frame);
        advance_arm(1000);CHECK(!arm_failed() && arm_pose_known() && !arm_busy() && !arm_stops);
    }
    puts("PASS wheel monitoring: delayed acquisition/stale replies and receive overflow retain session and recover; signed40RPM persists without stop; driver stall and bus-off still stop");
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
        if(mode<6) {
            CHECK(!arm_failed() && arm_pose_known() && !arm_stops);
            if(mode!=3)CHECK(arm_busy());
            arm_auto_reply=arm_ack=1;arm_missing_motor=arm_freeze=arm_bad_direction=0;
            if(mode==4 || mode==5)arm_encoder[0]=-(int64_t)25*65536u/3200u;
            advance_arm(1500);CHECK(!arm_busy() && !arm_failed() && arm_pose_known());
        } else CHECK(arm_stops>=2 && !strstr(output,"[arm done") && !position_count && !sync_count);
    }
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm platform 3");advance_arm(ARM_SERVO_TIME_MS/2u);
    {uint16_t held=arm_platform_us;stop_request();advance_arm(3000);CHECK(arm_platform_us==held);}
    reset_fixture();declare_pose();send_line("arm yawtest 0");advance_arm(8400);CHECK(arm_yaw_us==2500 && arm_claw_us==826 && !arm_positions);
    send_line("arm yawtest 360");advance_arm(8400);CHECK(arm_yaw_us==500);
    puts("PASS arm monitoring: feedback/ACK/action deadlines retain pose and recover on actual position; explicit stop, driver fault and failed CAN submission still stop");
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
    started=clock_ms;send_line("arm close");advance_arm(ARM_CLAW_TIME_MS/2u);CHECK(arm_busy());
    advance_arm(500);CHECK(!arm_busy() && !arm_failed() && arm_claw_us==604);
    CHECK(clock_ms-started>=ARM_CLAW_TIME_MS);
    send_line("arm yaw 256");advance_arm(ARM_SERVO_TIME_MS/2u);CHECK(arm_busy() && arm_yaw_us>553);
    advance_arm(1400);CHECK(!arm_busy() && arm_yaw_us==553);

    /* 即使已取消2秒下限，迟到的位置反馈和缺失ACK仍不能放行，非零轴速单独验证为只记日志。 */
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
        CHECK(!arm_failed() && arm_pose_known());
        if(missing==0)CHECK(!arm_busy());
        else {CHECK(arm_busy());arm_missing_motor=0;advance_arm(1500);CHECK(!arm_busy() && !arm_failed());}
    }

    printf("PASS arm speed: axes use fresh verified feedback; claw_ms=%u yaw_platform_ms=%u; position/status must arrive; missing ACK or RPM magnitude alone cannot cancel or gate measured completion\n",
        ARM_CLAW_TIME_MS,ARM_SERVO_TIME_MS);
}
static void axis_speed_log_only_tests(void)
{
    static const int16_t speeds[]={-3000,-15,-1,1,15,3000};
    unsigned axis,variant;
    for(axis=0;axis<2;axis++)for(variant=0;variant<sizeof(speeds)/sizeof(speeds[0]);variant++) {
        char expected[96];
        reset_fixture();declare_pose();arm_speed_bias[axis]=speeds[variant];
        send_line(axis ? "arm lift 100" : "arm slide 55");advance_arm(600);
        CHECK(!arm_busy() && !arm_failed() && !arm_stops && arm_positions==1);
        CHECK(arm_trace_address[0]==(axis ? 5 : 7));
        snprintf(expected,sizeof(expected),"stage=preflight addr=%u speed_rpm=%d; logged only",axis ? 5u : 7u,(int)speeds[variant]);
        CHECK(strstr(output,expected));
        snprintf(expected,sizeof(expected),"stage=complete addr=%u speed_rpm=%d; logged only",axis ? 5u : 7u,(int)speeds[variant]);
        CHECK(strstr(output,expected));
        send_line("arm yaw 125");advance_arm(1400);
        CHECK(!arm_busy() && !arm_failed() && !arm_stops); /* 舵机步骤也会检查两轴，不能被静止轴速读数拦截。 */
    }
    reset_fixture();declare_pose();arm_speed_bias[0]=15;arm_speed_bias[1]=-15;arm_freeze=1;
    send_line("arm slide 55");advance_arm(700);CHECK(arm_busy() && !arm_failed());
    arm_encoder[0]-=(int64_t)1401*65536u/3200u;
    advance_arm(350);CHECK(!arm_busy() && !arm_failed() && !arm_stops);
    reset_fixture();declare_pose();arm_speed_bias[0]=-15;arm_speed_bias[1]=15;arm_flags=1;
    send_line("arm lift 100");advance_arm(700);CHECK(arm_busy() && !arm_failed());
    arm_flags=3;advance_arm(350);CHECK(!arm_busy() && !arm_failed() && !arm_stops);
    puts("PASS arm axis speed log-only: both5/7 signed1/15/3000RPM pass preflight, position completion and servo steps; both nonzero axes still wait for actual target position and arrived flag");
}
static void parallel_platform_yaw_tests(void)
{
    uint32_t start,serial_ms;uint16_t yaw_start,platform_start,claw_start;unsigned positions;
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 125 45");
    start=clock_ms;CHECK(arm_platform_at_yaw(2,125,clock_ms));
    while(arm_busy() && clock_ms-start<2u*ARM_SERVO_TIME_MS)advance_arm(1);
    send_line("arm yaw 256");
    while(arm_busy() && clock_ms-start<3u*ARM_SERVO_TIME_MS)advance_arm(1);
    CHECK(arm_ready());serial_ms=clock_ms-start;
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 125 45");
    yaw_start=arm_yaw_us;platform_start=arm_platform_us;claw_start=arm_claw_us;positions=arm_positions;
    CHECK(!arm_platform_with_yaw(0,256,clock_ms) && !arm_platform_with_yaw(4,256,clock_ms));
    CHECK(!arm_platform_with_yaw(2,361,clock_ms));
    start=clock_ms;CHECK(arm_platform_with_yaw(2,256,clock_ms));
    advance_arm(ARM_SERVO_TIME_MS/2u+100u);
    CHECK(arm_busy() && arm_yaw_us!=yaw_start && arm_platform_us!=platform_start);
    CHECK(arm_claw_us==claw_start && arm_positions==positions);
    CHECK(!arm_platform_with_yaw(3,125,clock_ms));
    while(arm_busy() && clock_ms-start<2u*ARM_SERVO_TIME_MS)advance_arm(1);
    CHECK(arm_ready() && arm_platform_slot()==2 && clock_ms-start<2u*ARM_SERVO_TIME_MS);
    {float yaw;arm_pose_now(NULL,NULL,&yaw,NULL);CHECK(yaw==256.0f);}
    CHECK(clock_ms-start<serial_ms);
    printf("PARALLEL_ROTATION serial_ms=%lu parallel_ms=%lu servo_ms=%u; both PWM channels moved before completion\n",(unsigned long)serial_ms,(unsigned long)(clock_ms-start),(unsigned)ARM_SERVO_TIME_MS);
    CHECK(arm_platform_with_yaw(3,125,clock_ms));advance_arm(ARM_SERVO_TIME_MS/2u+100u);
    yaw_start=arm_yaw_us;platform_start=arm_platform_us;stop_request();advance_arm(2000);
    CHECK(arm_failed() && arm_yaw_us==yaw_start && arm_platform_us==platform_start);
    reset_fixture();declare_pose();send_line("arm platform pose 20");
    send_line("arm pose 1 135 125 45");CHECK(!arm_platform_with_yaw(2,256,clock_ms));
    send_line("arm pose 0 134 125 45");CHECK(!arm_platform_with_yaw(2,256,clock_ms));
    puts("PASS simultaneous platform/yaw: shared interpolation, no extension/lowering before completion, target cup/yaw committed together, explicit emergency holds both PWM channels");
}

int main(void)
{
    manual_and_recipe_tests();station_platform_tests();parallel_platform_yaw_tests();mutual_exclusion_tests();wheel_lock_failures();
    arm_faults_and_stop_tests();bounds_and_shared_service_tests();english_console_tests();speed_and_feedback_gates_tests();axis_speed_log_only_tests();
    printf("PASS arm integration: %u assertions; host simulation only, no hardware accessed\n",assertions);
    return 0;
}
