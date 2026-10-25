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
    send_line("arm yaw 126.5");advance_arm(2400);CHECK(arm_yaw_us==1272);expect_pose(0,135,126.5f,45);
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
    CHECK(host_tx_cancels[0] && host_tx_cancels[5] && host_tx_cancels[7]);
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
        if(mode)bus_off=1;else motor_flags[3]=11;
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
        } else if(mode==6) {
            CHECK(!arm_failed() && !arm_busy() && arm_pose_known() && !arm_stops);
            CHECK(strstr(output,"flags_dec=7") && strstr(output,"; logged only"));
        } else CHECK(arm_stops>=2 && !strstr(output,"[arm done") && !position_count && !sync_count);
    }
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm platform 3");advance_arm(ARM_SERVO_TIME_MS/2u);
    {uint16_t held=arm_platform_us;stop_request();advance_arm(3000);CHECK(arm_platform_us==held);}
    reset_fixture();declare_pose();send_line("arm yawtest 0");advance_arm(8400);CHECK(arm_yaw_us==2500 && arm_claw_us==826 && !arm_positions);
    send_line("arm yawtest 360");advance_arm(8400);CHECK(arm_yaw_us==500);
    puts("PASS arm monitoring: feedback/ACK/action deadlines retain pose and recover on actual position; axis flags logged only; explicit stop and failed CAN submission still stop");
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
    send_line("arm yaw 256");advance_arm(ARM_YAW_TIME_MS/2u);CHECK(arm_busy() && arm_yaw_us>553);
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

    printf("PASS arm speed: axes use fresh verified feedback; claw_ms=%u yaw_ms=%u platform_ms=%u close_hold_ms=%u; position/status must arrive; missing ACK or RPM magnitude alone cannot cancel or gate measured completion\n",
        ARM_CLAW_TIME_MS,ARM_YAW_TIME_MS,ARM_SERVO_TIME_MS,ARM_CLAW_CLOSE_HOLD_MS);
}
static void feedback_reuse_tests(void)
{
    unsigned variant;
    uint32_t saved=0;
    for(variant=0;variant<3;variant++) {
        uint32_t start,deadline;unsigned before;
        reset_fixture();declare_pose();send_line("arm slide 55");
        deadline=clock_ms+1000;
        while(arm_busy() && clock_ms<deadline)advance_arm(1);
        CHECK(!arm_busy() && arm_positions==1);
        if(variant==1)advance_arm(ARM_FEEDBACK_TIMEOUT_MS+1u);
        if(variant==2)send_line("arm pose 55 135 0 45");
        before=arm_positions;start=clock_ms;arm_freeze=1;
        send_line("arm lift 100");deadline=clock_ms+1000;
        while(arm_positions==before && clock_ms<deadline)advance_arm(1);
        CHECK(arm_positions==before+1 && arm_busy());
        if(!variant) {
            saved=arm_trace_ms[before]-start;
            CHECK(saved<40u && strstr(output,"reuse fresh completed-axis feedback"));
        } else CHECK(arm_trace_ms[before]-start>=50u);
        advance_arm(600);CHECK(arm_busy() && !arm_failed()); /* 旧到位数据不能完成真实新目标。 */
        arm_encoder[1]+=(int64_t)2800*65536u/3200u;
        advance_arm(100);CHECK(!arm_busy() && !arm_failed());
        if(!variant) {
            before=arm_positions;start=clock_ms;
            send_line("arm open");while(arm_busy() && clock_ms-start<1000)advance_arm(1);
            CHECK(!arm_busy() && arm_positions==before && clock_ms-start<ARM_CLAW_TIME_MS);
        }
    }
    printf("PASS feedback reuse: warm next-axis submit within%lums; stale/manual redeclaration reacquire; real changed-axis target feedback still required; unchanged servo target skips interpolation\n",(unsigned long)saved);
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
/* 5/7逐轴注入未使能、堵转、保护；不清flags，不虚构到位。 */
static void axis_status_log_only_tests(void)
{
    static const uint8_t flags[]={0,1,2,4,6,7,8,10,11,12,14,15};
    unsigned axis,i;
    for(axis=0;axis<2;axis++)for(i=0;i<sizeof(flags)/sizeof(flags[0]);i++) {
        char expected[128];unsigned before,seen;
        reset_fixture();declare_pose();arm_status_override[axis]=flags[i];
        send_line(axis ? "arm lift 100" : "arm slide 55");advance_arm(700);
        CHECK(!arm_failed() && arm_pose_known() && !arm_stops && arm_positions==1);
        if(flags[i]&2u)CHECK(!arm_busy());else CHECK(arm_busy());
        if(flags[i]!=1u) { /* enabled=1/arrived=0本来就不是本次异常条件。 */
            snprintf(expected,sizeof(expected),"[arm axis warning] addr=%u flags_dec=%u enabled=%u arrived=%u stalled=%u protected=%u",
                     axis ? 5u : 7u,(unsigned)flags[i],(unsigned)((flags[i]&1u)!=0),
                     (unsigned)((flags[i]&2u)!=0),(unsigned)((flags[i]&4u)!=0),(unsigned)((flags[i]&8u)!=0));
            CHECK(strstr(output,expected) && strstr(output,"target_slide_mm=") && strstr(output,"target_lift_mm=") && strstr(output,"; logged only"));
        }
        before=output_size;advance_arm(1500);CHECK(output_size==before); /* 相同异常不刷屏。 */
        arm_status_override[axis]=3;advance_arm(400);CHECK(!arm_busy() && !arm_failed() && !arm_stops);
        send_line("arm yaw 125");advance_arm(1);CHECK(arm_busy());
        arm_status_override[axis]=7;before=output_size;advance_arm(6u*ARM_POLL_MS+1u);CHECK(output_size>before); /* 两轴各三类反馈，一整轮后核对。 */
        seen=output_size;arm_status_override[axis]=11;advance_arm(6u*ARM_POLL_MS+1u);CHECK(output_size>seen); /* 标志变化同样记录。 */
        stop_request();CHECK(arm_stops>=2 && arm_failed());
    }
    reset_fixture();declare_pose();arm_status_override[0]=7;arm_freeze=1;
    send_line("arm slide 55");advance_arm(1000);CHECK(arm_busy() && !arm_failed() && !arm_stops);
    arm_freeze=0;arm_encoder[0]=-(int64_t)1401*65536u/3200u;
    advance_arm(400);CHECK(!arm_busy() && !arm_failed() && !arm_stops);
    reset_fixture();declare_pose();clock_ms=UINT32_MAX-100u;arm_status_override[1]=11;
    send_line("arm lift 100");advance_arm(700);CHECK(!arm_busy() && !arm_failed() && !arm_stops);
    puts("PASS arm axis status log-only: both5/7 disabled/stalled/protected combinations retain raw flags, per-axis/change/recovery diagnostics without repeated spam; actual arrived/position still required, explicit stop retained, timer wrap");
}
static void clearance_link_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        uint16_t yaw_start,platform_start,claw_start;
        reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 35 100 256 15");
        if(variant==1)clock_ms=UINT32_MAX-200u;
        yaw_start=arm_yaw_us;platform_start=arm_platform_us;claw_start=arm_claw_us;
        arm_freeze=1;CHECK(arm_retract_with_yaw(126.5f,2,clock_ms));advance_arm(200);
        CHECK(arm_positions==1 && arm_trace_address[0]==5 && arm_trace_pulses[0]==2800);
        CHECK(arm_yaw_us==yaw_start && arm_platform_us==platform_start && arm_claw_us==claw_start);
        arm_flags=1;
        arm_encoder[1]=-(int64_t)((ARM_CLEARANCE_MM-100-0.5f)*ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        advance_arm(200);CHECK(arm_positions==1 && arm_yaw_us==yaw_start && arm_platform_us==platform_start);
        if(variant==2) {
            stop_request();advance_arm(500);CHECK(arm_failed() && arm_positions==1 && arm_yaw_us==yaw_start);continue;
        }
        arm_encoder[1]=-(int64_t)((ARM_CLEARANCE_MM-100)*ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        advance_arm(60);CHECK(arm_positions==2 && arm_trace_address[1]==7 && arm_busy());
        advance_arm(100);CHECK(arm_yaw_us!=yaw_start && arm_platform_us!=platform_start && arm_claw_us==claw_start);
        if(variant==3) {
            yaw_start=arm_yaw_us;platform_start=arm_platform_us;stop_request();advance_arm(500);
            CHECK(arm_failed() && arm_positions==2 && arm_yaw_us==yaw_start && arm_platform_us==platform_start);continue;
        }
        advance_arm(500);CHECK(arm_busy() && arm_positions==2); /* 舵机已完成，升降仍在115，不能假报135。 */
        arm_encoder[1]=-(int64_t)arm_trace_pulses[0]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        arm_encoder[0]=(int64_t)arm_trace_pulses[1]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        arm_flags=3;advance_arm(250);
        CHECK(arm_ready() && arm_positions==2 && arm_platform_slot()==2);
        expect_pose(0,135,126.5f,15);
    }
    {
        char pose[64];
        reset_fixture();declare_pose();send_line("arm platform pose 20");
        snprintf(pose,sizeof(pose),"arm pose 35 %d 256 15",ARM_CLEARANCE_MM);send_line(pose);
        CHECK(arm_retract_with_yaw(126.5f,2,clock_ms));advance_arm(100);
        CHECK(arm_positions==(ARM_CLEARANCE_MM==ARM_LIFT_HOME_MM ? 1u : 2u) && arm_trace_address[0]==7);
    }
    puts("PASS clearance linking: one continuous lift command to135; actual clearance-minus0.5 holds horizontal/servo, actual clearance releases them while lift remains in transit; real final position still required; both PWM channels, held claw, initial clear pose, timer wrap, emergency before/after release");
}
static void yaw_settle_tests(void)
{
    unsigned variant;
    for(variant=0;variant<3;variant++) {
        uint32_t start,written;
        uint16_t prior;
        reset_fixture();declare_pose();send_line("arm platform pose 20");
        if(variant==1)send_line("arm pose 0 135 256 45");
        if(variant==2)clock_ms=UINT32_MAX-180u;
        start=clock_ms;prior=arm_yaw_us;
        send_line(variant==1 ? "arm yaw 0" : "arm yaw 256");
        while(arm_yaw_us!=(variant==1 ? 1975 : 553) && (uint32_t)(clock_ms-start)<1000u) {
            uint16_t before=arm_yaw_us;advance_arm(1);
            CHECK(variant==1 ? arm_yaw_us>=before : arm_yaw_us<=before);
        }
        written=clock_ms;
        CHECK(arm_yaw_us!=prior && arm_busy());
        advance_arm(ARM_YAW_SETTLE_MS-1u);CHECK(arm_busy());
        while(arm_busy() && (uint32_t)(clock_ms-start)<1000u)advance_arm(1);
        CHECK(arm_ready() && !arm_failed() && (uint32_t)(clock_ms-written)>=ARM_YAW_SETTLE_MS);
        CHECK((uint32_t)(clock_ms-start)>=ARM_YAW_TIME_MS+ARM_YAW_SETTLE_MS);
        start=clock_ms;send_line(variant==1 ? "arm yaw 0" : "arm yaw 256");
        while(arm_busy() && (uint32_t)(clock_ms-start)<1000u)advance_arm(1);
        CHECK(arm_ready() && (uint32_t)(clock_ms-start)<ARM_YAW_SETTLE_MS);
    }
    printf("PASS yaw settle: smooth monotonic180ms commands, final target PWM then hold%ums, both directions/time wrap, unchanged target no extra hold; no physical position feedback\n",ARM_YAW_SETTLE_MS);
}
static void independent_servo_and_grip_hold_tests(void)
{
    unsigned variant;
    uint32_t start,written,deadline;
    uint16_t yaw_start,platform_start;
    for(variant=0;variant<2;variant++) {
        reset_fixture();declare_pose();send_line("arm platform pose 20");
        if(variant)clock_ms=UINT32_MAX-180u;
        start=clock_ms;send_line("arm close");
        while(arm_claw_us!=604 && (uint32_t)(clock_ms-start)<1000u)advance_arm(1);
        written=clock_ms;
        CHECK(arm_claw_us==604 && arm_busy());
        advance_arm(ARM_CLAW_CLOSE_HOLD_MS-1u);CHECK(arm_busy());
        while(arm_busy() && (uint32_t)(clock_ms-written)<1000u)advance_arm(1);
        CHECK(arm_ready() && !arm_failed() && (uint32_t)(clock_ms-written)>=ARM_CLAW_CLOSE_HOLD_MS);
        start=clock_ms;send_line("arm close");
        while(arm_busy() && (uint32_t)(clock_ms-start)<1000u)advance_arm(1);
        CHECK(arm_ready() && (uint32_t)(clock_ms-start)<ARM_CLAW_CLOSE_HOLD_MS);
        start=clock_ms;send_line("arm open");
        while(arm_busy() && (uint32_t)(clock_ms-start)<1000u)advance_arm(1);
        CHECK(arm_ready() && arm_claw_us==826 && (uint32_t)(clock_ms-start)<ARM_CLAW_TIME_MS+ARM_CLAW_CLOSE_HOLD_MS);
    }
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 125 15");
    yaw_start=arm_yaw_us;platform_start=arm_platform_us;start=clock_ms;
    CHECK(arm_high_approach(35,ARM_CLEARANCE_MM,256,2,1,clock_ms));advance_arm(150);
    CHECK(arm_busy() && arm_claw_us==826 && arm_yaw_us!=yaw_start && arm_yaw_us!=553 && arm_platform_us!=platform_start);
    while(arm_yaw_us!=553 && clock_ms-start<1000u)advance_arm(1);
    CHECK(arm_yaw_us==553 && arm_busy() && clock_ms-start<ARM_SERVO_TIME_MS);
    while(arm_busy() && clock_ms-start<1000u)advance_arm(1);
    CHECK(arm_ready() && arm_platform_slot()==2 && clock_ms-start>=ARM_SERVO_TIME_MS);

    /* 真实load1顺序：合爪最终PWM写入后满保持时间，才提交抬升指令。 */
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 131 45");
    send_line("arm load1");deadline=clock_ms+5000u;written=0;
    while(clock_ms<deadline) {
        advance_arm(1);if(!written && arm_claw_us==604)written=clock_ms;
        if(arm_positions && arm_trace_address[arm_positions-1u]==5 && arm_trace_dir[arm_positions-1u]==1)break;
    }
    CHECK(written && clock_ms<deadline && arm_trace_address[arm_positions-1u]==5 && arm_trace_dir[arm_positions-1u]==1);
    CHECK(arm_trace_ms[arm_positions-1u]-written>=ARM_CLAW_CLOSE_HOLD_MS);
    printf("PASS independent servo/grip hold: yaw%ums, platform%ums, claw%ums; close target PWM then hold%ums before actual load1 lift; open/unchanged close no hold, timer wrap covered\n",
           ARM_YAW_TIME_MS,ARM_SERVO_TIME_MS,ARM_CLAW_TIME_MS,ARM_CLAW_CLOSE_HOLD_MS);
}
static void parallel_high_approach_tests(void)
{
    uint32_t start,serial_ms,parallel_ms,deadline;
    uint16_t yaw_start,platform_start,claw_start;
    unsigned positions;
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 125 15");
    start=clock_ms;CHECK(arm_platform_with_yaw(2,256,clock_ms));
    while(arm_busy() && clock_ms-start<3000)advance_arm(1);
    send_line("arm slide 35");while(arm_busy() && clock_ms-start<3000)advance_arm(1);
    send_line("arm open");while(arm_busy() && clock_ms-start<3000)advance_arm(1);
#if ARM_LIFT_PARALLEL_ENABLED
    send_line("arm lift " CAR_STRING_VALUE(ARM_CLEARANCE_MM));while(arm_busy() && clock_ms-start<3000)advance_arm(1);
#endif
    CHECK(arm_ready());serial_ms=clock_ms-start;

    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 125 15");
    yaw_start=arm_yaw_us;platform_start=arm_platform_us;claw_start=arm_claw_us;
    CHECK(!arm_high_approach(35,ARM_CLEARANCE_MM,256,4,1,clock_ms));
    CHECK(!arm_high_approach(ARM_SLIDE_MAX_MM+1,ARM_CLEARANCE_MM,256,2,1,clock_ms));
    start=clock_ms;CHECK(arm_high_approach(35,ARM_CLEARANCE_MM,256,2,1,clock_ms));
    advance_arm(ARM_SERVO_TIME_MS/2u+100u);
    CHECK(arm_busy() && arm_positions==(ARM_LIFT_PARALLEL_ENABLED ? 2u : 1u) && arm_trace_address[0]==7);
    CHECK(arm_trace_ms[0]-start<200u);
    CHECK(arm_yaw_us!=yaw_start && arm_platform_us!=platform_start && arm_claw_us!=claw_start);
    CHECK(!arm_high_approach(55,ARM_CLEARANCE_MM,125,0,0,clock_ms));
    while(arm_busy() && clock_ms-start<3000)advance_arm(1);
    parallel_ms=clock_ms-start;
    CHECK(arm_ready() && arm_platform_slot()==2 && parallel_ms<serial_ms);
    expect_pose(35,ARM_LIFT_PARALLEL_ENABLED ? ARM_CLEARANCE_MM : ARM_LIFT_HOME_MM,256,45);
    printf("HIGH_APPROACH_TIMING serial_ms=%lu parallel_ms=%lu; immediate stepper replies, not physical timing\n",
           (unsigned long)serial_ms,(unsigned long)parallel_ms);

    /* 原料直接降到105；平推/升降未实际到位时，即使插值结束也不能合爪。 */
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 131 45");
    arm_freeze=1;send_line("arm load1");start=clock_ms;
    advance_arm(ARM_SERVO_TIME_MS+500u);
    CHECK(arm_busy() && !arm_failed() && arm_positions==(ARM_LIFT_PARALLEL_ENABLED ? 2u : 1u) && arm_trace_address[0]==7);
    if(ARM_LIFT_PARALLEL_ENABLED)CHECK(arm_trace_address[1]==5 && arm_trace_pulses[1]==(ARM_LIFT_HOME_MM-ARM_PICK_LIFT_MM)*ARM_LIFT_PULSES_PER_MM);
    CHECK(arm_claw_us==826);
    arm_encoder[0]-=(int64_t)arm_trace_pulses[0]*65536u/3200u;
    if(ARM_LIFT_PARALLEL_ENABLED)arm_encoder[1]+=(int64_t)arm_trace_pulses[1]*65536u/3200u;
    arm_freeze=0;positions=arm_positions;
    deadline=clock_ms+1000;
    while(arm_positions==positions && clock_ms<deadline)advance_arm(1);
    CHECK(arm_positions==positions+1u && arm_trace_ms[positions]-start>=ARM_SERVO_TIME_MS+500u);
#if ARM_DIRECT_TRANSFER_ENABLED && ARM_LIFT_PARALLEL_ENABLED
    if(ARM_CLEARANCE_MM==ARM_PICK_LIFT_MM) {
        CHECK(arm_trace_address[positions]==7 && arm_trace_pulses[positions]==(uint32_t)((ARM_PICK_SLIDE_MM-ARM_PLACE_SLIDE_MM)*ARM_SLIDE_PULSES_PER_MM+0.5f));
    } else CHECK(arm_trace_address[positions]==5);
#else
    CHECK(arm_trace_address[positions]==5);
#endif

    /* 带物块的高位转向保留夹爪；同一步急停也同时取消轴和PWM。 */
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 256 15");
    claw_start=arm_claw_us;platform_start=arm_platform_us;
    CHECK(arm_high_approach(55,5,126.5f,0,0,clock_ms));advance_arm(ARM_YAW_TIME_MS/2u+100u);
    CHECK(arm_claw_us==claw_start && arm_platform_us==platform_start && arm_busy());
    yaw_start=arm_yaw_us;positions=arm_positions;stop_request();advance_arm(1500);
    CHECK(arm_failed() && !arm_pose_known() && arm_positions==positions && arm_yaw_us==yaw_start);
    reset_fixture();declare_pose();send_line("arm platform pose 20");
    send_line("arm pose 1 135 125 45");CHECK(!arm_high_approach(35,ARM_CLEARANCE_MM,256,2,1,clock_ms));
    send_line("arm pose 0 100 125 45");CHECK(!arm_high_approach(35,ARM_CLEARANCE_MM,256,2,1,clock_ms));
    puts("PASS high approach: raw direct105 and right-side direct5 overlap slide/yaw; platform retains clearance approach; actual axes must complete before grip/release; held claw and manual recipes retained; explicit emergency stops all");
}
static void direct_side_descent_tests(void)
{
    static const int targets[]={5,65,105};
    unsigned i;
    for(i=0;i<sizeof(targets)/sizeof(targets[0]);i++) {
        uint16_t claw,yaw;
        int slide,lift;uint32_t start;
        reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 0 135 256 15");
        arm_freeze=1;claw=arm_claw_us;yaw=arm_yaw_us;start=clock_ms;
        CHECK(arm_high_approach(55,targets[i],126.5f,0,0,clock_ms));
        advance_arm(500);
        CHECK(arm_busy() && !arm_failed() && arm_claw_us==claw && arm_yaw_us!=yaw);
        CHECK(arm_positions==2 && arm_trace_address[0]==7 && arm_trace_address[1]==5);
        CHECK(arm_trace_ms[1]-start<200u && arm_trace_dir[1]==0);
        CHECK(arm_trace_pulses[1]==(ARM_LIFT_HOME_MM-targets[i])*ARM_LIFT_PULSES_PER_MM);
        arm_target_now(&slide,&lift,NULL,NULL);CHECK(slide==55 && lift==targets[i]);
        /* 只提交最终高度一条下降命令；轴未到位时不把舵机结束当动作完成。 */
        arm_encoder[0]=-(int64_t)arm_trace_pulses[0]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        arm_encoder[1]=(int64_t)arm_trace_pulses[1]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        arm_freeze=0;advance_arm(200);
        CHECK(arm_ready() && arm_positions==2);expect_pose(55,targets[i],126.5f,15);
    }
    puts("PASS right-side direct descent: lift135 goes directly to5/65/105 with slide/yaw, one full-distance lift command, no clearance waypoint; closed claw held and actual axis completion required");
}
static void direct_transfer_tests(void)
{
    unsigned direction,i;uint16_t claw,yaw;
    for(direction=0;direction<3;direction++) {
        char pose[64];int slide,lift;
        int source_lift=direction==0 ? 100 : direction==1 ? 5 : ARM_PICK_LIFT_MM;
        int target_lift=direction ? ARM_PLACE_LIFT_MM : 5;
        unsigned before;uint32_t top_injected;
        reset_fixture();declare_pose();send_line("arm platform pose 20");
        snprintf(pose,sizeof(pose),"arm pose %d %d %.1f 15",direction ? 55 : 35,source_lift,direction==0 ? 256.0 : direction==1 ? 126.5 : ARM_PICK_YAW_DEG);send_line(pose);
        claw=arm_claw_us;yaw=arm_yaw_us;arm_freeze=1;
        CHECK(arm_transfer_to(direction ? 35 : 55,target_lift,direction ? 256 : 126.5f,2,clock_ms));
        advance_arm(90);
        CHECK(arm_positions==1 && arm_trace_address[0]==5 && arm_trace_dir[0]==ARM_LIFT_DIR_INVERT);
        CHECK(arm_trace_pulses[0]==(ARM_LIFT_HOME_MM-source_lift)*ARM_LIFT_PULSES_PER_MM && arm_yaw_us==yaw);
        /* 三种来源均提交一次连续升顶；CPOS实际109不能提前转，也不能开始下降计时。 */
        arm_encoder[1]=-(int64_t)((109-source_lift)*ARM_LIFT_PULSES_PER_MM)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        advance_arm(ARM_CUP_EXIT_DESCENT_DELAY_MS+100u);
        CHECK(arm_positions==1 && arm_yaw_us==yaw && arm_platform_slot()==1 && arm_busy());
        arm_encoder[1]=-(int64_t)((110-source_lift)*ARM_LIFT_PULSES_PER_MM)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        advance_arm(100);
        CHECK(arm_positions==2 && arm_trace_address[1]==7 && arm_yaw_us!=yaw && arm_busy());
        CHECK(arm_trace_pulses[1]==(uint32_t)(20*ARM_SLIDE_PULSES_PER_MM+0.5f));
        arm_target_now(&slide,&lift,NULL,NULL);CHECK(slide==(direction ? 35 : 55) && lift==135);
        advance_arm(ARM_CUP_EXIT_DESCENT_DELAY_MS+200u);
        CHECK(arm_positions==2 && arm_busy());
        arm_encoder[1]=-(int64_t)((ARM_LIFT_HOME_MM-source_lift)*ARM_LIFT_PULSES_PER_MM)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        top_injected=clock_ms;before=arm_positions;
#if ARM_CUP_EXIT_DESCENT_DELAY_MS > 0
        advance_arm(ARM_CUP_EXIT_DESCENT_DELAY_MS-1u);CHECK(arm_positions==before);
#endif
        while(arm_positions==before && clock_ms-top_injected<ARM_CUP_EXIT_DESCENT_DELAY_MS+500u)advance_arm(1);
        CHECK(arm_positions==3 && arm_trace_address[2]==5 && arm_trace_dir[2]!=ARM_LIFT_DIR_INVERT);
        CHECK(arm_trace_pulses[2]==(ARM_LIFT_HOME_MM-target_lift)*ARM_LIFT_PULSES_PER_MM);
        CHECK(arm_trace_ms[2]-top_injected+1u>ARM_CUP_EXIT_DESCENT_DELAY_MS);
        CHECK(arm_claw_us==claw);
        arm_target_now(&slide,&lift,NULL,NULL);CHECK(slide==(direction ? 35 : 55) && lift==target_lift);
        CHECK(!arm_translation_overlap_ready() && arm_busy());
        arm_encoder[0]=arm_encoder[1]=0;
        for(i=0;i<arm_positions;i++)arm_encoder[arm_trace_address[i]==5]+=(arm_trace_dir[i] ? -1 : 1)*(int64_t)arm_trace_pulses[i]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        arm_freeze=0;advance_arm(500);
        CHECK(arm_ready() && arm_positions==3 && arm_claw_us==claw && arm_platform_slot()==2);
        expect_pose(direction ? 35 : 55,target_lift,direction ? 256 : 126.5f,15);
    }
    /* 抬升、转向中升降未到顶、已到顶等待期间，急停都撤销剩余下降。 */
    for(i=0;i<3;i++) {
        unsigned count;
        reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 35 100 256 15");arm_freeze=1;
        CHECK(arm_transfer_to(55,5,126.5f,2,clock_ms));advance_arm(90);
        if(i) {
            arm_encoder[1]=-(int64_t)((i==2 ? 35 : 10)*ARM_LIFT_PULSES_PER_MM)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
            advance_arm(80);
        }
        stop_request();count=arm_positions;advance_arm(ARM_CUP_EXIT_DESCENT_DELAY_MS+500u);
        CHECK(arm_failed() && arm_positions==count);
    }
    /* 未实际收拢时不能启动底盘；实际两轴完成后，剩余旋转可以与平移重叠。 */
    reset_fixture();declare_pose();send_line("arm platform pose 20");send_line("arm pose 35 100 256 45");arm_freeze=1;
    CHECK(arm_retract_with_yaw(126.5f,2,clock_ms));advance_arm(80);
    CHECK(!arm_translation_overlap_ready() && arm_positions==1);
    arm_encoder[1]=-(int64_t)arm_trace_pulses[0]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
    advance_arm(40);CHECK(arm_positions==2 && !arm_translation_overlap_ready());
    arm_encoder[0]=(int64_t)arm_trace_pulses[1]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
    advance_arm(80);CHECK(arm_busy() && arm_translation_overlap_ready());
    stop_request();advance_arm(500);CHECK(arm_failed() && !arm_translation_overlap_ready());
    puts("PASS direct transfers: raw105/ground5/platform100 each continuous lift135; actual109 blocks yaw,110 releases yaw/slide while still lifting, no descent until measured135 plus configured delay; all original destinations retained; claw held, no slide0, real feedback and emergency at rise/turn/top-wait");
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
    manual_and_recipe_tests();station_platform_tests();parallel_platform_yaw_tests();parallel_high_approach_tests();direct_side_descent_tests();direct_transfer_tests();independent_servo_and_grip_hold_tests();clearance_link_tests();yaw_settle_tests();mutual_exclusion_tests();wheel_lock_failures();
    arm_faults_and_stop_tests();bounds_and_shared_service_tests();english_console_tests();speed_and_feedback_gates_tests();feedback_reuse_tests();axis_speed_log_only_tests();axis_status_log_only_tests();
    printf("PASS arm integration: %u assertions; host simulation only, no hardware accessed\n",assertions);
    return 0;
}
