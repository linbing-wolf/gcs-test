/* 复用真实固件入口与独立四轮惯性模型；不把编码器模型当作实车侧滑验证。 */
#define HEADING_TEST_MAIN retained_heading_suite
#include "verify_heading.c"

static void start_straight(int direction,int distance)
{
    /* 仅借用原入口的自检，替换本次几何段；不使用工位或MOVEXY执行器。 */
    send_line("runimu A");
    CHECK(run_active);
    run.translation_only=1;run.station_enabled=0;
    run.travel_axis=BODY_FORWARD;run.travel_sign=(int8_t)direction;
    run.remaining_mm=(uint16_t)distance;run.continuation_mm=0;
    run.target_heading=HEAD_Y_POS;run.heading=HEAD_Y_POS;
}

static void unequal_wheel_tests(void)
{
    unsigned direction,side,mask,i;
    double worst=0;
    for(direction=0;direction<2;direction++)for(side=0;side<2;side++)for(mask=0;mask<16;mask++) {
        uint32_t deadline;
        int corrected=0;
        double peak=0;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        for(i=0;i<4;i++) {
            encoder_polarity[i]=(mask&(1u<<i)) ? -1 : 1;
            /* 两组轮速偏差相反，产生平移侧偏而没有转向力矩。 */
            wheel_speed_scale[i]=(i==0 || i==2) ? (side ? 1.04 : 0.96) : (side ? 0.96 : 1.04);
        }
        start_straight(direction ? -1 : 1,1600);
        deadline=clock_ms+30000;
        while(run_active && (uint32_t)(deadline-clock_ms)<0x80000000u) {
            plant_step();
            if(fabs(field_x)>peak)peak=fabs(field_x);
            if(run.straight_hold.command_rpm) {
                corrected=1;
                CHECK(abs(run.straight_hold.command_rpm)<=CAR_STRAIGHT_HOLD_MAX_RPM);
            }
            for(i=0;i<4;i++)CHECK(abs(run.wheel_rpm[i])<=CAR_WHEEL_MAX_RPM);
        }
        if(run.state!=RUN_DONE)printf("STRAIGHT failed dir=%u side=%u mask=%u state=%d progress=%ld cross=%ld output=%s\n",direction,side,mask,run.state,(long)run.progress_mm,(long)run.straight_hold.lateral_mm,output);
        CHECK(run.state==RUN_DONE && !run_active && !position_count);
        CHECK(fabs(physical_yaw)<0.1); /* IMU看不出此处的侧偏。 */
        CHECK(run.progress_mm>=1599 && run.progress_mm<=1610);
        if(CAR_STRAIGHT_HOLD_ENABLED) {
            CHECK(corrected && fabs(field_x)<12 && peak<25);
        } else CHECK(!corrected && fabs(field_x)>50);
        if(fabs(field_x)>worst)worst=fabs(field_x);
    }
    printf("PASS straight unequal-wheel plant: enabled=%u, 64 forward/backward/left-right bias/encoder-polarity cases; worst terminal lateral displacement=%.2fmm; simulated only\n",(unsigned)CAR_STRAIGHT_HOLD_ENABLED,worst);
}

static void sample_alignment_tests(void)
{
    unsigned i,step;
    reset_plant(0,1.15);run.travel_axis=BODY_FORWARD;run.travel_sign=1;run.encoder_ready=15;
    /* 相同加速度、不同采样时刻；真实横向位移为0。 */
    clock_ms=UINT32_MAX-1000u;
    run.straight_hold.sample_ms=clock_ms;
    for(i=0;i<4;i++) {
        run.encoder_start[i]=0;
        run.encoder_sign[i]=1;
        run.straight_hold.latest[i]=0;run.straight_hold.latest_ms[i]=clock_ms;
    }
    for(step=1;step<250;step++) {
        unsigned wheel=(step-1u)%4u;
        int64_t counts=(int64_t)step*step*100;
        clock_ms+=15;
        for(i=0;i<4;i++) {
            motor_feedback[i].position=run.straight_hold.latest[i];
            motor_feedback[i].position_ms=run.straight_hold.latest_ms[i];
        }
        motor_feedback[wheel].position=counts;
        motor_feedback[wheel].position_ms=clock_ms;
        run_straight_observe();
        if(run.straight_hold.valid)CHECK(abs(run.straight_hold.lateral_mm)<=1);
    }
    if(CAR_STRAIGHT_HOLD_ENABLED)CHECK(run.straight_hold.valid);
    /* 不通过轮速读数推算位置；同一CPOS重复处理不能产生额外位移。 */
    if(CAR_STRAIGHT_HOLD_ENABLED) {
        int32_t before=run.straight_hold.lateral_mm;
        for(i=0;i<4;i++)motor_feedback[i].speed_rpm=300;
        run_straight_observe();CHECK(run.straight_hold.lateral_mm==before);
    }
    puts("PASS straight sampling: staggered accelerating CPOS, repeated samples and millisecond wrap; no speed-feedback extrapolation");
}

static void high_speed_and_unobserved_slip_tests(void)
{
    unsigned direction,i;
    for(direction=0;direction<2;direction++) {
        unsigned elapsed=0,peak=0;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        wheel_speed_scale[0]=wheel_speed_scale[2]=1.04;
        wheel_speed_scale[1]=wheel_speed_scale[3]=0.96;
        start_straight(direction ? -1 : 1,8000);
        while(run_active && elapsed++<60000) {
            unsigned speed;
            plant_step();
            speed=(unsigned)abs(run.forward_cmd_rpm);
            if(speed>peak)peak=speed;
            for(i=0;i<4;i++)CHECK(abs(run.wheel_rpm[i])<=CAR_WHEEL_MAX_RPM);
            if(CAR_STRAIGHT_HOLD_ENABLED && run.straight_hold.command_rpm) {
                CHECK(abs(run.forward_cmd_rpm)+abs(run.straight_hold.command_rpm)+abs(run.turn_cmd_rpm)<=CAR_WHEEL_MAX_RPM);
                if(run.forward_cmd_rpm) {
                    CHECK(run.wheel_rpm[0]*run.forward_cmd_rpm<0 && run.wheel_rpm[1]*run.forward_cmd_rpm>0);
                    CHECK(run.wheel_rpm[2]*run.forward_cmd_rpm>0 && run.wheel_rpm[3]*run.forward_cmd_rpm<0);
                }
            }
        }
        CHECK(run.state==RUN_DONE && peak>=280);
        if(CAR_STRAIGHT_HOLD_ENABLED)CHECK(fabs(field_x)<12);
        printf("HIGH_SPEED dir=%s peak_forward_command=%u terminal_lateral_mm=%.2f simulated\n",direction ? "BACKWARD" : "FORWARD",peak,field_x);
    }
    reset_plant(0,1.15);translation_yaw_disturbance=0;start_straight(1,1600);
    {
        unsigned elapsed=0,slipped=0;
        while(run_active && elapsed++<30000) {
            plant_step();
            if(!slipped && run.progress_mm>200) {field_x+=30;slipped=1;}
            CHECK(!run.straight_hold.command_rpm);
        }
        CHECK(run.state==RUN_DONE && slipped && fabs(field_x-30)<0.1);
    }
    puts("PASS high-speed headroom/forward-backward wheel signs; encoder-invisible physical slip remains unobserved as documented");
}

static void scope_and_failure_tests(void)
{
    unsigned test,i;
    for(test=0;test<4;test++) {
        reset_plant(0,1.15);run.travel_axis=BODY_FORWARD;run.travel_sign=1;
        run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;run.straight_hold.lateral_mm=20;
        if(test==0)run.corner.active=1;
        if(test==1)run.station_shift=1;
        if(test==2)run.vector_move=1;
        if(test==3)run.obstacle_phase=2;
        CHECK(!run_straight_hold_enabled());
        CHECK(!run_straight_correction(100,0,1));
    }
    reset_plant(0,1.15);run.travel_axis=BODY_FORWARD;run.travel_sign=1;
    run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;
    run.straight_hold.lateral_mm=100;run.control_ms=clock_ms-25;
    if(CAR_STRAIGHT_HOLD_ENABLED) {
        CHECK(run_straight_correction(100,0,1)<0);
        run.straight_hold.lateral_mm=-100;run.straight_hold.command_mrpm=0;
        CHECK(run_straight_correction(-100,0,1)>0);
        CHECK(!run_straight_correction(0,0,1));
        run.straight_hold.sample_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;
        CHECK(!run_straight_correction(100,0,1));
    }
    for(test=0;test<3;test++) {
        unsigned elapsed=0;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        wheel_speed_scale[0]=wheel_speed_scale[2]=1.04;
        wheel_speed_scale[1]=wheel_speed_scale[3]=0.96;
        start_straight(1,1600);
        while(run_active && !run.straight_hold.command_rpm && elapsed++<3000)plant_step();
        if(CAR_STRAIGHT_HOLD_ENABLED)CHECK(run.straight_hold.command_rpm);
        if(test==0)stop_request();
        if(test==1)bus_off=1;
        if(test==2)missing_feedback_motor=4;
        for(i=0;i<600;i++)plant_step();
        if(test<2) {
            if(run_active)printf("SAFETY test=%u state=%d can_fault=%u output=%s\n",test,run.state,can_rx_fault,output);
            CHECK(!run_active && (run.state==RUN_STOPPED || run.state==RUN_FAULT));
        }
        else {
            CHECK(run_active && !run.straight_hold.command_rpm);
            for(i=0;i<4;i++)CHECK(!run.wheel_rpm[i]);
            missing_feedback_motor=0;
            finish_plant(30000);
        }
    }
    puts("PASS straight scope/safety: station/corner/MOVEXY/obstacle excluded; sign, bounds, primary stop, stale feedback, emergency stop, CAN fault and recovery");
}

#ifndef STRAIGHT_HOLD_TEST_MAIN
#define STRAIGHT_HOLD_TEST_MAIN main
#endif
int STRAIGHT_HOLD_TEST_MAIN(void)
{
    unequal_wheel_tests();sample_alignment_tests();high_speed_and_unobserved_slip_tests();scope_and_failure_tests();
    printf("PASS straight hold: %lu assertions; software simulation, no hardware accessed\n",(unsigned long)assertions);
    return 0;
}
