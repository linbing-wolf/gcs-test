/* 新横移纠偏与原前后纠偏共用真实入口及独立轮速模型。 */
#define STRAIGHT_HOLD_TEST_MAIN retained_forward_hold_suite
#include "verify_straight_hold.c"

static void start_lateral(unsigned right)
{
    send_line("test");send_line("dist 1600");send_line("rpm 200");send_line(right ? "r" : "l");
    CHECK(run_active && run.translation_only && run.travel_axis==BODY_LEFT);
}

static void unequal_lateral_tests(void)
{
    unsigned direction,side,mask,i;double worst=0,peak_worst=0;
    for(direction=0;direction<2;direction++)for(side=0;side<2;side++)for(mask=0;mask<16;mask++) {
        unsigned corrected=0;uint32_t deadline;double peak=0;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        for(i=0;i<4;i++) {
            encoder_polarity[i]=(mask&(1u<<i)) ? -1 : 1;
            wheel_speed_scale[i]=(i==0 || i==2) ? (side ? 1.04 : 0.96) : (side ? 0.96 : 1.04);
        }
        start_lateral(direction);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            plant_step();if(fabs(field_y)>peak)peak=fabs(field_y);
            if(run.straight_hold.command_rpm) {
                corrected=1;CHECK(run.forward_cmd_rpm==run.straight_hold.command_rpm);
                CHECK(abs(run.forward_cmd_rpm)<=CAR_STRAIGHT_HOLD_MAX_RPM);
                CHECK(abs(run.forward_cmd_rpm)+abs(run.left_cmd_rpm)+abs(run.turn_cmd_rpm)<=200);
            }
            if(!run.left_cmd_rpm && CAR_STRAIGHT_HOLD_ENABLED)CHECK(!run.forward_cmd_rpm);
            for(i=0;i<4;i++)CHECK(abs(run.wheel_rpm[i])<=200);
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && !position_count);
        CHECK(abs(run.progress_mm-1600)<=CAR_DISTANCE_TOLERANCE_MM && fabs(physical_yaw)<0.1);
        CHECK(fabs(field_x-(direction ? -1600 : 1600))<15);
        if(CAR_STRAIGHT_HOLD_ENABLED) {
            /* 只在横移过程中限幅纠偏，不在末尾补位；15mm为此模型的回归预算，非实车精度。 */
            CHECK(corrected && fabs(field_y)<15 && peak<25);
        }
        else CHECK(!corrected && fabs(field_y)>20);
        if(fabs(field_y)>worst)worst=fabs(field_y);
        if(peak>peak_worst)peak_worst=peak;
    }
    printf("LATERAL_UNEQUAL enabled=%u cases=64 max_terminal_forward_mm=%.3f max_peak_forward_mm=%.3f; independent plant only\n",
        (unsigned)CAR_STRAIGHT_HOLD_ENABLED,worst,peak_worst);
}

static void injected_displacement_tests(void)
{
    static const int8_t fs[4]={-1,1,1,-1};unsigned direction,side,visible,i;
    for(direction=0;direction<2;direction++)for(side=0;side<2;side++)for(visible=0;visible<2;visible++) {
        unsigned injected=0,corrected=0;uint32_t deadline;int sign=side ? -1 : 1;
        reset_plant(0,1.15);translation_yaw_disturbance=0;start_lateral(direction);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            if(!injected && run.progress_mm>400) {
                field_y+=sign*30;
                if(visible)for(i=0;i<4;i++) {
                    encoder_value[i]+=encoder_polarity[i]*fs[i]*sign*30.0*CALIBRATED_PPM_Y*65536.0/3200.0;
                    simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
                }
                injected=1;
            }
            plant_step();
            if(run.straight_hold.command_rpm) {
                corrected=1;
                if(visible && abs(run.straight_hold.forward_mm)>5)CHECK(run.forward_cmd_rpm*run.straight_hold.forward_mm<0);
            }
        }
        CHECK(injected && clock_ms<deadline && run.state==RUN_DONE);
        if(CAR_STRAIGHT_HOLD_ENABLED) {
            if(visible)CHECK(corrected && fabs(field_y)<12);
            else CHECK(!corrected && fabs(field_y-sign*30)<0.2);
        }
        printf("LATERAL_INJECT enabled=%u right=%u side=%d encoder_visible=%u terminal_forward_mm=%.3f corrected=%u\n",
            (unsigned)CAR_STRAIGHT_HOLD_ENABLED,direction,sign,visible,field_y,corrected);
    }
}

static void lateral_sampling_tests(void)
{
    static const int8_t ls[4]={-1,-1,1,1};unsigned direction,i,step;
    for(direction=0;direction<2;direction++) {
        reset_plant(0,1.15);run.travel_axis=BODY_LEFT;run.travel_sign=direction ? -1 : 1;run.encoder_ready=15;
        clock_ms=UINT32_MAX-1000u;run.straight_hold.sample_ms=clock_ms;
        for(i=0;i<4;i++) {
            run.encoder_start[i]=0;run.encoder_sign[i]=ls[i]*run.travel_sign;
            run.straight_hold.latest[i]=0;run.straight_hold.latest_ms[i]=clock_ms;
        }
        for(step=1;step<250;step++) {
            unsigned wheel=(step-1u)%4u;int64_t counts=(int64_t)step*step*100;
            clock_ms+=15;
            for(i=0;i<4;i++) {
                motor_feedback[i].position=run.straight_hold.latest[i];
                motor_feedback[i].position_ms=run.straight_hold.latest_ms[i];
            }
            /* 叠加纯旋转轮行程；共同时间逆混控仍不应误报前后平移。 */
            motor_feedback[wheel].position=counts*ls[wheel]*run.travel_sign+(int64_t)step*step*37;
            motor_feedback[wheel].position_ms=clock_ms;run_straight_observe();
            if(run.straight_hold.valid)CHECK(abs(run.straight_hold.forward_mm)<=1);
        }
        if(CAR_STRAIGHT_HOLD_ENABLED) {
            int32_t before=run.straight_hold.forward_mm;
            CHECK(run.straight_hold.valid);
            for(i=0;i<4;i++)motor_feedback[i].speed_rpm=999;
            run_straight_observe();CHECK(run.straight_hold.forward_mm==before);
        }
    }
    puts("PASS lateral sampling: common-time accelerating/rotating CPOS, left/right, timestamp wrap and no speed-readout extrapolation");
}

static void scope_and_continuity_tests(void)
{
    unsigned test;
    for(test=0;test<4;test++) {
        reset_plant(0,1.15);run.travel_axis=BODY_LEFT;run.travel_sign=1;
        run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;run.straight_hold.forward_mm=30;
        if(test==0)run.station_shift=1;
        if(test==1)run.vector_move=1;
        if(test==2)run.obstacle_phase=2;
        if(test==3)run.corner.active=1;
        CHECK(!run_straight_hold_enabled() && !run_straight_correction(100,0,1));
    }
    reset_plant(0,1.15);run.travel_axis=BODY_LEFT;run.travel_sign=1;
    run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;run.straight_hold.forward_mm=30;
    run.control_ms=clock_ms-25;
    if(CAR_STRAIGHT_HOLD_ENABLED) {
        CHECK(run_straight_correction(100,0,1)<0);
        run.straight_hold.forward_mm=-30;run.straight_hold.command_mrpm=0;
        CHECK(run_straight_correction(-100,0,1)>0);
        CHECK(!run_straight_correction(0,0,1));
        run.straight_hold.sample_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;
        CHECK(!run_straight_correction(100,0,1));
    }
    reset_plant(0,1.15);run.plan=0;run.job=2;run.edge=3;run.segment=0;run.station_enabled=1;
    run.travel_axis=BODY_LEFT;run.travel_sign=1;run.heading=run.target_heading=HEAD_X_NEG;
    CHECK(road_plan(NODE_F,NODE_E,&run.road));run.action_mm=810;run.progress_mm=819;run.continuation_mm=810;
    run.straight_hold.forward_mm=20;run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;
    CHECK(run_pass_straight() && run.edge==4 && run.progress_mm==9);
    CHECK(run.straight_hold.forward_mm==20 && run.straight_hold.valid);
    reset_plant(0,1.15);start_lateral(0);plant_step();stop_request();
    CHECK(!run_active && run.state==RUN_STOPPED);
    puts("PASS lateral scope: original station/MOVEXY/corner/obstacle paths retained; correction signs, zero-primary exit, stale estimate, F-E-D baseline continuity and manual !");
}

int main(void)
{
    unequal_wheel_tests();sample_alignment_tests();
    unequal_lateral_tests();injected_displacement_tests();lateral_sampling_tests();scope_and_continuity_tests();
    printf("PASS lateral forward-position hold: %u assertions; encoder-visible drift only; no hardware accessed\n",assertions);
    return 0;
}
