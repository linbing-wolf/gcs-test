/* 复用实际F帧入口与独立四轮模型；不把指令换向复现当作实车震荡验收。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

#ifndef MOVEXY_FINE_BASELINE
#define MOVEXY_FINE_BASELINE 0
#endif
#ifndef MOVEXY_TERMINAL_BASELINE
#define MOVEXY_TERMINAL_BASELINE 0
#endif

#if !MOVEXY_TERMINAL_BASELINE
static void fine_command_replay(void)
{
    unsigned direction,total=0;
    for(direction=0;direction<4;direction++) {
        uint32_t deadline;
        int16_t translation[4];
        int last[4]={0,0,0,0};
        unsigned reversals[4]={0,0,0,0},i,n;
        char payload[40];
        prepare_vision_debug();imu_period_ms=20;
        send_line("vision debug F 2");wait_debug_request(1);
        snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",direction&1 ? -18 : 18,direction&2 ? 11 : -11);
        peer_reply(payload,0);peer_reply("STACK,305,194,-10,1",0);
        deadline=clock_ms+10000;
        while(clock_ms<deadline && !(run.vector_move && run.velocity_phase==VELOCITY_FORWARD
            && run.encoder_ready==15))plant_step();
        CHECK(clock_ms<deadline && run.station_shift);
        run.vector_profile_sign[0]=run.vector_target_mm[0]>0 ? 1 : -1;
        run.vector_profile_sign[1]=run.vector_target_mm[1]>0 ? 1 : -1;
        heading_control_translate_wheels(3*run.vector_profile_sign[0],3*run.vector_profile_sign[1],0,translation);
        for(n=0;n<32;n++) {
            /* 固定本段编码器反馈，只重放日志所示微小角差与双向角速度。 */
            sample_yaw(run.heading_control.last_yaw_cdeg);
            run.heading_control.sample_sequence=heading_sensor.sample.sequence;
            run.heading_control.sample_ms=clock_ms;
            run.heading_control.error_cdeg=-9;
            run.heading_control.rate_cdeg_s=n&1 ? -150 : 150;
            run.vector_mrpm[0]=run.vector_mrpm[1]=3000;
            run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
            for(i=0;i<4;i++)motor_feedback[i].position_ms=motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
            run.control_ms=clock_ms-CAR_VELOCITY_CONTROL_MS;
            run_velocity_update();
            CHECK(run_active && run.vector_move);
            for(i=0;i<4;i++) {
                int sign=run.wheel_rpm[i]>0 ? 1 : run.wheel_rpm[i]<0 ? -1 : 0;
                if(sign && last[i] && sign!=last[i])reversals[i]++;
                if(sign)last[i]=sign;
            }
#if !MOVEXY_FINE_BASELINE
            CHECK(run.turn_cmd_rpm==0);
            for(i=0;i<4;i++)CHECK(run.wheel_rpm[i]==translation[i]);
#endif
        }
        for(i=0;i<4;i++)if(!translation[i]) {
            total+=reversals[i];
#if MOVEXY_FINE_BASELINE
            CHECK(reversals[i]>0);
#else
            CHECK(reversals[i]==0);
#endif
        }
        printf("MOVEXY_FINE_REPLAY quadrant=%u reversals=%u,%u,%u,%u baseline=%u\n",
            direction,reversals[0],reversals[1],reversals[2],reversals[3],MOVEXY_FINE_BASELINE);
    }
    printf("%s low-effort cancelled-wheel reversals=%u; command replay only\n",MOVEXY_FINE_BASELINE ? "REPRO" : "PASS",total);
}

#if !MOVEXY_FINE_BASELINE
static void fine_boundary_and_scope_tests(void)
{
    static const int samples[][3]={{83,0,0},{84,0,1},{-83,0,0},{-84,0,-1},
        {-9,500,-2},{-9,-500,1},{100,0,1},{-100,0,-1}};
    unsigned n,i;
    uint32_t deadline;
    prepare_vision_debug();imu_period_ms=20;
    send_line("vision debug F 2");wait_debug_request(1);
    peer_reply("MOVEXY,18,-11",0);
    deadline=clock_ms+10000;
    while(clock_ms<deadline && !(run.vector_move && run.velocity_phase==VELOCITY_FORWARD
        && run.encoder_ready==15))plant_step();
    CHECK(clock_ms<deadline);
    run.vector_profile_sign[0]=run.vector_profile_sign[1]=-1;
    for(n=0;n<sizeof(samples)/sizeof(samples[0])+2;n++) {
        int expected;
        sample_yaw(run.heading_control.last_yaw_cdeg);
        run.heading_control.sample_sequence=heading_sensor.sample.sequence;
        run.heading_control.sample_ms=clock_ms;
        run.heading_control.error_cdeg=n<8 ? samples[n][0] : -9;
        run.heading_control.rate_cdeg_s=n<8 ? samples[n][1] : 150;
        run.vector_mrpm[0]=run.vector_mrpm[1]=3000;
        run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
        expected=n<8 ? samples[n][2] : -1;
        for(i=0;i<4;i++)motor_feedback[i].position_ms=motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
        if(n==8)run.station_shift=0;
        if(n==9) {
            run.station_shift=1;
            run.vector_mrpm[0]=(CAR_MOVEXY_FINE_MAX_RPM+1)*1000;
            run.vector_profile_ms[0]=clock_ms;
        }
        run.control_ms=clock_ms-CAR_VELOCITY_CONTROL_MS;
        run_velocity_update();
        CHECK(run_active && run.turn_cmd_rpm==expected);
    }
    puts("PASS fine scope: sub-1RPM boundary, meaningful yaw error/damping retained, station-only and low-speed-only");
}

static void reversal_diagnostic_tests(void)
{
    unsigned i;
    reset_plant(0,1.15);run_active=1;run.imu_mode=1;run.vector_move=1;
    CHECK(run_velocity_send(-3,-3,-1));
    run.velocity_waiting=0;run_clear_rx();CHECK(run_velocity_send(0,0,0));
    run.velocity_waiting=0;run_clear_rx();CHECK(run_velocity_send(-3,-3,1));
    CHECK(!run.vector_wheel_reversals[0] && run.vector_wheel_reversals[1]==1
        && !run.vector_wheel_reversals[2] && run.vector_wheel_reversals[3]==1);
    run.velocity_waiting=0;run_clear_rx();CHECK(run_velocity_send(-3,-3,1));
    CHECK(run.vector_wheel_reversals[1]==1);
    for(i=0;i<4;i++)run.vector_wheel_reversals[i]=65535u;
    run.velocity_waiting=0;run_clear_rx();CHECK(run_velocity_send(-3,-3,-1));
    for(i=0;i<4;i++)CHECK(run.vector_wheel_reversals[i]==65535u);
    run.velocity_phase=VELOCITY_FORWARD;run.log_ms=clock_ms-CAR_CONTROL_LOG_MS;
    output_size=0;output[0]=0;run_velocity_log();
    CHECK(strstr(output," wheel_rev=65535,65535,65535,65535"));
    puts("PASS wheel reversal log: submitted batches, zero preserves direction, unchanged batch not recounted, saturation and output");
}

static void field_geometry_tests(void)
{
    unsigned quadrant;
    for(quadrant=0;quadrant<4;quadrant++) {
        uint32_t deadline;
        unsigned seen=0,i;
        double x,y;
        int dx=quadrant&1 ? -18 : 18,dy=quadrant&2 ? 11 : -11;
        int final_x=0,final_y=0;
        char payload[40];
        prepare_vision_debug();imu_period_ms=20;
        for(i=0;i<4;i++)wheel_acceleration[i]=0.12+i*0.04;
        send_line("vision debug F 2");wait_debug_request(1);
        x=field_x;y=field_y;
        snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",dx,dy);peer_reply(payload,0);
        peer_reply("STACK,305,194,-10,1",0);deadline=clock_ms+15000;
        while(run.vision_debug && clock_ms<deadline) {
            plant_step();
            if(run.vector_move && run.station_shift) {
                if(!seen)for(i=0;i<4;i++)CHECK(!run.vector_wheel_reversals[i]);
                seen=1;
                final_x=run.vector_progress_mm[0];final_y=run.vector_progress_mm[1];
            }
        }
        CHECK(seen && clock_ms<deadline && run.state==RUN_IDLE);
        CHECK(abs(final_x+dx)<=CAR_DISTANCE_TOLERANCE_MM && abs(final_y-dy)<=CAR_DISTANCE_TOLERANCE_MM);
        CHECK(fabs(field_y-y+dx)<3.0 && fabs(field_x-x-dy)<3.0);
        printf("MOVEXY_FINE_FIELD quadrant=%u field_forward=%.2f field_left=%.2f target=%d,%d; simulation only\n",
            quadrant,field_y-y,field_x-x,-dx,dy);
    }
    puts("PASS field case: F2 18/11mm all quadrants at50Hz with unequal wheel response; both encoder1mm windows retained");
}
#endif

#endif

static void terminal_profile_tests(void)
{
    unsigned axis,negative,remaining;
    reset_plant(0,1.15);run.job=6;run.encoder_ready=15;
    for(axis=0;axis<2;axis++)for(negative=0;negative<2;negative++) {
        int sign=negative ? -1 : 1;
        run.vector_target_mm[axis]=sign*100;
        for(remaining=0;remaining<=8;remaining++) {
            int rpm;
            run.vector_progress_mm[axis]=sign*(100-(int)remaining);
            rpm=run_vector_target((uint8_t)axis);
            if(remaining<=1)CHECK(!rpm);
            if(remaining==2) {
                float ppm=axis ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
                float delay=CAR_DRIVE_DECEL_RPM_PER_S*CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0f;
                float braking=sqrtf(delay*delay+2.0f*CAR_DRIVE_DECEL_RPM_PER_S*60.0f*ppm/CAR_POSITION_PULSES_PER_REV)-delay;
                printf("MOVEXY_TERMINAL axis=%u sign=%d remaining_mm=2 brake_limit_rpm=%.3f command_rpm=%d baseline=%u\n",
                    axis,sign,braking,rpm,MOVEXY_TERMINAL_BASELINE);
#if MOVEXY_TERMINAL_BASELINE
                CHECK(rpm==sign*3 && abs(rpm)>braking);
#else
                CHECK(rpm==sign && abs(rpm)<=braking);
#endif
            }
#if !MOVEXY_TERMINAL_BASELINE
            if(remaining==3)CHECK(rpm==sign);
            if(remaining>=6)CHECK(rpm==sign*(int)(remaining/2));
#endif
        }
        run.vector_progress_mm[axis]=sign*102;
#if MOVEXY_TERMINAL_BASELINE
        CHECK(run_vector_target((uint8_t)axis)==-sign*3);
#else
        CHECK(run_vector_target((uint8_t)axis)==-sign);
#endif
    }
    puts(MOVEXY_TERMINAL_BASELINE ? "REPRO terminal3RPM floor overrides calculated brake limit" : "PASS terminal1RPM taper: 0/1mm window unchanged, 2/3mm low speed, signed overshoot recovery and >=6mm profile retained");
}

#if !MOVEXY_TERMINAL_BASELINE && !MOVEXY_FINE_BASELINE
static void terminal_field_tests(void)
{
    unsigned quadrant;
    for(quadrant=0;quadrant<4;quadrant++) {
        int dx=quadrant&1 ? 75 : -75,dy=quadrant&2 ? 44 : -44;
        uint32_t deadline;
        int final_x=0,final_y=0,seen=0,slow_seen=0;
        double x,y;
        char payload[40];
        prepare_vision_debug();imu_period_ms=20;
        send_line("vision debug F 2");wait_debug_request(1);x=field_x;y=field_y;
        snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",dx,dy);peer_reply(payload,0);
        peer_reply("STACK,330,197,15,1",0);deadline=clock_ms+20000;
        while(run.vision_debug && clock_ms<deadline) {
            plant_step();
            if(run.vector_move && run.station_shift) {
                seen=1;final_x=run.vector_progress_mm[0];final_y=run.vector_progress_mm[1];
                if(abs(run.forward_cmd_rpm)==1 || abs(run.left_cmd_rpm)==1)slow_seen=1;
            }
        }
        CHECK(clock_ms<deadline && seen && slow_seen && run.state==RUN_IDLE);
        CHECK(abs(final_x+dx)<=CAR_DISTANCE_TOLERANCE_MM && abs(final_y-dy)<=CAR_DISTANCE_TOLERANCE_MM);
        CHECK(fabs(field_y-y+dx)<3 && fabs(field_x-x-dy)<3);
        printf("MOVEXY_TERMINAL_FIELD quadrant=%u forward=%.2f left=%.2f target=%d,%d; simulation only\n",
            quadrant,field_y-y,field_x-x,-dx,dy);
    }
    puts("PASS F2 75/44mm terminal geometry all quadrants at50Hz; encoder1mm gate and independent physical-model endpoints");
}
#endif

int main(void)
{
    terminal_profile_tests();
#if !MOVEXY_TERMINAL_BASELINE
    fine_command_replay();
#if !MOVEXY_FINE_BASELINE
    fine_boundary_and_scope_tests();reversal_diagnostic_tests();field_geometry_tests();terminal_field_tests();movexy_tests();
    printf("PASS MOVEXY fine: %lu assertions; host command/plant verification, not vehicle acceptance\n",(unsigned long)assertions);
#endif
#endif
    return 0;
}
