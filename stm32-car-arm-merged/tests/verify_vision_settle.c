/* 真实视觉入口、末端重放和独立四轮模型；不连接车辆。 */
#define STATION_TEST_MAIN retained_vision_station_suite
#include "verify_station_vision.c"

static void start_visual_move(unsigned kind,int sign)
{
    char payload[40];uint32_t deadline;
    prepare_vision_debug();imu_period_ms=20;
    send_line("vision debug F 2");wait_debug_request(1);
    if(kind==0)snprintf(payload,sizeof(payload),"MOVE,%d",sign*45);
    else if(kind==1)snprintf(payload,sizeof(payload),"MOVEY,%d",sign*45);
    else snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",sign*45,-sign*38);
    peer_reply(payload,0);deadline=clock_ms+10000;
    while(clock_ms<deadline && !(run.station_shift && run.encoder_ready==15 && run.velocity_phase==VELOCITY_FORWARD))plant_step();
    CHECK(clock_ms<deadline && run.vector_move==(kind==2));
}

static void place_encoder_at_target(int residual)
{
    static const int fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    for(unsigned i=0;i<4;i++) {
        double counts;
        if(run.vector_move) {
            double f=run.vector_target_mm[0]-residual,l=run.vector_target_mm[1]-residual;
            counts=(fs[i]*f*CALIBRATED_PPM_Y+ls[i]*l*CALIBRATED_PPM_X)
                *CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV*run.encoder_sign[i];
        } else {
            float ppm=run.travel_axis==BODY_LEFT ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
            counts=(run.action_mm-residual)*ppm*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV*run.encoder_sign[i];
        }
        motor_feedback[i].position=run.encoder_start[i]+(int64_t)llround(counts);
        motor_feedback[i].position_ms=motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
    }
    run_clear_rx();run.pending=run.velocity_waiting=0;
    heading_sensor.sample.received_ms=clock_ms;run.control_ms=clock_ms-CAR_VELOCITY_CONTROL_MS;
}

static void terminal_replay(void)
{
    for(unsigned kind=0;kind<3;kind++)for(unsigned mirror=0;mirror<2;mirror++) {
        int sign=mirror ? -1 : 1;
        start_visual_move(kind,sign);place_encoder_at_target(0);
        run.velocity_phase=VELOCITY_STOP_FORWARD;run.forward_mrpm=0;
        run.vector_mrpm[0]=run.vector_mrpm[1]=0;run.forward_profile_ms=clock_ms;
        run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
        run.turn_cmd_rpm=0;run.heading_control.stable_samples=0;
        run.heading_control.error_cdeg=165*sign;run.heading_control.rate_cdeg_s=1000*sign;
        run_velocity_update();
        printf("VISION_STOP_REPLAY kind=%u sign=%d turn=%d\n",kind,sign,run.turn_cmd_rpm);
        CHECK(abs(run.turn_cmd_rpm)<=2);
        for(unsigned n=0;n<16;n++) {
            int previous=run.turn_cmd_rpm;
            clock_ms+=25;place_encoder_at_target(n&1 ? 2 : 0);
            run.heading_control.error_cdeg=600*sign;run.heading_control.rate_cdeg_s=0;
            run_velocity_update();CHECK(abs(run.turn_cmd_rpm-previous)<=2);
        }
        CHECK(abs(run.turn_cmd_rpm)>1);stop_request();
    }
}

static void geometry_and_cancel_tests(void)
{
    for(unsigned kind=0;kind<3;kind++)for(unsigned negative=0;negative<2;negative++)for(unsigned lag=0;lag<2;lag++) {
        int sign=negative ? -1 : 1,seen=0,last_mm=0,last_f=0,last_l=0;
        uint32_t deadline,start;uint16_t claw,platform;
        start_visual_move(kind,sign);
        if(lag)for(unsigned i=0;i<4;i++)wheel_acceleration[i]=0.12+i*0.02;
        claw=arm_claw_us;platform=arm_platform_us;start=clock_ms;deadline=start+20000;
        physical_yaw+=negative ? -165 : 165;peer_reply("STACK,330,197,15,1",0);
        while(run.vision_debug && clock_ms<deadline && run.state!=RUN_FAULT) {
            plant_step();
            if(run.station_shift) {seen=1;last_mm=run.progress_mm;last_f=run.vector_progress_mm[0];last_l=run.vector_progress_mm[1];}
        }
        CHECK(seen && clock_ms<deadline && run.state==RUN_IDLE && !run_active && !arm_positions);
        CHECK(arm_claw_us==claw && arm_platform_us==platform);
        if(kind==2)CHECK(abs(last_f+sign*45)<=1 && abs(last_l+sign*38)<=1);
        else CHECK(abs(last_mm-45)<=1);
        printf("VISION_SETTLE_MODEL kind=%u sign=%d lag=%u elapsed_ms=%lu terminal=%d,%d scalar=%d\n",
            kind,sign,lag,(unsigned long)(clock_ms-start),last_f,last_l,last_mm);stop_request();
    }
    start_visual_move(2,1);send_line("!");plant_step();CHECK(!run_active && !run.vision_debug && arm_failed());
}

static void noise_and_angle_boundary_tests(void)
{
    static const int samples[][3]={{83,0,1},{-83,0,-1},{79,0,0},{-79,0,0},
        {9,150,0},{-9,-150,0},{9,500,-1},{-9,500,-2}};
    for(unsigned kind=0;kind<3;kind++) {
        start_visual_move(kind,1);place_encoder_at_target(0);
        run.velocity_phase=VELOCITY_STOP_FORWARD;run.forward_mrpm=0;
        run.vector_mrpm[0]=run.vector_mrpm[1]=0;
        run.forward_profile_ms=run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
        run.turn_cmd_rpm=0;run.heading_control.stable_samples=0;
        for(unsigned n=0;n<sizeof(samples)/sizeof(samples[0]);n++) {
            int previous=run.turn_cmd_rpm;clock_ms+=25;place_encoder_at_target(0);
            run.heading_control.error_cdeg=samples[n][0];run.heading_control.rate_cdeg_s=samples[n][1];
            run_velocity_update();CHECK(run.turn_ref_rpm==samples[n][2]);
            CHECK(abs(run.turn_cmd_rpm-previous)<=2);
        }
        stop_request();
    }
    puts("PASS visual yaw boundary: sub-RPM noise suppressed inside0.8deg, residual outside window corrects at1RPM, useful damping retained");
}

static void fixed_index_scope_test(void)
{
    uint32_t deadline;
    prepare_vision_debug();send_line("vision debug F 2");wait_debug_request(1);
    CHECK(station_chassis_start(150));deadline=clock_ms+10000;
    while(clock_ms<deadline && !(run.station_shift && run.encoder_ready==15 && run.velocity_phase==VELOCITY_FORWARD))plant_step();
    CHECK(clock_ms<deadline);place_encoder_at_target(0);
    run.velocity_phase=VELOCITY_STOP_FORWARD;run.forward_mrpm=0;run.forward_profile_ms=clock_ms;
    run.turn_cmd_rpm=0;run.heading_control.error_cdeg=165;run.heading_control.rate_cdeg_s=1000;
    run.heading_control.stable_samples=0;run_velocity_update();
    CHECK(run.turn_cmd_rpm==CAR_HEADING_MIN_RPM);
    puts("PASS fixed index: original stationary-heading controller retained");stop_request();
}

static void tiny_and_fifo_tests(void)
{
    for(unsigned axis=0;axis<2;axis++)for(unsigned negative=0;negative<2;negative++)for(unsigned mm=2;mm<=3;mm++) {
        char payload[32];uint32_t deadline;int last=0,seen=0;
        prepare_vision_debug();send_line("vision debug F 2");wait_debug_request(1);
        snprintf(payload,sizeof(payload),axis ? "MOVEY,%d" : "MOVE,%d",negative ? -(int)mm : (int)mm);
        peer_reply(payload,0);peer_reply("STACK,330,197,15,1",0);deadline=clock_ms+15000;
        while(run.vision_debug && clock_ms<deadline && run.state!=RUN_FAULT) {
            plant_step();if(run.station_shift) {last=run.progress_mm;seen=1;}
        }
        CHECK(seen && clock_ms<deadline && run.state==RUN_IDLE && abs(last-(int)mm)<=1);
        printf("VISION_TINY axis=%u negative=%u target=%u end=%d\n",axis,negative,mm,last);stop_request();
    }
    {
        uint32_t deadline;unsigned count=0,was_shift=0;double x,y;
        prepare_vision_debug();send_line("vision debug F 2");wait_debug_request(1);x=field_x;y=field_y;
        peer_reply("MOVE,12",0);peer_reply("MOVEXY,-14,9",0);peer_reply("MOVEY,-6",0);peer_reply("STACK,330,197,15,1",0);
        deadline=clock_ms+30000;
        while(run.vision_debug && clock_ms<deadline) {
            plant_step();
            if(run.station_shift && !was_shift) {
                static const unsigned mm[]={12,14,6};CHECK(count<3 && run.remaining_mm==mm[count]);
                CHECK(run.vector_move==(count==1));count++;
            }
            was_shift=run.station_shift;CHECK(!arm_positions);
        }
        CHECK(count==3 && clock_ms<deadline && run.state==RUN_IDLE && !vision_motion_peek_xy(NULL,NULL,NULL));
        CHECK(fabs(field_y-y-2)<5 && fabs(field_x-x-3)<5 && vision_tx_count==8);
        puts("PASS visual FIFO: MOVE/MOVEXY/MOVEY order and early terminal drain retained");stop_request();
    }
}

int main(void)
{
    terminal_replay();geometry_and_cancel_tests();noise_and_angle_boundary_tests();fixed_index_scope_test();tiny_and_fifo_tests();
    movexy_transport_tests();movexy_debug_tests();movexy_polarity_response_tests();movexy_station_tests();
    printf("PASS vision settle: %lu assertions; UART and independent wheel model only\n",(unsigned long)assertions);return 0;
}
