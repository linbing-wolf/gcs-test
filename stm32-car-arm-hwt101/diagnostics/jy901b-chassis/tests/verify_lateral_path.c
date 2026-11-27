#define HEADING_TEST_MAIN retained_path_suite
#include "verify_heading.c"
static int16_t test_origin_yaw;

static void push_forward(double mm)
{
    const int signs[4]={-1,1,1,-1};
    field_y+=mm;
    for(unsigned i=0;i<4;i++) {
        encoder_value[i]+=encoder_polarity[i]*signs[i]*mm*CALIBRATED_PPM_Y*65536.0/3200.0;
        simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
    }
}
static void path_test(unsigned right,unsigned terminal,unsigned slow_yaw)
{
    uint32_t deadline;unsigned injected=0,terminal_correction=0,max_request=0;
    reset_plant(test_origin_yaw,1.15);diagnostic_init();translation_yaw_disturbance=0;
    if(slow_yaw)imu_period_ms=100;
    send_line("loop gyro on");send_line("loop forward on");send_line("loop encoder off");
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(right ? "r" : "l");
    deadline=clock_ms+40000;
    while(run_active && clock_ms<deadline) {
        if(!injected && run.progress_mm>(terminal ? 1997 : 300)) {
            if(terminal)push_forward(9);
            else if(!slow_yaw)physical_yaw+=test_origin_yaw<0 ? -1000 : 1000;
            injected=1;
        }
        if(slow_yaw)translation_yaw_disturbance=run.progress_mm>300 && run.progress_mm<700 ? 2.0 : 0;
        plant_step();
        if(abs(run.left_ref_rpm)>max_request)max_request=abs(run.left_ref_rpm);
        if(terminal && injected && run.progress_mm>=1999 && run.forward_cmd_rpm)terminal_correction=1;
        CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    }
    printf("PATH right=%u terminal=%u yaw_ms=%lu field_left=%.2f field_forward=%.2f end_yaw=%.2f encoder=%ld max_request=%u terminal_correction=%u\n",
        right,terminal,(unsigned long)imu_period_ms,field_x,field_y,(physical_yaw-test_origin_yaw)/100,(long)run.progress_mm,max_request,terminal_correction);
    CHECK(injected && !run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(fabs(field_y)<=5.0 && fabs(field_x-(right ? -2000 : 2000))<=5.0);
    CHECK(fabs(physical_yaw-test_origin_yaw)<=CAR_HEADING_TOLERANCE_CDEG+10 && max_request>=190);
    CHECK((int32_t)(run.lateral_pose.sample_ms-run.zero_ms)>0);
    if(terminal)CHECK(terminal_correction);
}
static void yaw_history_test(void)
{
    HeadingSample sample;float yaw;
    reset_plant(17900,1.15);run.translation_only=1;run.travel_axis=BODY_LEFT;
    run.lateral_pose.origin_yaw_cdeg=17900;
    clock_ms=UINT32_MAX-99u;sample_yaw(17900);sample=heading_snapshot();run_lateral_yaw_sample(sample);
    run_lateral_yaw_sample(sample);CHECK(run.lateral_pose.count==1);
    clock_ms+=100u;sample_yaw(-17900);run_lateral_yaw_sample(heading_snapshot());
    CHECK(run_lateral_yaw_at(clock_ms-50u,&yaw) && fabs(yaw-100)<0.01);
    CHECK(!run_lateral_yaw_at(clock_ms+1u,&yaw) && !run_lateral_yaw_at(clock_ms-101u,&yaw));
    CHECK(run_lateral_yaw_at(clock_ms,&yaw) && fabs(yaw-200)<0.01);
    for(unsigned i=0;i<100;i++) {clock_ms+=10;sample_yaw(-17900);run_lateral_yaw_sample(heading_snapshot());}
    CHECK(run.lateral_pose.count==64 && !run_lateral_yaw_at(clock_ms-640,&yaw));
    CHECK(run_lateral_yaw_at(clock_ms-625,&yaw) && fabs(yaw-200)<0.01);
    CHECK(run_lateral_path_enabled());
    lateral_hold_feedback_enabled=0;CHECK(!run_lateral_path_enabled());
    lateral_hold_feedback_enabled=1;gyro_drive_feedback_enabled=0;CHECK(!run_lateral_path_enabled());
}
static void cancel_terminal_test(void)
{
    uint32_t deadline;unsigned pushed=0,corrected=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line("l");
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        if(!pushed && run.progress_mm>1997) {push_forward(9);pushed=1;}
        plant_step();
        if(pushed && run.forward_cmd_rpm) {corrected=1;break;}
    }
    CHECK(corrected && run_active);queue_byte('!');plant_step();
    CHECK(!run_active && run.state==RUN_STOPPED && arm_stops==2);
    for(unsigned i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i]);
    for(unsigned i=0;i<1000;i++)plant_step();
    CHECK(!run_active && !arm_positions && arm_yaw_us==1975 && arm_claw_us==826 && arm_platform_us==648);
}
static void bypass_test(void)
{
    for(unsigned disable_gyro=0;disable_gyro<2;disable_gyro++) {
        uint32_t deadline;unsigned pushed=0;
        reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
        send_line(disable_gyro ? "loop gyro off" : "loop forward off");
        send_line("test");send_line("dist 2000");send_line("rpm 200");send_line("r");
        deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            if(!pushed && run.progress_mm>1997) {push_forward(9);pushed=1;}
            plant_step();CHECK(!run_lateral_path_enabled());
        }
        CHECK(pushed && run.state==RUN_DONE && !run_active && clock_ms<deadline && field_y>5);
    }
}
static void missing_yaw_interval_test(void)
{
    reset_plant(0,1.15);run.translation_only=1;run.travel_axis=BODY_LEFT;
    clock_ms=1000;sample_yaw(0);run_lateral_yaw_sample(heading_snapshot());
    run.lateral_pose.sample_ms=clock_ms;
    clock_ms=1020;sample_yaw(0);run_lateral_yaw_sample(heading_snapshot());
    run_lateral_pose_observe(0,10,1010);CHECK(run.lateral_pose.valid);
    clock_ms=1520;sample_yaw(100);run_lateral_yaw_sample(heading_snapshot());
    run_lateral_pose_observe(0,100,clock_ms);
    CHECK(run.lateral_pose.sample_ms==1010 && fabs(run.lateral_pose.left_mm-10)<0.01);
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(mode<6)path_test(mode&1u,(mode>>1)==1u,(mode>>1)==2u);
    else if(mode==6)yaw_history_test();
    else if(mode==7)cancel_terminal_test();
    else if(mode==8)bypass_test();
    else if(mode<11) {test_origin_yaw=mode==9 ? 17900 : -17900;path_test(mode==10,0,0);}
    else missing_yaw_interval_test();
    printf("PASS lateral path regression%u: %u assertions; model only\n",mode,assertions);return 0;
}
