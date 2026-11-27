#define HEADING_TEST_MAIN retained_forward_path_suite
#include "verify_heading.c"

static void forward_step(void)
{
    /* 此专项物理前后位移使用各方向实测比例，横向使用当前CALIBRATED_PPM_X；不把旧9.68当10.00。 */
    double old_x=field_x,old_y=field_y,angle=-(physical_yaw-origin_yaw)*3.141592653589793/18000.0;
    plant_step();
    {double dx=field_x-old_x,dy=field_y-old_y;
     double forward=(-sin(angle)*dx+cos(angle)*dy)*(CALIBRATED_PPM_Y/car_road_ppm_y(run.travel_sign)-1.0);
     field_x-=sin(angle)*forward;field_y+=cos(angle)*forward;}
}
static void push_left(double mm)
{
    const int signs[4]={-1,-1,1,1};field_x+=mm;
    for(unsigned i=0;i<4;i++) {
        encoder_value[i]+=encoder_polarity[i]*signs[i]*mm*CALIBRATED_PPM_X*65536.0/3200.0;
        simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
    }
}
static void forward_path(unsigned back,unsigned terminal,unsigned slow)
{
    uint32_t deadline;unsigned injected=0,terminal_correction=0,peak=0;
    double target=back ? -2000 : 2000,target_left=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;if(slow)imu_period_ms=100;
    send_line("loop gyro on");send_line("loop encoder on");send_line("loop forward off");
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(back ? "b" : "f");
    deadline=clock_ms+40000;
    while(run_active && clock_ms<deadline) {
        if(!injected && run.progress_mm>(terminal ? 1997 : 300)) {
            if(terminal)push_left(9);else if(!slow)physical_yaw+=1000;injected=1;
        }
        if(slow)translation_yaw_disturbance=run.progress_mm>300 && run.progress_mm<700 ? 2.0 : 0;
        forward_step();if(abs(run.forward_ref_rpm)>peak)peak=abs(run.forward_ref_rpm);
        if(terminal && injected && run.progress_mm>=1999 && run.left_cmd_rpm)terminal_correction=1;
        CHECK(!arm_positions && arm_yaw_us==1975 && arm_claw_us==826 && arm_platform_us==648);
    }
    printf("FORWARD_PATH back=%u terminal=%u yaw_ms=%lu field_forward=%.2f field_left=%.2f target_left=%.2f peak=%u terminal_correction=%u\n",
        back,terminal,(unsigned long)imu_period_ms,field_y,field_x,target_left,peak,terminal_correction);
    CHECK(injected && !run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(fabs(field_y-target)<=5 && fabs(field_x-target_left)<=5);
    CHECK(fabs(physical_yaw)<=CAR_HEADING_TOLERANCE_CDEG+10 && peak>=190);
    CHECK((int32_t)(run.lateral_pose.sample_ms-run.zero_ms)>0);
    if(terminal)CHECK(terminal_correction);
}
static void forward_cancel(unsigned back)
{
    uint32_t deadline;unsigned pushed=0,corrected=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    send_line("test");send_line("dist 300");send_line("rpm 120");send_line(back ? "b" : "f");
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        if(!pushed && run.progress_mm>297) {push_left(9);pushed=1;}
        forward_step();
        if(pushed && run.progress_mm>=299 && run.left_cmd_rpm) {corrected=1;break;}
    }
    CHECK(corrected && run_active);queue_byte('!');forward_step();
    CHECK(!run_active && run.state==RUN_STOPPED && arm_stops==2);
    for(unsigned i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i]);
    for(unsigned i=0;i<100;i++)forward_step();
    CHECK(!run_active && !arm_positions && arm_yaw_us==1975 && arm_claw_us==826 && arm_platform_us==648);
}
static void forward_bypass(unsigned back)
{
    for(unsigned gyro_off=0;gyro_off<2;gyro_off++) {
        uint32_t deadline;unsigned pushed=0;
        reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
        send_line(gyro_off ? "loop gyro off" : "loop encoder off");send_line("loop forward on");
        send_line("test");send_line("dist 300");send_line("rpm 120");send_line(back ? "b" : "f");
        deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            if(!pushed && run.progress_mm>297) {push_left(9);pushed=1;}
            forward_step();CHECK(!run_path_hold_enabled());
        }
        CHECK(pushed && !run_active && run.state==RUN_DONE && clock_ms<deadline);
    }
}
static void forward_overshoot(unsigned back)
{
    const int fs[4]={-1,1,1,-1};int sign=back ? -1 : 1;
    uint32_t deadline;unsigned pushed=0,reversed=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    send_line("test");send_line("dist 300");send_line("rpm 120");send_line(back ? "b" : "f");
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        if(!pushed && run.progress_mm>297) {
            field_y+=sign*12;
            for(unsigned i=0;i<4;i++) {
                encoder_value[i]+=encoder_polarity[i]*fs[i]*sign*12*car_road_ppm_y(sign)*65536.0/3200.0;
                simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
            }
            pushed=1;
        }
        forward_step();if(run.forward_cmd_rpm*sign<0)reversed=1;
    }
    CHECK(pushed && reversed && !run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(fabs(field_y-sign*300)<=5 && fabs(field_x)<=5);
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    /* 直行目标为起步参考线，短段和长段均不再请求固定侧移。 */
    CHECK(car_straight_left_trim(300)==0 && car_straight_left_trim(-300)==0);
    CHECK(car_straight_left_trim(1000)==0 && car_straight_left_trim(-1000)==0);
    CHECK(car_straight_left_trim(2000)==0 && car_straight_left_trim(-2000)==0);
    if(mode<6)forward_path(mode&1u,(mode>>1)==1u,(mode>>1)==2u);
    else if(mode<8)forward_cancel(mode&1u);
    else if(mode<10)forward_bypass(mode&1u);
    else forward_overshoot(mode&1u);
    printf("PASS forward/back path regression%u: %u assertions; model only\n",mode,assertions);return 0;
}
