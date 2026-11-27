/* 诊断底盘合入正式入口后的验收：编码器位移、起步坐标、分速航向和非目标作用域。 */
#define HEADING_TEST_MAIN retained_chassis_merge_heading_suite
#include "verify_heading.c"

static void merged_step(void)
{
    double old_x=field_x,old_y=field_y,angle=-(physical_yaw-origin_yaw)*3.141592653589793/18000.0;
    plant_step();
    if(run.travel_axis==BODY_FORWARD) {
        double dx=field_x-old_x,dy=field_y-old_y;
        double extra=(-sin(angle)*dx+cos(angle)*dy)*(CALIBRATED_PPM_Y/car_road_ppm_y(run.travel_sign)-1);
        field_x-=sin(angle)*extra;field_y+=cos(angle)*extra;
    }
}
static void scope_test(void)
{
    HeadingDriveGains gains;
    reset_plant(0,1.15);run.imu_mode=1;run.travel_axis=BODY_LEFT;run.forward_mrpm=160000;
    CHECK(run_path_hold_enabled());gains=run_drive_heading_gains();CHECK(gains.kp_x100==200 && gains.kd_x100==30);
    run.translation_only=1;CHECK(run_path_hold_enabled());
    run.station_shift=1;CHECK(!run_path_hold_enabled() && !run_drive_gain_schedule_enabled());run.station_shift=0;
    run.vector_move=1;CHECK(!run_path_hold_enabled());run.vector_move=0;
    run.obstacle_phase=2;CHECK(!run_path_hold_enabled());run.obstacle_phase=0;
    run.corner.active=1;CHECK(!run_path_hold_enabled());run.corner.active=0;
    send_line("loop forward off");CHECK(!run_path_hold_enabled());send_line("loop forward on");
    CHECK(run_path_hold_enabled());send_line("loop gyro off");CHECK(!run_path_hold_enabled());send_line("loop gyro on");
    for(unsigned speed=0;speed<=300;speed++) {
        gains=heading_control_drive_gains(0,(uint16_t)speed);CHECK(gains.kp_x100==150 && gains.kd_x100==30);
        gains=heading_control_drive_gains(1,(uint16_t)speed);CHECK(gains.kp_x100>=150 && gains.kp_x100<=220 && gains.kd_x100==30);
    }
    CHECK(CAR_IMU_UART_BAUD==115200 && CALIBRATED_PPM_X==10.92f);
    CHECK(CAR_FORWARD_LEFT_MM_PER_M==0 && CAR_BACKWARD_LEFT_MM_PER_M==0);
    puts("PASS merged scope: normal roads/manual use encoder pose and gain schedule; station/vector/corner/retreat preserved");
}
static void motion_test(unsigned direction,unsigned disturbed)
{
    const char *keys[]={"f","b","l","r"};unsigned injected=0,observed=0,peak=0;
    uint32_t deadline;
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    send_line("loop encoder on");send_line("loop gyro on");send_line("loop forward on");
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(keys[direction]);
    deadline=clock_ms+35000;
    while(run_active && clock_ms<deadline) {
        if(disturbed && !injected && run.progress_mm>300) {physical_yaw+=500;injected=1;}
        merged_step();
        if(run_path_hold_enabled() && run.lateral_pose.valid)observed=1;
        if(abs((int)(run.forward_mrpm/1000))>peak)peak=abs((int)(run.forward_mrpm/1000));
        for(unsigned i=0;i<4;i++)CHECK(abs(run.wheel_rpm[i])<=200);
    }
    printf("MERGED_PATH direction=%s disturb=%u state=%u forward=%.2f left=%.2f estimate=%ld peak=%u\n",
        keys[direction],disturbed,run.state,field_y,field_x,(long)run.progress_mm,peak);
    CHECK(observed && !run_active && run.state==RUN_DONE && clock_ms<deadline && peak>=190);
    CHECK(fabs((direction>=2 ? field_x : field_y)-(direction&1u ? -2000 : 2000))<=6);
    CHECK(fabs(direction>=2 ? field_y : field_x)<=6 && fabs(physical_yaw)<=90);
    CHECK(abs(run.progress_mm-2000)<=1 && abs(run.straight_hold.lateral_mm)<=2);
    CHECK((int32_t)(run.lateral_pose.sample_ms-run.zero_ms)>0);
    for(unsigned i=0;i<4;i++)CHECK(abs(reported_wheel_rpm[i])<=1);
}
static void sensor_and_disabled_test(void)
{
    HeadingSensor sensor;uint8_t frame[11]={0x55,0x52,0,0,0,0,0,0,0,0,0};
    reset_plant(0,1.15);heading_sensor_reset(&sensor);
    frame[6]=0xff;frame[7]=0x7f;
    for(unsigned i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    for(unsigned i=0;i<11;i++)heading_sensor_byte(&sensor,frame[i],100);
    CHECK(sensor.rate.rate_cdeg_s>190000 && !sensor.sample.valid);
    CHECK(heading_rate_fresh((const HeadingRate *)&sensor.rate,200,100));
    CHECK(!heading_rate_fresh((const HeadingRate *)&sensor.rate,201,100));
    send_line("loop displacement imu");CHECK(strstr(output,"rejected") && !run_active);
    send_line("loop displacement fused");CHECK(strstr(output,"rejected") && !run_active);
    puts("PASS source: independently fresh full-range gyro; inertial position modes absent from competition entry");
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(!mode){scope_test();sensor_and_disabled_test();}
    else motion_test((mode-1)%4,mode>4);
    printf("PASS chassis merge mode%u: %u assertions; model only\n",mode,assertions);return 0;
}
