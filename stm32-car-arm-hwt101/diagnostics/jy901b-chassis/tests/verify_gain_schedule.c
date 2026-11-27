#define HEADING_TEST_MAIN retained_gain_schedule_suite
#include "verify_heading.c"

static unsigned lateral_p(unsigned speed)
{
    if(speed<=60)return 150;
    if(speed<=120)return (150*(120-speed)+180*(speed-60)+30)/60;
    if(speed<=200)return (180*(200-speed)+220*(speed-120)+40)/80;
    return 220;
}
static void selector_test(void)
{
    HeadingControl control;HeadingDriveGains gains,previous={150,30};
    const HeadingDriveGains changed_d={220,60};
    memset(&control,0,sizeof(control));
    for(unsigned speed=0;speed<=300;speed++) {
        gains=heading_control_drive_gains(1,(uint16_t)speed);
        CHECK(gains.kp_x100==lateral_p(speed) && gains.kd_x100==30);
        CHECK(gains.kp_x100>=previous.kp_x100 && gains.kp_x100-previous.kp_x100<=1);
        previous=gains;gains=heading_control_drive_gains(0,(uint16_t)speed);
        CHECK(gains.kp_x100==150 && gains.kd_x100==30);
    }
    gains=heading_control_drive_gains(1,UINT16_MAX);CHECK(gains.kp_x100==220);
    control.error_cdeg=500;
    CHECK(heading_control_drive_rpm(&control,20)==8);
    CHECK(heading_control_drive_rpm_with_gains(&control,20,gains)==11);
    control.error_cdeg=-500;
    CHECK(heading_control_drive_rpm_with_gains(&control,20,gains)==-11);
    control.error_cdeg=500;control.rate_cdeg_s=1000;
    CHECK(heading_control_drive_rpm_with_gains(&control,20,gains)==8);
    CHECK(heading_control_drive_rpm_with_gains(&control,20,changed_d)==5);
    control.rate_cdeg_s=0;
    CHECK(heading_control_drive_rpm_with_gains(&control,6,gains)==6);
    control.error_cdeg=10;control.rate_cdeg_s=20;
    CHECK(heading_control_drive_rpm_with_gains(&control,20,gains)==0);
}
static void scope_test(void)
{
    HeadingDriveGains gains;
    reset_plant(0,1.15);diagnostic_init();run.translation_only=1;run.travel_axis=BODY_LEFT;
    run.lateral_rpm=200;run.forward_mrpm=30000;gains=run_drive_heading_gains();
    CHECK(gains.kp_x100==150); /* rpm上限200，斜坡30时仍是低速。 */
    run.forward_mrpm=160000;run.profile_sign=-1;gains=run_drive_heading_gains();
    CHECK(gains.kp_x100==(CAR_DRIVE_GAIN_SCHEDULE_ENABLED ? 200 : 150));
    run.travel_axis=BODY_FORWARD;gains=run_drive_heading_gains();CHECK(gains.kp_x100==150);
    run.travel_axis=BODY_LEFT;
    run.station_shift=1;CHECK(!run_drive_gain_schedule_enabled());run.station_shift=0;
    run.vector_move=1;CHECK(!run_drive_gain_schedule_enabled());run.vector_move=0;
    run.obstacle_phase=1;CHECK(!run_drive_gain_schedule_enabled());run.obstacle_phase=0;
    run.turn_only=1;CHECK(!run_drive_gain_schedule_enabled());run.turn_only=0;
    run.translation_only=0;CHECK(!run_drive_gain_schedule_enabled());run.translation_only=1;
    send_line("loop gyro off");CHECK(!run_drive_gain_schedule_enabled());
    gains=run_drive_heading_gains();CHECK(gains.kp_x100==150 && gains.kd_x100==30);
    CHECK(!arm_positions && arm_yaw_us==1975 && arm_claw_us==826 && arm_platform_us==648);
}
static int expected_turn(unsigned p)
{
    int error=run.heading_control.error_cdeg,rate=run.heading_control.rate_cdeg_s;
    int effort=error*(int)p-rate*30,rpm;
    if(abs(error)<=20 && abs(rate)<=50)return 0;
    rpm=effort>=0 ? (effort+5000)/10000 : -((-effort+5000)/10000);
    if(effort>0 && rpm<1)rpm=1;
    if(effort<0 && rpm> -1)rpm=-1;
    if(rpm>20)rpm=20;
    if(rpm< -20)rpm=-20;
    return rpm;
}
static void motion_test(unsigned direction,unsigned limit)
{
    const char *commands[]={"f","b","l","r"};char line[32];
    uint32_t deadline,previous_control=0;unsigned observed=0,settle_log=0,peak=0,checks=0;
    unsigned gain_peak=0,accelerated=0,decelerated=0,old_speed=0,disturbed=0,nonzero_checked=0;
    const char *schedule_log=CAR_DRIVE_GAIN_SCHEDULE_ENABLED ? "heading_schedule=1" : "heading_schedule=0";
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    send_line("test");send_line("dist 2000");snprintf(line,sizeof(line),"rpm %u",limit);send_line(line);send_line(commands[direction]);
    deadline=clock_ms+40000;
    while(run_active && clock_ms<deadline) {
        unsigned speed,p;
        if(!disturbed && run.progress_mm>300) {physical_yaw+=300;disturbed=1;}
        plant_step();
        if(strstr(output,schedule_log) && strstr(output,"heading_pd_mode=DRIVE")
            && strstr(output,"heading_kp_x100=") && strstr(output,"heading_kd_x100=")
            && strstr(output,"heading_speed_rpm="))observed=1;
        if(strstr(output,"heading_pd_mode=SETTLE")) {
            CHECK(strstr(output,"heading_schedule=0") && strstr(output,"heading_kp_x100=250"));settle_log=1;
        }
        CHECK(!arm_positions && arm_yaw_us==1975 && arm_claw_us==826 && arm_platform_us==648);
        if(!run_active || run.velocity_phase!=VELOCITY_FORWARD || run.control_ms==previous_control)continue;
        previous_control=run.control_ms;speed=(unsigned)(run.forward_mrpm/1000);
        p=direction>=2 && CAR_DRIVE_GAIN_SCHEDULE_ENABLED ? lateral_p(speed) : 150;
        if(speed>peak)peak=speed;
        if(p>gain_peak)gain_peak=p;
        if(speed>old_speed)accelerated=1;
        if(speed<old_speed && peak>=limit-5)decelerated=1;
        old_speed=speed;
        if(!run.settle_turn_active && speed) {
            if(run.turn_ref_rpm!=expected_turn(p))
                printf("GAIN_OUTPUT direction=%s speed=%u p=%u error=%ld rate=%ld expected=%d reference=%d phase=%u encoder=%u\n",
                    commands[direction],speed,p,(long)run.heading_control.error_cdeg,(long)run.heading_control.rate_cdeg_s,
                    expected_turn(p),run.turn_ref_rpm,run.velocity_phase,run.encoder_ready);
            CHECK(run.turn_ref_rpm==expected_turn(p));checks++;
            if(run.turn_ref_rpm)nonzero_checked++;
        }
    }
    CHECK(observed && settle_log && !run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(accelerated && decelerated && peak>=limit-5 && peak<=limit && checks>20 && nonzero_checked>5);
    CHECK(gain_peak==(direction>=2 && CAR_DRIVE_GAIN_SCHEDULE_ENABLED ? lateral_p(peak) : 150));
    printf("GAIN_MOTION direction=%s limit=%u peak_profile=%u peak_kp_x100=%u checked_outputs=%u nonzero_outputs=%u; model only\n",
        commands[direction],limit,peak,gain_peak,checks,nonzero_checked);
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(!mode){selector_test();scope_test();}
    else motion_test((mode-1)%4,mode<=4 ? 60 : mode<=8 ? 120 : 200);
    printf("PASS heading gain schedule: %u assertions; model only\n",assertions);return 0;
}
