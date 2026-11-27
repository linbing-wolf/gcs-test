#define HEADING_TEST_MAIN retained_lateral_hold_heading_suite
#include "verify_heading.c"

static double lateral_run(unsigned right,unsigned enabled)
{
    uint32_t deadline;unsigned corrected=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    wheel_speed_scale[0]=wheel_speed_scale[2]=1.03;
    wheel_speed_scale[1]=wheel_speed_scale[3]=0.97;
    send_line("loop gyro off");send_line("loop encoder off");
    send_line(enabled ? "loop forward on" : "loop forward off");
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(right ? "r" : "l");
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        plant_step();if(run.forward_cmd_rpm)corrected=1;
        CHECK(abs(run.forward_cmd_rpm)<=CAR_LATERAL_HOLD_MAX_RPM && !run.turn_cmd_rpm);
        CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    }
    CHECK(!run_active && run.state==RUN_DONE && abs(run.progress_mm-2000)<=2);
    if(enabled)CHECK(corrected);else CHECK(!corrected);
    printf("MODEL lateral right=%u hold=%u forward_drift_mm=%.2f\n",right,enabled,field_y);
    return fabs(field_y);
}
static void wheel_priority_test(void)
{
    for(int side=-1;side<=1;side+=2)for(int turn=-20;turn<=20;turn+=40) {
        reset_plant(0,1.15);run_active=run.imu_mode=1;run.travel_axis=BODY_LEFT;run.lateral_rpm=200;
        CHECK(run_velocity_send(8,(int16_t)(side*200),(int16_t)turn));
        CHECK((run.wheel_rpm[0]+run.wheel_rpm[1]+run.wheel_rpm[2]+run.wheel_rpm[3])/4==turn);
        for(unsigned i=0;i<4;i++)CHECK(abs(run.wheel_rpm[i])<=200);
    }
}
int main(void)
{
    for(unsigned right=0;right<2;right++) {
        double before=lateral_run(right,0),after=lateral_run(right,1);
        CHECK(before>20 && after<before*0.65);
    }
    wheel_priority_test();
    printf("PASS lateral hold/rotation priority: %u assertions; unequal-wheel model only, no physical acceptance\n",assertions);
    return 0;
}
