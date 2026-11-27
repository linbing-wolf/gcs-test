#define HEADING_TEST_MAIN retained_forward_d_suite
#include "verify_heading.c"

/* 相同位置误差，向外移动应更强回拉，已向目标靠近应更早制动。 */
static int command_at_ten(float start,float end,int primary)
{
    reset_plant(0,1.15);diagnostic_init();run.translation_only=1;run.travel_axis=BODY_LEFT;run.lateral_rpm=120;
    clock_ms=1000;run.lateral_pose.sample_ms=clock_ms;
    sample_yaw(0);run_lateral_yaw_sample(heading_snapshot());run_lateral_pose_observe(start,0,clock_ms);
    clock_ms=1100;sample_yaw(0);run_lateral_yaw_sample(heading_snapshot());run_lateral_pose_observe(end,0,clock_ms);
    run.straight_hold.lateral_mm=(int32_t)lroundf(end);run.straight_hold.valid=1;run.straight_hold.sample_ms=clock_ms;
    run.control_ms=clock_ms-50;
    return run_straight_correction((int16_t)primary,0,1);
}
int main(void)
{
    int stationary=command_at_ten(10,10,100),away=command_at_ten(8,10,100),toward=command_at_ten(12,10,100);
    printf("FORWARD_D same_error10 stationary=%d away=%d toward=%d\n",stationary,away,toward);
    CHECK(away<stationary && toward>stationary);
    CHECK(command_at_ten(8,10,0)<command_at_ten(10,10,0)); /* 主横移归零仍有D。 */
    CHECK(command_at_ten(-8,-10,100)>command_at_ten(-10,-10,100));
    CHECK(abs(command_at_ten(100,100,0))<=CAR_LATERAL_HOLD_MAX_RPM);
    CHECK(fabs(run.lateral_pose.forward_rate_mm_s)<0.01); /* 初始大偏移不能产生差分尖峰。 */
    (void)command_at_ten(8,10,100);
    {float rate=run.lateral_pose.forward_rate_mm_s;
     run_lateral_pose_observe(10,0,clock_ms);CHECK(run.lateral_pose.forward_rate_mm_s==rate);
     for(unsigned i=0;i<10;i++) {
         clock_ms+=100;sample_yaw(0);run_lateral_yaw_sample(heading_snapshot());run_lateral_pose_observe(10,0,clock_ms);
     }
     CHECK(fabs(run.lateral_pose.forward_rate_mm_s)<0.02);}
    CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    puts("PASS forward position damping; independent moving-away/approaching response");return 0;
}
