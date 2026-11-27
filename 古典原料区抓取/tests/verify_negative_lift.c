/* 真实串口入口与轴反馈模型：允许0以下目标，但保留最高135和原净空。 */
#define STATION_TEST_MAIN inherited_negative_lift_station_tests
#include "verify_station_vision.c"

static void expect_pose(int expected_slide,int expected_lift,float expected_yaw,float expected_claw)
{
    int slide,lift;float yaw,claw;arm_pose_now(&slide,&lift,&yaw,&claw);
    if(lift!=expected_lift)fprintf(stderr,"lift expected=%d actual=%d\n",expected_lift,lift);
    CHECK(arm_ready() && slide==expected_slide && lift==expected_lift && yaw==expected_yaw && claw==expected_claw);
}
int main(void)
{
    unsigned before,down_dir;
#ifdef NEGATIVE_STATION_REJECT
    prepare_arm_auto(0);send_line("task 134+123+314+231");
    CHECK(station_start(NODE_D,2,clock_ms)<0 && !arm_positions);
    puts("PASS station height below -20 rejected before actuation");
    return 0;
#endif
    prepare_arm_auto(0);
    send_line("arm lift 0");tick(2500);expect_pose(0,0,0,45);
    down_dir=arm_trace_dir[0];before=arm_positions;
    send_line("arm lift -5");tick(2500);expect_pose(0,-5,0,45);
    CHECK(arm_positions==before+1 && arm_trace_address[before]==5
        && arm_trace_pulses[before]==400 && arm_trace_dir[before]==down_dir);
    before=arm_positions;send_line("arm lift -20");tick(2500);expect_pose(0,-20,0,45);
    CHECK(arm_positions==before+1 && arm_trace_pulses[before]==1200);
    before=arm_positions;send_line("arm lift -21");send_line("arm lift 136");tick(100);
    CHECK(arm_positions==before && arm_ready());expect_pose(0,-20,0,45);
    send_line("arm home");tick(5000);expect_pose(0,135,0,45);
    CHECK(arm_trace_pulses[before]==12400 && arm_trace_dir[before]!=down_dir);
    CHECK(ARM_LIFT_MAX_MM==135 && ARM_LIFT_HOME_MM==135 && ARM_CLEARANCE_MM==125);
    puts("PASS negative lift: 0 -> -5 -> -20, correct downward pulses; -21/136 rejected; signed return135 and clearance125 retained");
    return 0;
}
