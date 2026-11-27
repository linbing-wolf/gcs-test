/* 用字面量回放同一段CPOS，核对左右独立标定与非目标作用域。 */
#define HEADING_TEST_MAIN retained_lateral_direction_heading_suite
#include "verify_heading.c"

static void distance_test(int sign,float ppm)
{
    unsigned i;
    reset_plant(0,1.15);
    run.imu_mode=1;run.translation_only=1;run.travel_axis=BODY_LEFT;run.travel_sign=(int8_t)sign;
    CHECK(fabsf(run_single_axis_ppm()-ppm)<0.0001f);
    gyro_drive_feedback_enabled=0;
    run.encoder_ready=ALL_MOTORS_MASK;
    for(i=0;i<4;i++) {
        run.encoder_sign[i]=1;run.encoder_start[i]=0;
        motor_feedback[i].position=(int64_t)llround(2000.0*ppm*65536.0/3200.0);
    }
    CHECK(run_velocity_distance());CHECK(abs(run.progress_mm-2000)<=1);
    run.station_shift=1;CHECK(run_single_axis_ppm()==10.92f);run.station_shift=0;
    run.obstacle_phase=1;CHECK(fabsf(run_single_axis_ppm()-ppm)<0.0001f);
    run.obstacle_phase=2;CHECK(fabsf(run_single_axis_ppm()-ppm)<0.0001f);run.obstacle_phase=0;
    run.travel_axis=BODY_FORWARD;
    CHECK(run_single_axis_ppm()==(sign>0 ? 10.00f : 9.98f));
}
int main(void)
{
    distance_test(1,11.495f);distance_test(-1,11.136f);
    printf("PASS independent lateral calibration: left11.495/right11.136; CPOS, obstacle stop/return continuity, station/forward scopes; %u assertions; model only\n",assertions);
    return 0;
}
