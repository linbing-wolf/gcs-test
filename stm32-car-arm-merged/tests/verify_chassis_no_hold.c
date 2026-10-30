/* 真实主控/航向模块：取消额外稳定保持，保留新鲜反馈、角度/角速度及机械臂时序。 */
#define STATION_TEST_MAIN retained_chassis_hold_station_suite
#include "verify_station_vision.c"

static void first_heading_sample_tests(void)
{
    HeadingControl c;HeadingSample sample;
    reset_heading_fixture();clock_ms=1000;sample_yaw(0);sample=heading_snapshot();
    CHECK(heading_control_start(&c,sample,0,clock_ms));
    CHECK(heading_control_update(&c,sample,clock_ms)==HEADING_WAIT);
    clock_ms+=10;sample_yaw(0);sample=heading_snapshot();
    CHECK(heading_control_update(&c,sample,clock_ms)==HEADING_DONE && c.stable_samples==1);
    clock_ms+=10;sample_yaw(50);sample=heading_snapshot();
    CHECK(heading_control_update(&c,sample,clock_ms)==HEADING_CORRECT); /* 角度内但仍在快速转动。 */
    clock_ms+=10;sample_yaw(50);sample=heading_snapshot();
    CHECK(heading_control_update(&c,sample,clock_ms)==HEADING_DONE);
    clock_ms+=10;sample_yaw(200);sample=heading_snapshot();
    CHECK(heading_control_update(&c,sample,clock_ms)==HEADING_CORRECT);
    CHECK(heading_control_update(&c,sample,clock_ms+CAR_IMU_MAX_AGE_MS+1)==HEADING_FAULT);
    puts("PASS heading: first fresh in-window low-rate sample completes; repeated/stale/out-of-window/high-rate samples do not");
}
static void zero_feedback_tests(void)
{
    reset_heading_fixture();clock_ms=1000;run.zero_ms=clock_ms;
    for(unsigned i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];f->position_valid=f->speed_valid=1;f->status_replies=1;
        f->position_ms=f->speed_ms=f->status_ms=clock_ms;f->status_flags=MOTOR_FLAG_ENABLED;
    }
    motor_feedback[0].speed_ms=clock_ms-1;CHECK(!run_velocity_stopped());
    motor_feedback[0].speed_ms=clock_ms;CHECK(run_velocity_stopped());
    run.obstacle_phase=1;motor_feedback[0].speed_rpm=CAR_VELOCITY_STOP_RPM+1;
    CHECK(!run_velocity_stopped());motor_feedback[0].speed_rpm=0;CHECK(run_velocity_stopped());
    motor_feedback[0].position_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;CHECK(!run_velocity_stopped());
    puts("PASS zero feedback: first fresh post-zero reply completes without100/300ms delay; old/stale replies and obstacle moving wheels still block");
}
static void interlock_tests(void)
{
    reset_heading_fixture();clock_ms=1000;arm_lock.state=1;arm_lock.started_ms=clock_ms;
    for(unsigned i=0;i<4;i++) {
        arm_lock.seen[i]=3;motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
    }
    arm_lock.seen[3]=1;arm_lock_update();CHECK(arm_lock.state==1);
    arm_lock.seen[3]=3;arm_lock_update();CHECK(arm_lock.state==2 && arm_enabled());
    CHECK(ARM_CUP_EXIT_DESCENT_DELAY_MS==200u && ARM_YAW_SETTLE_MS==40u && ARM_CLAW_CLOSE_HOLD_MS==50u);
    stop_request();CHECK(arm_failed() && !run_active);
    puts("PASS interlock: first complete fresh four-wheel reply acquires without200ms hold; missing reply blocks; arm200/40/50ms and manual ! retained");
}
static void rough_turn_overlap_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned batch=0;batch<2;batch++) {
        unsigned offset=0,last_job=0,positions,job=batch ? 5u : 2u;uint32_t deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+200000;
        while(clock_ms<deadline && !(run.job==job && run.station_align_pending && run.velocity_phase==VELOCITY_TURN)) {
            serve_peer(&offset);plant_step();if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && abs(run.heading_control.target_cdeg)>=17900);positions=arm_positions;
        while(arm_yaw_us==1975 && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && run.station_entry_overlap && run.turning);
        CHECK(abs(run.heading_control.progress_cdeg)<9000 && run.turn_cmd_rpm && arm_yaw_us!=1975 && arm_positions==positions);
        stop_request();CHECK(!run_active && !station_active() && arm_failed());
    }
    puts("PASS both rough180 visits on A/C: actual observation PWM still starts during chassis turn; ! cancels");
}
int main(void)
{
    first_heading_sample_tests();zero_feedback_tests();interlock_tests();rough_turn_overlap_tests();
    printf("PASS chassis no hold: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
