#define HEADING_TEST_MAIN retained_manual_stop_suite
#include "verify_heading.c"

static void stopped_fixture(unsigned turn)
{
    reset_plant(0,1.15);diagnostic_init();clock_ms=1000;
    run_active=run.imu_mode=1;run.translation_only=!turn;run.turn_only=turn;
    run.velocity_phase=turn ? VELOCITY_TURN : VELOCITY_STOP_FORWARD;
    run.zero_ms=900;run.velocity_waiting=0;
    for(unsigned i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        f->position_valid=f->speed_valid=1;f->status_replies=1;
        f->position_ms=f->speed_ms=f->status_ms=clock_ms;f->speed_rpm=0;
    }
}
static void each_wheel_test(void)
{
    const int speeds[]={-7,-4,-2,2,4,7};
    for(unsigned turn=0;turn<2;turn++)for(unsigned wheel=0;wheel<4;wheel++) {
        stopped_fixture(turn);CHECK(run_velocity_stopped());
        for(unsigned v=0;v<sizeof(speeds)/sizeof(speeds[0]);v++) {
            motor_feedback[wheel].speed_rpm=speeds[v];
            CHECK(!run_velocity_stopped());
            CHECK(run_active && run.state!=RUN_FAULT);
        }
        motor_feedback[wheel].speed_rpm=CAR_VELOCITY_STOP_RPM;CHECK(run_velocity_stopped());
        motor_feedback[wheel].speed_rpm=-CAR_VELOCITY_STOP_RPM;CHECK(run_velocity_stopped());
    }
    /* 本次只收紧手动会话，不改变被诊断入口禁用的旧工位判据。 */
    stopped_fixture(0);run.station_shift=run.station_enabled=1;motor_feedback[0].speed_rpm=7;
    CHECK(run_velocity_stopped());
}
static void fresh_stop_test(void)
{
    for(unsigned turn=0;turn<2;turn++)for(unsigned wheel=0;wheel<4;wheel++) {
        stopped_fixture(turn);motor_feedback[wheel].speed_ms=run.zero_ms;
        CHECK(!run_velocity_stopped()); /* 同一毫秒不是零速请求之后的新速度证据。 */
        motor_feedback[wheel].speed_ms=run.zero_ms-1;CHECK(!run_velocity_stopped());
        motor_feedback[wheel].speed_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;CHECK(!run_velocity_stopped());
        motor_feedback[wheel].speed_ms=clock_ms;CHECK(run_velocity_stopped());
    }
}
static void stop_hold_test(unsigned direction,unsigned cancel)
{
    const char *commands[]={"f","b","l","r","left90","right90"};
    uint32_t deadline,hold_started=0;unsigned injected=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    send_line(direction<4 ? "test" : "turn");send_line("dist 300");send_line("rpm 60");send_line(commands[direction]);
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        if(!injected && (direction<4 ? run.progress_mm>100 : abs(run.heading_control.progress_cdeg)>1000)) {
            moving_at_stop_motor=2;injected=1;
        }
        plant_step();
        if(injected && run.zero_ms && motor_feedback[1].speed_rpm==16) {
            if(!hold_started)hold_started=clock_ms;
            if(clock_ms-hold_started>=500)break;
        }
    }
    CHECK(injected && hold_started && run_active && run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline);
    for(unsigned i=0;i<4;i++)CHECK(!run.wheel_rpm[i]);
    CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    if(cancel) {
        queue_byte('!');plant_step();CHECK(!run_active && run.state==RUN_STOPPED && arm_stops==2);
        for(unsigned i=0;i<100;i++)plant_step();
        CHECK(!run_active);
    } else {
        moving_at_stop_motor=0;deadline=clock_ms+3000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && run.state==RUN_DONE && clock_ms<deadline);
        for(unsigned i=0;i<4;i++) {
            CHECK(abs(motor_feedback[i].speed_rpm)<=CAR_VELOCITY_STOP_RPM);
            CHECK((int32_t)(motor_feedback[i].speed_ms-run.zero_ms)>0);
        }
    }
    printf("PASS manual stop %s cancel=%u: outstanding wheel speed retains task, fresh stop then completes/cancels\n",commands[direction],cancel);
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(mode==0)each_wheel_test();
    else if(mode==1)fresh_stop_test();
    else if(mode==2)for(unsigned d=0;d<6;d++)stop_hold_test(d,0);
    else stop_hold_test(0,1);
    printf("PASS manual stop regression%u: %u assertions; model only\n",mode,assertions);return 0;
}
