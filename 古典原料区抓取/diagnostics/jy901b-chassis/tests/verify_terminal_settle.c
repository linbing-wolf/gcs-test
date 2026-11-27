/* 真实停车路径重放及独立惯性模型；不连接车辆。 */
#define HEADING_TEST_MAIN retained_heading_suite
#include "verify_heading.c"

static void endpoint_fixture(BodyAxis axis,int error,int rate)
{
    uint32_t deadline;
    reset_plant(0,1.15);CHECK(start_heading_shift(810,200));run.travel_axis=axis;
    deadline=clock_ms+10000;
    while(run_active && run.progress_mm<100 && clock_ms<deadline)plant_step();
    CHECK(run_active && run.velocity_phase==VELOCITY_FORWARD);
    CHECK(run_velocity_distance());run.action_mm=(uint16_t)run.progress_mm;
    run.continuation_mm=0;run.velocity_phase=VELOCITY_STOP_FORWARD;
    run.forward_mrpm=0;run.forward_profile_ms=clock_ms;
    run.turn_cmd_rpm=0;run.velocity_waiting=0;
    run.heading_control.error_cdeg=error;run.heading_control.rate_cdeg_s=rate;
    run.heading_control.stable_samples=0;
    run.control_ms=clock_ms-CAR_VELOCITY_CONTROL_MS;
}

static void command_replay(int baseline)
{
    unsigned axis,mirror;
    for(axis=0;axis<2;axis++)for(mirror=0;mirror<2;mirror++) {
        int sign=mirror ? -1 : 1;
        endpoint_fixture(axis ? BODY_FORWARD : BODY_LEFT,165*sign,1000*sign);
        run_velocity_update();
        printf("LOG_REPLAY axis=%u error=%d rate=%d turn=%d\n",axis,165*sign,1000*sign,run.turn_cmd_rpm);
        if(!baseline)CHECK(run.turn_cmd_rpm==sign); /* 原PD约1RPM，被10RPM下限抬高；修复后保留小速度制动。 */
        endpoint_fixture(axis ? BODY_FORWARD : BODY_LEFT,50*sign,1000*sign);
        run_velocity_update();
        if(!baseline)CHECK(run.turn_cmd_rpm==-sign); /* 角度已入窗，但角速度未稳，仍应制动。 */
        endpoint_fixture(axis ? BODY_FORWARD : BODY_LEFT,50*sign,0);
        run_velocity_update();CHECK(!run.turn_cmd_rpm);
    }
    endpoint_fixture(BODY_LEFT,0,0);run.velocity_phase=VELOCITY_FORWARD;
    printf("FINE_TARGET remaining=2 rpm=%d\n",run_forward_target(2));
    if(!baseline)CHECK(run_forward_target(2)==1 && run_forward_target(-2)==-1);
    CHECK(!run_forward_target(1) && !run_forward_target(-1));
    run.travel_axis=BODY_FORWARD;CHECK(run_forward_target(2)==CAR_DISTANCE_MIN_RPM);
    run.travel_axis=BODY_LEFT;run.station_shift=1;CHECK(run_forward_target(2)==CAR_DISTANCE_MIN_RPM);
    run.heading_control.error_cdeg=165;run.heading_control.rate_cdeg_s=1000;
    CHECK(heading_control_rpm(&run.heading_control,300)==CAR_HEADING_MIN_RPM);
    CHECK(CAR_DISTANCE_TOLERANCE_MM==1 && CAR_HEADING_TOLERANCE_CDEG==80);
}

static void scope_and_slew_tests(void)
{
    unsigned kind,i;
    for(kind=0;kind<4;kind++) {
        endpoint_fixture(BODY_LEFT,165,1000);
        if(kind==0)run.station_shift=1;
        if(kind==1)run.vector_move=1;
        if(kind==2)run.obstacle_phase=2;
        if(kind==3) {run.velocity_phase=VELOCITY_TURN;run.turn_only=run.turning=1;}
        run_velocity_update();CHECK(run.turn_cmd_rpm==CAR_HEADING_MIN_RPM);
    }
    endpoint_fixture(BODY_LEFT,600,0);run.turn_cmd_rpm=2;
    for(i=0;i<30;i++) {
        int previous=run.turn_cmd_rpm;
        run.heading_control.error_cdeg=i<6 ? 600 : -600;
        /* 固定位置与IMU样本，仅隔离反馈导致phase切换时的输出连续性。 */
        heading_sensor.sample.received_ms=clock_ms;run.pending=0;
        for(unsigned wheel=0;wheel<4;wheel++) {
            motor_feedback[wheel].speed_ms=motor_feedback[wheel].status_ms=clock_ms;
            motor_feedback[wheel].position_ms=clock_ms;
        }
        run.velocity_waiting=0;run.control_ms=clock_ms-CAR_VELOCITY_CONTROL_MS;
        run_velocity_update();
        printf("SLEW_REPLAY i=%u before=%d after=%d phase=%u left=%d error=%ld\n",i,previous,run.turn_cmd_rpm,run.velocity_phase,run.left_cmd_rpm,(long)run.heading_control.error_cdeg);
        CHECK(abs(run.turn_cmd_rpm-previous)<=2);
        CHECK(abs(run.turn_cmd_rpm)<=CAR_HEADING_DRIVE_TRIM_RPM);
        previous=run.turn_cmd_rpm;run_velocity_update();CHECK(run.turn_cmd_rpm==previous);
        clock_ms+=CAR_VELOCITY_CONTROL_MS;
    }
    CHECK(run.turn_cmd_rpm<0);
}

static void inertia_cases(int baseline)
{
    unsigned axis,direction,lag;
    for(axis=0;axis<2;axis++)for(direction=0;direction<2;direction++)for(lag=0;lag<3;lag++) {
        uint32_t start,deadline,terminal=0;int disturbed=0,last_turn=0,last_left=0;
        unsigned turn_reversals=0,left_reversals=0;double max_yaw=0;
        reset_plant(0,lag==2 ? 1.5 : 1.15);imu_period_ms=20;
        CHECK(start_heading_shift(direction ? -1620 : 1620,200));
        run.travel_axis=axis ? BODY_FORWARD : BODY_LEFT;
        if(lag==1)wheel_acceleration[1]=0.12;
        if(lag==2)for(unsigned i=0;i<4;i++)wheel_acceleration[i]=0.12;
        start=clock_ms;deadline=start+45000;
        while(run_active && clock_ms<deadline) {
            if(!disturbed && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>1540) {
                physical_yaw+=direction ? -600 : 600;disturbed=1;terminal=clock_ms;
            }
            plant_step();
            if(disturbed) {
                int turn=run.turn_cmd_rpm>0 ? 1 : run.turn_cmd_rpm<0 ? -1 : 0;
                int left=run.left_cmd_rpm>0 ? 1 : run.left_cmd_rpm<0 ? -1 : 0;
                if(turn && last_turn && turn!=last_turn)turn_reversals++;
                if(left && last_left && left!=last_left)left_reversals++;
                if(turn)last_turn=turn;
                if(left)last_left=left;
                if(fabs(physical_yaw)>max_yaw)max_yaw=fabs(physical_yaw);
            }
        }
        printf("SETTLE baseline=%d axis=%u direction=%u lag=%u done=%u mm=%ld yaw=%.2f settle_ms=%lu turn_reversals=%u lateral_reversals=%u peak_yaw=%.2f\n",
            baseline,axis,direction,lag,run.state==RUN_DONE,(long)run.progress_mm,physical_yaw/100,
            (unsigned long)(clock_ms-terminal),turn_reversals,left_reversals,max_yaw/100);
        CHECK(disturbed && !run_active && run.state==RUN_DONE && clock_ms<deadline);
        if(!axis)CHECK(abs(run.progress_mm-1620)<=CAR_DISTANCE_TOLERANCE_MM);
        else CHECK(run.progress_mm>=1620-CAR_DISTANCE_TOLERANCE_MM); /* 原前后单轴不反向修复超程。 */
        CHECK(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
        CHECK(abs(run.heading_control.rate_cdeg_s)<=CAR_HEADING_STABLE_RATE_CDEG_S);
        if(!baseline)CHECK(clock_ms-terminal<8000);
    }
}

int main(int argc,char **argv)
{
    int baseline=argc>1 && !strcmp(argv[1],"baseline");
    command_replay(baseline);
    if(!baseline)scope_and_slew_tests();
    inertia_cases(baseline);
    if(!baseline) {continuous_turn_tests();lateral_heading_damping_tests();}
    printf("PASS terminal settle: %lu assertions; current speed tiers and arrival windows retained; host model only\n",(unsigned long)assertions);
    return 0;
}
