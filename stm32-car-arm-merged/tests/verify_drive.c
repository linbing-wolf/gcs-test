/* 复用真实入口和独立车辆模型；对比数据只描述仿真，不代表实车改善幅度。 */
#define HEADING_TEST_MAIN heading_regression_main
#include "verify_heading.c"

#ifndef DRIVE_BASELINE_BENCHMARK
static void drive_core_tests(void)
{
    HeadingControl h;
    static const struct {int error,rate,sign;} cases[]={
        {-12,360,-1},{37,-510,1},{100,1000,-1},{-100,-1000,1},{79,0,1},{-79,0,-1}
    };
    unsigned i;
    memset(&h,0,sizeof(h));h.error_cdeg=10;h.rate_cdeg_s=30;
    CHECK(heading_control_drive_rpm(&h,20)==0);
    for(i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        int16_t rpm;
        h.error_cdeg=cases[i].error;h.rate_cdeg_s=cases[i].rate;
        rpm=heading_control_drive_rpm(&h,20);
        CHECK(rpm*cases[i].sign>0 && abs(rpm)<=20);
    }
    h.error_cdeg=10000;h.rate_cdeg_s=-10000;CHECK(heading_control_drive_rpm(&h,20)==20);
    h.error_cdeg=-10000;h.rate_cdeg_s=10000;CHECK(heading_control_drive_rpm(&h,20)==-20);
    h.error_cdeg=200;h.rate_cdeg_s=0;CHECK(heading_control_rpm(&h,60)==CAR_HEADING_MIN_RPM);
    h.error_cdeg=37;h.rate_cdeg_s=-510;CHECK(heading_control_rpm(&h,60)==0);
    reset_fixture();run.travel_sign=1;clock_ms=run.forward_profile_ms=10;
    for(i=0;i<30;i++) {
        int32_t before=run.forward_mrpm;
        clock_ms+=25;run_forward_slew(90);
        CHECK(run.forward_mrpm-before<=3000 && run.forward_mrpm>=before);
        CHECK(run.forward_mrpm<=90000);
    }
    CHECK(run.forward_mrpm==90000);
    clock_ms+=1000;CHECK(run_forward_slew(0)==84); /* 延迟后最多吸收50ms，不追补成一次骤停。 */
    run.forward_mrpm=0;run.travel_sign=-1;clock_ms=run.forward_profile_ms=UINT32_MAX-10u;
    clock_ms+=25;CHECK(run_forward_slew(-90)==-3);
    CHECK(run_forward_target(0)==0 && run_forward_target(1)==0);
    for(i=10;i<=780;i+=10) {
        double rpm=abs(run_forward_target((int32_t)i));
        double factor=CAR_POSITION_PULSES_PER_REV/(60.0*CALIBRATED_PPM_Y);
        double stop=rpm*factor*CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0
                    +rpm*rpm*factor/(2.0*CAR_DRIVE_DECEL_RPM_PER_S);
        CHECK(rpm<=CAR_RUN_RPM && stop<=i-CAR_DISTANCE_TOLERANCE_MM+0.01);
    }
    puts("PASS drive core: logged small-angle/rate cases, predictive damping sign, updated turn minimum with independent drive PD, bounded ramps, delayed tick, reverse, wrap and braking envelope");
}

static void drive_stop_and_log_tests(void)
{
    RoadPoint start;
    uint32_t limit;
    int saw_log=0;
    reset_plant(0,1);road_point(NODE_A,&start);
    field_x=planned_x=start.x;field_y=planned_y=start.y;
    send_line("runimu A");limit=clock_ms+20000;
    while(run_active && clock_ms<limit && run.forward_cmd_rpm<30) {
        int32_t before=run.forward_mrpm;
        uint32_t profile_time=run.forward_profile_ms;
        plant_step();
        if(run.forward_profile_ms!=profile_time)
            CHECK(abs(run.forward_mrpm-before)<=CAR_DRIVE_ACCEL_RPM_PER_S*CAR_DRIVE_SLEW_MAX_DT_MS);
        if(strstr(output," v_ref=")) {
            CHECK(strstr(output," turn_ref=") && strstr(output," wheel_cmd_rpm=")
                  && strstr(output," wheel_rpm_raw=") && strstr(output," control_dt_ms=")
                  && strstr(output," speed_age_max_ms=") && strstr(output," yaw_win="));
            saw_log=1;
        }
    }
    CHECK(run_active && run.forward_cmd_rpm>=30);
    while(run_active && !saw_log && clock_ms<limit) {plant_step();if(strstr(output," v_ref="))saw_log=1;}
    CHECK(saw_log);
    {uint32_t before=stop_count;
        stop_request();
        CHECK(!run_active && run.state==RUN_STOPPED && stop_count>=before+4);
        CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
    }
    reset_plant(0,1);road_point(NODE_A,&start);field_x=planned_x=start.x;field_y=planned_y=start.y;
    send_line("runimu A");
    while(run_active && !run.forward_cmd_rpm && clock_ms<20000)plant_step();
    CHECK(run_active && run.forward_cmd_rpm>0 && run.forward_cmd_rpm<=3);
    stop_request();
    puts("PASS drive execution: ramp applied at submit opportunities, command/feedback logs, immediate emergency bypass and fresh profile after restart");
}

static void drive_heading_window_tests(void)
{
    static const int offsets[3]={250,-175,0};
    int32_t mm[3];
    uint32_t ms[3],before_velocity,before_sync;
    int16_t target;
    RoadPoint start;
    char expected[160];
    unsigned i;
    reset_plant(17900,1);
    road_point(NODE_C,&start);field_x=planned_x=start.x;field_y=planned_y=start.y;initial_field_heading=2;
    send_line("runimu C");
    while(run_active && (run.travel_axis!=BODY_FORWARD || run.velocity_phase!=VELOCITY_FORWARD))plant_step();
    CHECK(run_active);
    target=wrapped_yaw(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg);
    memset(&run.heading_window,0,sizeof(run.heading_window));
    run.log_ms=clock_ms;
    for(i=0;i<3;i++) {
        clock_ms+=20;sample_yaw(wrapped_yaw(target+offsets[i]));firmware_step();
        CHECK(run_active && run.heading_control.error_cdeg==-offsets[i]);
        mm[i]=run.progress_mm;ms[i]=run.heading_control.sample_ms;
        firmware_step(); /* 同一IMU序号即使处理多次，也不重复计数。 */
    }
    before_velocity=velocity_count;before_sync=sync_count;
    output_size=0;output[0]=0;
    run.log_ms=clock_ms-CAR_CONTROL_LOG_MS+1;
    run_velocity_log();CHECK(!output_size); /* 到500ms前不额外打印。 */
    clock_ms++;run_velocity_log();
    snprintf(expected,sizeof(expected)," yaw_win=3,-250,%ld,%lu,175,%ld,%lu",
             (long)mm[0],(unsigned long)ms[0],(long)mm[1],(unsigned long)ms[1]);
    CHECK(strstr(output," error_cdeg=0 ") && strstr(output,expected));
    CHECK(velocity_count==before_velocity && sync_count==before_sync);
    CHECK(!run.heading_window.samples);
    firmware_step();CHECK(!run.heading_window.samples); /* 换窗口后不重记上一样本。 */
    clock_ms+=20;sample_yaw(target);firmware_step();
    CHECK(run.heading_window.samples==1);
    output_size=0;output[0]=0;run.log_ms=clock_ms-CAR_CONTROL_LOG_MS;run_velocity_log();
    CHECK(strstr(output," yaw_win=1,0,") && !strstr(output,",-250,"));
    stop_request();send_line("runimu C");CHECK(!run.heading_window.samples && !run.heading_window.seen);
    stop_request();

    reset_fixture();run.velocity_phase=VELOCITY_FORWARD;run.travel_axis=BODY_LEFT;
    clock_ms=UINT32_MAX-10u;run.log_ms=clock_ms;
    run.heading_control.sample_sequence=UINT32_MAX;run.heading_control.sample_ms=clock_ms;
    run.heading_control.error_cdeg=-320;run.progress_mm=30;run_heading_window_sample();
    clock_ms+=20;run.heading_control.sample_sequence=0;run.heading_control.sample_ms=clock_ms;
    run.heading_control.error_cdeg=40;run.progress_mm=53;run_heading_window_sample();
    clock_ms+=480;output_size=0;output[0]=0;run_velocity_log();
    CHECK(strstr(output," yaw_win=2,-320,30,4294967285,40,53,9"));
    puts("PASS heading windows: transient peaks survive zero-error log snapshot, signed extrema positions/timestamps, sample deduplication across windows, periodic output, restart, lateral and sequence/time wrap");
}

static void drive_window_segment_tests(void)
{
    unsigned done=0;
    int last_step=-1;
    RoadPoint start;
    reset_plant(0,1);
    road_point(NODE_C,&start);field_x=planned_x=start.x;field_y=planned_y=start.y;initial_field_heading=2;
    send_line("runimu C");
    while(run_active && done<2 && clock_ms<20000) {
        /* 将定期打印推迟，强制验证不足一个输出窗口也会在段末完整落盘。 */
        run.log_ms=clock_ms;
        plant_step();
        if(run.velocity_phase==VELOCITY_FORWARD && run.steps!=last_step) {
            CHECK(run.heading_window.samples<=1);
            last_step=run.steps;
        }
        if(strstr(output,"[control segment done]")) {
            const char *window=strstr(output," yaw_win=");
            CHECK(window && window<strstr(output,"[control segment done]"));
            done++;
        }
    }
    CHECK(done==2 && run_active);stop_request();
    puts("PASS heading window boundaries: lateral/forward segment-final flush and reset at next encoder baseline");
}
#endif

static void drive_dynamics_benchmark(void)
{
    unsigned plan;
    for(plan=0;plan<4;plan++) {
        RoadPoint start;
        double max_error[2]={0,0},cross[2]={0,0},initial_y[2]={0,0};
        uint8_t entered[2]={0,0};
        uint32_t deadline;
        reset_plant(17900,1.15);
        simulate_uart_cost=(uint8_t)(plan>=2);
        road_point((plan&1)?NODE_C:NODE_A,&start);
        planned_x=field_x=start.x;planned_y=field_y=start.y;initial_field_heading=(plan&1)?2:0;
        imu_period_ms=20;
        send_line((plan&1)?"runimu C":"runimu A");deadline=clock_ms+120000;
        while(run_active && run.edge<8 && clock_ms<deadline) {
            int leg=run.road.from==NODE_E && run.road.to==NODE_H ? 0
                    :run.road.from==NODE_H && run.road.to==NODE_E ? 1:-1;
            unsigned i;
            for(i=0;i<4;i++)wheel_acceleration[i]=0.18;
            if(leg>=0 && !run.turning && run.velocity_phase==VELOCITY_FORWARD) {
                /* 固定车身方向上的两组不同动态，模拟起步左偏、制动右偏的扰动来源。
                 * 这些值不是由操作者日志识别出的真实电机参数。 */
                wheel_acceleration[0]=wheel_acceleration[3]=0.14;
                wheel_acceleration[1]=wheel_acceleration[2]=0.20;
                if(!entered[leg]){entered[leg]=1;initial_y[leg]=field_y;}
            }
            plant_step();
            if(leg>=0 && entered[leg] && !run.turning && run.road.to==(leg?NODE_E:NODE_H)) {
                double error=fabs(run.heading_control.error_cdeg/100.0),drift=fabs(field_y-initial_y[leg]);
                if(error>max_error[leg])max_error[leg]=error;
                if(drift>cross[leg])cross[leg]=drift;
            }
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"Dynamics fault: %s\n",output);
        CHECK(run_active && run.edge>=8 && entered[0] && entered[1] && clock_ms<deadline);
#ifndef DRIVE_BASELINE_BENCHMARK
        CHECK(max_error[0]<3.0 && max_error[1]<3.0 && cross[0]<30.0 && cross[1]<30.0);
#endif
        printf("DYNAMICS %c uart_cost=%u E-H yaw_max=%.3fdeg cross_max=%.3fmm; H-E yaw_max=%.3fdeg cross_max=%.3fmm; simulated, not hardware\n",
               (plan&1)?'C':'A',simulate_uart_cost,max_error[0],cross[0],max_error[1],cross[1]);
        stop_request();
    }
}

int main(void)
{
#ifndef DRIVE_BASELINE_BENCHMARK
    drive_core_tests();drive_stop_and_log_tests();drive_heading_window_tests();drive_window_segment_tests();
#endif
    drive_dynamics_benchmark();
    continuous_route_tests();
    printf("PASS drive dynamics: %u assertions; independent wheel-response model, hardware accuracy unverified\n",assertions);
    return 0;
}
