/* 行驶纠偏独立开关：真实命令/控制器，CAN与IMU主机模型。 */
#define STATION_TEST_MAIN retained_drive_loops_station_suite
#include "verify_station_vision.c"

static void command_tests(void)
{
    unsigned before;
    reset_fixture();before=velocity_batches;
    send_line("loop status");
    CHECK(strstr(output,CAR_ENCODER_DRIFT_DEFAULT_ON ? "encoder=ON" : "encoder=OFF"));
    CHECK(strstr(output,CAR_GYRO_HEADING_DEFAULT_ON ? "gyro=ON" : "gyro=OFF"));
    for(unsigned e=0;e<2;e++)for(unsigned g=0;g<2;g++) {
        send_line(e ? "loop encoder on" : "loop encoder off");
        send_line(g ? "loop gyro on" : "loop gyro off");
        CHECK(encoder_drift_feedback_enabled==e && gyro_drive_feedback_enabled==g);
        send_line("loop");CHECK(strstr(output,e ? "encoder=ON" : "encoder=OFF"));
        CHECK(strstr(output,g ? "gyro=ON" : "gyro=OFF"));
        send_line("loop gyro maybe");CHECK(strstr(output,"loop rejected"));
        CHECK(encoder_drift_feedback_enabled==e && gyro_drive_feedback_enabled==g);
    }
    send_line("loop gyro off");send_line("loop encoder off");
    send_line("test");send_line("nav");send_line("stop");
    CHECK(!encoder_drift_feedback_enabled && !gyro_drive_feedback_enabled);
    CHECK(velocity_batches==before && !position_count);
    reset_fixture();CHECK(encoder_drift_feedback_enabled==CAR_ENCODER_DRIFT_DEFAULT_ON);
    CHECK(gyro_drive_feedback_enabled==CAR_GYRO_HEADING_DEFAULT_ON);
    puts("PASS serial: independent combinations, status, invalid input, mode/stop persistence and reset defaults");
}

static int16_t settled_cross(int16_t forward,int error,int valid)
{
    int16_t result=0;
    run.travel_axis=BODY_FORWARD;run.straight_hold.lateral_mm=error;
    run.straight_hold.valid=(uint8_t)valid;
    for(unsigned i=0;i<6;i++) {
        clock_ms+=50;run.control_ms=clock_ms-50;run.straight_hold.sample_ms=clock_ms;
        result=run_straight_correction(forward,0,1);
    }
    return result;
}

static void encoder_tests(void)
{
    for(int sign=-1;sign<=1;sign+=2) {
        int16_t ff=(int16_t)lroundf(car_straight_left_trim(sign*100)*CALIBRATED_PPM_X/car_road_ppm_y(sign));
        reset_fixture();send_line("loop encoder off");
        CHECK(settled_cross((int16_t)(sign*100),20,0)==ff);
        CHECK(settled_cross((int16_t)(sign*100),-20,1)==ff);
        send_line("loop encoder on");CHECK(settled_cross((int16_t)(sign*100),20,1)<ff);
        CHECK(settled_cross((int16_t)(sign*100),-20,1)>ff);
        send_line("loop encoder off");CHECK(settled_cross((int16_t)(sign*100),20,1)==ff);
        CHECK(!settled_cross(0,20,1));
    }
    puts("PASS encoder toggle: both travel signs, feedback direction, fixed feed-forward retained, no extra end movement");
}

static void translation_tests(void)
{
    const char *directions[]={"f","b","l","r"};
    for(unsigned direction=0;direction<4;direction++)for(unsigned e=0;e<2;e++)for(unsigned g=0;g<2;g++) {
        unsigned disturbed=0,corrected=0;uint32_t deadline;int target;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        send_line(e ? "loop encoder on" : "loop encoder off");
        send_line(g ? "loop gyro on" : "loop gyro off");
        send_line("test");send_line("dist 400");send_line("rpm 100");send_line(directions[direction]);
        CHECK(run_active);target=run.heading_control.target_cdeg;deadline=clock_ms+18000;
        while(run_active && clock_ms<deadline) {
            if(!disturbed && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>60) {
                physical_yaw+=400;disturbed=1;
            }
            plant_step();
            if(run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD) {
                if(!g)CHECK(!run.turn_cmd_rpm && !run.turn_ref_rpm);
                else if(disturbed && run.turn_cmd_rpm<0)corrected=1;
            }
        }
        CHECK(clock_ms<deadline && !run_active && run.state==RUN_DONE && disturbed);
        CHECK(abs(run.progress_mm-400)<=CAR_DISTANCE_TOLERANCE_MM+4);
        CHECK(run.heading_control.target_cdeg==target);
        if(g)CHECK(corrected && fabs(physical_yaw)<=CAR_HEADING_TOLERANCE_CDEG+10);
        else CHECK(fabs(physical_yaw-400)<1); /* 已到距离但航向仍偏4度，也应完成。 */
        printf("PASS translation %s encoder=%u gyro=%u: distance stop, heading policy, fixed reference\n",directions[direction],e,g);
    }
}

static void live_toggle_and_turn_tests(void)
{
    uint32_t deadline;int target;unsigned corrected=0;
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    send_line("loop gyro off");send_line("test");send_line("dist 600");send_line("rpm 100");send_line("f");
    deadline=clock_ms+18000;
    while(run_active && run.progress_mm<100 && clock_ms<deadline)plant_step();
    CHECK(run_active && clock_ms<deadline);target=run.heading_control.target_cdeg;
    physical_yaw+=400;
    for(unsigned i=0;i<100;i++) {plant_step();CHECK(!run.turn_cmd_rpm);}
    send_line("loop gyro on");CHECK(run.heading_control.target_cdeg==target);
    while(run_active && clock_ms<deadline) {plant_step();if(run.turn_cmd_rpm<0)corrected=1;}
    CHECK(!run_active && run.state==RUN_DONE && corrected);
    for(unsigned right=0;right<2;right++) {
        reset_plant(0,1.15);send_line("loop gyro off");send_line("loop encoder off");
        send_line("turn");send_line("rpm 60");send_line(right ? "right90" : "left90");
        deadline=clock_ms+20000;corrected=0;
        while(run_active && clock_ms<deadline) {plant_step();if(run.turn_cmd_rpm)corrected=1;}
        CHECK(!run_active && run.state==RUN_DONE && corrected);
        CHECK(fabs(physical_yaw-(right ? -9000 : 9000))<=CAR_HEADING_TOLERANCE_CDEG+10);
        CHECK(!gyro_drive_feedback_enabled && !encoder_drift_feedback_enabled);
    }
    puts("PASS live gyro enable: original heading recovered; both90-degree angle turns retained with drive feedback OFF");
}

static void production_and_feedback_tests(void)
{
    RoadPoint start;uint32_t deadline;unsigned disturbed=0,finished=0;
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    CHECK(road_point(NODE_A,&start));planned_x=field_x=start.x;planned_y=field_y=start.y;
    initial_field_heading=HEAD_X_NEG;
    send_line("loop encoder off");send_line("loop gyro off");send_line("runimu A");
    deadline=clock_ms+60000;
    while(run_active && clock_ms<deadline) {
        if(!disturbed && run.edge==1 && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>60) {
            physical_yaw+=400;disturbed=1;
        }
        plant_step();
        if(run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)
            CHECK(!run.turn_cmd_rpm && !run.turn_ref_rpm);
        if(disturbed && strstr(output,"[control segment done]")) {finished=1;break;}
    }
    CHECK(clock_ms<deadline && run_active && disturbed && finished);
    CHECK(!run.translation_only && !run.corner.active);
    CHECK(CAR_RUN_RPM==250 && CAR_LATERAL_SHORT_RPM==250 && CAR_LATERAL_LONG_RPM==200);
    stop_request();
    reset_plant(0,1.15);send_line("loop encoder off");send_line("loop gyro off");
    send_line("test");send_line("dist 400");send_line("f");deadline=clock_ms+10000;
    while(run_active && run.progress_mm<60 && clock_ms<deadline)plant_step();
    CHECK(run_active && clock_ms<deadline);
    {
        int32_t progress=run.progress_mm;unsigned batches=velocity_batches;
        clock_ms+=CAR_IMU_MAX_AGE_MS+1;run.pending=run.velocity_waiting=0;run.warning_seen=0;run_velocity_update();
        CHECK(run_active && run.progress_mm==progress && velocity_batches==batches);
        CHECK(strstr(output,"IMU sample stale/invalid"));
        sample_yaw(wrapped_yaw(physical_yaw));output_size=0;output[0]=0;run.warning_seen=0;run.pending=0;run_velocity_update();
        CHECK(run_active && run.progress_mm==progress && velocity_batches==batches);
        CHECK(strstr(output,"four-wheel feedback incomplete/stale"));
    }
    puts("PASS production runimu A: switches applied to route, race speeds retained; OFF still requires fresh IMU and encoders");
}

int main(void)
{
    command_tests();encoder_tests();translation_tests();live_toggle_and_turn_tests();production_and_feedback_tests();
    printf("PASS drive loops: %u assertions; host model only\n",assertions);
    return 0;
}
