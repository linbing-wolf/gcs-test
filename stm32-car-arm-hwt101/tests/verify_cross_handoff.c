/* 比赛普通横移末段：使用主控与编码器/IMU替身，注入前后残差。 */
#define HEADING_TEST_MAIN retained_cross_handoff_suite
#include "verify_heading.c"

static void inject_forward_error(int mm)
{
    static const int8_t forward_sign[4]={-1,1,1,-1};
    for(unsigned i=0;i<4;i++)
        encoder_value[i]+=mm*CALIBRATED_PPM_Y*CAR_ENCODER_COUNTS_PER_REV
            /CAR_POSITION_PULSES_PER_REV*forward_sign[i]*encoder_polarity[i];
}

static void cross_handoff_test(int work, int bias)
{
    RoadPoint origin;
    char pose[64];
    uint32_t deadline;
    int injected=0,stopped_with_error=0;
    reset_plant(0,1.15);
    translation_yaw_disturbance=0;
    send_line("imutest extra 0");
    send_line("arm on");
    for(unsigned ms=0;ms<500;ms++)plant_step();
    snprintf(pose,sizeof(pose),"arm pose %d %d %.1f %.1f",
        ARM_SLIDE_HOME_MM,ARM_LIFT_HOME_MM,(double)ARM_YAW_HOME_DEG,(double)ARM_CLAW_OPEN_DEG);
    send_line(pose);
    snprintf(pose,sizeof(pose),"arm platform pose %.1f",(double)ARM_PLATFORM_FIRST_DEG);
    send_line(pose);
    CHECK(arm_ready() && arm_parked() && arm_platform_slot()==1);
    run.edge=work ? 4u : 5u;run.job=checked_job=work ? 2u : 3u;
    run.station_enabled=1;run.imu_mode=run_active=1;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
    initial_field_heading=run.heading=work ? HEAD_X_NEG : HEAD_X_POS;
    run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*(int32_t)run.heading*9000);
    CHECK(road_point(work ? NODE_E : NODE_D,&origin));
    planned_x=field_x=origin.x;planned_y=field_y=origin.y;
    CHECK(run_road());
    deadline=clock_ms+35000;
    while(run_active && run.edge==(work ? 4u : 5u) && clock_ms<deadline) {
        plant_step();
        if(!injected && run.velocity_phase==VELOCITY_FORWARD
            && run.encoder_ready==ALL_MOTORS_MASK && run.progress_mm>run.action_mm-12) {
            inject_forward_error(bias);injected=1;
        }
        if(injected && run.velocity_phase==VELOCITY_STOP_FORWARD
            && abs(run.straight_hold.lateral_mm)>CAR_STRAIGHT_HOLD_DEADBAND_MM && !run.left_cmd_rpm) {
            CHECK(!run.forward_cmd_rpm);
            stopped_with_error=1;
        }
    }
    if(!injected || !stopped_with_error || clock_ms>=deadline)
        fprintf(stderr,"CROSS work=%d bias=%d injected=%d stopped=%d edge=%u phase=%u state=%u progress=%ld cross=%ld remaining=%ld: %s\n",
            work,bias,injected,stopped_with_error,run.edge,run.velocity_phase,run.state,
            (long)run.progress_mm,(long)run.straight_hold.lateral_mm,(long)(deadline-clock_ms),output);
    CHECK(injected && stopped_with_error && clock_ms<deadline);
    if(work) {
        CHECK(run.edge==5 && run.station_align_pending && !run.cross_carry_mm);
    } else {
        int32_t carried=run.cross_carry_mm;
        CHECK(run.edge==6 && !run.station_align_pending && carried*bias>0);
        while(run_active && run.velocity_phase!=VELOCITY_FORWARD && clock_ms<deadline)plant_step();
        CHECK(run_active && clock_ms<deadline && run.travel_axis==BODY_FORWARD && run.travel_sign==-1);
        while(run_active && !run.lateral_pose.valid && clock_ms<deadline)plant_step();
        CHECK(run_active && clock_ms<deadline);
        CHECK(!run.cross_carry_mm && run.progress_mm*bias<0);
        CHECK(run.route_origin_along_mm==-carried);
    }
    stop_request();
    printf("PASS cross handoff work=%d bias=%d\n",work,bias);
}

static void carry_coordinates_tests(void)
{
    static const int forward[]={15,0,-15,0},left[]={0,15,0,-15};
    for(unsigned heading=0;heading<4;heading++)for(unsigned axis=0;axis<2;axis++) {
        reset_heading_fixture();
        run.cross_carry_mm=15;run.cross_carry_heading=HEAD_Y_POS;
        run.target_heading=(RoadHeading)heading;run.travel_axis=(BodyAxis)axis;run.travel_sign=-1;
        run_apply_cross_carry();
        int along=axis==BODY_LEFT ? left[heading] : forward[heading];
        int cross=axis==BODY_LEFT ? forward[heading] : left[heading];
        CHECK(run.route_origin_along_mm==-along && run.encoder_offset_mm==along);
        CHECK(run.lateral_pose.cross_mm==cross && !run.cross_carry_mm);
        CHECK(run_save_leg(50-along));
        CHECK(run.road_path_count==1 && run.road_path[0].mm==50 && run.road_path[0].sign==-1);
    }
    reset_heading_fixture();run.cross_carry_mm=-15;run.cross_carry_heading=HEAD_Y_POS;
    run.target_heading=HEAD_X_NEG;run.corner.active=1;run_apply_cross_carry();
    CHECK(run.corner.x_mm==-15 && run.corner.y_mm==0 && !run.encoder_offset_mm);
    reset_heading_fixture();run.imu_mode=1;run.travel_axis=BODY_LEFT;
    CHECK(run_defer_lateral_finish());
    run.translation_only=1;CHECK(!run_defer_lateral_finish());
    run.translation_only=0;run.station_shift=1;CHECK(!run_defer_lateral_finish());
    run.station_shift=0;run.obstacle_phase=2;CHECK(!run_defer_lateral_finish());
    run.obstacle_phase=0;run.edge=FULL_ROUTE_NODES-2u;run.road.count=1;
    CHECK(!run_defer_lateral_finish());
    puts("PASS carry coordinates: four headings, both axis projections, physical retreat distance, corner origin and scope");
}

int main(void)
{
    carry_coordinates_tests();
    cross_handoff_test(1,-15);
    cross_handoff_test(1,15);
    cross_handoff_test(0,-15);
    cross_handoff_test(0,15);
    return 0;
}
