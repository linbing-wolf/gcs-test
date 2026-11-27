#define HEADING_TEST_MAIN retained_race_accel_extra_suite
#include "verify_heading.c"

static void publish_accel(int16_t left_mg)
{
    heading_sensor.acceleration.x_mg=21;
    heading_sensor.acceleration.y_mg=left_mg+35;
    heading_sensor.acceleration.z_mg=1000;
    heading_sensor.acceleration.received_ms=clock_ms;
    heading_sensor.acceleration.sequence++;
    heading_sensor.acceleration.valid=1;
}

static void start_extra_test(const char *direction,int enabled)
{
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    CHECK(accel_extra_enabled);
    if(!enabled)send_line("imutest extra 0");
    CHECK(accel_extra_enabled==enabled);
    send_line("test");send_line("dist 1000");send_line("rpm 120");send_line(direction);
    CHECK(run_active);
}

static void motion_test(const char *direction,int enabled,int supply_accel)
{
    uint32_t deadline;
    uint16_t frozen_mm=0;
    float previous_left_mm_s=0;
    int froze=0,attempted_toggle=0;
    start_extra_test(direction,enabled);
    deadline=clock_ms+20000;
    while(run_active && clock_ms<deadline) {
        if(supply_accel && clock_ms%20u==0u) {
            float left_mm_s=run.profile_sign*run.forward_mrpm*
                (CAR_POSITION_PULSES_PER_REV/(60000.0f*run_single_axis_ppm()));
            float accel_mg=(left_mm_s-previous_left_mm_s)/(0.020f*9.80665f);
            if(run.velocity_phase==VELOCITY_FORWARD && supply_accel==1)accel_mg-=60*run.travel_sign;
            publish_accel((int16_t)lroundf(accel_mg));
            previous_left_mm_s=left_mm_s;
        }
        plant_step();
        if(run.accel_extra.state==ACCEL_EXTRA_FROZEN) {
            if(!froze){froze=1;frozen_mm=run.accel_extra.mm;}
            CHECK(run.accel_extra.mm==frozen_mm);
        }
        if(enabled && supply_accel && run.accel_extra.mm && !attempted_toggle) {
            attempted_toggle=1;send_line("imutest extra 0");
            CHECK(accel_extra_enabled && strstr(output,"[imutest extra rejected: active segment]"));
        }
    }
    CHECK(!run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(run.action_mm==1000 && abs(run.progress_mm-(int)run_target_mm())<=CAR_DISTANCE_TOLERANCE_MM);
    printf("Extra motion %s enable=%d accel=%d target=%u distance=%ld\n",
           direction,enabled,supply_accel,run_target_mm(),(long)run.progress_mm);
    if(enabled && supply_accel==1 && direction[0]!='f') {
        CHECK(froze && frozen_mm>0 && frozen_mm<CAR_ACCEL_EXTRA_LIMIT_MM && attempted_toggle);
        CHECK(run.accel_extra.samples==CAR_ACCEL_EXTRA_CAL_SAMPLES);
        CHECK(run.accel_extra.bias_y_mg==35 && strstr(output," nominal_mm=1000 accel_extra_mm="));
    } else {
        CHECK(!run.accel_extra.mm);
        CHECK(run.accel_extra.state==(enabled && !supply_accel ? ACCEL_EXTRA_UNAVAILABLE
            : enabled && supply_accel==2 ? ACCEL_EXTRA_FROZEN : ACCEL_EXTRA_IDLE));
    }
    send_line("imutest extra 0");CHECK(!accel_extra_enabled);
}

static void limits_test(void)
{
    reset_plant(0,1.15);
    accel_extra_enabled=run.imu_mode=run.translation_only=1;
    run.travel_axis=BODY_LEFT;run.travel_sign=1;run.profile_sign=1;run.lateral_rpm=200;
    run.velocity_phase=VELOCITY_FORWARD;run.action_mm=2000;
    run.encoder_ready=ALL_MOTORS_MASK;
    run.accel_extra.state=ACCEL_EXTRA_READY;
    run.accel_extra.deficit_mm=70;run.accel_extra.sample_ms=clock_ms;
    clock_ms+=20;publish_accel(0);sample_yaw(0);accel_extra_update(heading_snapshot());
    CHECK(!run.accel_extra.mm);
    run.accel_extra.deficit_mm=400;
    accel_extra_update(heading_snapshot());CHECK(!run.accel_extra.mm); /* 同帧不计算两次。 */
    clock_ms+=20;publish_accel(0);sample_yaw(0);accel_extra_update(heading_snapshot());
    CHECK(run.accel_extra.mm==300 && run_target_mm()==2300 && run.accel_extra.state==ACCEL_EXTRA_FROZEN);
    for(unsigned condition=0;condition<4;condition++) {
        run.accel_extra.state=ACCEL_EXTRA_READY;run.accel_extra.mm=100;
        run.accel_extra.sample_ms=clock_ms;run.accel_extra.sequence=heading_sensor.acceleration.sequence;
        run.progress_mm=0;run.forward_mrpm=0;
        clock_ms+=20;publish_accel(-500);
        sample_yaw(0);
        if(condition==0){run.progress_mm=1900;run.forward_mrpm=120000;}
        if(condition==1){clock_ms+=101;sample_yaw(0);}
        if(condition==2)heading_sensor_publish_attitude(&heading_sensor,200,0,0,clock_ms);
        if(condition==3)run.velocity_phase=VELOCITY_STOP_FORWARD;
        accel_extra_update(heading_snapshot());
        CHECK(run.accel_extra.state==ACCEL_EXTRA_FROZEN && run.accel_extra.mm==100 && run_target_mm()==2100);
        clock_ms+=20;publish_accel(-500);sample_yaw(0);accel_extra_update(heading_snapshot());
        CHECK(run.accel_extra.mm==100);
    }
    puts("PASS extra limits: deadband, duplicate, cap, nominal brake, stale sensors, tilt and stop freeze");
}

/* 从D工位后的路线现场启动D->E，入口和停车均走比赛道路状态机。 */
static void route_test(void)
{
    uint32_t deadline;
    uint16_t stop_target=0;
    int32_t stop_distance=0;
    float previous_left_mm_s=0;
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    send_line("imutest extra");
    CHECK(accel_extra_enabled && strstr(output,"[imutest extra] enabled=1"));
    sample_yaw(0);
    run_active=run.imu_mode=1;run.plan=1;run.edge=5;run.job=checked_job=3;
    run.heading=run.target_heading=HEAD_X_POS;
    run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*9000*(int)run.heading);
    initial_field_heading=HEAD_X_POS;
    field_x=planned_x=1200;field_y=planned_y=2060;
    CHECK(run_road() && run.travel_axis==BODY_LEFT && !run.continuation_mm);
    deadline=clock_ms+20000;
    while(run_active && run.edge==5 && clock_ms<deadline) {
        if(clock_ms%20u==0u) {
            float left_mm_s=run.profile_sign*run.forward_mrpm*
                (CAR_POSITION_PULSES_PER_REV/(60000.0f*run_single_axis_ppm()));
            float accel_mg=(left_mm_s-previous_left_mm_s)/(0.020f*9.80665f);
            if(run.velocity_phase==VELOCITY_FORWARD)accel_mg-=100*run.travel_sign;
            publish_accel((int16_t)lroundf(accel_mg));
            previous_left_mm_s=left_mm_s;
        }
        plant_step();
        if(run.edge==5 && run.velocity_phase==VELOCITY_STOP_FORWARD)stop_target=run_target_mm();
        if(strstr(output,"[control segment done]"))stop_distance=run.progress_mm;
    }
    if(!(run.edge==6 && clock_ms<deadline && stop_target>860 && stop_target<=1160))
        fprintf(stderr,"D-E: edge=%u state=%u phase=%u progress=%ld target=%u extra_state=%u ready=%u now=%lu %s\n",
            run.edge,run.state,run.velocity_phase,(long)run.progress_mm,stop_target,
            run.accel_extra.state,run.ready,(unsigned long)clock_ms,output);
    CHECK(run.edge==6 && clock_ms<deadline && stop_target>860 && stop_target<=1160);
    CHECK(abs(stop_distance-stop_target)<=CAR_DISTANCE_TOLERANCE_MM);
    CHECK(run.road.from==NODE_E && run.road.to==NODE_H && run.remaining_mm==ROAD_STORAGE_LEG_MM);
    while(run_active && run.velocity_phase!=VELOCITY_FORWARD && clock_ms<deadline)plant_step();
    CHECK(run_active && clock_ms<deadline && run.action_mm==ROAD_STORAGE_LEG_MM);
    CHECK(!run.accel_extra.mm && run.accel_extra.state==ACCEL_EXTRA_IDLE);
    printf("PASS race D-E: encoders stop at %ld of %u mm, E-H distance stays %u mm\n",
        (long)stop_distance,stop_target,(unsigned)ROAD_STORAGE_LEG_MM);
}

static void scope_test(void)
{
    reset_plant(0,1.15);
    send_line("imutest extra 1");
    run.imu_mode=1;run.travel_axis=BODY_LEFT;run.road.count=1;run.road.to=NODE_E;
    CHECK(accel_extra_eligible());
    run.continuation_mm=400;CHECK(!accel_extra_eligible());run.continuation_mm=0;
    run.road.count=2;CHECK(!accel_extra_eligible());run.road.count=1;
    run.station_enabled=1;run.job=3;run.road.to=NODE_H;CHECK(!accel_extra_eligible());
    run.road.to=NODE_A;CHECK(!accel_extra_eligible());run.road.to=NODE_E;
    run.station_shift=1;CHECK(!accel_extra_eligible());run.station_shift=0;
    run.vector_move=1;CHECK(!accel_extra_eligible());run.vector_move=0;
    run.obstacle_phase=2;CHECK(!accel_extra_eligible());run.obstacle_phase=0;
    send_line("imutest extra 0");CHECK(!accel_extra_eligible());
    send_line("imutest extra");CHECK(strstr(output,"[imutest extra] enabled=0"));
    send_line("imutest extra 1");CHECK(accel_extra_eligible());
    puts("PASS race scope: compound/continuous roads, station approach and positioning, return and idle toggle");
}

int main(int argc,char **argv)
{
    int mode=argc>1 ? atoi(argv[1]) : 0;
    if(mode==0)motion_test("l",0,1);
    else if(mode==1)motion_test("l",1,1);
    else if(mode==2)motion_test("r",1,1);
    else if(mode==3)motion_test("l",1,0);
    else if(mode==4)motion_test("f",1,1);
    else if(mode==5)limits_test();
    else if(mode==6)motion_test("l",1,2);
    else if(mode==7)route_test();
    else scope_test();
    return 0;
}
