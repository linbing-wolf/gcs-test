/* 分段比赛换向停车：真实主控、CAN/IMU替身，不连接车辆。 */
#define STATION_TEST_MAIN retained_node_stop_station_suite
#include "verify_station_vision.c"

static void stopped_fixture(uint32_t now)
{
    reset_heading_fixture();clock_ms=now;
    run_active=run.imu_mode=1;run.state=RUN_MOVING;
    run.velocity_phase=VELOCITY_STOP_FORWARD;run.zero_ms=now;
    for(unsigned i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        f->position_valid=f->speed_valid=1;f->status_replies=1;f->status_flags=MOTOR_FLAG_ENABLED;
        f->position_ms=f->speed_ms=f->status_ms=now;
    }
}
static void fresh_at(uint32_t now)
{
    clock_ms=now;
    for(unsigned i=0;i<4;i++)motor_feedback[i].position_ms=motor_feedback[i].speed_ms=motor_feedback[i].status_ms=now;
}

static void node_stop_gate_tests(void)
{
    stopped_fixture(1000);
    motor_feedback[3].speed_rpm=CAR_VELOCITY_STOP_RPM+1;
    CHECK(!run_velocity_stopped() && !run.stopped_ms);
    motor_feedback[3].speed_rpm=0;
    CHECK(!run_velocity_stopped());
    fresh_at(1199);CHECK(!run_velocity_stopped());
    fresh_at(1200);CHECK(run_velocity_stopped());
    motor_feedback[2].speed_rpm=-CAR_VELOCITY_STOP_RPM-1;
    CHECK(!run_velocity_stopped() && !run.stopped_ms);
    motor_feedback[2].speed_rpm=0;fresh_at(1201);CHECK(!run_velocity_stopped());
    fresh_at(1401);CHECK(run_velocity_stopped());
    motor_feedback[1].speed_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;
    CHECK(!run_velocity_stopped());
    stopped_fixture(UINT32_MAX-100);CHECK(!run_velocity_stopped());
    fresh_at(98);CHECK(!run_velocity_stopped());fresh_at(99);CHECK(run_velocity_stopped());
    stopped_fixture(1000);run.translation_only=1;CHECK(run_velocity_stopped());
    stopped_fixture(1000);run.station_shift=1;CHECK(run_velocity_stopped());
    stopped_fixture(1000);run.velocity_phase=VELOCITY_TURN;CHECK(run_velocity_stopped());
    motor_feedback[0].speed_rpm=2;CHECK(!run_velocity_stopped());
    motor_feedback[0].speed_rpm=0;CHECK(run_velocity_stopped()); /* 转头完成后也先停稳，但不另加200ms。 */
    stopped_fixture(1000);run.obstacle_phase=2;
    motor_feedback[0].speed_rpm=2;CHECK(!run_velocity_stopped());
    motor_feedback[0].speed_rpm=0;CHECK(run_velocity_stopped());
    CHECK(!velocity_batches && !arm_stops);
    puts("PASS node stop: actual wheel speed,200ms dwell/restart, fresh feedback/wrap; manual, station micro-move, heading and obstacle policies retained");
}

static void start_and_b_corner_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        RoadPoint start;uint32_t deadline,last_zero=0;unsigned stops=0,seen=0;
        uint32_t completed_zero[2]={0};
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        CHECK(road_point(plan ? NODE_C : NODE_A,&start));
        planned_x=field_x=start.x;planned_y=field_y=start.y;initial_field_heading=HEAD_X_NEG;
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+60000;
        while(run_active && clock_ms<deadline) {
            if(run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>30
                && (plan ? run.edge==0 && run.segment==1 : run.edge==1))break;
            plant_step();CHECK(!run.corner.active);
            if(run.velocity_phase==VELOCITY_STOP_FORWARD && run.stopped_ms)last_zero=run.stopped_ms;
            if(strstr(output,"[control segment done]")) {
                CHECK(last_zero && (uint32_t)(clock_ms-last_zero)>=200);
                for(unsigned i=0;i<4;i++)CHECK(abs(motor_feedback[i].speed_rpm)<=CAR_VELOCITY_STOP_RPM);
                CHECK(stops<2);completed_zero[stops++]=clock_ms;last_zero=0;
            }
            if(strstr(output,"[route pass]"))CHECK(0); /* 起点折线和B换向都不能不断速穿过。 */
            if(run.velocity_phase==VELOCITY_FORWARD
                && (plan ? run.edge==0 && run.segment==1 && run.left_cmd_rpm : run.edge==1 && run.forward_cmd_rpm))seen=1;
        }
        CHECK(clock_ms<deadline && run_active && stops==(plan ? 1u : 2u) && seen);
        CHECK(run.target_heading==HEAD_X_NEG && !run.continuation_mm);
        if(plan)CHECK(run.edge==0 && run.segment==1 && run.travel_axis==BODY_LEFT && run.travel_sign==-1);
        else {
            CHECK(completed_zero[0]<completed_zero[1] && run.travel_axis==BODY_FORWARD);
            CHECK(run.road.from==NODE_B && run.road.to==NODE_E);
        }
        stop_request();
        printf("PASS direction change plan=%u: %u measured stop/handoff(s), no arc or mixed-axis blend\n",plan,stops);
    }
}

static void work_turn_policy_tests(void)
{
    for(unsigned batch=0;batch<2;batch++) {
        reset_heading_fixture();run.station_enabled=1;run.job=(uint8_t)(batch ? 5 : 2);
        run.heading=HEAD_X_NEG;run.plan=0;run.edge=(uint8_t)(batch ? 10 : 4);
        CHECK(run_plan_road(run.edge,&run.road));run.segment=(uint8_t)(run.road.count-1u);
        run.action_mm=860;run.progress_mm=855;run.encoder_ready=15;
        CHECK(!run_station_turn_early());
    }
    CHECK(CAR_RUN_CORNER_ENABLED==0);
    CHECK(CAR_LATERAL_SHORT_RPM==250 && CAR_LATERAL_LONG_RPM==200 && CAR_RUN_RPM==250);
    CHECK(LEFT_FORWARD_MM_PER_M==28 && RIGHT_FORWARD_MM_PER_M==35);
    puts("PASS work turns: both rough180 visits cannot start before translation stop; race speed/calibration retained");
}

int main(void)
{
    node_stop_gate_tests();work_turn_policy_tests();start_and_b_corner_tests();
    printf("PASS stop at direction nodes: %u assertions; host model only\n",assertions);
    return 0;
}
