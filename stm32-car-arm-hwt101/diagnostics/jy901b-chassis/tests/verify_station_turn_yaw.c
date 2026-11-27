/* 用比赛入口验证到站前机构和视觉保持空闲，四轮交接后才开始作业。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void station_arrival_tests(void)
{
    static const uint8_t predecessor_edges[]={1,3,5,7,9,11};
    static const RoadHeading predecessor_headings[]={HEAD_X_NEG,HEAD_X_NEG,HEAD_X_POS,HEAD_Y_POS,HEAD_X_NEG,HEAD_X_POS};
    static const char requests[]={'H','D','E','H','D','F'};
    unsigned plan,visit;
    for(plan=0;plan<2;plan++)for(visit=0;visit<6;visit++) {
        RoadPoint predecessor;
        uint16_t idle_yaw,idle_claw,idle_platform;
        unsigned tx,positions,turning=0,continuous=0,yaw_moved=0;
        uint32_t deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");
        send_line(plan ? "runimu C" : "runimu A");
        CHECK(run_active && run.state==RUN_PREFLIGHT);
        run.job=(uint8_t)(visit+1u);run.edge=predecessor_edges[visit];checked_job=run.job;
        run.heading=run.target_heading=predecessor_headings[visit];
        run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*(int32_t)run.heading*9000);
        initial_field_heading=run.heading;
        CHECK(road_point(full_route_nodes[plan][run.edge],&predecessor));
        field_x=planned_x=predecessor.x;field_y=planned_y=predecessor.y;
        idle_yaw=arm_yaw_us;idle_claw=arm_claw_us;idle_platform=arm_platform_us;
        tx=vision_tx_count;positions=arm_positions;
        deadline=clock_ms+60000;
        while(run_active && clock_ms<deadline) {
            plant_step();
            if(run.station_align_pending && run.turn_cmd_rpm)turning=1;
            if(run.corner.active && (run.forward_cmd_rpm || run.left_cmd_rpm))continuous=1;
            CHECK(arm_yaw_us==idle_yaw && arm_claw_us==idle_claw && arm_platform_us==idle_platform);
            CHECK(arm_positions==positions && vision_tx_count==tx && !station_active());
        }
        CHECK(clock_ms<deadline && !run_active && run.state==RUN_STATION && arm_lock.state==1);
        CHECK(run.job==visit+1u && full_route_nodes[plan][run.edge]==full_route_jobs[plan][run.job]);
        if(visit==1 || visit==4)CHECK(turning);
        if(CAR_RUN_CORNER_ENABLED && (visit==2 || visit==3 || visit==5))CHECK(continuous);
        missing_feedback_motor=4;tick(500);
        CHECK(arm_yaw_us==idle_yaw && arm_claw_us==idle_claw && arm_platform_us==idle_platform);
        CHECK(!station_active() && arm_positions==positions && vision_tx_count==tx);
        missing_feedback_motor=0;
        while(vision_tx_count<tx+6u && clock_ms<deadline) {
            plant_step();
            if(arm_yaw_us!=idle_yaw) {
                CHECK((run.state==RUN_STATION || run.station_shift) && arm_lock.state==2);
                yaw_moved=1;
            }
        }
        CHECK(clock_ms<deadline && yaw_moved && vision_tx_bytes[tx+3u]==requests[visit]);
        printf("ARRIVAL plan=%u job=%u command=%c; arm and vision waited for wheel handoff\n",plan,run.job,requests[visit]);
        stop_request();CHECK(!run_active && !station_active());
    }
}

int main(void)
{
    station_arrival_tests();
    printf("PASS station arrival: %lu assertions; host model only\n",(unsigned long)assertions);
    return 0;
}
