/* 暂存新增50mm撤回，粗加工原50mm保持；独立坐标和真实发车入口共同验证。 */
#define START_SCAN_TEST_MAIN retained_complete_start_suite
#include "verify_start_scan.c"

static void requested_inset_tests(void)
{
    static const uint8_t nodes[]={NODE_F,NODE_D,NODE_H};
    static const RoadPoint expected[]={{1200,390},{1200,2010},{420,1200}};
    static const unsigned distances[]={810,810,780};
    unsigned i;
    RoadPoint point;
    RoadPlan road,back;
    CHECK(CAR_DRIVE_DECEL_RPM_PER_S==240);
    for(i=0;i<3;i++) {
        CHECK(road_point(nodes[i],&point) && point.x==expected[i].x && point.y==expected[i].y);
        CHECK(road_plan(NODE_E,nodes[i],&road) && road.length==distances[i]);
        CHECK(road_plan(nodes[i],NODE_E,&back) && back.length==road.length);
        CHECK(road.count==1 && back.count==1 && back.segments[0].mm==-road.segments[0].mm);
    }
    CHECK(road_point(NODE_I,&point) && point.x==340 && point.y==2010);
    CHECK(road_point(NODE_G,&point) && point.x==340 && point.y==290);
    CHECK(EXPECTED_FULL_ROUTE_MM==13760);
    puts("PASS requested insets: F1200/390 retained, H420/1200 restored, D1200/2010 and I340/2010 retained, H420/1200 restored; E distances810/810/780; deceleration240");
}

static void blended_start_return_tests(void)
{
    unsigned plan,stage;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(plan=0;plan<2;plan++)for(stage=0;stage<3;stage++) {
        uint32_t deadline;
        RoadPoint start;
        prepare_arm_auto(plan);send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+30000;
        CHECK(road_point(plan ? NODE_C : NODE_A,&start));
        while(run_active && !(run.corner.active && run.corner.s_mm>=100+stage*300) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && run.edge==0 && !run.job);
        send_line("obstacle");
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_WAIT_OBSTACLE && !run.job);
        CHECK(hypot(field_x-start.x,field_y-start.y)<8);
        stop_request();
    }
    puts("PASS retained start blend: A/C100/400/700mm obstacle retraces to original start within8mm in host model");
}

int main(void)
{
    requested_inset_tests();start_blend_geometry_test();start_geometry_and_scan_tests();
    approach_cancellation_tests();scan_during_travel_tests();scan_handoff_tests();
    automatic_obstacle_return_test();blended_start_return_tests();
    printf("PASS station inset/start focused suite: corner=%u assertions=%u; host only\n",(unsigned)CAR_RUN_CORNER_ENABLED,assertions);
    return 0;
}
