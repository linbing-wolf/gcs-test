/* 验证 A/C 回程把启停区偏移 并入圆角轨迹，使用真实主控和四轮反馈替身。 */
#define STATION_TEST_MAIN retained_final_return_suite
#include "verify_station_vision.c"

int main(void)
{
    for(int plan=1;plan>=0;plan--)for(unsigned from_b=0;from_b<2;from_b++) {
        const uint8_t final_edge=FULL_ROUTE_NODES-2u;
        uint8_t home_node=plan ? NODE_C : NODE_A;
        RoadPlan road;
        unsigned main_mm=ROAD_START_TO_B_MM;
        RoadPoint start,home;
        uint32_t deadline;
        unsigned final_seen=0,blended_motion=0;

        prepare_arm_auto((unsigned)plan);
        translation_yaw_disturbance=0;
        run.plan=(uint8_t)plan;
        run.job=checked_job=FULL_ROUTE_JOBS-1u;run.station_enabled=1;
        CHECK(station_command("task 134+123+314+231") && station_task_ready());
        CHECK(road_plan(home_node,NODE_B,&road));
        CHECK(road.count==2 && road.segments[road.count-1u].axis==BODY_FORWARD);
        CHECK(road_plan(NODE_B,home_node,&road) && road.count==2
            && road.length==ROAD_START_TO_B_MM+CAR_START_INSET_MM);
        CHECK(run_plan_road(final_edge,&road) && road.to==home_node);
        CHECK(road.count==(CAR_RUN_CORNER_ENABLED ? 1 : 2));
        CHECK(road.length==(main_mm+(CAR_RUN_CORNER_ENABLED ? 0u : CAR_START_INSET_MM)));
        CHECK(road.segments[0].axis==BODY_FORWARD && road.segments[0].mm==(plan ? (int)main_mm : -(int)main_mm));

        CHECK(road_point(from_b ? NODE_B : NODE_E,&start) && road_point(home_node,&home));
        planned_x=field_x=start.x;planned_y=field_y=start.y;initial_field_heading=HEAD_Y_POS;
        run.edge=(uint8_t)(from_b ? final_edge : final_edge-1u);
        run.heading=HEAD_Y_POS;run.completed_roads=0xffffu;
        run.imu_mode=run_active=1;
        CHECK(run_road());
        deadline=clock_ms+45000;
        while(run_active && clock_ms<deadline) {
            plant_step();
            if(run_active && run.edge==final_edge && run.velocity_phase==VELOCITY_FORWARD) {
                final_seen=1;
                CHECK(run.job==FULL_ROUTE_JOBS-1u);
                if(!run.segment)CHECK(run.travel_axis==BODY_FORWARD && run.action_mm==main_mm);
                else CHECK(!CAR_RUN_CORNER_ENABLED && run.segment==1
                    && run.travel_axis==BODY_LEFT && run.action_mm==CAR_START_INSET_MM);
                CHECK(run.corner.active==CAR_RUN_CORNER_ENABLED);
                if(run.corner.active) {
                    const CornerSpan *last=&run.corner.spans[run.corner.span_count-1u];
                    CornerPoint begin=run_corner_point(last->s);
                    CornerPoint halfway=run_corner_point(last->s+last->blend_length/2);
                    CornerPoint end=run_corner_point(last->s+last->blend_length);
                    CHECK(last->blend_length==CAR_RETURN_BLEND_MM);
                    CHECK(last->blend_x==CAR_START_INSET_MM && !last->blend_y);
                    int separate_home=run.corner.start_edge==final_edge;
                    float base_x=separate_home ? 0.0f : ROAD_B_E_MM;
                    float base_y=0;
                    CHECK(fabsf(begin.x-base_x)<0.01f && fabsf(halfway.x-(base_x+CAR_START_INSET_MM/2.0f))<0.01f);
                    CHECK(fabsf(end.x-(base_x+CAR_START_INSET_MM))<0.01f && fabsf(end.y-begin.y)>500);
                    CHECK(fabsf(begin.dx)<0.001f && fabsf(end.dx)<0.001f);
                    CHECK(fabsf(run.corner.first_mm-(base_x+CAR_START_INSET_MM))<0.01f);
                    CHECK(fabsf(run.corner.second_mm-(base_y+(plan ? -(float)main_mm : (float)main_mm)))<0.01f);
                    if(run.corner.s_mm>last->s+30 && run.corner.s_mm<last->s+last->blend_length-30
                        && run.forward_cmd_rpm && run.left_cmd_rpm)blended_motion=1;
                }
            }
        }
        if(!final_seen || clock_ms>=deadline || run.state!=RUN_DONE)
            fprintf(stderr,"HOME plan=%d from_B=%u state=%u job=%u edge=%u: %s\n",plan,from_b,run.state,run.job,run.edge,output);
        CHECK(final_seen && clock_ms<deadline && run.state==RUN_DONE);
        CHECK(blended_motion==CAR_RUN_CORNER_ENABLED);
        CHECK(!run_active && run.job==FULL_ROUTE_JOBS && run.distance_mm==main_mm+CAR_START_INSET_MM+(from_b ? 0u : ROAD_B_E_MM));
        CHECK(run.heading==HEAD_Y_POS && planned_x==home.x && planned_y==home.y);
        for(unsigned wheel=0;wheel<4;wheel++)
            CHECK(!simulated_wheel_rpm[wheel] && abs(reported_wheel_rpm[wheel])<=CAR_VELOCITY_STOP_RPM);
        printf("PASS %s-%c final return: reaches configured home endpoint, wheel feedback stopped; corner=%u\n",
            from_b ? "B" : "E",plan ? 'C' : 'A',(unsigned)CAR_RUN_CORNER_ENABLED);
    }
    return 0;
}
