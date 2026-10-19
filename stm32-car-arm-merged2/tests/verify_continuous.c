/* 使用真实入口和独立车辆模型；不连接串口或车辆。 */
#define HEADING_TEST_MAIN heading_regression_main
#include "verify_heading.c"

static void begin_route(unsigned plan, unsigned uart_cost)
{
    RoadPoint start;
    reset_plant(17900,1.15);simulate_uart_cost=(uint8_t)uart_cost;imu_period_ms=100;
    CHECK(road_point(plan?NODE_C:NODE_A,&start));
    field_x=planned_x=start.x;field_y=planned_y=start.y;initial_field_heading=plan?2:0;
    send_line(plan?"runimu C":"runimu A");CHECK(run_active);
}

static void continuous_pass_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        uint32_t deadline;
        unsigned passes=0,injections=0;
        begin_route(variant&1,variant/2);deadline=clock_ms+300000;
        while(run_active && clock_ms<deadline) {
            int64_t baseline[4];
            int8_t signs[4];
            uint8_t edge=run.edge;
            RoadHeading heading=run.target_heading;
            memcpy(baseline,run.encoder_start,sizeof(baseline));
            memcpy(signs,run.encoder_sign,sizeof(signs));
            plant_step();
            if (((run.edge==3 && injections==0) || (run.edge==9 && injections==1)) && run.velocity_phase==VELOCITY_FORWARD
                && run.encoder_ready==0 && run.progress_mm==0 && !run.encoder_previous_delta[0]) {
                unsigned wheel;
                clock_ms++;
                for(wheel=0;wheel<4;wheel++) {
                    int sign=(wheel==0 || wheel==3) ? -1 : 1;
                    motor_feedback[wheel].position=run.encoder_start[wheel]+sign*(wheel==variant ? -128 : 128);
                    motor_feedback[wheel].position_ms=clock_ms;
                }
                CHECK(run_velocity_distance() && !run.encoder_ready);
                injections++;
            }
            if(strstr(output,"[route pass]")) {
                CHECK(edge==3 || edge==9 || edge==13);
                CHECK(run.edge==edge+1 && run.velocity_phase==VELOCITY_FORWARD);
                CHECK(run.target_heading==heading && run.forward_mrpm>0 && run.forward_cmd_rpm>0);
                CHECK(!memcmp(baseline,run.encoder_start,sizeof(baseline)));
                CHECK(!memcmp(signs,run.encoder_sign,sizeof(signs)) && run.encoder_ready==15u);
                CHECK(run.encoder_offset_mm>0 && run.road.from==NODE_E);
                passes++;
            }
            if(run.velocity_phase==VELOCITY_STOP_FORWARD)
                CHECK(run.edge!=3 && run.edge!=9 && run.edge!=13);
        }
        if(run.state!=RUN_DONE)fprintf(stderr,"Route fault: %s\n",output);
        CHECK(run.state==RUN_DONE && run.job==8 && passes==3 && injections==2);
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && baseline_transitions==15);
        printf("PASS nonstop route %c uart_cost=%u: three E passes preserve heading, ramp and encoder baseline; all jobs stop; configured_run_rpm=%u\n",
               (variant&1)?'C':'A',variant/2,(unsigned)CAR_RUN_RPM);
    }
}

static void reach_leg(unsigned edge,unsigned segment,unsigned mm)
{
    uint32_t deadline=clock_ms+300000;
    while(run_active && !(run.edge==edge && run.segment==segment
          && run.velocity_phase==VELOCITY_FORWARD && !run.turning && run.progress_mm>=(int32_t)mm)
          && clock_ms<deadline)plant_step();
    CHECK(run_active && run.edge==edge && run.segment==segment && clock_ms<deadline);
}

static void return_cases(void)
{
    static const struct {unsigned edge,segment,mm;} cases[]={
        {0,0,20},{0,1,200},{3,0,200},{4,0,200},{6,0,200},{14,0,150},{15,0,200},{15,1,20}
    };
    unsigned plan,test;
    for(plan=0;plan<2;plan++)for(test=0;test<sizeof(cases)/sizeof(cases[0]);test++) {
        uint32_t deadline;
        unsigned i,job;
        uint8_t node,road;
        RoadPoint point;
        begin_route(plan,test&1);reach_leg(cases[test].edge,cases[test].segment,cases[test].mm);
        node=full_route_nodes[plan][run.edge];road=run.road.road;job=run.job;
        send_line("obstacle");CHECK(run.obstacle_phase==1 && run.state==RUN_OBSTACLE_STOP);
        for(i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i]);
        deadline=clock_ms+60000;
        while(run_active && clock_ms<deadline) {
            plant_step();
            if(run.obstacle_phase==2 && !run.turning)
                CHECK(abs(run.forward_cmd_rpm)<=CAR_OBSTACLE_RETURN_RPM);
        }
        if(run.state!=RUN_WAIT_OBSTACLE)fprintf(stderr,"Return fault plan=%u case=%u: %s\n",plan,test,output);
        CHECK(run.state==RUN_WAIT_OBSTACLE && run.obstacle_node==node && run.job==job);
        CHECK(run.blocked_roads&(uint16_t)(1u<<road));
        CHECK(!(run.completed_roads&(uint16_t)(1u<<road)) && !pose_set && !nav.position_confirmed);
        CHECK(road_point(node,&point) && hypot(field_x-point.x,field_y-point.y)<80);
        for(i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i] && abs(reported_wheel_rpm[i])<=CAR_VELOCITY_STOP_RPM);
        {uint32_t batches=velocity_batches;
            for(i=0;i<1000;i++)plant_step();
            CHECK(!run_active && velocity_batches==batches);
            sample_yaw(wrapped_yaw(physical_yaw));send_line(plan?"runimu C":"runimu A");
            CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE && strstr(output,"[rejected"));
        }
    }
    puts("PASS obstacle return: A/C before/after E, ordinary road, start lateral+forward and return backward+lateral; stop first, return to latest node, block road, wait without job advance or restart");
}

static void return_failure_tests(void)
{
    unsigned test;
    for(test=0;test<7;test++) {
        uint32_t deadline;
        uint32_t before_batches;
        begin_route(1,0);reach_leg(4,0,120);send_line("obstacle");
        before_batches=velocity_batches;
        if(test==0)missing_feedback_motor=4;
        if(test==1)moving_at_stop_motor=4;
        if(test>=2) {
            while(run_active && run.obstacle_phase!=2)plant_step();
            CHECK(run_active);
            if(test==2)heading_sensor_invalidate(&heading_sensor);
            if(test==3)missing_velocity_ack=4;
            if(test==4)route_test_can_error(1);
            if(test==5)stop_request();
            if(test==6)send_line("obstacle");
        }
        deadline=clock_ms+25000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        CHECK(run.job==2);
        if(test<2)CHECK(velocity_batches==before_batches);
    }
    begin_route(1,0);send_line("obstacle");
    {uint32_t deadline=clock_ms+10000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_WAIT_OBSTACLE && !run.road_path_count && run.obstacle_node==NODE_C && !run.blocked_roads);
    }
    for(test=0;test<2;test++) {
        uint32_t deadline;
        uint8_t node,job;
        begin_route(1,0);deadline=clock_ms+300000;
        while(run_active && !(test ? (run.edge==6 && run.turning && !run.station_align_pending)
                                  : run.station_align_pending) && clock_ms<deadline)plant_step();
        CHECK(run_active && clock_ms<deadline);
        node=full_route_nodes[run.plan][run.edge];job=run.job;
        send_line("obstacle");deadline=clock_ms+10000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_WAIT_OBSTACLE && run.obstacle_node==node && run.job==job && !run.road_path_count);
    }
    reset_plant(0,1);clock_ms=UINT32_MAX-200u;plant_ms=imu_ms=clock_ms;sample_yaw(0);
    send_line("runimu C");send_line("obstacle");
    for(test=0;run_active && test<10000;test++)plant_step();
    CHECK(run.state==RUN_WAIT_OBSTACLE && run.obstacle_node==NODE_C);
    puts("PASS obstacle failures: missing feedback/ACK, wheel4 not stopped, IMU invalid, CAN fault, emergency stop, second obstacle and preflight obstacle; no false arrival");
}

int main(void)
{
    continuous_pass_tests();return_cases();return_failure_tests();
    printf("PASS nonstop/obstacle: %u assertions; run_rpm=%u return_rpm=%u; simulation, not vehicle acceptance\n",
           assertions,(unsigned)CAR_RUN_RPM,(unsigned)CAR_OBSTACLE_RETURN_RPM);
    return 0;
}
