/* 真实粗加工状态机与独立场地模型；检查最后抓取后连续经过2环附近离站。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void through_geometry_tests(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    unsigned plan,batch,order;
    for(plan=0;plan<2;plan++)for(batch=0;batch<2;batch++)for(order=0;order<6;order++) {
        char task[48];unsigned offset=0,grips=0,returns=0,launched=0,joined=0,moving_return=0,front_seen=0;
        uint32_t deadline;float previous=ARM_CLAW_OPEN_DEG;double closest=10000;
        int expected=test_station_move_mm(0,orders[order][2]-'0',2);
        snprintf(task,sizeof(task),"task 134+%s+314+%s",orders[order],orders[order]);
        prepare_rough(batch,task);run.plan=(uint8_t)plan;deadline=clock_ms+150000;
        while(clock_ms<deadline && run.state!=RUN_FAULT) {
            float yaw,claw,target_yaw;int lift,slide;
            serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_CLOSE_DEG && yaw==STN_ROUGH_YAW_DEG && lift==STN_ROUGH_LIFT_MM)grips++;
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG && lift==ARM_PLACE_LIFT_MM) {
                returns++;
                if(returns==3 && (run.forward_cmd_rpm || run.left_cmd_rpm))moving_return=1;
            }
            previous=claw;
            if(run.departure_overlap==2u && !launched) {
                int16_t mm;
                CHECK(grips==3 && arm_busy() && station_through_departure(&mm) && mm==expected);
                CHECK(run.job==(batch ? 6 : 3) && !run.station_shift && run.station_return_mm==expected);
                CHECK(hypot(field_x-(1200-expected),field_y-2010)<30);launched=1;
            }
            if(launched) {
                if(returns==3 && arm_parked())front_seen=1;
                if(returns==3 && run.departure_overlap) {arm_target_now(NULL,NULL,&target_yaw,NULL);CHECK(target_yaw!=STN_ROUGH_YAW_DEG);}
                CHECK(!strstr(output,"[station index] ring=2")); /* 没有另一个停车索引。 */
                if(run.corner.active) {
                    double a=run.corner.start_heading*3.141592653589793/2;
                    double end_x=1200-expected+cos(a)*run.corner.first_mm+sin(a)*run.corner.second_mm;
                    double end_y=2010+sin(a)*run.corner.first_mm-cos(a)*run.corner.second_mm;
                    CHECK(hypot(end_x-420,end_y-1200)<0.01); /* 不以重新定位模型掩盖环位偏移。 */
                    CHECK(run.corner.prefix_mm==expected);
                    if(expected) {
                        float join=run.corner.legs[0].end_s;
                        CHECK(run.corner.legs[0].length==150 && run.corner.legs[0].radius==75);
                        if(fabs(run.corner.s_mm-join)<20) {
                            CHECK(run.wheel_rpm[0] || run.wheel_rpm[1] || run.wheel_rpm[2] || run.wheel_rpm[3]);
                            joined=1;
                        }
                        if(run.corner.s_mm<join+30) {
                            double d=hypot(field_x-1200,field_y-2010);if(d<closest)closest=d;
                        }
                    } else joined=1;
                }
                if(run.state==RUN_STATION)break;
            }
        }
        CHECK(clock_ms<deadline && run.state==RUN_STATION && launched && joined && grips==3 && returns==3);
        CHECK(peer_disc_count==1 && !peer_move_count && arm_ready() && front_seen && arm_platform_slot()==1);
        CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+5);
        CHECK(!expected || closest<50); /* 圆角仅经过2环附近，不声称停车到环中心。 */
        printf("RING2_THROUGH plan=%u batch=%u order=%s return_mm=%d closest_ring2_mm=%.2f moving_last_return=%u endpoint_mm=%.2f,%.2f\n",
            plan,batch,orders[order],expected,expected ? closest : 0,moving_return,field_x,field_y);
        CHECK(moving_return);stop_request();
    }
}

static void through_pause_and_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<5;variant++) {
        unsigned offset=0;uint32_t deadline;uint16_t yaw;unsigned positions;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+100000;
        while(clock_ms<deadline && !(run.corner.active && run.corner.prefix_mm && run.forward_cmd_rpm)) {
            serve_peer(&offset);plant_step();
        }
        CHECK(clock_ms<deadline && run.departure_overlap==2u && run_active);
        if(!variant) {
            missing_feedback_motor=4;tick(1500);
            CHECK(run_active && run.corner.active && run.corner.prefix_mm==150 && run.job==3 && run.state!=RUN_FAULT);
            CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
            missing_feedback_motor=0;
            while(run_active && clock_ms<deadline && run.state!=RUN_FAULT)plant_step();
            CHECK(clock_ms<deadline && run.state==RUN_STATION && hypot(field_x-420,field_y-1200)<55);
            stop_request();continue;
        }
        if(variant==1)stop_request();
        if(variant==2){bus_off=1;plant_step();}
        if(variant==3)send_line("obstacle");
        if(variant==4)send_line("arm stop");
        positions=arm_positions;yaw=arm_yaw_us;tick(500);
        CHECK(!run_active && !station_active() && arm_failed() && !run.obstacle_phase && arm_positions==positions && arm_yaw_us==yaw);
        CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
    }
    puts("PASS through departure: missing wheel feedback pauses same prefix and resumes; explicit emergency, bus fault, obstacle and arm stop cancel chassis/arm together");
}

static void last_grip_clearance_tests(void)
{
    unsigned variant;
    for(variant=0;variant<3;variant++) {
        unsigned offset=0,grips=0,positions;uint32_t deadline;
        float previous=ARM_CLAW_OPEN_DEG,yaw,claw;int lift;int64_t base,counts;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+100000;
        while(grips<3u && clock_ms<deadline) {
            serve_peer(&offset);plant_step();arm_pose_now(NULL,&lift,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_CLOSE_DEG && yaw==STN_ROUGH_YAW_DEG && lift==STN_ROUGH_LIFT_MM)grips++;
            previous=claw;
        }
        CHECK(grips==3u && run.job==2 && !run_active);
        base=arm_encoder[1];positions=arm_positions;arm_freeze=1;
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[positions]==ARM_LIFT_ADDRESS);
        counts=(int64_t)((ARM_PLATFORM_TURN_START_LIFT_MM-STN_ROUGH_LIFT_MM)*ARM_LIFT_PULSES_PER_MM)*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        if(ARM_LIFT_DIR_INVERT)counts=-counts;
        counts*=ARM_LIFT_ENCODER_SIGN;
        arm_encoder[1]=base+counts-(counts>0 ? 1 : -1)*(int64_t)(ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        tick(600);CHECK(run.job==2 && !run_active && !station_through_departure(NULL));
        if(variant==2) {stop_request();tick(100);CHECK(!run_active && !station_active() && arm_failed());continue;}
        if(variant==1) {
            arm_missing_motor=ARM_LIFT_ADDRESS;arm_encoder[1]=base+counts;
            tick(ARM_FEEDBACK_TIMEOUT_MS+200u);CHECK(run.job==2 && !run_active && !station_through_departure(NULL));
            arm_missing_motor=0;
        }
        arm_encoder[1]=base+counts;
        while(run.job==2 && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.job==3 && run.departure_overlap==2u && run.station_return_mm==150);
        stop_request();
    }
    puts("PASS final-grip clearance: measured109mm and missing lift feedback cannot start departure; actual110mm releases; explicit stop cancels pending departure");
}

int main(void)
{
    CHECK(STN_ROUGH_THROUGH_ENABLED);through_geometry_tests();through_pause_and_cancel_tests();last_grip_clearance_tests();
    printf("PASS ring2 through: %u assertions; simulated feedback/path/command timing, not physical grip or field acceptance\n",assertions);
    return 0;
}
