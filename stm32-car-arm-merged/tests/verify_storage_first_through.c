/* 真实首次暂存编排与独立场地积分，检查末环直接连续离站。 */
#define STATION_TEST_MAIN retained_storage_through_station_suite
#include "verify_station_vision.c"

static void first_storage_tests(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    unsigned plan,order;
    for(plan=0;plan<2;plan++)for(order=0;order<6;order++) {
        char task[48];uint32_t deadline,start,depart=0;unsigned offset=0,placed=0,launched=0,joined=0;
        unsigned retract_moving=0,parked=0;float previous=ARM_CLAW_OPEN_DEG;RoadPoint h,f;
        int expected=(2-(orders[order][2]-'0'))*STN_STORAGE_INDEX_SIGN*STN_STORAGE_INDEX_MM;
        snprintf(task,sizeof(task),"task 134+%s+314+231",orders[order]);
        prepare_storage(0,task);run.plan=(uint8_t)plan;CHECK(road_point(NODE_H,&h)&&road_point(NODE_F,&f));
        start=clock_ms;deadline=clock_ms+120000;
        while(clock_ms<deadline) {
            float yaw,claw;int16_t mm;
            serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(strstr(output,"[station departure] parking finished"))parked=1;
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
            if(run.job==4 && !launched) {
                launched=1;depart=clock_ms;CHECK(placed==3 && run_active && !run.station_shift);
                if(STN_STORAGE_FIRST_THROUGH_ENABLED) {
                    CHECK(station_through_departure(&mm) && mm==expected && run.departure_overlap==2u);
                    CHECK(run.station_return_mm==expected && arm_busy());
                    CHECK(hypot(field_x-h.x,field_y-(h.y-expected))<35);
                } else CHECK(!station_through_departure(NULL) && !run.station_return_mm);
            }
            if(launched && STN_STORAGE_FIRST_THROUGH_ENABLED) {
                float target_yaw;
                if(run.departure_overlap==2u && arm_busy() && (run.forward_cmd_rpm || run.left_cmd_rpm))retract_moving=1;
                if(run.departure_overlap==2u && arm_busy()){arm_target_now(NULL,NULL,&target_yaw,NULL);CHECK(target_yaw==ARM_YAW_HOME_DEG);}
                CHECK(!strstr(output,"[station index] ring=2"));
                if(run.corner.active) {
                    double end_x=h.x+run.corner.first_mm;
                    double end_y=h.y-expected-run.corner.second_mm;
                    CHECK(run.corner.start_heading==HEAD_Y_POS && run.corner.prefix_mm==expected);
                    CHECK(hypot(end_x-f.x,end_y-f.y)<0.01);
                    if(!expected)joined=1;
                    else if(fabs(run.corner.s_mm-run.corner.legs[0].end_s)<25) {
                        CHECK(run.wheel_rpm[0]||run.wheel_rpm[1]||run.wheel_rpm[2]||run.wheel_rpm[3]);joined=1;
                    }
                }
                if(run.state==RUN_STATION)break;
            } else if(launched)break;
        }
        CHECK(clock_ms<deadline && launched && placed==3 && peer_disc_count==1 && !peer_stack_count);
        if(STN_STORAGE_FIRST_THROUGH_ENABLED) {
            CHECK(joined && retract_moving && run.state==RUN_STATION && run.job==4);
            CHECK(parked && !run.departure_overlap && arm_pose_known() && !arm_failed() && arm_platform_slot()==1);
            CHECK(hypot(field_x-f.x,field_y-f.y)<80);
        }
        printf("STORAGE_FIRST plan=%u order=%s through=%u return_mm=%d depart_ms=%lu total_ms=%lu overlap=%u end_mm=%.3f,%.3f\n",
            plan,orders[order],(unsigned)STN_STORAGE_FIRST_THROUGH_ENABLED,expected,
            (unsigned long)(depart-start),(unsigned long)(clock_ms-start),retract_moving,field_x,field_y);
        stop_request();
    }
}

static void second_storage_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        uint32_t deadline;unsigned offset=0;
        prepare_storage(1,"task 134+123+314+231");run.plan=(uint8_t)plan;deadline=clock_ms+150000;
        while(run.job==6 && clock_ms<deadline) {
            serve_peer(&offset);plant_step();CHECK(!station_through_departure(NULL));
        }
        CHECK(clock_ms<deadline && run.job==7 && !run.station_return_mm);
        CHECK(peer_disc_count==0 && peer_stack_count==3 && (arm_ready() || arm_departure_rotation_active()));
        stop_request();
    }
    puts("PASS second storage: three color F requests, no E, no through-prefix, original stacking and return path retained");
}

static void measured_clearance_tests(void)
{
    unsigned variant;
    if(!STN_STORAGE_FIRST_THROUGH_ENABLED)return;
    for(variant=0;variant<3;variant++) {
        uint32_t deadline;unsigned offset=0,placed=0,positions;int64_t base,counts,one;
        float previous=ARM_CLAW_OPEN_DEG;
        prepare_storage(0,"task 134+123+314+231");deadline=clock_ms+100000;
        while(placed<3 && clock_ms<deadline) {
            float yaw,claw;serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
        }
        CHECK(placed==3 && run.job==3 && !run_active);
        base=arm_encoder[1];positions=arm_positions;arm_freeze=1;
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[positions]==ARM_LIFT_ADDRESS);
        counts=(int64_t)((ARM_CLEARANCE_MM-STN_STORAGE_LIFT_MM)*ARM_LIFT_PULSES_PER_MM)
            *ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        if(ARM_LIFT_DIR_INVERT)counts=-counts;
        counts*=ARM_LIFT_ENCODER_SIGN;
        one=(int64_t)(ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        arm_encoder[1]=base+counts-(counts>0 ? one : -one);
        tick(600);CHECK(run.job==3 && !run_active && !station_through_departure(NULL));
        if(variant==2) {stop_request();tick(100);CHECK(!run_active && !station_active() && arm_failed());continue;}
        if(variant==1) {
            arm_missing_motor=ARM_LIFT_ADDRESS;arm_encoder[1]=base+counts;
            tick(ARM_FEEDBACK_TIMEOUT_MS+200);CHECK(run.job==3 && !run_active && !station_through_departure(NULL));
            arm_missing_motor=0;
        }
        arm_encoder[1]=base+counts;
        while(run.job==3 && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.job==4 && run.departure_overlap==2 && run.station_return_mm==150);
        stop_request();
    }
    puts("PASS first-storage clearance:124mm or missing lift feedback holds station, actual125mm releases; manual ! cancels");
}

int main(void)
{
    first_storage_tests();second_storage_tests();measured_clearance_tests();
    printf("PASS first storage through: %u assertions; fixed ring geometry and command overlap only, not physical placement acceptance\n",assertions);
    return 0;
}
