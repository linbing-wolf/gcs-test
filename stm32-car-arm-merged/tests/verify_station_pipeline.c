/* 运行真实状态机；独立控制升降编码器回包，验证离物、并行和地面动作汇合。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static int64_t axis_delta_counts(int axis,float mm)
{
    float ppm=axis ? ARM_LIFT_PULSES_PER_MM : ARM_SLIDE_PULSES_PER_MM;
    int64_t pulses=(int64_t)(fabsf(mm)*ppm+0.5f);
    int sign=mm<0 ? -1 : 1;
    if(axis ? ARM_LIFT_DIR_INVERT : ARM_SLIDE_DIR_INVERT)sign=-sign;
    return sign*pulses*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
}

static void clearance_and_join_tests(void)
{
    unsigned area,variant;
    for(area=0;area<3u;area++)for(variant=0;variant<3u;variant++) {
        unsigned offset=0,last_positions,j;uint32_t deadline;
        int slide,lift,target_slide,target_lift,threshold;
        int64_t lift_base,slide_base;float yaw,claw,previous=ARM_CLAW_OPEN_DEG;
        if(area==1)prepare_storage(0,"task 134+123+314+231");
        else prepare_rough(0,"task 134+123+314+231");
        deadline=clock_ms+150000;
        while(clock_ms<deadline && run.state!=RUN_FAULT) {
            serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if(claw!=previous && yaw==STN_ROUGH_YAW_DEG && lift==STN_ROUGH_LIFT_MM
                && slide==STN_ROUGH_SLIDE_MM && claw==(area==2 ? ARM_CLAW_CLOSE_DEG : ARM_CLAW_OPEN_DEG))break;
            previous=claw;
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && !run_active);
        lift_base=arm_encoder[1];slide_base=arm_encoder[0];arm_freeze=1;last_positions=arm_positions;
        while(arm_positions==last_positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[last_positions]==5);
        threshold=area==2 ? ARM_PLATFORM_TURN_START_LIFT_MM : ARM_CLEARANCE_MM;
        tick(500);CHECK(!run_active && !station_index_overlap_active() && !arm_station_clearance_ready(clock_ms));
        arm_encoder[1]=lift_base+axis_delta_counts(1,(float)(threshold-1-STN_ROUGH_LIFT_MM));
        tick(200);CHECK(!run_active && !station_index_overlap_active());
        if(variant==1) {
            arm_missing_motor=5;
            arm_encoder[1]=lift_base+axis_delta_counts(1,(float)(threshold-STN_ROUGH_LIFT_MM));
            tick(ARM_FEEDBACK_TIMEOUT_MS+200u);
            CHECK(!run_active && !station_index_overlap_active());
            arm_missing_motor=0;
        }
        if(variant==2) {
            stop_request();tick(100);
            CHECK(!run_active && !station_active() && arm_failed() && !station_index_overlap_active());
            continue;
        }
        arm_encoder[1]=lift_base+axis_delta_counts(1,(float)(threshold-STN_ROUGH_LIFT_MM));
        while(!run.station_shift && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && station_index_overlap_active() && arm_busy());
        CHECK(threshold<ARM_LIFT_HOME_MM); /* 实际未升到顶，也未完整收臂，就已启动索引。 */
        missing_feedback_motor=4; /* 底盘停留在预检，车载搬运可以完成，下一地面动作不能开始。 */
        arm_target_now(&target_slide,&target_lift,NULL,NULL);
        arm_encoder[1]=lift_base+axis_delta_counts(1,(float)(ARM_LIFT_HOME_MM-STN_ROUGH_LIFT_MM));
        arm_encoder[0]=slide_base+axis_delta_counts(0,(float)(target_slide-STN_ROUGH_SLIDE_MM));
        arm_freeze=0;
        for(j=0;j<2500u;j++) {
            plant_step();arm_target_now(NULL,&target_lift,NULL,NULL);
            CHECK(run_active && station_index_overlap_active() && target_lift>=STN_ROUGH_PLATFORM_TAKE_LIFT_MM);
            CHECK(run.state!=RUN_FAULT);
        }
        missing_feedback_motor=0;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.station_relock && arm_lock.state==2);
        while(station_index_overlap_active() && clock_ms<deadline)plant_step();
        CHECK(!station_index_overlap_active());
        printf("PIPELINE_CLEARANCE area=%u delayed_lift=%u height_mm=%d; no index below clearance, next ground action waits for chassis\n",area,variant,threshold);
        stop_request();
    }
}

static void full_route_pipeline_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2u;plan++) {
        unsigned offset=0,entry=0,early=0,pwm_move=0,board_move=0,entry_tx=0,last_job=0;
        uint32_t deadline,start;int previous_index=0;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=route_timing.last_ms;deadline=clock_ms+300000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            uint16_t pwm=arm_yaw_us;unsigned positions=arm_positions;
            int was_entry=run.station_entry_overlap;
            serve_peer(&offset);plant_step();
            if(run.station_entry_overlap && !was_entry) {
                RoadHeading heading;
                CHECK(run_station_heading(full_route_jobs[run.plan][run.job],&heading));
                CHECK(abs(heading_wrap_cdeg(CAR_MAP_YAW_SIGN*(int32_t)heading*9000
                    - ((int32_t)heading_snapshot().yaw_cdeg-run.yaw_zero_cdeg)))<=STN_APPROACH_OVERLAP_YAW_CDEG);
                entry|=1u<<run.job;entry_tx=vision_tx_count;
                if(run.corner.active && run.corner.total_mm-run.corner.s_mm>100)early|=1u<<run.job;
                if(!run.corner.active && !run.station_align_pending && run.action_mm-run.progress_mm>100)early|=1u<<run.job;
            }
            if(run.station_entry_overlap) {
                int slide,lift;arm_pose_now(&slide,&lift,NULL,NULL);
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM && arm_positions==positions);
                CHECK(vision_tx_count==entry_tx);
                if(pwm!=arm_yaw_us && (run.forward_cmd_rpm || run.left_cmd_rpm))pwm_move|=1u<<run.job;
            }
            if(station_index_overlap_active() && run.station_shift) {
                CHECK(run.job==2 || run.job==3 || run.job==5); /* 第二批F/STACK索引未改为并行车载作业。 */
                if(previous_index && arm_positions>positions && (run.forward_cmd_rpm || run.left_cmd_rpm))board_move|=1u<<run.job;
            }
            previous_index=run.station_shift && station_index_overlap_active();
            if(run.job!=last_job) {last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        CHECK(entry==0x7eu && (early&0x5au)==0x5au && (pwm_move&0x5au)==0x5au && board_move==0x2cu);
        CHECK(peer_area_count[1]==1 && peer_area_count[2]==1 && peer_area_count[3]==1
            && peer_area_count[4]==1 && peer_area_count[5]==1 && !peer_area_count[6] && peer_stack_count==3);
        CHECK(arm_ready() && arm_parked() && arm_platform_slot()==1);
        printf("PIPELINE_FULL plan=%u total_ms=%lu entry=%02x early=%02x moving_yaw=%02x moving_board_axes=%02x; six visits,8 jobs,13960mm,three second-storage F requests\n",
            plan,(unsigned long)(clock_ms-start),entry,early,pwm_move,board_move);
        stop_request();
    }
}

int main(void)
{
    clearance_and_join_tests();full_route_pipeline_tests();
    printf("PASS station pipeline: %u assertions; command timing and feedback simulation only, no physical grip/clearance acceptance\n",assertions);
    return 0;
}
