/* 真实入口/机构/工位与独立轮速模型：离站必须已放下末块，收臂与底盘并行。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void rough_release_before_departure(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    for(unsigned batch=0;batch<2;batch++)for(unsigned order=0;order<6;order++) {
        char task[48];unsigned offset=0,grips=0,releases=0,moving_home=0;uint32_t deadline;
        float previous=ARM_CLAW_OPEN_DEG;int expected=(2-(orders[order][2]-'0'))*STN_ROUGH_INDEX_SIGN*STN_ROUGH_INDEX_MM;
        snprintf(task,sizeof(task),"task 134+%s+314+%s",orders[order],orders[order]);
        prepare_rough(batch,task);deadline=clock_ms+150000;
        while(clock_ms<deadline) {
            int lift,slide;float yaw,claw;
            serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_CLOSE_DEG && yaw==STN_ROUGH_YAW_DEG)grips++;
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG) {
                CHECK(slide==ARM_PLACE_SLIDE_MM && lift==ARM_PLACE_LIFT_MM);releases++;
            }
            previous=claw;
            if(grips==3 && releases<3)CHECK(run.job==(batch ? 5 : 2) && !run_active && !station_through_departure(NULL));
            if(run.departure_overlap==2) {
                CHECK(releases==3 && grips==3 && claw==ARM_CLAW_OPEN_DEG);
                CHECK(run.station_return_mm==expected || run.corner.prefix_mm==expected);
                if(arm_busy() && (run.forward_cmd_rpm || run.left_cmd_rpm))moving_home=1;
            }
            if(run.job==(batch ? 6 : 3) && !run.departure_overlap)break;
        }
        CHECK(clock_ms<deadline && releases==3 && moving_home && arm_parked());stop_request();
    }
    puts("PASS rough release: both batches/all6 ring orders; never depart holding final block; original ring2 prefix; return overlaps chassis");
}

static void raw_departure_during_lift(void)
{
    for(unsigned batch=0;batch<2;batch++) {
        unsigned offset=0,last_job=0,releases=0;uint32_t deadline;float previous=ARM_CLAW_OPEN_DEG;
        int64_t lift_position;unsigned source=batch ? 4u : 1u;
        prepare_arm_auto(0);send_line("task 134+123+314+231");send_line("runimu A");deadline=clock_ms+200000;
        while(clock_ms<deadline) {
            float yaw,claw;int lift;
            unsigned prior_job=run.job;
            serve_peer(&offset);plant_step();arm_pose_now(NULL,&lift,&yaw,&claw);
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
            if(prior_job==source && claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG)releases++;
            previous=claw;
            if(releases==3) {CHECK(lift==ARM_PLACE_LIFT_MM);break;}
        }
        CHECK(clock_ms<deadline && releases==3);arm_freeze=1;lift_position=arm_encoder[1];
        deadline=clock_ms+2000;
        while(!(run.departure_overlap && (run.forward_cmd_rpm || run.left_cmd_rpm)) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.job==source+1 && arm_busy() && arm_encoder[1]==lift_position);
        CHECK(arm_departure_return_active());stop_request();CHECK(!run_active && arm_failed());
    }
    puts("PASS raw departure: both visits move while released gripper still lifting from platform110mm, no full-retract wait; ! retained");
}

static void full_departure_and_turn_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,departure_source=0,mask=0,turns=0,nonzero_turn_entries=0;
        uint32_t deadline,start;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+400000;
        while(run.state!=RUN_DONE && clock_ms<deadline) {
            unsigned edge=run.edge;int was_active=run_active,was_speed=run.forward_cmd_rpm || run.left_cmd_rpm;
            serve_peer(&offset);plant_step();
            if(strstr(output,"[station turn overlap]")) {
                CHECK(was_active && run.edge==edge+1u && run.station_align_pending && run.velocity_phase==VELOCITY_TURN);
                CHECK(!run.stopped_ms);
                turns++;if(was_speed)nonzero_turn_entries++;
            }
            if(run.job!=last_job) {
                if(run.departure_overlap)departure_source=last_job;
                last_job=run.job;arm_positions=0;
            }
            if(run.departure_overlap && arm_busy() && (run.forward_cmd_rpm || run.left_cmd_rpm))mask|=1u<<departure_source;
            CHECK(!strstr(output,"arm left parked pose during route"));
        }
        CHECK(clock_ms<deadline && run.job==8 && mask==0x7eu && turns>=2 && nonzero_turn_entries>=2);
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && peer_stack_count==3);
        printf("DEPARTURE_ROUTE plan=%u elapsed_ms=%lu all6_overlap=%u station_turns=%u before_zero_command=%u endpoint_model_mm=%.3f\n",
            plan,(unsigned long)(clock_ms-start),mask,turns,nonzero_turn_entries,checkpoint_max_error_mm);
    }
}

static void platform_release_timing_tests(void)
{
    for(unsigned source=0;source<2;source++) {
        uint32_t deadline,yaw_target=0,ready;char pose[64];
        prepare_arm_auto(0);
        snprintf(pose,sizeof(pose),"arm pose %d %d %.1f %.1f",source ? ARM_PICK_SLIDE_MM : STN_ROUGH_SLIDE_MM,
            source ? ARM_PICK_LIFT_MM : STN_ROUGH_LIFT_MM,
            (double)(source ? ARM_PICK_YAW_DEG : STN_ROUGH_YAW_DEG),(double)ARM_CLAW_CLOSE_DEG);
        send_line(pose);CHECK(arm_ready());
        CHECK(arm_transfer_to(ARM_PLACE_SLIDE_MM,ARM_PLACE_LIFT_MM,ARM_PLACE_YAW_DEG,1,clock_ms));
        deadline=clock_ms+5000;
        while(!arm_ready() && clock_ms<deadline) {
            plant_step();if(arm_yaw_us==553 && !yaw_target)yaw_target=clock_ms;
            CHECK(arm_claw_us==604);
        }
        ready=clock_ms;CHECK(ready<deadline && yaw_target);
#if ARM_CUP_EXIT_DESCENT_DELAY_MS == 0
        CHECK(ready-yaw_target<ARM_YAW_SETTLE_MS); /* 云台最后到位时也不再多等40ms。 */
#endif
        {int slide,lift;float yaw;arm_pose_now(&slide,&lift,&yaw,NULL);
         CHECK(slide==ARM_PLACE_SLIDE_MM && lift==ARM_PLACE_LIFT_MM && yaw==ARM_PLACE_YAW_DEG);}
        CHECK(arm_command("claw angle 45",clock_ms));
        while(arm_claw_us==604 && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && clock_ms-ready<=ARM_CLAW_TIME_MS+20u);
        printf("PLATFORM_RELEASE source=%s yaw_to_ready_ms=%lu ready_to_open_pwm_ms=%lu descent_delay_ms=%u\n",
            source ? "raw" : "rough",(unsigned long)(ready-yaw_target),(unsigned long)(clock_ms-ready),(unsigned)ARM_CUP_EXIT_DESCENT_DELAY_MS);
        stop_request();
    }
}

static void storage_index_clearance_retained(void)
{
    for(unsigned batch=0;batch<2;batch++) {
        unsigned offset=0,placed=0,positions;uint32_t deadline;float previous=ARM_CLAW_OPEN_DEG;
        int64_t base,counts,one;int source=batch ? 6 : 3;
        prepare_storage(batch,"task 134+123+314+231");deadline=clock_ms+100000;
        while(placed<1 && clock_ms<deadline) {
            float yaw,claw;serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
        }
        CHECK(clock_ms<deadline && run.job==source && !run_active);
        base=arm_encoder[1];positions=arm_positions;arm_freeze=1;
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[positions]==ARM_LIFT_ADDRESS);
        counts=(int64_t)((ARM_CLEARANCE_MM-STN_STORAGE_LIFT_MM-(batch ? STN_STORAGE_LAYER_LIFT_MM : 0))*ARM_LIFT_PULSES_PER_MM)
            *ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        if(ARM_LIFT_DIR_INVERT)counts=-counts;
        counts*=ARM_LIFT_ENCODER_SIGN;
        one=(int64_t)(ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        arm_encoder[1]=base+counts-(counts>0 ? one : -one);
        tick(500);CHECK(run.job==source && !run_active && !station_through_departure(NULL));
        arm_encoder[1]=base+counts;
        while(!(run.forward_cmd_rpm || run.left_cmd_rpm) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.job==source && run_active && station_index_overlap_active()
            && arm_busy() && arm_encoder[1]==base+counts);
        stop_request();
    }
    puts("PASS storage index: both visits retain actual124/125mm boundary between rings; final departure tested separately");
}

int main(void)
{
    platform_release_timing_tests();rough_release_before_departure();raw_departure_during_lift();storage_index_clearance_retained();full_departure_and_turn_tests();
    printf("PASS departure overlap: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}

