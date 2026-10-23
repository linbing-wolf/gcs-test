/* 真实工位/底盘/机构状态机联合验证，不连接硬件。 */
#define STATION_TEST_MAIN retained_station_regression
#include "verify_station_vision.c"

static void prepare_overlap(unsigned area)
{
    uint32_t deadline;unsigned offset=0;
    /* 首个目标环位2，使D/E后直接靠近；F后始终是平台取料靠近。 */
    if(area==0)prepare_rough(0,"task 134+213+314+123");
    else prepare_storage(area==2,"task 134+213+314+123");
    peer_stack_hold=1;deadline=clock_ms+60000;
    while(clock_ms<deadline) {
        if(area==2){serve_peer(&offset);if(peer_stack_count)break;}
        else if(vision_align_state()==VISION_PICK_WAITING && vision_tx_count>=6)break;
        plant_step();
    }
    CHECK(clock_ms<deadline && arm_ready());
}

static void overlap_tests(void)
{
    unsigned area,variant,direction;
    for(area=0;area<3u;area++)for(direction=0;direction<2u;direction++)for(variant=0;variant<5u;variant++) {
        uint32_t deadline,started=0,completed=0;unsigned positions,concurrent=0;
        float initial_claw,claw;int initial_slide,initial_lift,slide,lift;
        prepare_overlap(area);positions=arm_positions;
        arm_pose_now(&initial_slide,&initial_lift,NULL,&initial_claw);
        peer_reply(variant==1 ? (direction ? "MOVE,75" : "MOVE,-75")
                   : variant==2 ? (direction ? "MOVEY,44" : "MOVEY,-44")
                   : (direction ? "MOVEXY,75,44" : "MOVEXY,-75,-44"),0);
        peer_reply(area==2 ? "STACK,320,240,0,1" : "DISC,320,240,0,1",0);
        deadline=clock_ms+30000;
        while(clock_ms<deadline && !run.station_approach_overlap && run.state!=RUN_FAULT)plant_step();
        CHECK(clock_ms<deadline && run.station_approach_overlap && run_active && arm_busy());
        CHECK(abs(run.forward_cmd_rpm)<=6 && abs(run.left_cmd_rpm)<=6);
        CHECK(run.forward_cmd_rpm || run.left_cmd_rpm);
        if(run.vector_move) {
            CHECK(abs(run.vector_target_mm[0]-run.vector_progress_mm[0])<=5);
            CHECK(abs(run.vector_target_mm[1]-run.vector_progress_mm[1])<=5);
        } else CHECK(abs(run.action_mm-run.progress_mm)<=5);
        started=clock_ms;
        if(variant==3) {
            send_line("!");tick(100);
            CHECK(!run_active && !station_active() && arm_failed() && arm_lock.state==3);
            CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
            continue;
        }
        if(variant==4) {
            bus_off=1;tick(20);
            CHECK(!run_active && !station_active() && arm_failed());continue;
        }
        /* 故意中断一轮反馈，让靠近先完成；必须仍停留在当前修正步骤，不能闭爪。 */
        if(variant==0) {
            missing_feedback_motor=4;
            {unsigned hold;for(hold=0;hold<4000 && run_active;hold++)plant_step();}
            if(!(run_active && station_active() && !arm_busy() && arm_positions>positions))fprintf(stderr,"HOLD state=%d active=%d station=%d busy=%d positions=%u/%u %s\n",run.state,run_active,station_active(),arm_busy(),(unsigned)arm_positions,positions,output);
            CHECK(run_active && station_active() && !arm_busy() && arm_positions>positions);
            arm_pose_now(&slide,&lift,NULL,&claw);
            CHECK(claw==initial_claw && (slide!=initial_slide || lift!=initial_lift));
            missing_feedback_motor=0;
        }
        while(clock_ms<deadline && run_active && run.state!=RUN_FAULT) {
            plant_step();
            arm_pose_now(NULL,NULL,NULL,&claw);CHECK(claw==initial_claw);
            if(run_active && arm_positions>positions && (run.forward_cmd_rpm || run.left_cmd_rpm))concurrent++;
        }
        CHECK(clock_ms<deadline && run.state==RUN_STATION && arm_lock.state==2);
        completed=clock_ms;CHECK(arm_positions>positions && (concurrent || variant==0));
        CHECK(!run.station_relock);
        while(clock_ms<deadline) {
            plant_step();arm_pose_now(NULL,NULL,NULL,&claw);
            if(claw==ARM_CLAW_CLOSE_DEG)break;
        }
        CHECK(clock_ms<deadline && clock_ms>completed && arm_positions<=positions+3u);
        printf("PASS approach overlap area=%u axis=%u direction=%u start_ms=%lu correction_done_ms=%lu grip_ms=%lu\n",
               area,variant,direction,(unsigned long)started,(unsigned long)completed,(unsigned long)clock_ms);
        stop_request();
    }
}

static void eligibility_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4u;variant++) {
        uint32_t deadline;unsigned positions;
        prepare_overlap(0);positions=arm_positions;
        peer_reply("MOVEXY,-75,-44",0);
        if(variant==1)peer_reply("MOVE,-20",0);
        if(variant==1 || variant==2)peer_reply("DISC,320,240,0,1",0);
        deadline=clock_ms+10000;
        while(clock_ms<deadline && (!run.station_shift || run.velocity_phase!=VELOCITY_FORWARD))plant_step();
        CHECK(clock_ms<deadline && run_active);
        if(variant==2) {
            CHECK(abs(run.vector_target_mm[0]-run.vector_progress_mm[0])>5);
            CHECK(!station_chassis_approach_begin());
            stop_request();continue;
        }
        if(variant==3){missing_feedback_motor=4;peer_reply("DISC,320,240,0,1",0);}
        while(clock_ms<deadline && run.station_shift) {
            plant_step();CHECK(arm_positions==positions && !run.station_approach_overlap);
            if(variant==1 && run.vector_progress_mm[0]>=74)break;
        }
        CHECK(run.state!=RUN_FAULT);
        if(variant==3)CHECK(run_active && !arm_busy());
        stop_request();
    }
    puts("PASS overlap eligibility: terminal required, pending FIFO retained, far target and stale feedback cannot start approach");
}

#ifndef APPROACH_TEST_MAIN
#define APPROACH_TEST_MAIN main
#endif
int APPROACH_TEST_MAIN(void)
{
    overlap_tests();eligibility_tests();
    printf("PASS final-correction approach: %u assertions; host simulation only\n",assertions);
    return 0;
}
