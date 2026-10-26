/* 请求须早于下位机停稳确认；立即回包也不得提前执行下一动作。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void raw_h_and_c_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned correction=0;correction<3;correction++) {
        uint32_t deadline;unsigned positions;int saw_c=0,last_f=0,last_l=0,saw_shift=0;
        prepare_arm_auto(plan);imu_period_ms=20;send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+30000;
        while(vision_tx_count<6 && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && vision_tx_bytes[3]=='H');
        printf("EARLY_H plan=%u correction=%u active=%u entry=%u state=%u\n",plan,correction,run_active,run.station_entry_overlap,run.state);
        CHECK(run_active && run.station_entry_overlap);positions=arm_positions;
        if(correction)peer_reply("MOVEXY,12,5",0);
        if(correction==2)peer_reply("MOVEXY,-7,-9",0);
        peer_reply("DISC,320,240,0,1",0);
        while(clock_ms<deadline && !saw_c) {
            plant_step();
            if(run.station_shift) {saw_shift=1;last_f=run.vector_progress_mm[0];last_l=run.vector_progress_mm[1];}
            if(vision_tx_count>=14) {
                CHECK(vision_tx_bytes[9]=='C' && vision_tx_bytes[11]=='1');
                CHECK(run_active && (correction ? run.station_shift : run.station_entry_overlap));
                saw_c=1;peer_reply("PICK,1,320,240",0);
                printf("EARLY_C plan=%u correction=%u active=%u shift=%u\n",plan,correction,run_active,run.station_shift);
            }
            CHECK(arm_positions==positions);
        }
        CHECK(saw_c && clock_ms<deadline);
        while(run_active && clock_ms<deadline) {
            plant_step();
            if(run.station_shift) {last_f=run.vector_progress_mm[0];last_l=run.vector_progress_mm[1];}
            if(run_active)CHECK(arm_positions==positions);
        }
        CHECK(clock_ms<deadline && run.job==1);
        if(correction)CHECK(saw_shift && abs(last_f-(correction==2 ? 7 : -12))<=1 && abs(last_l-(correction==2 ? -9 : 5))<=1);
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_positions>positions && vision_tx_count==14);
        stop_request();CHECK(!station_active() && vision_pick_state()==VISION_PICK_IDLE);
    }
}

static void stack_early_reply_test(void)
{
    unsigned offset=0,positions;uint32_t deadline;
    prepare_storage(1,"task 134+231+314+123");peer_stack_hold=1;deadline=clock_ms+30000;
    while(!peer_stack_count && clock_ms<deadline){serve_peer(&offset);plant_step();}
    CHECK(clock_ms<deadline && peer_stack_count==1 && run_active && run.station_shift && !run.station_visual_shift);
    positions=arm_positions;peer_reply("MOVEY,8",0);peer_reply("STACK,320,240,0,1",0);
    while(run.station_shift && !run.station_visual_shift && clock_ms<deadline) {
        CHECK(arm_positions==positions);plant_step();
    }
    while(!run.station_visual_shift && clock_ms<deadline)plant_step();
    CHECK(clock_ms<deadline && run.remaining_mm==8 && run.travel_axis==BODY_LEFT);
    CHECK(vision_tx_count==8);stop_request();CHECK(!station_active() && vision_stack_state()==VISION_PICK_IDLE);
    peer_reply("MOVEY,99",0);peer_reply("STACK,320,240,0,1",0);tick(500);CHECK(!run_active);
    puts("PASS early F: request before fixed index completion; immediate MOVEY/STACK retained, actions wait; ! cancels");
}

static void entry_early_reply_tests(void)
{
    for(unsigned target=2;target<=3;target++) {
        unsigned offset=0,last_job=0,positions;uint32_t deadline;
        prepare_arm_auto(0);send_line("task 134+123+314+231");send_line("runimu A");deadline=clock_ms+200000;
        while(clock_ms<deadline) {
            if(run.job==target && vision_tx_count>=offset+6u)break;
            if(run.job!=target)serve_peer(&offset);
            plant_step();if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && vision_tx_bytes[offset+3u]==(target==2 ? 'D' : 'E'));
        CHECK(run_active && run.station_entry_overlap);positions=arm_positions;
        peer_reply("MOVEXY,15,7",0);peer_reply("DISC,320,240,0,1",0);
        while(run.station_entry_overlap && clock_ms<deadline) {
            CHECK(arm_positions==positions);plant_step();
        }
        while(!run.station_visual_shift && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.vector_target_mm[0]==-15 && run.vector_target_mm[1]==7);
        CHECK(vision_tx_count==offset+6u);stop_request();
        printf("PASS early %c: MOVE/terminal received during entry retained; correction begins after entry completion\n",target==2 ? 'D' : 'E');
    }
}

static void early_full_routes(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,counts[256]={0},early[256]={0};uint32_t start,deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+500000;
        while(run.state!=RUN_DONE && clock_ms<deadline) {
            unsigned before=offset;serve_peer(&offset);
            while(before<offset) {
                unsigned command=vision_tx_bytes[before+3];counts[command]++;
                if(station_chassis_busy())early[command]++;
                before+=vision_tx_bytes[before+2]+5u;
            }
            plant_step();
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && !run_active);
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        CHECK(counts['H']==2 && counts['D']==2 && counts['E']==1 && counts['F']==3 && counts['C']==6);
        CHECK(early['H']==2 && early['D']==2 && early['E']==1 && early['F']==3 && early['C']==2);
        printf("EARLY_ROUTE plan=%u total_ms=%lu distance=%lu H=%u/%u D=%u/%u E=%u/%u F=%u/%u C=%u/%u; host independently judges wheel stop\n",
            plan,(unsigned long)(clock_ms-start),(unsigned long)run.distance_mm,early['H'],counts['H'],early['D'],counts['D'],
            early['E'],counts['E'],early['F'],counts['F'],early['C'],counts['C']);
    }
}

int main(void)
{
    raw_h_and_c_tests();stack_early_reply_test();entry_early_reply_tests();early_full_routes();
    printf("PASS early vision: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
