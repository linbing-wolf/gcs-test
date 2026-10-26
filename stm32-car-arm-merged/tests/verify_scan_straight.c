/* 首码可延迟到B后直行或首次F停车；真实入口/路线/串口解析，主机模型。 */
#define STATION_TEST_MAIN retained_scan_station_suite
#include "verify_station_vision.c"

static void accept_qr(void)
{
    scanner_receive((const uint8_t *)"bad-task\r",9);CHECK(!station_task_ready());
    scanner_receive((const uint8_t *)"134+123+314+231\r",8);CHECK(!station_task_ready());
    scanner_receive((const uint8_t *)"134+123+314+231\r"+8,8);CHECK(station_task_ready());
}

static void late_scan_tests(void)
{
    unsigned plan,mode,baud;
    for(plan=0;plan<2;plan++)for(mode=0;mode<6;mode++)for(baud=0;baud<2;baud++) {
        uint32_t deadline,straight=0,waiting=0;unsigned initialised,positions;
        int sent=0,reached_f=0;
        prepare_arm_auto(plan);imu_period_ms=20;scanner.baud=baud ? 115200u : 9600u;
        scanner.state=SCANNER_READY;scanner_receive((const uint8_t *)"314+231+",8);
        initialised=scanner_init_calls;
        if(mode==3)send_line("task 134+123+314+231");
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+45000;positions=arm_positions;
        CHECK(scanner_init_calls==initialised+1 && !scanner.length);
        CHECK(scanner.baud==(baud ? 115200u : 9600u));
        while(clock_ms<deadline && run.state!=RUN_FAULT) {
            if(mode!=2)scanner_peer_ack(); /* 缺配置ACK也继续解析任务文本。 */
            plant_step();
            CHECK(!(run.state==RUN_STATION && !run.job)); /* B不因缺码停等。 */
            if(mode==4 && !sent && !run.edge && run.travel_axis==BODY_LEFT && run.progress_mm>=150) {
                accept_qr();sent=1;
            }
            if(mode==5 && !sent && run.edge==2 && run.job==1 && run.left_cmd_rpm) {
                accept_qr();sent=1;
            }
            if(run.edge==1 && run.job==1 && run.travel_axis==BODY_FORWARD && run.forward_cmd_rpm) {
                if(!straight)straight=clock_ms;
                if(mode==0 && !sent && clock_ms-straight>=500) {
                    CHECK(run_active && !station_task_ready());accept_qr();sent=1;
                }
            }
            if(!station_task_ready())CHECK(arm_positions==positions);
            if(run.state==RUN_STATION && run.job==1 && run.edge==3 && !run.station_phase) {
                CHECK(!run_active && !station_active());reached_f=1;
                if(!waiting)waiting=clock_ms;
                if(mode==1 && !sent && clock_ms-waiting>=1000) {accept_qr();sent=1;}
                if(mode==2 && clock_ms-waiting>=1200)break;
            }
            if(station_active() && run.job==1 && run.station_phase)break;
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && straight);
        if(mode==2) {
            CHECK(reached_f && !station_task_ready() && !run_active && !station_active());
            CHECK(arm_positions==positions && scanner.state==SCANNER_FAULT);
            CHECK(strstr(output,"station action refused")==NULL);
            accept_qr();sent=1;
            for(unsigned j=0;j<500 && !station_active();j++)plant_step();
        }
        CHECK(station_task_ready() && station_active() && run.job==1 && run.station_phase);
        CHECK(mode==3 || sent);CHECK((mode!=1 && mode!=2) || reached_f);
        CHECK(scanner_init_calls==initialised+1);
        scanner_receive((const uint8_t *)"314+231+134+123\r",16);send_line("station");
        CHECK(strstr(output,"batch1 colors=134 rings=123") && strstr(output,"batch2 colors=314 rings=231"));
        printf("SCAN_STRAIGHT corners=%u plan=%u mode=%u baud=%lu straight_ms=%lu f_wait=%u init_once=1\n",
            (unsigned)CAR_RUN_CORNER_ENABLED,plan,mode,(unsigned long)scanner.baud,(unsigned long)straight,reached_f);
        stop_request();CHECK(!station_task_ready());
        scanner_receive((const uint8_t *)"134+123+314+231\r",16);tick(20);CHECK(!station_task_ready());
    }
}

static void full_round_late_scan_tests(void)
{
    unsigned plan;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0;int sent=0,repeat=0;uint32_t deadline,start;
        int32_t first,second;
        prepare_arm_auto(plan);imu_period_ms=20;send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+300000;
        while(clock_ms<deadline && run.state!=RUN_DONE && run.state!=RUN_FAULT) {
            scanner_peer_ack();
            if(!sent && run.edge==1 && run.job==1 && run.forward_cmd_rpm) {accept_qr();sent=1;}
            if(!repeat && run.job>=4 && run.job<7) {
                scanner_receive((const uint8_t *)"314+231+134+123\r",16);repeat=1;
            }
            if(sent && run.job<8) {
                station_task_words(&first,&second);CHECK(first==134123 && second==314231);
            }
            serve_peer(&offset);plant_step();
            if(run.job!=last_job) {last_job=run.job;arm_positions=0;}
        }
        CHECK(sent && repeat && run.state==RUN_DONE && run.job==8 && clock_ms<deadline);
        CHECK(scanner_init_calls==1 && !station_task_ready() && arm_ready() && arm_parked());
        CHECK(peer_area_count[1]==1 && peer_area_count[2]==1 && peer_area_count[3]==1
            && peer_area_count[4]==1 && peer_area_count[5]==1 && peer_area_count[6]==0 && peer_stack_count==3);
        scanner_receive((const uint8_t *)"134+123+314+231\r",16);tick(20);CHECK(!station_task_ready());
        printf("SCAN_FULL_ROUND plan=%u jobs=%u distance_mm=%u model_ms=%lu init_once=1 task_expired=1\n",
            plan,run.job,run.distance_mm,(unsigned long)(clock_ms-start));
        stop_request();
    }
}

static void manual_and_cancel_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        uint32_t deadline;
        prepare_arm_auto(plan);send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+20000;
        while(clock_ms<deadline && !(run.edge==1 && run.forward_cmd_rpm))plant_step();
        CHECK(clock_ms<deadline && run_active && !station_task_ready());
        send_line("!");plant_step();CHECK(!run_active && !station_task_ready());
        scanner_receive((const uint8_t *)"134+123+314+231\r",16);tick(20);CHECK(!station_task_ready());
        prepare_arm_auto(plan);send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+45000;
        while(clock_ms<deadline && !(run.state==RUN_STATION && run.job==1 && run.edge==3))plant_step();
        CHECK(clock_ms<deadline && !station_task_ready());send_line("task 134+123+314+231");
        for(unsigned j=0;j<500 && !station_active();j++)plant_step();
        if(!station_task_ready() || !station_active())fprintf(stderr,"MANUAL_F plan=%u task=%u active=%u lock=%u phase=%u scan=%u output=%s\n",plan,station_task_ready(),station_active(),arm_lock.state,run.station_phase,run.scan_turn_stage,output);
        CHECK(station_task_ready() && station_active());stop_request();
    }
}

int main(void)
{
    late_scan_tests();manual_and_cancel_tests();full_round_late_scan_tests();
    printf("PASS scan during straight: %lu assertions; no B scan wait; task required at first F; host model only\n",(unsigned long)assertions);
    return 0;
}
