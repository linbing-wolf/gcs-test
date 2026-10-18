/* 真实入口、机构、工位、视觉模块联合模拟；不连接硬件。 */
#define HEADING_TEST_MAIN heading_regression_main
#include "verify_heading.c"

static uint8_t peer_crc(const uint8_t *bytes,unsigned count)
{
    uint8_t crc=0;unsigned i,bit;
    for(i=0;i<count;i++){crc^=bytes[i];for(bit=0;bit<8;bit++)crc=(uint8_t)((crc<<1)^((crc&128)?7:0));}
    return crc;
}
static void peer_reply(const char *payload,int bad_crc)
{
    uint8_t frame[64];unsigned n=(unsigned)strlen(payload),i;
    CHECK(n<=48);frame[0]=0xaa;frame[1]=0x55;frame[2]=(uint8_t)n;
    memcpy(frame+3,payload,n);frame[n+3]=(uint8_t)(peer_crc(frame+2,n+1)^bad_crc);frame[n+4]=13;
    for(i=0;i<n+5;i++)vision_receive_byte(frame[i],0);
}
static void tick(unsigned ms)
{
    unsigned i;for(i=0;i<ms;i++)plant_step();
}
static void prepare_arm_auto(unsigned plan)
{
    RoadPoint start;
    reset_plant(0,1.15);CHECK(road_point(plan?NODE_C:NODE_A,&start));
    planned_x=start.x;planned_y=start.y;field_x=start.x;field_y=start.y;initial_field_heading=plan?2:0;
    send_line("arm on");tick(500);CHECK(arm_enabled() && arm_lock.state==2);
    CHECK(!arm_claw_us && !arm_yaw_us && !arm_platform_us);
    send_line("arm pose 0 135 0 45");send_line("arm platform pose 20");
    CHECK(arm_ready() && arm_parked() && arm_platform_slot()==1);
    send_line("station auto on");CHECK(station_auto_enabled());
}
static void task_and_stop_tests(void)
{
    char status[1024];unsigned stops;
    static const char *bad[]={"task 534+123+314+231","task 134+126+314+231","task 134+123+314",
        "task 134++123+314+231","task 134+123+314+231+","task 1341233142319","task 134+123314231"};
    unsigned i;
    reset_plant(0,1.0);station_init();CHECK(station_auto_enabled());
    send_line("task 134+125+314+251");CHECK(strstr(output,"[task accepted"));
    stops=stop_count;send_line("station");CHECK(stop_count==stops);
    CHECK(strstr(output,"colors=134 rings=125") && strstr(output,"colors=314 rings=251"));
    CHECK(strlen(output)<sizeof(status));strcpy(status,output);
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        send_line(bad[i]);CHECK(strstr(output,"[task rejected"));send_line("station");CHECK(!strcmp(output,status));
    }
    send_line("task 134125314251");CHECK(strstr(output,"[task accepted"));send_line("station");CHECK(!strcmp(output,status));
    send_line("test");send_line("f");CHECK(drive.state==MOTION_RUNNING);
    send_line("s");firmware_step();CHECK(drive.state==MOTION_IDLE && !run_active);
    puts("PASS task/stop: compact and separated groups, rings 4/5, invalid atomic rollback, station command does not trigger s shortcut; newline s and immediate ! retained");
}
static void vision_tests(void)
{
    uint8_t color;int16_t x,y;unsigned i;
    reset_plant(0,1.0);
    send_line("vision pick 1extra");CHECK(vision_pick_state()==VISION_PICK_IDLE);
    vision_tx_ready=0;vision_request_pick(1,clock_ms);tick(20);
    CHECK(!vision_tx_count && vision_pick_state()==VISION_PICK_WAITING);
    send_line("imu");CHECK(strstr(output,"[imu]"));
    vision_tx_ready=1;tick(12);CHECK(vision_tx_count==8);
    CHECK(vision_tx_bytes[0]==0xaa && vision_tx_bytes[1]==0x55 && vision_tx_bytes[2]==3);
    CHECK(!memcmp(vision_tx_bytes+3,"C,1",3) && vision_tx_bytes[6]==peer_crc(vision_tx_bytes+2,4));
    peer_reply("PICK,1,12,-3",1);tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    peer_reply("PICK,2,12,-3",0);tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    peer_reply("PICK,1,99999999999999999999999999999999999,0",0);tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    peer_reply("PICK,1,12",0);tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    peer_reply("PICK,1,32767,-32768",0);tick(1);CHECK(vision_pick_state()==VISION_PICK_READY);
    vision_pick_result(&color,&x,&y);CHECK(color==1 && x==32767 && y==(-32767-1));
    vision_cancel_pick();peer_reply("PICK,1",0);vision_request_pick(1,clock_ms);tick(12);
    CHECK(vision_pick_state()==VISION_PICK_WAITING); /* cached pre-request bytes were fenced */
    vision_receive_byte(0,1);tick(1);CHECK(vision_pick_state()==VISION_PICK_FAULT);
    vision_cancel_pick();vision_request_pick(2,clock_ms);tick(12);
    vision_receive_byte(0xaa,0);vision_receive_byte(0x55,0);vision_receive_byte(12,0);tick(1);
    clock_ms+=201;vision_update(clock_ms);peer_reply("PICK,2",0);vision_update(clock_ms);
    CHECK(vision_pick_state()==VISION_PICK_READY);
    vision_cancel_pick();vision_request_pick(2,clock_ms);tick(12);
    for(i=0;i<300;i++)vision_receive_byte(0x55,0);
    tick(1);CHECK(vision_pick_state()==VISION_PICK_FAULT);
    vision_cancel_pick();vision_request_pick(3,clock_ms);tick(12);
    clock_ms+=20000;peer_reply("PICK,3",0);vision_update(clock_ms);CHECK(vision_pick_state()==VISION_PICK_TIMEOUT);
    vision_cancel_pick();clock_ms=UINT32_MAX-100u;vision_request_pick(4,clock_ms);
    clock_ms+=20000;vision_update(clock_ms);CHECK(vision_pick_state()==VISION_PICK_TIMEOUT);
    stop_request();CHECK(vision_pick_state()==VISION_PICK_IDLE);
    puts("PASS vision: exact CRC frame, nonblocking blocked TX, wrong/bad/stale/overflow/error frames, bounded signed coordinates, timeout and timer wrap; emergency cancellation");
}
static void loading_tests(void)
{
    unsigned i;uint32_t deadline;
    prepare_arm_auto(0);send_line("@");CHECK(arm_busy());deadline=clock_ms+180000;
    while(arm_busy() && clock_ms<deadline)plant_step();
    CHECK(!arm_failed() && arm_ready() && arm_parked() && arm_platform_slot()==1 && clock_ms<deadline);
    for(i=0;i<3;i++) {
        send_line("arm load1");CHECK(arm_busy());deadline=clock_ms+50000;
        while(arm_busy() && clock_ms<deadline)plant_step();
        CHECK(arm_ready() && arm_platform_slot()==(int)((i+1)%3+1) && clock_ms<deadline);
    }
    send_line("arm home");while(arm_busy())plant_step();CHECK(arm_parked());
    send_line("arm load3");tick(100);stop_request();
    CHECK(arm_failed() && !arm_pose_known() && !arm_platform_known() && !arm_platform_slot() && arm_lock.state==3);
    puts("PASS loading: @/load3 and three load1 actions cycle slots, leave cup before rotation, retain explicit pose/stop semantics and no arm-on PWM jump");
}
static unsigned serve_peer(unsigned *offset)
{
    unsigned served=0;
    while(vision_tx_count>=*offset+3u) {
        unsigned n=vision_tx_bytes[*offset+2u],total=n+5u;char message[24];
        if(vision_tx_count<*offset+total)break;
        CHECK(vision_tx_bytes[*offset]==0xaa && vision_tx_bytes[*offset+1]==0x55);
        CHECK(vision_tx_bytes[*offset+total-2u]==peer_crc(&vision_tx_bytes[*offset+2u],n+1u));
        if(n==3 && vision_tx_bytes[*offset+3u]=='C') {
            int slide,lift;float yaw,claw;
            arm_pose_now(&slide,&lift,&yaw,&claw);
            CHECK(run.state==RUN_STATION && (run.job==1 || run.job==4) && !run_active);
            CHECK(!arm_busy() && arm_ready());
            CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM);
            CHECK(yaw==ARM_PICK_YAW_DEG && claw==ARM_CLAW_OPEN_DEG);
            snprintf(message,sizeof(message),"PICK,%c,0,0",vision_tx_bytes[*offset+5u]);peer_reply(message,0);served++;
        }
        *offset+=total;
    }
    return served;
}
static void scan_handoff_tests(void)
{
    unsigned plan,preloaded;
    for(plan=0;plan<2;plan++)for(preloaded=0;preloaded<2;preloaded++) {
        uint32_t deadline;double scan_yaw;
        prepare_arm_auto(plan);
        if(preloaded)send_line("task 134+123+314+231");
        send_line(plan?"runimu C":"runimu A");deadline=clock_ms+30000;
        while(run.state!=RUN_STATION && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_STATION && !run.job && run.edge==1 && run.heading==HEAD_X_NEG);
        scan_yaw=physical_yaw;
        if(!preloaded)send_line("task 134+123+314+231");
        deadline=clock_ms+12000;
        while(run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();
            CHECK(run.target_heading==HEAD_X_NEG && fabs(physical_yaw-scan_yaw)<500);
            CHECK(!arm_positions);
            if(run.job==1 && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>=50)break;
        }
        CHECK(clock_ms<deadline && run_active && run.job==1 && run.edge==1 && run.progress_mm>=50);
        stop_request();
    }
    for(plan=0;plan<2;plan++) {
        uint32_t deadline;unsigned wheel;
        prepare_arm_auto(plan);send_line(plan?"runimu C":"runimu A");deadline=clock_ms+30000;
        while(run.state!=RUN_STATION && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_STATION && !run.job);
        missing_feedback_motor=4;send_line("task 134+123+314+231");deadline=clock_ms+5000;
        while(run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();
            for(wheel=0;wheel<4;wheel++)CHECK(!simulated_wheel_rpm[wheel]);
        }
        CHECK(run.state==RUN_FAULT && !run_active && !arm_positions && clock_ms<deadline);
    }
    puts("PASS B handoff A/C: scanned or preloaded task keeps -X heading directly into B-E without a 90-degree return; missing fresh wheel-4 feedback prevents departure");
}
static void raw_preparation_timing_tests(void)
{
    unsigned changed;
    for(changed=0;changed<2;changed++) {
        uint32_t start,deadline,first_yaw=0;uint16_t home_pwm;int slide,lift;float yaw,claw;
        prepare_arm_auto(0);
        if(changed)send_line("arm pose 10 110 0 0");
        send_line("task 134+123+314+231");home_pwm=arm_yaw_us;start=clock_ms;
        CHECK(station_start(NODE_F,1,clock_ms)==1);deadline=clock_ms+(changed?12000:3000);
        while(!station_failed() && vision_tx_count<8 && clock_ms<deadline) {
            station_update(clock_ms);plant_step();
            if(!first_yaw && arm_yaw_us!=home_pwm) {
                first_yaw=clock_ms;arm_pose_now(&slide,&lift,&yaw,&claw);
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM);
            }
        }
        CHECK(!station_failed() && clock_ms<deadline && vision_tx_count==8 && first_yaw);
        CHECK(clock_ms-start>=ARM_SERVO_TIME_MS);
        arm_pose_now(&slide,&lift,&yaw,&claw);
        CHECK(!arm_busy() && yaw==ARM_PICK_YAW_DEG && claw==ARM_CLAW_OPEN_DEG);
        if(!changed) {
            CHECK(first_yaw-start<500 && clock_ms-start<ARM_SERVO_TIME_MS+500 && !arm_positions);
            printf("RAW_READY_TIMING yaw_start_ms=%lu vision_request_ms=%lu; simulation only\n",
                (unsigned long)(first_yaw-start),(unsigned long)(clock_ms-start));
        } else {
            CHECK(arm_positions==2 && arm_trace_address[0]==5 && arm_trace_address[1]==7);
            CHECK(arm_trace_yaw_us[0]==home_pwm && arm_trace_yaw_us[1]==home_pwm);
            CHECK(clock_ms-start>=4u*ARM_SERVO_TIME_MS);
        }
        stop_request();
    }
    puts("PASS raw preparation: already parked/open pose starts yaw within 500ms and requests vision only after full 2s yaw; changed lift/slide/claw targets still execute in safe order with fresh feedback");
}
static void automatic_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,replies=0,waiting=0,last_job=0,raw_job=0;
        uint32_t deadline,raw_start=0;int sent=0;
        prepare_arm_auto(plan);send_line(plan?"runimu C":"runimu A");CHECK(run_active && run.station_enabled);
        scanner.state=SCANNER_READY;scanner_receive((const uint8_t *)"134+123+314+231\r\n",17);tick(1);
        send_line("station");CHECK(strstr(output,"colors=000 rings=000"));
        send_line("arm yaw 126");CHECK(strstr(output,"route/station owns motion"));
        deadline=clock_ms+2000000;
        while(run.state!=RUN_FAULT && run.state!=RUN_DONE && clock_ms<deadline) {
            if(run.state==RUN_STATION && run.job==0 && !sent) {
                waiting++;CHECK(!run_active && !arm_positions);
                /* 二维码板在+X，车尾装扫码器：实际车头须为-X，而非+X。 */
                CHECK(run.heading==HEAD_X_NEG && run.target_heading==HEAD_X_NEG && !run.scan_turn_stage);
                CHECK((run.heading+2u)%4u==HEAD_X_POS);
                /* 作业时四轮反馈由机构互锁消费，直接核对独立模拟轮速。 */
                {unsigned wheel;for(wheel=0;wheel<4;wheel++)CHECK(fabs(actual_rpm[wheel])<=CAR_VELOCITY_STOP_RPM);}
                if(waiting==1000) {
                    send_line("station auto off");CHECK(strstr(output,"rejected") && station_auto_enabled());
                    if(plan) {scanner.state=SCANNER_READY;scanner_receive((const uint8_t *)"134+123+314+231\r\n",17);}
                    else send_line("task 134+123+314+231");
                    sent=1;
                }
            }
            if(run.state==RUN_STATION && (run.job==1 || run.job==4) && raw_job!=run.job) {
                raw_job=run.job;raw_start=clock_ms;
            }
            {
                unsigned served=serve_peer(&offset);
                if(served) {
                    static const uint8_t colors[2][3]={{1,3,4},{3,1,4}};
                    CHECK(raw_start && clock_ms-raw_start>=ARM_SERVO_TIME_MS);
                    CHECK(served==1 && arm_platform_slot()==(int)(replies%3u+1u));
                    CHECK(vision_tx_bytes[offset-3u]==(uint8_t)('0'+colors[run.job==4][replies%3u]));
                }
                replies+=served;
            }
            plant_step();
            if(run.job!=last_job) {last_job=run.job;arm_positions=0;} /* 单工位保留完整轴轨迹，避免跨工位填满测试记录。 */
        }
#if STN_ROUGH_PLACE_READY && STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY
        if(run.state!=RUN_DONE)fprintf(stderr,"MOCK FLOW job=%u state=%u: %s\n",run.job,run.state,output);
        CHECK(sent && waiting>=1000 && replies==6 && clock_ms<deadline);
        CHECK(run.state==RUN_DONE && !run_active && run.job==8 && run.edge==16);
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && heading_reuses==3 && arm_ready() && arm_parked());
#else
        CHECK(sent && waiting>=1000 && replies==3 && clock_ms<deadline);
        CHECK(run.state==RUN_FAULT && !run_active && run.job==2 && run.edge==5);
        CHECK(!arm_pose_known() && !arm_platform_known());
        CHECK(strstr(output,"station action refused") || strstr(output,"station work failed"));
#endif
    }
#if STN_ROUGH_PLACE_READY && STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY
    puts("PASS automatic A/C full mock flow: 8 jobs, 13960mm and three uninterrupted E passes retained; test-only READY overrides with placeholder poses, NOT calibration or hardware acceptance");
#else
    puts("PASS automatic A/C: rear scanner faces +X only after heading -X and four-wheel stop; F yaw 126 and open claw before each vision request, ordered three slots/colors and parked resume; D uncalibrated fault without job advance");
#endif
}
static void automatic_fault_tests(void)
{
    unsigned fault;
    for(fault=0;fault<4;fault++) {
        uint32_t deadline;
        prepare_arm_auto(0);send_line("runimu A");deadline=clock_ms+30000;
        while(run.state!=RUN_STATION && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_STATION && !run.job && !arm_positions);
        if(fault==0)stop_request();
        else if(fault==1)send_line("obstacle");
        else if(fault==2)send_line("arm stop");
        else {moving_at_stop_motor=4;tick(2500);}
        CHECK(!run_active && !arm_positions && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        tick(100);CHECK(!run_active && !station_active());
    }
    puts("PASS station failures: emergency, obstacle during work, arm stop and wheel-4 motion stop all work with no automatic resume");
}
static void raw_preparation_fault_tests(void)
{
    unsigned fault;
    for(fault=0;fault<2;fault++) {
        uint32_t deadline;uint16_t home_pwm;
        prepare_arm_auto(0);send_line("task 134+123+314+231");
        CHECK(station_start(NODE_F,1,clock_ms)==1);home_pwm=arm_yaw_us;
        if(fault)arm_missing_motor=5;
        deadline=clock_ms+20000;
        while(station_active() && clock_ms<deadline) {
            station_update(clock_ms);plant_step();
            CHECK(!vision_tx_count && vision_pick_state()==VISION_PICK_IDLE && !arm_positions);
            if(!fault && arm_yaw_us!=home_pwm) {
                CHECK(arm_busy());stop_request();break;
            }
        }
        CHECK(clock_ms<deadline && !station_active() && arm_failed() && !arm_pose_known());
        if(fault)CHECK(station_failed());
        tick(100);CHECK(!vision_tx_count && !arm_positions);
    }
    puts("PASS raw preparation failures: emergency during yaw interpolation or missing lift feedback cannot issue vision request or begin pickup");
}
static void automatic_obstacle_return_test(void)
{
    uint32_t deadline;
    prepare_arm_auto(0);send_line("runimu A");deadline=clock_ms+30000;
    while(run_active && !(run.edge==0 && run.segment==1 && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>=180)
        && clock_ms<deadline)plant_step();
    CHECK(run_active && !run.job && run.segment==1 && arm_ready() && arm_parked());
    send_line("obstacle");CHECK(run.obstacle_phase==1 && run.state==RUN_OBSTACLE_STOP);
    while(run_active && clock_ms<deadline)plant_step();
    CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE && !run.job && clock_ms<deadline);
    CHECK(arm_ready() && arm_parked() && arm_lock.state==2 && !station_active() && !arm_positions);
    tick(500);CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE);
    puts("PASS automatic moving obstacle: parked enabled arm retained while current encoder return stops at previous node; no station job or automatic restart");
}
int main(void)
{
    task_and_stop_tests();vision_tests();loading_tests();scan_handoff_tests();raw_preparation_timing_tests();automatic_route_tests();automatic_fault_tests();raw_preparation_fault_tests();automatic_obstacle_return_test();
    printf("PASS station/vision integration: %u assertions; ready_flags=%d/%d/%d; simulation only\n",assertions,STN_ROUGH_PLACE_READY,STN_STORAGE_PLACE_READY,STN_PLATFORM_TAKE_READY);
    return 0;
}
