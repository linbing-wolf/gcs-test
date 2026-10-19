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
    {
        char longest[49];
        memset(longest,'Z',48);longest[48]=0;
        peer_reply(longest,0);tick(1);send_line("vision status");
        CHECK(strstr(output,"[vision] state=idle") && strstr(output," timeout=0\r\n"));
        CHECK(strstr(output,longest) && strstr(output,"last=\"") && strstr(output,"\"\r\n"));
    }
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

static void task_round_lifetime_tests(void)
{
    unsigned ending;
    for(ending=0;ending<3;ending++) {
        reset_plant(0,1.15);
        send_line("task 134+123+314+231");
        if(ending==0)run_done();
        else if(ending==1) {run.station_enabled=1;run_fault("test round cancellation");}
        else stop_request();
        send_line("station");
        CHECK(strstr(output,"colors=000 rings=000") && !strstr(output,"colors=134"));
        CHECK(station_start(NODE_B,0,clock_ms)==1 && station_active());
        station_update(clock_ms+1000);CHECK(station_active());
        CHECK(station_command("task 314+231+134+123"));
        station_update(clock_ms+1001);CHECK(!station_active() && !station_failed());
        station_status();CHECK(strstr(output,"colors=314 rings=231") && strstr(output,"colors=134 rings=123"));
        station_abort();station_status();CHECK(strstr(output,"colors=000 rings=000"));
    }
    /* 整圈结束后相机仍吐出旧码，也不能在闲置状态把它重新缓存。 */
    reset_plant(0,1.15);send_line("task 134+123+314+231");run_done();
    scanner.state=SCANNER_READY;scanner_receive((const uint8_t *)"134+123+314+231\r\n",17);tick(1);
    send_line("station");CHECK(strstr(output,"colors=000 rings=000"));
    /* 新一轮明确手输仍可预置；独立定角/横移完成不算整轮结束。 */
    send_line("task 314+231+134+123");run.turn_only=1;run_done();
    send_line("station");CHECK(strstr(output,"colors=314 rings=231"));
    run.turn_only=0;run.translation_only=1;run_done();
    send_line("station");CHECK(strstr(output,"colors=314 rings=231"));
    puts("PASS task lifetime: full round completion/fault/stop expire both batches; B waits until new input; idle scanner cannot reload old code; fresh manual preload and standalone diagnostics retained");
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
static unsigned peer_geometry, peer_disc_count, peer_move_count;
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
        } else if(n==1 && vision_tx_bytes[*offset+3u]=='R') {
            peer_reply("READY",0);
        } else if(n==1 && vision_tx_bytes[*offset+3u]=='D') {
            int dx=0;
            CHECK(run.state==RUN_STATION && (run.job==2 || run.job==5) && !run_active);
            CHECK(station_chassis_pose_safe() && arm_lock.state==2);
            peer_disc_count++;
            if(peer_geometry==2) dx=30;
            else if(peer_geometry) {
                RoadPoint point;double target;
                CHECK(road_point(NODE_D,&point));
                target=point.x+round((field_x-point.x)/150.0)*150.0;
                dx=(int)round((target-field_x)*3.0);
            }
            if(abs(dx)>5) {
                if(dx>40)dx=40;
                if(dx< -40)dx=-40;
                snprintf(message,sizeof(message),"MOVE,%d",dx);peer_move_count++;
            } else snprintf(message,sizeof(message),"DISC,%d,240,%d,1",320+dx,dx);
            peer_reply(message,0);
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
            CHECK(clock_ms-start>=ARM_SERVO_TIME_MS+ARM_CLAW_TIME_MS);
            CHECK(clock_ms-start<ARM_SERVO_TIME_MS+ARM_CLAW_TIME_MS+1000u);
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
        /* 同一上电会话再跑一轮：不得继承上一轮码；首个B必须等新码。 */
        send_line("station");CHECK(strstr(output,"task_ready=0") && strstr(output,"colors=000 rings=000"));
        send_line(plan?"runimu C":"runimu A");deadline=clock_ms+30000;
        while(run.state!=RUN_STATION && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_STATION && run.job==0 && run.edge==1);
        tick(1000);CHECK(run.state==RUN_STATION && !run.job && !run_active);
        send_line("task 314+231+134+123");deadline=clock_ms+3000;
        while(!run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run_active && run.job==1 && run.edge==1);
        stop_request();
#else
        CHECK(sent && waiting>=1000 && replies==3 && clock_ms<deadline);
#if STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM)
        CHECK(run.state==RUN_FAULT && !run_active && run.job==3 && run.edge==7);
#else
        CHECK(run.state==RUN_FAULT && !run_active && run.job==2 && run.edge==5);
#endif
        CHECK(!arm_pose_known() && !arm_platform_known());
        CHECK(strstr(output,"station action refused") || strstr(output,"station work failed"));
#endif
        send_line("station");CHECK(strstr(output,"task_ready=0") && strstr(output,"colors=000 rings=000"));
    }
#if STN_ROUGH_PLACE_READY && STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY
    puts("PASS automatic A/C full mock flow: 8 jobs, 13960mm and three uninterrupted E passes retained; test-only READY overrides with placeholder poses, NOT calibration or hardware acceptance");
#else
    puts("PASS automatic A/C: raw loading retained; configured rough completes before protected storage; disabled rough refuses before job advance");
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

static void prepare_rough(unsigned batch, const char *task)
{
    RoadPoint point;
    prepare_arm_auto(batch & 1u);send_line(task);
    CHECK(road_point(NODE_D,&point));
    field_x=planned_x=point.x;field_y=planned_y=point.y;initial_field_heading=HEAD_X_POS;
    physical_yaw=origin_yaw=0;sample_yaw(0);
    run.state=RUN_STATION;run.station_enabled=1;run.plan=(uint8_t)(batch & 1u);
    run.job=(uint8_t)(batch ? 5 : 2);run.edge=(uint8_t)(batch ? 11 : 5);
    run.heading=run.target_heading=HEAD_X_POS;run.yaw_zero_cdeg=-9000;
    run.distance_mm=4567;run.steps=19;run.completed_roads=0x123;
    CHECK(road_plan(NODE_E,NODE_D,&run.road));
    checked_job=run.job;
    peer_geometry=peer_disc_count=peer_move_count=0;
}

static void rough_index_tests(void)
{
    static const uint8_t permutations[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    unsigned batch,variant,i;
    int configured=STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY
        && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM || STN_ROUGH_YAW_DEG);
    if(!configured) {
        prepare_rough(0,"task 134+123+314+231");firmware_step();
        CHECK(run.state==RUN_FAULT && !run_active && !arm_positions && !velocity_batches);
        CHECK(strstr(output,"not calibrated"));
        puts("PASS rough calibration guard: rough/platform take not ready or zero rough pose refuses before arm/index movement");
        return;
    }
    for(batch=0;batch<2;batch++)for(variant=0;variant<6;variant++) {
        char task[48];uint32_t deadline;unsigned offset=0,events=0,expected_count=0,placed=0,taken=0;
        uint8_t targets[16],ring=2;int moves[16];float prior_claw=ARM_CLAW_OPEN_DEG;
        unsigned positions_during_index=0,was_index=0;
        const uint8_t *order=permutations[variant];
        snprintf(task,sizeof(task),"task 134+%u%u%u+314+%u%u%u",order[0],order[1],order[2],order[0],order[1],order[2]);
        prepare_rough(batch,task);
        for(i=0;i<7;i++) {
            uint8_t target=i==6 ? 2u : (uint8_t)(i%3+1);
            if(ring!=2 && ring!=target) {targets[expected_count]=2;moves[expected_count++]=(2-(int)ring)*STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN;ring=2;}
            if(ring!=target) {targets[expected_count]=target;moves[expected_count++]=((int)target-ring)*STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN;ring=target;}
        }
        deadline=clock_ms+400000;
        while(run.state!=RUN_FAULT && run.job==(batch ? 5 : 2) && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            serve_peer(&offset);
            plant_step();
            if(strstr(output,"[station index] ring=")) {
                unsigned target;int mm;
                CHECK(sscanf(strstr(output,"[station index] ring="),"[station index] ring=%u move_mm=%d",&target,&mm)==2);
                CHECK(events<expected_count && target==targets[events] && mm==moves[events]);
                CHECK(abs(mm)==STN_ROUGH_INDEX_MM && run.station_shift && arm_parked());
                positions_during_index=arm_positions;events++;
            }
            if(run.station_shift) {
                CHECK(arm_parked() && arm_positions==positions_during_index);
                CHECK(abs(run.forward_cmd_rpm)<=STN_ROUGH_INDEX_RPM);
            } else if(was_index) {
                CHECK(run.state==RUN_STATION && run.station_relock && arm_lock.state==1);
                CHECK(run.distance_mm==4567 && run.steps==19 && run.completed_roads==0x123);
                CHECK(run.edge==(batch ? 11 : 5) && run.plan==(batch & 1u));
            }
            was_index=run.station_shift;
            arm_pose_now(&slide,&lift,&yaw,&claw);
            if(claw!=prior_claw && slide==STN_ROUGH_SLIDE_MM && lift==STN_ROUGH_LIFT_MM && yaw==STN_ROUGH_YAW_DEG) {
                RoadPoint point;double physical_ring;CHECK(road_point(NODE_D,&point));
                physical_ring=2+(field_x-point.x)/(STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN);
                if(claw==ARM_CLAW_OPEN_DEG) {
                    CHECK(placed<3 && taken==0 && fabs(physical_ring-(placed+1))<0.15);
                    for(i=0;order[i]!=placed+1;i++){}
                    CHECK(arm_platform_slot()==(int)i+1);placed++;
                } else if(claw==ARM_CLAW_CLOSE_DEG) {CHECK(placed==3 && taken<3 && fabs(physical_ring-(taken+1))<0.15);taken++;}
            }
            prior_claw=claw;
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"ROUGH batch=%u permutation=%u: %s\n",batch,variant,output);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==(batch ? 6 : 3));
        CHECK(placed==3 && taken==3 && events==expected_count && arm_parked() && arm_platform_slot()==1);
        CHECK(run.distance_mm==4567 && run.completed_roads==0x123);
        {RoadPoint point;CHECK(road_point(NODE_D,&point) && hypot(field_x-point.x,field_y-point.y)<25);}
        send_line("station");CHECK(strstr(output,"task_ready=1"));
        stop_request();
    }
    /* 非排列/超界环号不得执行任何机械或底盘动作，任务码解析接口本身保持兼容。 */
    for(variant=0;variant<2;variant++) {
        prepare_rough(0,variant ? "task 134+125+314+231" : "task 134+113+314+231");
        firmware_step();CHECK(run.state==RUN_FAULT && !arm_positions && !velocity_batches);
    }
    /* 索引接口不能绕开已收臂、零速、工位类型和新鲜IMU限制。 */
    for(variant=0;variant<7;variant++) {
        prepare_rough(0,"task 134+123+314+231");run.station_phase=1;
        if(variant==0)CHECK(arm_command("arm pose 1 135 0 45",clock_ms));
        if(variant==1)heading_sensor_invalidate(&heading_sensor);
        if(variant==2)motor_feedback[3].speed_rpm=1;
        if(variant==3)motor_feedback[3].speed_ms=clock_ms-ARM_FEEDBACK_TIMEOUT_MS-1u;
        if(variant==4)arm_lock.state=1;
        if(variant==5)run.job=1;
        CHECK(!station_chassis_start(variant==6 ? 2*STN_ROUGH_INDEX_MM : STN_ROUGH_INDEX_MM));
        CHECK(!run_active && !run.station_shift && !velocity_batches);
    }
    /* 索引期间异常取消整轮，不能退回路线旧节点或继续夹取。 */
    for(variant=0;variant<7;variant++) {
        uint32_t deadline;unsigned positions,offset=0;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+10000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        CHECK(run.station_shift && run_active);positions=arm_positions;
        if(variant==0)missing_feedback_motor=4;
        if(variant==1)missing_velocity_ack=4;
        if(variant==2) {frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        if(variant==3)route_test_can_error(1);
        if(variant==4)stop_request();
        if(variant==5)send_line("obstacle");
        if(variant==6)send_line("arm stop");
        while(run_active && clock_ms<deadline)plant_step();
        if(!(clock_ms<deadline && (run.state==RUN_FAULT || run.state==RUN_STOPPED)))
            fprintf(stderr,"ROUGH failure variant=%u state=%u active=%u: %s\n",variant,(unsigned)run.state,run_active,output);
        CHECK(clock_ms<deadline && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        tick(500);CHECK(!run_active && !station_active() && arm_positions==positions && !run.obstacle_phase);
        send_line("station");CHECK(strstr(output,"task_ready=0"));
    }
    {
        uint32_t deadline;unsigned positions,offset=0;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+10000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        positions=arm_positions;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.station_relock && arm_lock.state==1);
        diagnostic_auto_reply=status_auto_reply=0;tick(2500);
        CHECK(run.state==RUN_FAULT && arm_lock.state==3 && arm_positions==positions);
    }
    puts("PASS rough indexing: both batches/all six task permutations; fixed 1/2/3 visit order, original cup mapping, front1/rear3, initial/final ring2 vision, route state and strict zero interlock retained; fault coverage");
}

static void align_transport_tests(void)
{
    unsigned variant;
    for(variant=0;variant<9;variant++) {
        reset_plant(0,1.0);vision_request_align(2,clock_ms);tick(8);
        CHECK(vision_tx_count==6 && vision_tx_bytes[3]=='R');
        peer_reply("MOVE,30",0);peer_reply("DISC,320,240,0,1",0);tick(1);
        CHECK(vision_align_state()==VISION_PICK_WAITING); /* 未握手的旧数据不能启动动作。 */
        peer_reply("READY",0);tick(8);
        CHECK(vision_tx_count==12 && vision_tx_bytes[9]=='D');
        if(variant==0) {
            peer_reply("MOVE,30",0);tick(8);
            CHECK(vision_align_state()==VISION_PICK_WAITING && vision_tx_count==18 && vision_tx_bytes[15]=='R');
            peer_reply("DISC,320,240,0,1",0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_WAITING);
            peer_reply("READY",0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_READY && vision_align_offset_px()==30);
        } else if(variant==1) {
            peer_reply("DISC,318,242,-2,1",0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_READY && !vision_align_offset_px());
        } else if(variant==2) {peer_reply("NONE,D",0);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        else if(variant==3) {peer_reply("DISC,320,240,0,0",0);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        else if(variant==4) {peer_reply("MOVE,41",0);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        else if(variant==5) {peer_reply("DISC,340,240,0,1",0);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        else if(variant==6) {peer_reply("MOVE,-20",1);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);clock_ms+=25000;vision_update(clock_ms);CHECK(vision_align_state()==VISION_PICK_TIMEOUT);}
        else if(variant==7) {vision_receive_byte(0,1);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        else {peer_reply("ERR,camera",0);tick(1);CHECK(vision_align_state()==VISION_PICK_FAULT);}
        vision_cancel_pick();CHECK(vision_align_state()==VISION_PICK_IDLE);
    }
    reset_plant(0,1.0);clock_ms=UINT32_MAX-100u;vision_request_align(2,clock_ms);
    clock_ms+=2000u;vision_update(clock_ms);CHECK(vision_align_state()==VISION_PICK_TIMEOUT);
    puts("PASS rough vision transport: actual R/READY-D-MOVE/DISC frames; MOVE requires cancellation READY, old/bad/malformed/un-aligned replies, bounds/UART/timeouts/wrap rejected");
}

static void rough_alignment_tests(void)
{
    unsigned variant;
    if(!STN_ROUGH_PLACE_READY || !STN_ROUGH_PLATFORM_TAKE_READY
        || !(STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM || STN_ROUGH_YAW_DEG))return;
    for(variant=0;variant<2;variant++) {
        unsigned offset=0,positions=0,micro=0,was_shift=0;uint32_t deadline;
        prepare_rough(0,"task 134+231+314+123");
        field_x+=variant ? -9.0 : 9.0;peer_geometry=1;deadline=clock_ms+500000;
        while(run.state!=RUN_FAULT && run.job==2 && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
            if(run.station_shift && !was_shift) {
                positions=arm_positions;
                if(run.remaining_mm==STN_ROUGH_ALIGN_STEP_MM) {
                    micro++;
                    peer_reply("DISC,320,240,0,1",0);peer_reply("MOVE,-30",0);
                }
            }
            if(run.station_shift)CHECK(station_chassis_pose_safe() && arm_positions==positions);
            was_shift=run.station_shift;
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"ALIGN variant=%u: %s\n",variant,output);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==3);
        CHECK(micro>=2 && peer_move_count==micro && peer_disc_count>=24+micro);
        CHECK(arm_parked() && arm_platform_slot()==1 && run.distance_mm==4567 && run.completed_roads==0x123);
        stop_request();
    }
    for(variant=0;variant<5;variant++) {
        unsigned offset=0,positions;uint32_t deadline;
        prepare_rough(0,"task 134+123+314+231");field_x+=9;peer_geometry=1;deadline=clock_ms+30000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        CHECK(run.station_shift && run.remaining_mm==STN_ROUGH_ALIGN_STEP_MM);
        positions=arm_positions;
        if(variant==0)stop_request();
        else if(variant==1)send_line("obstacle");
        else if(variant==2)send_line("arm stop");
        else if(variant==3)missing_feedback_motor=4;
        else {frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        tick(500);CHECK(!station_active() && arm_positions==positions);
    }
    prepare_rough(0,"task 134+123+314+231");tick(8000);
    CHECK(run.state==RUN_FAULT && !velocity_batches); /* 无READY不能移动。 */
    {
        unsigned offset=0;uint32_t deadline;
        prepare_rough(0,"task 134+123+314+231");peer_geometry=2;deadline=clock_ms+90000;
        while(run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        CHECK(run.state==RUN_FAULT && clock_ms<deadline && !arm_positions && peer_move_count<=26);
        CHECK(strstr(output,"alignment travel limit") || strstr(output,"alignment timeout"));
        CHECK(!station_active() && !run_active);
    }
    puts("PASS rough closed loop: independent simulated geometry +/-9mm converges using 3mm steps and fresh observations; no arm motion during chassis movement; stop/obstacle/arm/IMU/CAN feedback faults and missing camera hold vehicle");
}
int main(void)
{
    task_and_stop_tests();vision_tests();align_transport_tests();task_round_lifetime_tests();loading_tests();scan_handoff_tests();raw_preparation_timing_tests();automatic_route_tests();automatic_fault_tests();raw_preparation_fault_tests();automatic_obstacle_return_test();rough_index_tests();rough_alignment_tests();
    printf("PASS station/vision integration: %u assertions; ready_flags=%d/%d/%d; simulation only\n",assertions,STN_ROUGH_PLACE_READY,STN_STORAGE_PLACE_READY,STN_PLATFORM_TAKE_READY);
    return 0;
}
