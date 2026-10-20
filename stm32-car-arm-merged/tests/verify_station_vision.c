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
static unsigned peer_geometry, peer_disc_count, peer_move_count;
static unsigned peer_session_active, peer_area_count[7];
static uint32_t peer_next_ms;
static void prepare_arm_auto(unsigned plan)
{
    RoadPoint start;
    peer_session_active=0;peer_next_ms=0;memset(peer_area_count,0,sizeof(peer_area_count));
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
        "task 134++123+314+231","task 134+123+314+231+","task 1341233142319","task 134+123314231",
        "task 134+125+314+231","task 134+113+314+231","task 114+123+314+231",
        "task 134+123+324+231","task 134+123+311+231","task 134+123+314+221"};
    unsigned i;
    reset_plant(0,1.0);station_init();CHECK(station_auto_enabled());
    send_line("task 134+231+314+312");CHECK(strstr(output,"[task accepted"));
    stops=stop_count;send_line("station");CHECK(stop_count==stops);
    CHECK(strstr(output,"colors=134 rings=231") && strstr(output,"colors=314 rings=312"));
    CHECK(strlen(output)<sizeof(status));strcpy(status,output);
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        send_line(bad[i]);CHECK(strstr(output,"[task rejected"));send_line("station");CHECK(!strcmp(output,status));
    }
    send_line("task 134231314312");CHECK(strstr(output,"[task accepted"));send_line("station");CHECK(!strcmp(output,status));
    send_line("test");send_line("f");CHECK(drive.state==MOTION_RUNNING);
    send_line("s");firmware_step();CHECK(drive.state==MOTION_IDLE && !run_active);
    puts("PASS task/stop: compact/separated groups; unique same-set colors, rings permutation123, invalid atomic rollback; newline s and immediate ! retained");
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
    vision_receive_byte(0,1);tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    vision_cancel_pick();vision_request_pick(2,clock_ms);tick(12);
    vision_receive_byte(0xaa,0);vision_receive_byte(0x55,0);vision_receive_byte(12,0);tick(1);
    clock_ms+=201;vision_update(clock_ms);peer_reply("PICK,2",0);vision_update(clock_ms);
    CHECK(vision_pick_state()==VISION_PICK_READY);
    vision_cancel_pick();vision_request_pick(2,clock_ms);tick(12);
    for(i=0;i<300;i++)vision_receive_byte(0x55,0);
    tick(1);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    vision_cancel_pick();vision_request_pick(3,clock_ms);tick(12);
    clock_ms+=20000;peer_reply("PICK,3",0);vision_update(clock_ms);CHECK(vision_pick_state()==VISION_PICK_READY);
    vision_cancel_pick();clock_ms=UINT32_MAX-100u;vision_request_pick(4,clock_ms);
    clock_ms+=20000;vision_update(clock_ms);CHECK(vision_pick_state()==VISION_PICK_WAITING);
    peer_reply("PICK,4",0);vision_update(clock_ms);CHECK(vision_pick_state()==VISION_PICK_READY);
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
        } else if(n==1 && (vision_tx_bytes[*offset+3u]=='D' || vision_tx_bytes[*offset+3u]=='E')) {
            CHECK(run.state==RUN_STATION && (run.job==2 || run.job==3 || run.job==5 || run.job==6) && !run_active);
            CHECK(station_chassis_pose_safe() && arm_lock.state==2);
            {float yaw;arm_pose_now(NULL,NULL,&yaw,NULL);
                CHECK(yaw==((run.job==3 || run.job==6) ? STN_STORAGE_ALIGN_YAW_DEG : STN_ROUGH_YAW_DEG));}
            CHECK(!peer_session_active); /* 未结束的D任务不能再收到D。 */
            CHECK(vision_tx_bytes[*offset+3u]==((run.job==3 || run.job==6) ? 'E' : 'D'));
            CHECK(++peer_area_count[run.job]==1); /* 同次进站不得再次请求。 */
            peer_disc_count++;peer_session_active=1;peer_next_ms=clock_ms;
        } else CHECK(0); /* 自动流程只能发C/D/E，禁止新增R停止帧。 */
        *offset+=total;
    }
    /* 一个D启动持续反馈，后续MOVE不依赖新的下行请求；300ms间隔源自相机配置。 */
    if(peer_session_active && (int32_t)(clock_ms-peer_next_ms)>=0) {
        int dx=0;char message[48];
        if(peer_geometry==3)dx=91; /* 视觉已判成功，STM32不再以偏差91否决。 */
        else if(peer_geometry==2)dx=30;
        else if(peer_geometry) {
            RoadPoint point;double target,position,baseline;
            int storage=run.job==3 || run.job==6;
            CHECK(road_point(storage ? NODE_H : NODE_D,&point));
            position=storage ? field_y : field_x;baseline=storage ? point.y : point.x;
            target=baseline+round((position-baseline)/150.0)*150.0;
            /* 本轮模拟相机反向安装：前进时同一目标的x增大，独立于固件方向常量。 */
            dx=(int)round((position-target)*3.0);
        }
        if(abs(dx)>5 && peer_geometry!=3) {
            if(dx>40)dx=40;
            if(dx< -40)dx=-40;
            snprintf(message,sizeof(message),"MOVE,%d",dx);peer_move_count++;
            peer_next_ms=clock_ms+300u;
        } else {
            snprintf(message,sizeof(message),"DISC,%d,240,%d,1",320+dx,dx);
            peer_session_active=0;
        }
        peer_reply(message,0);
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
        CHECK(run.state!=RUN_FAULT && (run_active || run.state==RUN_STATION) && !arm_positions && clock_ms>=deadline);
        missing_feedback_motor=0;deadline=clock_ms+15000;
        while(run.state!=RUN_FAULT && (run.state==RUN_STATION || run.velocity_phase!=VELOCITY_FORWARD) && clock_ms<deadline)plant_step();
        CHECK(run_active && run.state!=RUN_FAULT && run.velocity_phase==VELOCITY_FORWARD);stop_request();
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
static void raw_last_return_tests(void)
{
    unsigned batch;
    for(batch=0;batch<2;batch++) {
        unsigned offset=0,replies=0,final_released=0;
        uint32_t deadline;float prior_claw=ARM_CLAW_OPEN_DEG;
        const char *colors=batch ? "314" : "134";
        prepare_arm_auto(0);send_line("task 134+123+314+231");
        CHECK(station_start(NODE_F,(uint8_t)(batch ? 4 : 1),clock_ms)==1);
        deadline=clock_ms+150000;
        while(station_active() && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            station_update(clock_ms);plant_step();
            if(vision_tx_count>=offset+8) {
                char message[32];
                CHECK(replies<3 && vision_tx_bytes[offset+2]==3 && vision_tx_bytes[offset+3]=='C');
                CHECK(vision_tx_bytes[offset+5]==(uint8_t)colors[replies]);
                arm_pose_now(&slide,&lift,&yaw,&claw);
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM
                    && yaw==ARM_PICK_YAW_DEG && claw==ARM_CLAW_OPEN_DEG);
                CHECK(arm_platform_slot()==(int)replies+1);
                snprintf(message,sizeof(message),"PICK,%c,320,240",colors[replies++]);
                peer_reply(message,0);offset+=8;
            }
            arm_pose_now(&slide,&lift,&yaw,&claw);
            if(replies==3 && prior_claw==ARM_CLAW_CLOSE_DEG && claw==ARM_CLAW_OPEN_DEG
                && slide==ARM_PLACE_SLIDE_MM && lift==ARM_PLACE_LIFT_MM && yaw==ARM_PLACE_YAW_DEG)
                final_released=1;
            if(final_released)CHECK(yaw!=ARM_PICK_YAW_DEG);
            prior_claw=claw;
        }
        CHECK(clock_ms<deadline && !station_failed() && replies==3 && final_released);
        CHECK(arm_parked() && arm_platform_slot()==1);
        stop_request();
    }
    puts("PASS raw final block: both batches return directly from platform yaw to front0 without intermediate pick131 target; first two return to pick angle, all three vision gates and platform1 finish retained");
}
static void automatic_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,replies=0,waiting=0,last_job=0,raw_job=0;
        uint32_t deadline,raw_start=0;int sent=0;
        prepare_arm_auto(plan);arm_speed_bias[0]=plan ? -15 : 1;arm_speed_bias[1]=plan ? 15 : -1;
        send_line(plan?"runimu C":"runimu A");CHECK(run_active && run.station_enabled);
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
#if STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM || STN_ROUGH_PLACE_LIFT_MM) && STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY && STN_STORAGE_LAYER_LIFT_MM > 0 && STN_STORAGE_LAYER_LIFT_MM <= ARM_LIFT_MAX_MM-STN_STORAGE_LIFT_MM
        if(run.state!=RUN_DONE)fprintf(stderr,"FLOW job=%u state=%u: %s\n",run.job,run.state,output);
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
        CHECK(sent && waiting>=1000 && clock_ms<deadline);
#if STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM)
#if STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY
        CHECK(replies==6 && run.state==RUN_FAULT && !run_active && run.job==6 && run.edge==13);
        CHECK(strstr(output,"layer height not calibrated") || strstr(output,"second layer exceeds"));
#else
        CHECK(replies==3);
        CHECK(run.state==RUN_FAULT && !run_active && run.job==3 && run.edge==7);
#endif
#else
        CHECK(replies==3);
        CHECK(run.state==RUN_FAULT && !run_active && run.job==2 && run.edge==5);
#endif
        CHECK(!arm_pose_known() && !arm_platform_known());
        CHECK(strstr(output,"station action refused") || strstr(output,"station work failed"));
#endif
        send_line("station");CHECK(strstr(output,"task_ready=0") && strstr(output,"colors=000 rings=000"));
    }
#if STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM || STN_ROUGH_PLACE_LIFT_MM) && STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY && STN_STORAGE_LAYER_LIFT_MM > 0 && STN_STORAGE_LAYER_LIFT_MM <= ARM_LIFT_MAX_MM-STN_STORAGE_LIFT_MM
    puts("PASS automatic A/C full configured flow: both batches and storage stacking, 8 jobs, 13960mm and three uninterrupted E passes; persistent nonzero5/7RPM, simulation only, NOT hardware acceptance");
#else
    puts("PASS automatic A/C: configured stages finish; missing/out-of-travel storage height or disabled calibration refuses before job advance");
#endif
}
static void raw_wheel_log_only_tests(void)
{
    unsigned mode,wheel;
    for(mode=0;mode<4;mode++) {
        uint32_t deadline;unsigned positions;
        prepare_arm_auto(0);send_line("task 134+123+314+231");send_line("runimu A");
        deadline=clock_ms+60000u;
        while(!(run.state==RUN_STATION && run.job==1) && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_STATION && run.job==1 && clock_ms<deadline);
        for(wheel=0;wheel<4;wheel++)wheel_feedback_bias[wheel]=(int16_t)(wheel&1u ? -40 : 40);
        tick(6000); /* 超过旧3000ms门槛；不能取消仍在等待中的C/PICK。 */
        CHECK(run.state==RUN_STATION && arm_lock.state==2 && station_active());
        CHECK(vision_pick_state()==VISION_PICK_WAITING && !arm_positions);
        positions=arm_positions;
        if(mode==0) {
            peer_reply("PICK,1,262,176",0);
            deadline=clock_ms+2000u;
            while(!arm_busy() && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            CHECK(run.state!=RUN_FAULT && arm_busy() && clock_ms<deadline);
            stop_request();
        } else {
            if(mode==1)motor_flags[2]=7;
            if(mode==2)speed_auto_reply=0;
            if(mode==3){frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
            tick(700);
            if(mode==1)CHECK(run.state==RUN_FAULT && !run_active && arm_positions==positions);
            else {
                CHECK(run.state==RUN_STATION && station_active() && arm_positions==positions);
                speed_auto_reply=1;frozen_imu=0;tick(200);
                CHECK(run.state==RUN_STATION && !arm_failed());stop_request();
            }
        }
    }
    puts("PASS raw speed logging: +/-40RPM persists6s without cancelling C/PICK, received PICK starts loading; driver stall stops; missing speed/IMU replies retain the waiting task and recover");
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
        else {motor_flags[3]=7;tick(2500);}
        CHECK(!run_active && !arm_positions && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        tick(100);CHECK(!run_active && !station_active());
    }
    puts("PASS station failures: emergency, obstacle during work, arm stop and wheel-4 stall stop all work with no automatic resume");
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
        if(fault) {
            CHECK(clock_ms>=deadline && station_active() && arm_busy() && arm_pose_known());
            arm_missing_motor=0;deadline=clock_ms+3000;
            while(!vision_tx_count && clock_ms<deadline){station_update(clock_ms);plant_step();}
            CHECK(vision_tx_count && !arm_failed() && station_active());stop_request();
        } else {
            CHECK(clock_ms<deadline && !station_active() && arm_failed() && !arm_pose_known());
            tick(100);CHECK(!vision_tx_count && !arm_positions);
        }
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
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=1;run.state=RUN_STATION;run.station_enabled=1;run.plan=(uint8_t)(batch & 1u);
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
        char task[48];uint32_t deadline;unsigned offset=0,aligned=0,events=0,expected_count=0,loaded=0,placed=0,taken=0,returned=0;
        uint8_t targets[16],ring=2;int moves[16];float prior_claw=ARM_CLAW_OPEN_DEG;
        unsigned positions_during_index=0,was_index=0,yaw_changes=0;
        float prior_yaw=ARM_YAW_HOME_DEG;
        uint16_t index_yaw_pwm=0;
        const uint8_t *order=permutations[variant];
        snprintf(task,sizeof(task),"task 134+%u%u%u+314+%u%u%u",order[0],order[1],order[2],order[0],order[1],order[2]);
        prepare_rough(batch,task);
        for(i=0;i<7;i++) {
            uint8_t target=i==6 ? 2u : order[i%3];
            if(ring!=2 && ring!=target) {targets[expected_count]=2;moves[expected_count++]=(2-(int)ring)*STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN;ring=2;}
            if(ring!=target) {targets[expected_count]=target;moves[expected_count++]=((int)target-ring)*STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN;ring=target;}
        }
        deadline=clock_ms+400000;
        while(run.state!=RUN_FAULT && run.job==(batch ? 5 : 2) && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            serve_peer(&offset);plant_step();
            if(strstr(output,"[station aligned]")) {
                unsigned job,target;
                CHECK(sscanf(strstr(output,"[station aligned]"),"[station aligned] job=%u ring=%u",&job,&target)==2);
                CHECK(aligned==0 && job==(batch ? 5u : 2u) && target==2);aligned++;
            }
            if(strstr(output,"[station index] ring=")) {
                unsigned target;int mm;
                CHECK(sscanf(strstr(output,"[station index] ring="),"[station index] ring=%u move_mm=%d",&target,&mm)==2);
                CHECK(events<expected_count && target==targets[events] && mm==moves[events]);
                arm_pose_now(&slide,&lift,&yaw,&claw);
                CHECK(abs(mm)==STN_ROUGH_INDEX_MM && run.station_shift
                    && slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM && yaw==STN_ROUGH_YAW_DEG);
                positions_during_index=arm_positions;index_yaw_pwm=arm_yaw_us;events++;
            }
            if(run.station_shift) {
                arm_pose_now(&slide,&lift,&yaw,&claw);
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM && yaw==STN_ROUGH_YAW_DEG
                    && arm_positions==positions_during_index && arm_yaw_us==index_yaw_pwm);
                CHECK(claw==ARM_CLAW_OPEN_DEG && loaded==placed && taken==returned);
                CHECK(abs(run.forward_cmd_rpm)<=STN_ROUGH_INDEX_RPM);
            } else if(was_index) {
                CHECK(run.state==RUN_STATION && run.station_relock && arm_lock.state==1);
                CHECK(run.distance_mm==4567 && run.steps==19 && run.completed_roads==0x123);
                CHECK(run.edge==(batch ? 11 : 5) && run.plan==(batch & 1u));
            }
            was_index=run.station_shift;
            arm_pose_now(&slide,&lift,&yaw,&claw);
            if(yaw!=prior_yaw) {
                if(!yaw_changes)CHECK(yaw==STN_ROUGH_YAW_DEG);
                else if(yaw==ARM_YAW_HOME_DEG)CHECK(loaded==3 && placed==3 && taken==3 && returned==3 && events==expected_count);
                else CHECK((prior_yaw==STN_ROUGH_YAW_DEG && yaw==ARM_PLACE_YAW_DEG)
                    || (prior_yaw==ARM_PLACE_YAW_DEG && yaw==STN_ROUGH_YAW_DEG));
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM);
                yaw_changes++;
            }
            prior_yaw=yaw;
            if(claw!=prior_claw && claw==ARM_CLAW_CLOSE_DEG && yaw==ARM_PLACE_YAW_DEG
                && slide==ARM_PLACE_SLIDE_MM && lift==STN_ROUGH_PLATFORM_TAKE_LIFT_MM) {
                RoadPoint point;double physical_ring;
                CHECK(loaded<3 && taken==0 && loaded==placed && arm_platform_slot()==(int)loaded+1);
                CHECK(aligned==1);
                CHECK(road_point(NODE_D,&point));
                physical_ring=2+(field_x-point.x)/(STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN);
                CHECK(fabs(physical_ring-order[loaded])<0.15);loaded++;
            }
            if(claw!=prior_claw && slide==STN_ROUGH_SLIDE_MM && yaw==STN_ROUGH_YAW_DEG) {
                RoadPoint point;double physical_ring;CHECK(road_point(NODE_D,&point));
                physical_ring=2+(field_x-point.x)/(STN_ROUGH_INDEX_MM*STN_ROUGH_INDEX_SIGN);
                if(claw==ARM_CLAW_OPEN_DEG) {
                    CHECK(lift==STN_ROUGH_PLACE_LIFT_MM);
                    CHECK(placed<3 && taken==0 && fabs(physical_ring-order[placed])<0.15);
                    CHECK(arm_platform_slot()==(int)placed+1);placed++;
                } else if(claw==ARM_CLAW_CLOSE_DEG) {
                    CHECK(lift==STN_ROUGH_LIFT_MM);
                    CHECK(placed==3 && taken<3 && aligned==1 && fabs(physical_ring-order[taken])<0.15);taken++;
                }
            }
            if(claw!=prior_claw && claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG
                && slide==ARM_PLACE_SLIDE_MM && lift==ARM_PLACE_LIFT_MM) {
                CHECK(returned<3 && taken==returned+1 && arm_platform_slot()==(int)returned+1);
                returned++;
            }
            prior_claw=claw;
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"ROUGH batch=%u permutation=%u: %s\n",batch,variant,output);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==(batch ? 6 : 3));
        CHECK(aligned==1 && peer_disc_count==1 && !peer_move_count && loaded==3 && placed==3 && taken==3 && returned==3 && yaw_changes==14
            && events==expected_count && arm_parked() && arm_platform_slot()==1);
        CHECK(run.distance_mm==4567 && run.completed_roads==0x123);
        {RoadPoint point;CHECK(road_point(NODE_D,&point) && hypot(field_x-point.x,field_y-point.y)<25);}
        send_line("station");CHECK(strstr(output,"task_ready=1"));
        stop_request();
    }
    /* 索引接口不能绕开已收臂、轮速容差、工位类型和新鲜IMU限制。 */
    for(variant=0;variant<7;variant++) {
        prepare_rough(0,"task 134+123+314+231");run.station_phase=1;
        if(variant==0)CHECK(arm_command("arm pose 1 135 0 45",clock_ms));
        if(variant==1)heading_sensor_invalidate(&heading_sensor);
        if(variant==2)motor_feedback[3].status_flags=0;
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
        if(variant<4) {
            uint32_t pause_until=clock_ms+1200;
            while(run_active && clock_ms<pause_until)plant_step();
            CHECK(run.state!=RUN_FAULT && station_active());
            missing_feedback_motor=missing_velocity_ack=0;frozen_imu=0;
            while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            CHECK(run.state!=RUN_FAULT && clock_ms<deadline);stop_request();
        } else {
            while(run_active && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
            tick(500);CHECK(!run_active && !station_active() && arm_positions==positions && !run.obstacle_phase);
            send_line("station");CHECK(strstr(output,"task_ready=0"));
        }
    }
    {
        uint32_t deadline;unsigned positions,offset=0;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+10000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        positions=arm_positions;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.station_relock && arm_lock.state==1);
        diagnostic_auto_reply=status_auto_reply=0;tick(2500);
        CHECK(run.state==RUN_STATION && arm_lock.state==1 && arm_positions==positions);
        diagnostic_auto_reply=status_auto_reply=1;tick(500);
        CHECK(arm_lock.state==2 && run.state!=RUN_FAULT);stop_request();
    }
    puts("PASS rough indexing: both batches/all six permutations; move before pickup100, place5/retrieve5, direct125/256 with no intermediate0, empty retracted index, original cups, final ring2 and departure0; strict feedback/fault gates retained");
}

static void align_transport_tests(void)
{
    unsigned variant;
    for(variant=0;variant<9;variant++) {
        reset_plant(0,1.0);
        peer_reply("MOVE,30",0); /* 请求之前已在本地缓存的数据必须清除。 */
        vision_request_align(variant==1 ? 'E' : 'D',2,clock_ms);vision_tx_ready=0;
        peer_reply("DISC,320,240,0,1",0);tick(1);
        CHECK(vision_align_state()==VISION_PICK_WAITING && !vision_tx_count);
        vision_tx_ready=1;tick(8);
        CHECK(vision_tx_count==6 && vision_tx_bytes[3]==(variant==1 ? 'E' : 'D'));
        CHECK(vision_tx_bytes[4]==peer_crc(vision_tx_bytes+2,2));
        peer_reply("READY",0);tick(1);
        CHECK(vision_align_state()==VISION_PICK_WAITING && vision_tx_count==6);
        if(variant==0) {
            int16_t offset;uint32_t received;
            peer_reply("MOVE,30",0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_WAITING && vision_align_offset_px()==30);
            CHECK(vision_align_take_move(&offset,&received) && offset==30 && received==clock_ms);
            CHECK(!vision_align_take_move(&offset,&received));
            peer_reply("MOVE,-30",0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_WAITING && vision_tx_count==6);
            CHECK(vision_align_take_move(&offset,&received) && offset== -30);
            peer_reply("MOVE,20",0);peer_reply("MOVE,10",0);tick(1);
            CHECK(vision_align_take_move(&offset,&received) && offset==10);
            CHECK(!vision_align_take_move(&offset,&received));
            peer_reply("MOVE,-20",0);peer_reply("DISC,320,240,0,1",0);tick(8);
            CHECK(vision_align_state()==VISION_PICK_READY && !vision_align_offset_px() && vision_tx_count==6);
            CHECK(!vision_align_take_move(&offset,&received));
            peer_reply("MOVE,-30",0);peer_reply("DISC,350,240,30,0",0);tick(8);
            CHECK(vision_align_state()==VISION_PICK_READY && vision_tx_count==6);
            vision_request_align('D',1,clock_ms);tick(8); /* 下一个目标才开始新的D任务。 */
            CHECK(vision_tx_count==12 && vision_tx_bytes[9]=='D');
            peer_reply("DISC,320,240,0,1",0);tick(8);
            CHECK(vision_align_state()==VISION_PICK_READY && !vision_align_offset_px() && vision_tx_count==12);
        } else if(variant==1) {
            peer_reply("DISC,318,242,-2,1",0);tick(8);
            CHECK(vision_align_state()==VISION_PICK_READY && !vision_align_offset_px() && vision_tx_count==6);
        } else if(variant==2) {peer_reply("NONE,D",0);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else if(variant==3) {peer_reply("DISC,320,240,0,0",0);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else if(variant==4) {peer_reply("MOVE,41",0);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else if(variant==5) {peer_reply("DISC,340,240,0,1",0);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else if(variant==6) {peer_reply("MOVE,-20",1);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);clock_ms+=25000;vision_update(clock_ms);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else if(variant==7) {vision_receive_byte(0,1);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        else {peer_reply("ERR,camera",0);tick(1);CHECK(vision_align_state()==VISION_PICK_WAITING);}
        {unsigned sent=vision_tx_count;
            vision_cancel_pick();tick(8);CHECK(vision_align_state()==VISION_PICK_IDLE && vision_tx_count==sent);}
    }
    /* 回归实际出现的大偏差ok=1，并覆盖正负两侧；只由视觉决定是否到位。 */
    {
        static const int offsets[]={54,84,77,80,91,-91,6,-6,319,-320};
        unsigned i;
        for(i=0;i<sizeof(offsets)/sizeof(offsets[0]);i++) {
            char payload[48];int16_t move;
            reset_plant(0,1.0);vision_request_align(i&1u ? 'E' : 'D',2,clock_ms);tick(8);
            peer_reply("MOVE,30",0);tick(1);
            snprintf(payload,sizeof(payload),"DISC,%d,168,%d,1",320+offsets[i],offsets[i]);
            peer_reply(payload,0);tick(1);
            CHECK(vision_align_state()==VISION_PICK_READY && !vision_align_take_move(&move,NULL));
            CHECK(vision_tx_count==6);
            vision_cancel_pick();
        }
    }
    reset_plant(0,1.0);clock_ms=UINT32_MAX-100u;vision_request_align('D',2,clock_ms);tick(8);
    clock_ms+=24900u;peer_reply("MOVE,20",0);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_WAITING);
    clock_ms+=100u;vision_update(clock_ms);CHECK(vision_align_state()==VISION_PICK_WAITING);
    puts("PASS single D session: large-offset DISC ok=1 accepted, ok=0/malformed rejected; multiple MOVE keep waiting; latest sample consumed once, final DISC ends task, next explicit visit may start a new D/E; no automatic R/READY or resend, terminal duplicates ignored; CRC/bounds/UART/deadline/wrap covered");
}

static void prepare_storage(unsigned batch, const char *task)
{
    RoadPoint point;
    prepare_arm_auto(batch & 1u);send_line(task);
    CHECK(road_point(NODE_H,&point));
    field_x=planned_x=point.x;field_y=planned_y=point.y;initial_field_heading=HEAD_Y_POS;
    physical_yaw=origin_yaw=0;sample_yaw(0);
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=1;run.state=RUN_STATION;run.station_enabled=1;run.plan=(uint8_t)(batch & 1u);
    run.job=(uint8_t)(batch ? 6 : 3);run.edge=(uint8_t)(batch ? 13 : 7);
    run.heading=run.target_heading=HEAD_Y_POS;run.yaw_zero_cdeg=0;
    run.distance_mm=4567;run.steps=19;run.completed_roads=0x123;
    CHECK(road_plan(NODE_G,NODE_H,&run.road));checked_job=run.job;
    peer_geometry=peer_disc_count=peer_move_count=0;
}

static void storage_index_tests(void)
{
    static const uint8_t permutations[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    unsigned batch,variant,color_variant,i;
    for(batch=0;batch<2;batch++)for(variant=0;variant<6;variant++)for(color_variant=0;color_variant<(batch ? 6u : 1u);color_variant++) {
        char task[64];uint32_t deadline;uint8_t ring=2,targets[8],colors[3];
        unsigned offset=0,aligned=0,expected_count=0,events=0,taken=0,placed=0,positions=0,was_shift=0;
        int moves[8],can_run=STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY;
        int height=STN_STORAGE_LIFT_MM+(batch ? STN_STORAGE_LAYER_LIFT_MM : 0);
        int take_height=batch ? ARM_PLACE_LIFT_MM : STN_ROUGH_PLATFORM_TAKE_LIFT_MM;
        float previous=ARM_CLAW_OPEN_DEG;uint16_t yaw_pwm=0;
        const uint8_t *order=permutations[variant],*color_order=permutations[color_variant];
        static const uint8_t first_colors[3]={1,3,4};
        for(i=0;i<3;i++)colors[i]=first_colors[color_order[i]-1];
        snprintf(task,sizeof(task),"task 134+%u%u%u+%u%u%u+312",order[0],order[1],order[2],colors[0],colors[1],colors[2]);
        prepare_storage(batch,task);
        if(batch && (STN_STORAGE_LAYER_LIFT_MM<=0 || height>ARM_LIFT_MAX_MM))can_run=0;
        if(!can_run) {
            firmware_step();CHECK(run.state==RUN_FAULT && !run_active && !arm_positions && !velocity_batches);
            CHECK(strstr(output,"not calibrated") || strstr(output,"second layer exceeds"));
            continue;
        }
        for(i=0;i<4;i++) {
            uint8_t target=i==3 ? 2u : order[batch ? color_order[i]-1u : i];
            if(ring!=2 && ring!=target) {targets[expected_count]=2;moves[expected_count++]=(2-(int)ring)*STN_STORAGE_INDEX_MM*STN_STORAGE_INDEX_SIGN;ring=2;}
            if(ring!=target) {targets[expected_count]=target;moves[expected_count++]=((int)target-ring)*STN_STORAGE_INDEX_MM*STN_STORAGE_INDEX_SIGN;ring=target;}
        }
        deadline=clock_ms+250000;
        while(run.state!=RUN_FAULT && run.job==(batch ? 6 : 3) && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            serve_peer(&offset);plant_step();
            if(strstr(output,"[station aligned]")) {
                unsigned job,target;
                CHECK(sscanf(strstr(output,"[station aligned]"),"[station aligned] job=%u ring=%u",&job,&target)==2);
                CHECK(aligned==0 && job==(batch ? 6u : 3u) && target==2);aligned++;
            }
            arm_pose_now(&slide,&lift,&yaw,&claw);
            if(strstr(output,"[station index] ring=")) {
                unsigned target;int mm;
                CHECK(sscanf(strstr(output,"[station index] ring="),"[station index] ring=%u move_mm=%d",&target,&mm)==2);
                CHECK(events<expected_count && target==targets[events] && mm==moves[events]);events++;
                CHECK(abs(mm)==150 && station_chassis_pose_safe() && yaw==STN_STORAGE_ALIGN_YAW_DEG && claw==ARM_CLAW_OPEN_DEG);
                positions=arm_positions;yaw_pwm=arm_yaw_us;
            }
            if(run.station_shift) {
                CHECK(station_chassis_pose_safe() && yaw==STN_STORAGE_ALIGN_YAW_DEG
                    && arm_positions==positions && arm_yaw_us==yaw_pwm && claw==ARM_CLAW_OPEN_DEG);
                CHECK(abs(run.forward_cmd_rpm)<=STN_STORAGE_INDEX_RPM);
            } else if(was_shift) {
                CHECK(run.state==RUN_STATION && run.station_relock && arm_lock.state==1);
                CHECK(run.distance_mm==4567 && run.steps==19 && run.completed_roads==0x123);
                CHECK(run.edge==(batch ? 13 : 7));
            }
            was_shift=run.station_shift;
            if(claw!=previous && yaw==ARM_PLACE_YAW_DEG && slide==ARM_PLACE_SLIDE_MM) {
                CHECK(lift==take_height);
                CHECK(claw==ARM_CLAW_CLOSE_DEG && taken==placed && taken<3 && arm_platform_slot()==(int)taken+1);taken++;
                CHECK(aligned==1);
            }
            if(claw!=previous && yaw==STN_STORAGE_YAW_DEG && slide==STN_STORAGE_SLIDE_MM && lift==height) {
                RoadPoint point;double actual;
                uint8_t target;
                CHECK(placed<3);target=order[batch ? color_order[placed]-1u : placed];
                CHECK(road_point(NODE_H,&point));
                actual=2+(field_y-point.y)/(STN_STORAGE_INDEX_MM*STN_STORAGE_INDEX_SIGN);
                CHECK(claw==ARM_CLAW_OPEN_DEG && taken==placed+1 && placed<3);
                CHECK(fabs(actual-target)<0.15 && arm_platform_slot()==(int)placed+1);placed++;
            }
            previous=claw;
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"STORAGE batch=%u rings=%u colors=%u: %s\n",batch,variant,color_variant,output);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==(batch ? 7 : 4));
        CHECK(aligned==1 && peer_disc_count==1 && !peer_move_count && placed==3 && taken==3
            && events==expected_count && arm_parked() && arm_platform_slot()==1);
        CHECK(run.distance_mm==4567 && run.completed_roads==0x123);
        {RoadPoint point;CHECK(road_point(NODE_H,&point) && hypot(field_x-point.x,field_y-point.y)<25);}
        stop_request();
    }
    /* 暂存区同样拒绝未收回、低位、忙碌和超轮速容差，不能绕过工位联锁。 */
    for(variant=0;variant<4;variant++) {
        prepare_storage(0,"task 134+231+314+123");run.station_phase=1;
        if(variant==0)CHECK(arm_command("arm pose 1 135 0 45",clock_ms));
        if(variant==1)CHECK(arm_command("arm pose 0 105 0 45",clock_ms));
        if(variant==2)motor_feedback[0].status_flags=7;
        if(variant==3)CHECK(arm_command("arm yaw 10",clock_ms));
        CHECK(!station_chassis_start(STN_STORAGE_INDEX_MM) && !run_active && !velocity_batches);
    }
    puts("PASS storage: first G2 placement, second same-color G1/G2 lookup independent of G4, all six ring/color permutations, layer height/travel guards, 150mm segments and final ring2, empty retracted arm while moving");
}

static int alignment_station_enabled(unsigned storage)
{
    return storage ? STN_STORAGE_PLACE_READY && STN_PLATFORM_TAKE_READY :
        STN_ROUGH_PLACE_READY && STN_ROUGH_PLATFORM_TAKE_READY
        && (STN_ROUGH_SLIDE_MM || STN_ROUGH_LIFT_MM || STN_ROUGH_YAW_DEG);
}

static void station_launch_tolerance_tests(void)
{
    unsigned storage,variant,move,wheel;
    static const int distances[]={3,-3,150,-150};
    for(storage=0;storage<2;storage++) {
        if(!alignment_station_enabled(storage))continue;
        for(move=0;move<4;move++)for(variant=0;variant<11;variant++) {
            static const int16_t logged[]={0,4,2,-2};
            if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
            run.station_phase=1;
            for(wheel=0;wheel<4;wheel++) {
                motor_feedback[wheel].speed_rpm=variant==0 ? logged[wheel] :
                    variant==1 ? (wheel&1u ? -15 : 15) : variant==2 ? (wheel&1u ? 15 : -15) : 0;
            }
            if(variant>=3)motor_feedback[(variant-3u)/2u].speed_rpm=(variant&1u) ? 16 : -16;
            CHECK(station_chassis_start((int16_t)distances[move]));
            CHECK(run_active && run.station_shift && run.travel_axis==BODY_FORWARD);
            CHECK(run.travel_sign==(distances[move]>0 ? 1 : -1));
            CHECK(!arm_positions && !velocity_batches);
            stop_request();
        }
    }
    puts("PASS station launch tolerance: rough/storage +/-3mm and +/-150mm accept logged0/4/2/-2RPM and +/-15RPM on all wheels; each wheel +/-16RPM logged only; fresh/status/pose/IMU gates retained");
}

static void station_alignment_tests(void)
{
    unsigned storage,variant;
    for(storage=0;storage<2;storage++)for(variant=0;variant<5;variant++) {
        uint32_t deadline;unsigned offset=0,micro=0,was_shift=0,positions=0;uint16_t yaw_pwm=0;
        RoadPoint point;uint8_t job=(uint8_t)(storage ? 3 : 2);
        if(!alignment_station_enabled(storage))continue;
        if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
        if(variant<2) {
            if(storage)field_y+=variant ? -9.0 : 9.0;else field_x+=variant ? -9.0 : 9.0;
        }
        if(variant>=3) {
            static const int16_t bias[2][4]={{5,3,-3,5},{16,-16,40,-40}};
            unsigned wheel;
            for(wheel=0;wheel<4;wheel++)wheel_feedback_bias[wheel]=bias[variant-3u][wheel];
            arm_speed_bias[0]=variant==3 ? 1 : -15;arm_speed_bias[1]=variant==3 ? -1 : 15;
            arm_lock_begin(); /* 自动进站也要能获取新鲜状态，不受持续非零轮速阻塞。 */
            if(storage)field_y+=9.0;else field_x+=9.0;
        }
        peer_geometry=variant==2 ? 3u : 1u;deadline=clock_ms+500000;
        while(run.state!=RUN_FAULT && run.job==job && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
            if(strstr(output,"[station correction]")) {
                unsigned logged_job,logged_ring,total;int pixels,mm;
                CHECK(sscanf(strstr(output,"[station correction]"),
                    "[station correction] job=%u ring=%u offset_px=%d move_mm=%d total_mm=%u",
                    &logged_job,&logged_ring,&pixels,&mm,&total)==5);
                CHECK(logged_job==job && logged_ring>=1 && logged_ring<=3 && pixels!=0);
                CHECK(mm==(pixels>0 ? -3 : 3)); /* 正偏差后退；负偏差前进。 */
            }
            if(run.station_shift && !was_shift) {
                positions=arm_positions;yaw_pwm=arm_yaw_us;
                if(run.remaining_mm==STN_ALIGN_STEP_MM) {
                    micro++;
                    /* 移动期间的反向旧偏差不能排队执行；最终DISC另测保留。 */
                    peer_reply("MOVE,-30",0);
                }
            }
            if(run.station_shift)CHECK(station_chassis_pose_safe() && arm_positions==positions && arm_yaw_us==yaw_pwm);
            was_shift=run.station_shift;
        }
        if(run.state==RUN_FAULT)fprintf(stderr,"ALIGN storage=%u direction=%u: %s\n",storage,variant,output);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==job+1u && (variant==2 ? !micro && !peer_move_count : micro>=2 && peer_move_count>=micro) && peer_disc_count==1);
        CHECK(arm_parked() && arm_platform_slot()==1 && run.distance_mm==4567 && run.completed_roads==0x123);
        CHECK(road_point(storage ? NODE_H : NODE_D,&point) && hypot(field_x-point.x,field_y-point.y)<25);
        if(variant>=3) {
            CHECK(clock_ms<deadline && run.station_departing && arm_lock.state==2);
            while(run.state!=RUN_MOVING && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            if(run.state!=RUN_MOVING || run.station_departing)fprintf(stderr,"DEPART storage=%u variant=%u state=%u departing=%u: %s\n",storage,variant,run.state,run.station_departing,output);
            CHECK(run.state==RUN_MOVING && clock_ms<deadline && !run.station_departing);
        }
        stop_request();
    }
    for(storage=0;storage<2;storage++) {
        uint32_t deadline;unsigned offset=0,positions;uint8_t job=(uint8_t)(storage ? 3 : 2);
        if(!alignment_station_enabled(storage))continue;
        if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
        peer_geometry=2;deadline=clock_ms+30000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(run.station_shift && peer_disc_count==1);positions=arm_positions;
        peer_session_active=0;peer_reply("DISC,411,183,91,1",0);tick(1);
        CHECK(vision_align_state()==VISION_PICK_READY && arm_positions==positions);
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            CHECK(arm_positions==positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && arm_lock.state==2);
        CHECK(strstr(output,"[station aligned]") && peer_disc_count==1 && run.job==job);
        stop_request();
    }
    for(storage=0;storage<2;storage++)for(variant=0;variant<5;variant++) {
        uint32_t deadline;unsigned offset=0,positions;
        if(!alignment_station_enabled(storage))continue;
        if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
        if(storage)field_y+=9;else field_x+=9;
        peer_geometry=1;deadline=clock_ms+30000;
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        CHECK(run.station_shift && run.remaining_mm==STN_ALIGN_STEP_MM);positions=arm_positions;
        if(variant==0)stop_request();
        if(variant==1)send_line("obstacle");
        if(variant==2)send_line("arm stop");
        if(variant==3)missing_feedback_motor=4;
        if(variant==4){frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        if(variant>=3) {
            uint32_t pause_until=clock_ms+1200;
            while(run_active && clock_ms<pause_until)plant_step();
            CHECK(run.state!=RUN_FAULT && station_active() && arm_positions==positions);
            missing_feedback_motor=0;frozen_imu=0;
            while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            CHECK(run.state!=RUN_FAULT && clock_ms<deadline);stop_request();
        } else {
            while(run_active && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && !run_active && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
            tick(500);CHECK(!station_active() && arm_positions==positions);
        }
    }
    for(storage=0;storage<2;storage++) {
        uint32_t deadline;unsigned offset=0;
        if(!alignment_station_enabled(storage))continue;
        if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
        deadline=clock_ms+35000;
        while(run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(clock_ms>=deadline && run.state==RUN_STATION && station_active() && !velocity_batches && !arm_positions && vision_tx_count==6);
        /* 原请求保留，迟到的成功DISC才放行；不重发D/E，也不伪造对准成功。 */
        peer_reply("DISC,320,240,0,1",0);tick(500);
        CHECK(run.state!=RUN_FAULT && vision_tx_count==6);stop_request();
        if(storage)prepare_storage(0,"task 134+231+314+123");else prepare_rough(0,"task 134+231+314+123");
        peer_geometry=2;deadline=clock_ms+90000;
        while(run.state!=RUN_FAULT && clock_ms<deadline) {serve_peer(&offset);plant_step();}
        CHECK(run.state!=RUN_FAULT && clock_ms>=deadline && station_active() && !arm_positions && peer_disc_count==1 && peer_move_count>0);
        CHECK(vision_tx_count==6);stop_request();
    }
    puts("PASS station vision: rough D once / storage E once at entry ring2 yaw125; no target/return detection, continuous MOVE/DISC, no R; one D per target drives independent +/-9mm geometry in fresh3mm steps, in-motion MOVE discarded and terminal DISC retained until stop commands confirmed; persistent +/-5 and +/-40RPM feedback crosses initial acquisition, PREFLIGHT, micro/index, reacquisition, all grips and departure; missing camera/deadlines retain task, valid late DISC resumes; explicit stop and actual driver faults cancel");
}

int main(void)
{
    task_and_stop_tests();vision_tests();align_transport_tests();task_round_lifetime_tests();loading_tests();scan_handoff_tests();raw_preparation_timing_tests();raw_last_return_tests();automatic_route_tests();raw_wheel_log_only_tests();automatic_fault_tests();raw_preparation_fault_tests();automatic_obstacle_return_test();rough_index_tests();storage_index_tests();station_launch_tolerance_tests();station_alignment_tests();
    printf("PASS station/vision integration: %u assertions; ready_flags=%d/%d/%d; simulation only\n",assertions,STN_ROUGH_PLACE_READY,STN_STORAGE_PLACE_READY,STN_PLATFORM_TAKE_READY);
    return 0;
}
