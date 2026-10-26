/* 由verify_station_vision.c引入：真实UART帧→工位→四轮CAN→独立惯性/编码器场地模型。 */
static void movexy_transport_tests(void)
{
    static const char *const good[]={"MOVEXY,+300,-200","MOVEXY 300 -200","movexy +300 -200","MOVEXY\t300\t-200","MOVEXY,-32768,+32767","MOVEXY,0,-0"};
    static const int xs[]={300,300,300,300,-32768,0},ys[]={-200,-200,-200,-200,32767,0};
    static const char *const bad[]={"MOVEXY,1","MOVEXY 1","MOVEXY,1,2,3","MOVEXY 1 2 3","MOVEXY,+32768,0","MOVEXY,0,-32769","MOVEXY,+,3","MOVEXY,1,2x","MOVEXY,1 2","MOVEXY 1,2","MOVEXY,,2"};
    unsigned area,i;int16_t x,y;uint32_t ms;
    for(area=0;area<3;area++) {
        reset_plant(0,1.0);
        if(area==2)vision_request_stack(1,clock_ms);else vision_request_align(area ? 'E' : 'D',2,clock_ms);
        tick(10);
        for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
            peer_reply(bad[i],0);tick(1);CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
        }
        peer_reply("MOVEXY,10,20",1);tick(1);CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
        for(i=0;i<sizeof(good)/sizeof(good[0]);i++) {
            peer_reply(good[i],0);tick(1);
            CHECK(vision_motion_peek_xy(&x,&y,&ms) && x==xs[i] && y==ys[i] && ms==clock_ms);
            CHECK(!vision_align_take_move(NULL,NULL) && !vision_align_take_axis(1,NULL,NULL));
            vision_motion_take();CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
        }
        for(i=0;i<33;i++){peer_reply("MOVEXY,7,-9",0);tick(1);}
        CHECK(strstr(output,"motion queue full"));
        peer_reply(area==2 ? "STACK,700,-50,620,1" : "DISC,700,-50,620,1",0);tick(1);
        peer_reply("MOVEXY,99,99",0);tick(1);
        for(i=0;i<32;i++) {
            CHECK(vision_motion_peek_xy(&x,&y,NULL) && x==7 && y== -9);vision_motion_take();
        }
        CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));vision_cancel_pick();
    }
    reset_plant(0,1.0);clock_ms=UINT32_MAX-5u;vision_request_align('D',2,clock_ms);tick(10);
    peer_reply("MOVEXY,+12,-21",0);tick(1);
    CHECK(vision_motion_peek_xy(&x,&y,&ms) && x==12 && y== -21 && ms==clock_ms);
    vision_cancel_pick();CHECK(!vision_motion_peek_xy(NULL,NULL,NULL));
    puts("PASS MOVEXY transport: D/E/F comma and whitespace forms, optional plus, signed16 bounds, malformed arity/range and CRC, atomic32-pair FIFO, no legacy split, terminal drain/fencing, cancellation and timer wrap");
}
static void movexy_debug_tests(void)
{
    static const int dx[]={40,-40,40,-40,300,-17,0,40,0,1,3,40};
    static const int dy[]={30,30,-30,-30,-200,90,30,0,0,-1,-3,1};
    static const char *const start[]={"vision debug D","vision debug E","vision debug F 1","vision debug F 4"};
    unsigned area,v;
    for(area=0;area<4;area++)for(v=0;v<sizeof(dx)/sizeof(dx[0]);v++) {
        uint32_t deadline;unsigned segments=0,was_shift=0,concurrent=0,i;
        double x,y;uint16_t claw,platform;char payload[48];
        prepare_vision_debug();send_line(start[area]);wait_debug_request(area>=2);
        if(v==3)for(i=0;i<4;i++)encoder_polarity[i]=(i&1) ? -1 : 1;
        x=field_x;y=field_y;claw=arm_claw_us;platform=arm_platform_us;
        snprintf(payload,sizeof(payload),v&1 ? "movexy %+d %+d" : "MOVEXY,%+d,%+d",dx[v],dy[v]);peer_reply(payload,0);
        peer_reply(area>=2 ? "STACK,700,-50,620,1" : "DISC,700,-50,620,1",0);
        deadline=clock_ms+90000;
        while(run.vision_debug && run.state!=RUN_FAULT && clock_ms<deadline) {
            double forward,left;
            plant_step();
            if(run.station_shift && !was_shift) {
                segments++;CHECK(segments==1);
                CHECK(run.vector_move==(dx[v]!=0 && dy[v]!=0));
                if(run.vector_move)CHECK(run.vector_target_mm[0]== -dx[v] && run.vector_target_mm[1]==dy[v]);
            }
            was_shift=run.station_shift;
            forward=(-simulated_wheel_rpm[0]+simulated_wheel_rpm[1]+simulated_wheel_rpm[2]-simulated_wheel_rpm[3])/4.0;
            left=(-simulated_wheel_rpm[0]-simulated_wheel_rpm[1]+simulated_wheel_rpm[2]+simulated_wheel_rpm[3])/4.0;
            if(run.station_shift && fabs(forward)>1 && fabs(left)>1)concurrent++;
            CHECK(!arm_positions && arm_claw_us==claw && arm_platform_us==platform);
        }
        if(clock_ms>=deadline || run.state==RUN_FAULT)fprintf(stderr,"MOVEXY failed area=%u v=%u %s progress=%ld,%ld pos=%.2f,%.2f state=%u signs=%d,%d,%d,%d\n",area,v,output,(long)run.vector_progress_mm[0],(long)run.vector_progress_mm[1],field_x-x,field_y-y,run.state,run.encoder_sign[0],run.encoder_sign[1],run.encoder_sign[2],run.encoder_sign[3]);
        CHECK(clock_ms<deadline && run.state==RUN_IDLE && !run_active && !station_active());
        CHECK(segments==(unsigned)(dx[v]!=0 || dy[v]!=0));
        if(dx[v] && dy[v] && abs(dx[v])>3 && abs(dy[v])>3)CHECK(concurrent>0);
        if(fabs(field_x-x-dy[v])>=3.0 || fabs(field_y-y+dx[v])>=3.0)fprintf(stderr,"MOVEXY endpoint area=%u v=%u actual=%.2f,%.2f expected=%d,%d\n",area,v,field_x-x,field_y-y,dy[v],-dx[v]);
        CHECK(fabs(field_x-x-dy[v])<3.0 && fabs(field_y-y+dx[v])<3.0);
        CHECK(vision_tx_count==(area>=2 ? 8u : 6u));
        x=field_x;y=field_y;tick(1000);CHECK(fabs(field_x-x)<0.01 && fabs(field_y-y)<0.01);
    }
    puts("PASS MOVEXY actual motion: D/E/F1/F4 debug, all signs/zeros, +300/-200 full magnitude, unequal X/Y calibration, inverted encoder polarities; same four-wheel batch has both components, independent field endpoint, no grip or inverse return");
}
static void movexy_polarity_response_tests(void)
{
    unsigned mask,i;
    for(mask=0;mask<16;mask++) {
        uint32_t deadline;unsigned concurrent=0,disturbed=0;
        double x,y;
        prepare_vision_debug();send_line("vision debug E");wait_debug_request(0);
        x=field_x;y=field_y;
        for(i=0;i<4;i++) {
            encoder_polarity[i]=(mask&(1u<<i)) ? -1 : 1;
            wheel_acceleration[i]=0.12+i*0.04;
        }
        peer_reply(mask&1 ? "MOVEXY,-80,+80" : "MOVEXY,+80,-80",0);
        peer_reply("DISC,700,-50,620,1",0);deadline=clock_ms+90000;
        while(run.vision_debug && run.state!=RUN_FAULT && clock_ms<deadline) {
            double forward,left;
            plant_step();
            if(run.vector_move && abs(run.vector_progress_mm[0])>20 && !disturbed) {
                physical_yaw+=100;disturbed=1;
            }
            forward=(-simulated_wheel_rpm[0]+simulated_wheel_rpm[1]+simulated_wheel_rpm[2]-simulated_wheel_rpm[3])/4.0;
            left=(-simulated_wheel_rpm[0]-simulated_wheel_rpm[1]+simulated_wheel_rpm[2]+simulated_wheel_rpm[3])/4.0;
            if(fabs(forward)>1 && fabs(left)>1)concurrent++;
        }
        if(run.state!=RUN_IDLE || fabs(field_x-x-(mask&1 ? 80 : -80))>=3.0 || fabs(field_y-y-(mask&1 ? 80 : -80))>=3.0)
            fprintf(stderr,"MOVEXY polarity mask=%u state=%u field=%.2f,%.2f progress=%ld,%ld %s\n",mask,run.state,field_x-x,field_y-y,(long)run.vector_progress_mm[0],(long)run.vector_progress_mm[1],output);
        CHECK(clock_ms<deadline && run.state==RUN_IDLE && concurrent && disturbed);
        CHECK(fabs(field_x-x-(mask&1 ? 80 : -80))<3.0 && fabs(field_y-y-(mask&1 ? 80 : -80))<3.0);
    }
    puts("PASS MOVEXY encoder projection: all16 feedback polarities, unequal wheel acceleration and1deg injected yaw disturbance; diagonal near-cancelling wheels do not require false four-wheel sign learning; independent two-axis endpoint");
}
static void movexy_fifo_and_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<5;variant++) {
        uint32_t deadline;unsigned segments=0,was_shift=0,positions=0,started=0;int16_t px,py;
        double x,y;
        prepare_vision_debug();send_line("vision debug F 2");wait_debug_request(1);x=field_x;y=field_y;
        if(variant==1){missing_feedback_motor=4;motor_feedback[3].speed_valid=0;}
        peer_reply("MOVE,-10",0);peer_reply("MOVEXY,+40,-20",0);peer_reply("MOVEXY,0,0",0);
        peer_reply("MOVEY,+5",0);peer_reply("MOVEXY,-15,+30",0);peer_reply("STACK,700,-50,620,1",0);
        if(variant==1) {
            tick(1000);CHECK(!run.station_shift && vision_motion_peek_xy(&px,&py,NULL) && px== -10 && py==0);
            missing_feedback_motor=0;
        }
        deadline=clock_ms+90000;
        while(run.vision_debug && run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();
            if(run.station_shift && !was_shift) {
                static const int targets[]={10,40,5,30};
                CHECK(segments<4 && run.remaining_mm==targets[segments]);
                CHECK(run.vector_move==(segments==1 || segments==3));segments++;
            }
            was_shift=run.station_shift;
            if(run.vector_move && run.velocity_phase==VELOCITY_FORWARD && abs(run.vector_progress_mm[0])>5 && !started) {
                started=1;
                if(variant==2)stop_request();
                if(variant==3)bus_off=1;
                if(variant==4){missing_feedback_motor=4;tick(600);CHECK(run.vision_debug && run.state!=RUN_FAULT && !run.vector_mrpm[0] && !run.vector_mrpm[1]);missing_feedback_motor=0;}
            }
            CHECK(arm_positions==positions);
        }
        CHECK(clock_ms<deadline && !run.vision_debug && !vision_motion_peek_xy(NULL,NULL,NULL));
        if(variant==2 || variant==3) {
            CHECK(started && !run_active && arm_failed());
            peer_reply("MOVEXY,100,100",0);tick(1000);CHECK(!run_active && !station_active());
        } else {
            CHECK(run.state==RUN_IDLE && segments==4 && started);
            CHECK(fabs(field_x-x-15)<5.0 && fabs(field_y-y+15)<5.0);
        }
    }
    puts("PASS MOVEXY mixed FIFO: legacy/vector/zero entries retained in receive order, same early STACK drains four actions; missing launch and in-motion feedback retains work, emergency and bus-off cancel pair and late commands");
}
static void movexy_station_tests(void)
{
    unsigned area;
    for(area=0;area<3;area++) {
        uint32_t deadline;unsigned offset=0,positions,finished=0,concurrent=0;
        double x,y;
        if(area==0 && !alignment_station_enabled(0))continue;
        if(area==1 && !alignment_station_enabled(1))continue;
        if(area==2 && (!STN_STORAGE_PLACE_READY || !STN_PLATFORM_TAKE_READY || STN_STORAGE_LAYER_LIFT_MM<=0
            || STN_STORAGE_LAYER_LIFT_MM>ARM_LIFT_MAX_MM-STN_STORAGE_LIFT_MM))continue;
        if(area==0)prepare_rough(0,"task 134+231+314+123");else prepare_storage(area==2,"task 134+231+314+123");
        peer_stack_hold=1;deadline=clock_ms+60000;
        while(clock_ms<deadline && (area==2 ? !peer_stack_count : vision_align_state()!=VISION_PICK_WAITING || vision_tx_count<6)) {
            if(area==2)serve_peer(&offset);
            plant_step();
        }
        /* F现在可早于固定索引完成发出；端点位移从该索引结束处计算。 */
        while(station_chassis_busy() && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline);x=field_x;y=field_y;positions=arm_positions;
        peer_reply("MOVEXY,+40,-25",0);peer_reply(area==2 ? "STACK,700,-50,620,1" : "DISC,700,-50,620,1",0);
        while(clock_ms<deadline && run.state!=RUN_FAULT && !finished) {
            plant_step();check_correction_arm(positions);
            if(run.vector_move && abs(run.forward_cmd_rpm)>1 && abs(run.wheel_rpm[0]-run.wheel_rpm[1])>2)concurrent++;
            if(strstr(output,area==2 ? "[station stack]" : "[station aligned]"))finished=1;
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && finished && concurrent);
        CHECK(fabs(field_x-x-(area ? -25 : -40))<3.0 && fabs(field_y-y-(area ? -40 : 25))<3.0);
        CHECK(vision_tx_count==(area==2 ? 8u : 6u));stop_request();
    }
    puts("PASS MOVEXY automatic jobs: rough D, first storage E, second storage direct F; true two-axis endpoint before grip; final approach may overlap; no extra requests or terminal-coordinate movement");
}
static void movexy_storage_retained_tests(void)
{
    static const int xp[3]={20,-15,5},yp[3]={-10,25,-5},rings[3]={3,2,1};
    unsigned offset=0,sent=0,placed=0,started=0,was_shift=0;
    uint32_t deadline;RoadPoint point;
    if(!STN_STORAGE_PLACE_READY || !STN_PLATFORM_TAKE_READY || STN_STORAGE_LAYER_LIFT_MM<=0
       || STN_STORAGE_LAYER_LIFT_MM>ARM_LIFT_MAX_MM-STN_STORAGE_LIFT_MM)return;
    prepare_storage(1,"task 134+231+314+123");peer_stack_hold=1;
    CHECK(road_point(NODE_H,&point));deadline=clock_ms+250000;
    while((run.job==6 || run.departure_overlap) && run.state!=RUN_FAULT && clock_ms<deadline) {
        int slide,lift;float yaw,claw;
        serve_peer(&offset);
        if(peer_stack_count>sent) {
            char payload[40];
            CHECK(peer_stack_count==sent+1u && sent<3);
            snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",xp[sent],yp[sent]);peer_reply(payload,0);
            peer_reply("STACK,700,-50,620,1",0);sent++;
        }
        plant_step();CHECK(!strstr(output,"[stack return]"));
        if(run.station_shift && !was_shift) {
            CHECK(started<7);
            if(started&1u) {
                unsigned n=started/2u;
                CHECK(run.vector_move && run.vector_target_mm[0]== -xp[n] && run.vector_target_mm[1]==yp[n]);
            } else CHECK(!run.vector_move && run.travel_axis==BODY_FORWARD && run.remaining_mm==150
                         && run.travel_sign==((started==0 || started==6) ? -1 : 1));
            started++;
        }
        was_shift=run.station_shift;
        arm_pose_now(&slide,&lift,&yaw,&claw);
        if(placed<sent && slide==55 && lift==STN_STORAGE_LIFT_MM+STN_STORAGE_LAYER_LIFT_MM
           && yaw==126.5f && claw==ARM_CLAW_OPEN_DEG) {
            unsigned i;int sum_x=0,sum_y=0;
            for(i=0;i<=placed;i++){sum_x+=yp[i];sum_y-=xp[i];}
            CHECK(fabs(field_x-point.x-sum_x)<8.0);
            CHECK(fabs(field_y-point.y-(rings[placed]-2)*STN_STORAGE_INDEX_MM*STN_STORAGE_INDEX_SIGN-sum_y)<8.0);placed++;
        }
    }
    CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==7 && sent==3 && placed==3 && started==7);
    CHECK(peer_stack_count==3 && !peer_disc_count && arm_parked() && arm_platform_slot()==1);
    CHECK(fabs(field_x-point.x-10)<8.0 && fabs(field_y-point.y+10)<8.0);stop_request();
    puts("PASS MOVEXY storage retained: all three F pairs execute as3 concurrent segments plus4 original indexes, exact7 segments; placements and final cumulative offsets retained, no inverse movement/E/resend");
}
static void movexy_tests(void)
{
    movexy_transport_tests();movexy_debug_tests();movexy_polarity_response_tests();movexy_fifo_and_cancel_tests();movexy_station_tests();movexy_storage_retained_tests();
}
