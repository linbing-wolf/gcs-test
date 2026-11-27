/* 新H的双色终包与真实原料工位联合模拟；不连接车辆或相机。 */
#define RAW_PLATE_TEST_MAIN retained_raw_plate_main
#include "verify_raw_plate.c"

static void manual_h(void)
{
    prepare_arm_auto(0);send_line("vision h");tick(10);
    CHECK(vision_tx_count==6 && vision_tx_bytes[3]=='H');
    CHECK(vision_align_state()==VISION_PICK_WAITING && !arm_positions && !station_active());
}

static void material_protocol_tests(void)
{
    static const char *bad[]={"COLORS,3,1","DISC,300,240,-20,3,1","HCOLORS,1,1,1","HCOLORS,0,3,1",
        "HCOLORS,3,5,1","HCOLORS,3,1,01","HCOLORS,3,1,+1","HCOLORS,3,1,2","HCOLORS,3,1,-1",
        "HCOLORS,3,1,1 ","HCOLORS,3,1,CW","HCOLORS,3,1,UNKNOWN","HCOLORS,3,1,",
        "HDISC,300,240,-20,3,3,1","HDISC,300,240,-20,0,1,1","HDISC,300,240,-20,3,5,1",
        "HDISC,300,240,-20,3,1,1,4","HDISC,300,240,-20,3,x,1"};
    uint8_t first=0,second=0;
    unsigned i;
    manual_h();
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        peer_reply(bad[i],0);vision_update(clock_ms);
        CHECK(vision_align_state()==VISION_PICK_WAITING && !vision_material_colors(NULL,NULL));
    }
    peer_reply("HDISC,300,240,-20,3,1,1",1);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_WAITING);
    /* 终包不依赖HCOLORS预报；旧H格式不再放行。 */
    peer_reply("HDISC,300,240,-20,3,1,1",0);
    vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_READY);
    CHECK(vision_material_colors(&first,&second) && first==3 && second==1);
    tick(100);CHECK(!arm_positions && !run_active && !station_active());
    send_line("vision h");tick(10);
    CHECK(!vision_material_colors(NULL,NULL));
    peer_reply("HCOLORS,2,4,0",0);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_WAITING && !vision_material_colors(NULL,NULL));
    peer_reply("HCOLORS,2,4,0",0);peer_reply("HCOLORS,4,2,0",0);
    peer_reply("HDISC,300,240,0,2,4,1",0);peer_reply("HDISC,300,240,0,4,2,0",0);
    peer_reply("DISC,300,240,0,1",0);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_WAITING);
    peer_reply("HDISC,300,240,0,2,4,0",0);vision_update(clock_ms);
    CHECK(vision_material_colors(&first,&second) && first==2 && second==4 && vision_material_direction()==0);
    vision_request_pick(4,clock_ms);CHECK(!vision_material_colors(NULL,NULL));
    for(i=0;i<2;i++) {
        vision_request_align(i ? 'E' : 'D',2,clock_ms);tick(10);
        peer_reply("COLORS,3,1",0);peer_reply("DISC,300,240,0,3,1",0);vision_update(clock_ms);
        CHECK(vision_align_state()==VISION_PICK_WAITING && !vision_material_colors(NULL,NULL));
        peer_reply("DISC,300,240,0,1",0);vision_update(clock_ms);
        CHECK(vision_align_state()==VISION_PICK_READY && !vision_material_colors(NULL,NULL));
    }
    vision_request_align('H',2,clock_ms);tick(10);
    peer_reply("DISC,300,240,0,1",0);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_WAITING && !vision_material_colors(NULL,NULL));
    peer_reply("HDISC,300,240,0,3,1,1",0);vision_update(clock_ms);
    CHECK(vision_material_colors(&first,&second) && first==3 && second==1);
    vision_cancel_pick();CHECK(!vision_material_colors(NULL,NULL));
    stop_request();
    puts("PASS H protocol: HCOLORS/HDISC direct/preview, strict0/1 direction, CRC/color/order agreement, old H rejected, D/E retained; manual H never moves");
}

static float material_slot_angle(unsigned slot)
{
    const float angles[]={ARM_PLATFORM_FIRST_DEG,ARM_PLATFORM_SECOND_DEG,ARM_PLATFORM_THIRD_DEG};
    CHECK(slot>=1 && slot<=3);return angles[slot-1u];
}

/* 用不同于生产表的独立期待值核对实际抓取完成姿态，包含各自高度参数。 */
static void material_expected_pose(unsigned pose,int *slide,int *lift,float *yaw)
{
    if(pose==0) {*slide=ARM_RAW_PAST_SLIDE_MM;*lift=ARM_RAW_PAST_LIFT_MM;*yaw=ARM_RAW_PAST_YAW_DEG;}
    else if(pose==1) {*slide=ARM_RAW_CURRENT_SLIDE_MM;*lift=ARM_RAW_CURRENT_LIFT_MM;*yaw=ARM_RAW_CURRENT_YAW_DEG;}
    else {*slide=ARM_RAW_NEXT_SLIDE_MM;*lift=ARM_RAW_NEXT_LIFT_MM;*yaw=ARM_RAW_NEXT_YAW_DEG;}
}

static void material_loading_tests(void)
{
    unsigned omitted,batch,a,b,direction,cases=0;
    for(omitted=1;omitted<=4;omitted++)for(batch=0;batch<2;batch++)
        for(a=0;a<3;a++)for(b=0;b<3;b++)if(a!=b)for(direction=0;direction<2;direction++) {
            uint8_t colors[3],by_slot[3];
            unsigned i,n=0,gripped=0,released=0,saw_motion=0,saw_order=0,aligned=0;
            int32_t words_before[2],words_after[2];
            int forward=0,left=0;
            double x,y,aligned_x=0,aligned_y=0;
            char task[48],payload[48];
            float previous=ARM_CLAW_OPEN_DEG;
            uint32_t deadline;
            uint32_t j_started_ms=0;unsigned j_done=0,j_positions=0;
            uint8_t job=(uint8_t)(batch ? 4 : 1);
            for(i=1;i<=4;i++)if(i!=omitted)colors[n++]=(uint8_t)i;
            for(i=0;i<3;i++)by_slot[i]=colors[(i+(batch ? 2u : 0u))%3u];
            snprintf(task,sizeof(task),"task %u%u%u+231+%u%u%u+312",
                colors[0],colors[1],colors[2],colors[2],colors[0],colors[1]);
            prepare_raw_plate_task(batch,task);station_task_words(&words_before[0],&words_before[1]);
            x=field_x;y=field_y;
            if(cases%3u==1u) {peer_reply("MOVE,12",0);forward=-12;}
            if(cases%3u==2u) {peer_reply("MOVEXY,-18,+11",0);forward=18;left=11;}
            if(cases&1u) {
                snprintf(payload,sizeof(payload),"HCOLORS,%u,%u,%u",colors[a],colors[b],direction);peer_reply(payload,0);tick(25);
                CHECK(vision_tx_count==6 && !arm_positions && vision_align_state()==VISION_PICK_WAITING);
            }
            snprintf(payload,sizeof(payload),"HDISC,300,240,-20,%u,%u,%u",colors[a],colors[b],direction);peer_reply(payload,0);
            deadline=clock_ms+150000;
            while(run.job==job && clock_ms<deadline && run.state!=RUN_FAULT) {
                int slide,lift;float yaw,claw;
                plant_step();
                CHECK(vision_tx_count>=6 && vision_tx_count<=18);
                if(vision_material_state()==VISION_PICK_WAITING && vision_tx_count==12u+6u*j_done) {
                    CHECK(gripped==j_done+1u && released==gripped && arm_ready());
                    if(!j_started_ms) {
                        j_started_ms=clock_ms;j_positions=arm_positions;
                        peer_reply("MSTATE,0,0",0);peer_reply("MSTATE,0,1,1,320,240,2",0);
                    }
                    CHECK(arm_positions==j_positions);
                    if(clock_ms-j_started_ms>=500u) {
                        /* 当前任务目标色移动到新抓点，不沿用H预排。 */
                        snprintf(payload,sizeof(payload),"MSTATE,1,1,%u,%d,%d,1",by_slot[j_done+1u],j_done ? 310 : 136,j_done ? 315 : 44);
                        peer_reply(payload,0);j_done++;j_started_ms=0;
                    }
                }
                if(strstr(output,"[station raw order]"))saw_order=1;
                if(strstr(output,"[station raw aligned]")) {aligned=1;aligned_x=field_x;aligned_y=field_y;}
                if(aligned && run.job==job) {
                    CHECK(!station_chassis_busy() && !run.station_shift && !run_active);
                    CHECK(fabs(field_x-aligned_x)<0.01 && fabs(field_y-aligned_y)<0.01);
                }
                if(station_chassis_busy()) {
                    saw_motion=1;CHECK(!arm_positions && !gripped);
                }
                arm_pose_now(&slide,&lift,&yaw,&claw);
                if(previous==ARM_CLAW_OPEN_DEG && claw==ARM_CLAW_CLOSE_DEG) {
                    unsigned pose;int expected_slide,expected_lift;float expected_yaw;
                    CHECK(gripped<3 && aligned);
                    pose=gripped==1u ? 2u : gripped==2u ? 1u
                        : by_slot[0]==colors[a] ? (direction ? 0u : 2u)
                        : by_slot[0]==colors[b] ? 1u : (direction ? 2u : 0u);
                    CHECK(j_done==gripped);
                    material_expected_pose(pose,&expected_slide,&expected_lift,&expected_yaw);
                    CHECK(slide==expected_slide && lift==expected_lift && yaw==expected_yaw);
                    CHECK(arm_platform_slot()==(int)(gripped+1u));
                    CHECK(!station_chassis_busy() && !run.station_shift);
                    gripped++;
                }
                if(previous==ARM_CLAW_CLOSE_DEG && claw==ARM_CLAW_OPEN_DEG) {
                    CHECK(released<3 && slide==ARM_PLACE_SLIDE_MM && lift==ARM_PLACE_LIFT_MM && yaw==ARM_PLACE_YAW_DEG);
                    CHECK(arm_platform_angle()==material_slot_angle(released+1u));
                    CHECK(arm_platform_slot()==(int)(released+1u));released++;
                }
                previous=claw;
            }
            CHECK(clock_ms<deadline && run.state!=RUN_FAULT && run.job==job+1u);
            CHECK(gripped==3 && released==3 && vision_tx_count==18 && aligned && j_done==2);
            CHECK(vision_tx_bytes[3]=='H' && vision_tx_bytes[9]=='J' && vision_tx_bytes[15]=='J');
            CHECK((forward || left) ? saw_motion : !saw_motion);
            CHECK(fabs(field_x-x+forward)<3.0 && fabs(field_y-y-left)<3.0);
            while(arm_busy() && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && arm_ready() && arm_parked() && arm_platform_slot()==1);
            station_task_words(&words_after[0],&words_after[1]);
            CHECK(words_before[0]==words_after[0] && words_before[1]==words_after[1]);
            CHECK(saw_order && !station_failed());
            stop_request();cases++;
        }
    printf("PASS H/J fixed grips: %u cases, all color sets / six orders / both directions / both batches; exact H-grip-J-grip-J-grip, J target coordinates refresh second/third fixed poses, exact cups and slot1 departure\n",cases);
}

static void material_wait_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        prepare_raw_plate(0);
        peer_reply(variant==1 ? "HDISC,300,240,0,2,1,1" : "HCOLORS,3,1,1",0);tick(1000);
        CHECK(vision_tx_count==6 && !arm_positions && station_active());
        if(variant==0) {
            peer_reply("NONE,H",0);tick(61000);
            CHECK(vision_tx_count==6 && !arm_positions && station_active());
            peer_reply("HDISC,300,240,0,1,3,1",0);tick(100);
            CHECK(vision_tx_count==6 && !arm_positions);
            peer_reply("HDISC,300,240,0,3,1,1",0);tick(100);
            CHECK(vision_tx_count==6 && station_active());
        }
        if(variant==3) {
            peer_reply("MOVE,-80",0);peer_reply("HDISC,300,240,0,3,1,1",0);tick(100);
            CHECK(station_chassis_busy() && !arm_positions);
        }
        stop_request();tick(100);
        CHECK(!station_active() && !run_active && !vision_material_colors(NULL,NULL));
        if(variant!=0)CHECK(!arm_positions);
    }
    puts("PASS H waits/cancel: preview/timeout/mismatch/non-task color never grip; emergency clears waiting/moving material session without C");
}

static void material_j_wait_cancel_tests(void)
{
    for(unsigned gate=1;gate<=2;gate++) {
        unsigned first_done=0,positions,target=6u+6u*gate;uint32_t deadline;
        prepare_raw_plate(0);peer_reply("HDISC,320,240,0,3,1,1",0);deadline=clock_ms+60000;
        while(vision_tx_count<target && clock_ms<deadline) {
            plant_step();
            if(gate==2 && !first_done && vision_tx_count==12 && vision_material_state()==VISION_PICK_WAITING) {
                peer_reply("MSTATE,1,1,3,136,44,1",0);first_done=1;
            }
        }
        CHECK(clock_ms<deadline && vision_tx_count==target && vision_material_state()==VISION_PICK_WAITING);
        positions=arm_positions;
        peer_reply("MSTATE,1,0",0);peer_reply("MSTATE,1,1,1,320,240,0",0);
        peer_reply("MSTATE,0,0",0);tick(100);
        CHECK(arm_positions==positions && station_active());
        peer_reply("MSTATE,0,1,1,320,240,2",0);peer_reply("NONE,J",0);peer_reply("BUSY",0);tick(61000);
        CHECK(arm_positions==positions && station_active() && arm_ready());
        CHECK(vision_tx_count>=target+6u*121u && vision_tx_count<=target+6u*123u
            && (vision_tx_count-target)%6u==0);
        for(unsigned at=target;at<vision_tx_count;at+=6u) {
            CHECK(vision_tx_bytes[at]==0xaa && vision_tx_bytes[at+1u]==0x55
                && vision_tx_bytes[at+2u]==1 && vision_tx_bytes[at+3u]=='J'
                && vision_tx_bytes[at+4u]==peer_crc(vision_tx_bytes+at+2u,2)
                && vision_tx_bytes[at+5u]==13);
        }
        target=vision_tx_count;
        stop_request();tick(20);
        CHECK(!station_active() && !run_active && vision_material_state()==VISION_PICK_IDLE);
        CHECK(vision_tx_count==target+6u && vision_tx_bytes[target+3u]=='R' && arm_positions==positions);
        tick(1000);CHECK(vision_tx_count==target+6u && !station_active());
    }
    puts("PASS automatic J gates: both waits survive invalid/moving/empty/insufficient/NONE/BUSY, repeat valid J every500ms after zero for61s without arm movement; stop sends R and stops repeats");
}

int main(void)
{
    loading_tests();
    material_protocol_tests();
    material_loading_tests();
    material_wait_cancel_tests();
    material_j_wait_cancel_tests();
    printf("PASS raw material: %lu assertions; host model only\n",(unsigned long)assertions);
    return 0;
}
