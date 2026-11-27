#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static const char task_code[]="134+123+314+231";

static void expect_frame(unsigned *offset,const char *payload)
{
    unsigned length=(unsigned)strlen(payload),total=length+5u;
    uint32_t deadline=clock_ms+30000u;
    while(vision_tx_count<*offset+total && clock_ms<deadline)plant_step();
    if(vision_tx_count<*offset+total)
        fprintf(stderr,"waiting for %s, tx=%u offset=%u station_failed=%d: %s\n",
            payload,(unsigned)vision_tx_count,*offset,station_failed(),output);
    CHECK(vision_tx_count==*offset+total && vision_tx_bytes[*offset]==0xaau);
    CHECK(vision_tx_bytes[*offset+1u]==0x55u && vision_tx_bytes[*offset+2u]==length);
    CHECK(!memcmp(&vision_tx_bytes[*offset+3u],payload,length));
    CHECK(vision_tx_bytes[*offset+total-2u]==peer_crc(&vision_tx_bytes[*offset+2u],length+1u));
    *offset+=total;
}

static void raw_batch(unsigned batch,int fail_second)
{
    static const int coordinates[3][2]={{491,32},{136,44},{310,315}};
    static const float yaw[3]={ARM_RAW_PAST_YAW_DEG,ARM_RAW_NEXT_YAW_DEG,ARM_RAW_CURRENT_YAW_DEG};
    const char *colors=batch ? "314" : "134";
    unsigned offset=0;
    RoadPoint point;
    prepare_arm_auto(batch);
    send_line("task 134+123+314+231");
    CHECK(road_point(NODE_F,&point) && road_plan(NODE_E,NODE_F,&run.road));
    field_x=planned_x=point.x;field_y=planned_y=point.y;
    run.imu_mode=run.station_enabled=1;run.state=RUN_STATION;
    run.heading=run.target_heading=initial_field_heading=HEAD_X_NEG;
    run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_NEG*9000;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
    run.job=(uint8_t)(batch ? 4u : 1u);run.edge=(uint8_t)(batch ? 9u : 3u);
    checked_job=FULL_ROUTE_JOBS;
    expect_frame(&offset,"T,134+123+314+231");
    peer_reply("BUSY",0);tick(200);CHECK(vision_tx_count==offset);
    peer_reply("TASK,134,314",0);
    expect_frame(&offset,"H");
    peer_reply("HCOLORS,1,3,0",0);peer_reply("HCOLORS,1,3,0",0);
    peer_reply("MOVEXY,18,-12",0);
    tick(100);CHECK(station_chassis_busy() && vision_tx_count==offset && !arm_positions);
    /* H 当中的位置与随后各件 PICK 不同，仍按本次 C 选择抓点。 */
    peer_reply(batch ? "HDISC2,1,3,4,120,40,1,310,315,3,490,32"
                     : "HDISC2,0,3,4,120,40,1,310,315,3,490,32",0);
    for(unsigned slot=0;slot<3u;slot++) {
        char request[4],reply[24];
        uint32_t deadline;
        int saw_grip=0;
        snprintf(request,sizeof(request),"C,%c",colors[slot]);
        expect_frame(&offset,request);
        CHECK(!station_chassis_busy() && peer_wheels_stopped() && !arm_busy());
        CHECK(arm_platform_slot()==(int)slot+1 && vision_material_h_result(NULL,NULL)==(int)batch);
        if(!slot)CHECK(hypot(field_x-point.x,field_y-point.y)>10.0);
        peer_reply("BUSY",0);tick(100);CHECK(vision_tx_count==offset && !arm_busy());
        if(fail_second && slot==1u) {
            unsigned positions=arm_positions;
            peer_reply("NONE,C",0);expect_frame(&offset,"R");
            peer_reply("READY",0);tick(400);
            CHECK(station_failed() && vision_tx_count==offset && arm_positions==positions && !arm_busy());
            stop_request();return;
        }
        snprintf(reply,sizeof(reply),"PICK,%c,%d,%d",colors[slot],coordinates[slot][0],coordinates[slot][1]);
        peer_reply(reply,0);peer_reply(reply,0);
        deadline=clock_ms+15000u;
        while(!saw_grip && clock_ms<deadline && !station_failed()) {
            float angle,claw;
            plant_step();arm_pose_now(NULL,NULL,&angle,&claw);
            CHECK(vision_tx_count==offset);
            if(claw==ARM_CLAW_CLOSE_DEG){CHECK(angle==yaw[slot]);saw_grip=1;}
        }
        CHECK(saw_grip);
        printf("RAW_C batch=%u slot=%u color=%c x=%d y=%d yaw=%.1f\n",
            batch+1u,slot+1u,colors[slot],coordinates[slot][0],coordinates[slot][1],(double)yaw[slot]);
    }
    {
        uint32_t deadline=clock_ms+30000u;
        while(station_active() && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && !station_failed() && arm_parked() && arm_platform_slot()==1);
        CHECK(vision_tx_count==offset);
    }
    stop_request();
}

static unsigned open_camera_session(unsigned stage)
{
    unsigned offset=0;
    reset_plant(0,1.15);vision_init();
    vision_request_raw_task(task_code,clock_ms);
    expect_frame(&offset,"T,134+123+314+231");
    if(stage==0)return offset;
    peer_reply("TASK,134,314",0);tick(1);CHECK(vision_raw_task_state()==VISION_PICK_READY);
    vision_request_raw_align(clock_ms);expect_frame(&offset,"H");
    if(stage==1)return offset;
    peer_reply("HDISC2,1,1,3,340,500",1); /* 首次 CRC 坏帧由下一份有效帧恢复。 */
    tick(1);CHECK(vision_raw_rx_pending() && vision_align_state()==VISION_PICK_WAITING);
    peer_reply("HDISC2,1,1,3,340,110",0);tick(1);
    CHECK(!vision_raw_rx_pending() && vision_align_state()==VISION_PICK_READY);
    vision_request_pick(1,clock_ms);expect_frame(&offset,"C,1");
    return offset;
}

static void raw_protocol_failures(void)
{
    static const struct {unsigned stage;const char *reply;} cases[]={
        {0,"TASK,314,134"},{1,"NONE,H"},{1,"ERR,H"},{1,"HDISC2,0,0"},
        {1,"HDISC2,1,2,1,320,240,1,120,30"},{1,"HDISC2,1,2,1,320,240"},
        {1,"HDISC2,1,1,3,invalid,110"}, {2,"NONE,C"},{2,"PICK,3,300,200"},
        {2,"PICK,1"},{2,"PICK,1,640,200"},{2,"PICK,1,300,200,1"},{2,"READY"}
    };
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        unsigned offset=open_camera_session(cases[i].stage);
        peer_reply(cases[i].reply,0);tick(1);
        CHECK(vision_raw_failed() && vision_pick_state()!=VISION_PICK_READY && !arm_positions);
        expect_frame(&offset,"R");
        peer_reply("READY",0);tick(20);CHECK(vision_tx_count==offset);
    }
    for(unsigned failure=0;failure<4u;failure++) {
        unsigned offset=open_camera_session(2);
        if(failure<=1u) {peer_reply("PICK,1,300,200",1);tick(1);CHECK(vision_raw_rx_pending());}
        if(failure==0)peer_reply("PICK,1,300,200",1);
        else if(failure==1)vision_update(clock_ms+1100u);
        else if(failure==2)vision_receive_byte(0,1);
        else vision_update(clock_ms+26000u);
        tick(1);CHECK(vision_raw_failed() && !arm_positions);expect_frame(&offset,"R");
    }
    for(unsigned stage=1;stage<=2u;stage++) {
        unsigned offset=open_camera_session(stage);
        vision_cancel_pick();expect_frame(&offset,"R");
        vision_request_raw_task(task_code,clock_ms);
        peer_reply("PICK,1,310,315",0);tick(200);
        CHECK(vision_tx_count==offset && vision_pick_state()==VISION_PICK_IDLE);
        peer_reply("READY",0);expect_frame(&offset,"T,134+123+314+231");
    }
    puts("PASS raw protocol: strict TASK/HDISC2/PICK, CRC recovery, timeout, reset, one R and READY isolation");
}

int main(void)
{
    raw_batch(0,0);raw_batch(1,0);raw_batch(0,1);raw_protocol_failures();
    printf("PASS H once/C3: %u assertions; host model only\n",assertions);
    return 0;
}
