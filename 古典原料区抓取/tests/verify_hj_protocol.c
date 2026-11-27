/* 新H方向格式/J状态解析、长帧和人工取消；主机模型，不访问相机。 */
#define STATION_TEST_MAIN retained_hj_station_suite
#include "verify_station_vision.c"

static void j_begin(void)
{
    reset_plant(0,1.15);send_line("vision j");tick(10);
    CHECK(vision_tx_count==6 && vision_tx_bytes[3]=='J' && vision_material_state()==VISION_PICK_WAITING);
    CHECK(!arm_positions && !velocity_batches && !station_active());
}

static void state_protocol_tests(void)
{
    static const char *bad[]={"MSTATE,1,0","MSTATE,2,1,1,1,1,1","MSTATE,1,5",
        "MSTATE,1,1,0,320,240,1","MSTATE,1,1,5,320,240,1","MSTATE,1,1,1,320,240,0",
        "MSTATE,1,1,1,320,240,2","MSTATE,1,1,1,320,240,3","MSTATE,1,1,1,32768,240,1",
        "MSTATE,1,1,1,320,-32769,1","MSTATE,1,1,1,320,240,1,9","MSTATE,1,2,1,0,0,1,1,0,0,1",
        "MSTATE,1,1,1,320,240"};
    const char *four="MSTATE,1,4,1,320,240,1,2,320,240,1,3,320,240,1,4,320,240,1";
    VisionMaterial samples[4];
    j_begin();peer_reply("MSTATE,0,0",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_WAITING && !vision_material_state_result(samples));
    peer_reply("MSTATE,0,2,1,320,240,0,3,450,210,2",0);vision_update(clock_ms);
    CHECK(vision_material_state_result(samples)==2 && samples[0].state==0 && samples[1].state==2);
    peer_reply("MOVEXY,100,100",0);peer_reply("HDISC,320,240,0,3,1,1",0);vision_update(clock_ms);
    CHECK(!vision_motion_peek_xy(NULL,NULL,NULL) && vision_material_state()==VISION_PICK_WAITING);
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        peer_reply(bad[i],0);vision_update(clock_ms);CHECK(vision_material_state()==VISION_PICK_WAITING);
    }
    peer_reply(four,1);vision_update(clock_ms);CHECK(vision_material_state()==VISION_PICK_WAITING);
    clock_ms+=61000;vision_update(clock_ms);CHECK(vision_tx_count==6 && vision_material_state()==VISION_PICK_WAITING);
    CHECK(strlen(four)==58 && strlen(four)>48);peer_reply(four,0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==4);
    for(unsigned i=0;i<4;i++)CHECK(samples[i].color==i+1 && samples[i].state==1 && samples[i].x==320 && samples[i].y==240);
    peer_reply("MSTATE,0,0",0);vision_update(clock_ms);CHECK(vision_material_state_result(samples)==4);
    CHECK(!arm_positions && !velocity_batches && vision_tx_count==6);
    send_line("vision j");tick(10);CHECK(!vision_material_state_result(samples));
    peer_reply("MSTATE,1,1,2,-32768,32767,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==1);
    CHECK(samples[0].x==-32768 && samples[0].y==32767);
    puts("PASS J: exact field counts, unique colors, empty/moving/insufficient waits, signed bounds, CRC,58-byte frame, no timeout/retry/motion");
}

static void unordered_result_test(void)
{
    VisionMaterial samples[4];
    j_begin();peer_reply("MSTATE,1,2,3,491,32,1,1,310,315,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==2);
    CHECK(samples[0].color==3 && samples[0].x==491 && samples[1].color==1 && samples[1].y==315);
    puts("PASS handover J: distinct colors need no record ordering");
}

static void cancel_ready_test(void)
{
    reset_plant(0,1.15);send_line("vision h");tick(10);
    CHECK(vision_tx_count==6 && vision_align_state()==VISION_PICK_WAITING);
    send_line("vision ping");tick(10);CHECK(vision_tx_count==6 && vision_align_state()==VISION_PICK_WAITING);
    vision_cancel_pick();vision_cancel_pick();tick(10);
    CHECK(vision_tx_count==12 && vision_tx_bytes[9]=='R');
    /* R已发送仍不是取消完成；下一请求只在相机READY后发送。 */
    vision_request_material_state(clock_ms);tick(10);
    CHECK(vision_tx_count==12 && vision_material_state()==VISION_PICK_WAITING);
    peer_reply("HDISC,371,298,51,2,1,0",0);
    peer_reply("MSTATE,1,1,1,310,315,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_WAITING && !vision_motion_peek_xy(NULL,NULL,NULL));
    peer_reply("READY",0);vision_update(clock_ms);tick(10);
    CHECK(vision_tx_count==18 && vision_tx_bytes[15]=='J');
    peer_reply("MSTATE,1,1,1,310,315,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY);
    vision_cancel_pick();tick(10);CHECK(vision_tx_count==18); /* 成功交接不发R。 */

    j_begin();vision_cancel_pick();tick(10);
    CHECK(vision_tx_count==12 && vision_tx_bytes[9]=='R');
    vision_request_align('H',2,clock_ms);tick(10);CHECK(vision_tx_count==12);
    peer_reply("BUSY",0);vision_update(clock_ms);tick(10);CHECK(vision_tx_count==12);
    peer_reply("READY",0);vision_update(clock_ms);tick(10);
    CHECK(vision_tx_count==18 && vision_tx_bytes[15]=='H');
    peer_reply("HDISC,371,298,51,2,1,0",0);vision_update(clock_ms);
    CHECK(vision_align_state()==VISION_PICK_READY && vision_material_direction()==0);
    puts("PASS handover cancellation: H/J -> one R -> READY -> next request, stale packets ignored");
}

static void session_cancel_tests(void)
{
    j_begin();send_line("vision h");send_line("vision ping");tick(10);CHECK(vision_tx_count==6 && vision_material_state()==VISION_PICK_WAITING);
    send_line("vision range");tick(10);
    CHECK(vision_material_state()==VISION_PICK_IDLE && vision_tx_count==12 && vision_tx_bytes[9]=='R');
    peer_reply("MSTATE,1,1,1,320,240,1",0);vision_update(clock_ms);CHECK(vision_material_state()==VISION_PICK_IDLE);
    j_begin();queue_byte('!');firmware_step();firmware_step();tick(10);
    CHECK(vision_material_state()==VISION_PICK_IDLE && vision_tx_count==12 && vision_tx_bytes[9]=='R');
    CHECK(!arm_positions && !run_active && !station_active());
    reset_plant(0,1.15);vision_tx_ready=0;send_line("vision j");tick(10);
    peer_reply("MSTATE,1,1,1,320,240,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_WAITING && !vision_tx_count);
    vision_tx_ready=1;tick(10);CHECK(vision_tx_count==6 && vision_material_state()==VISION_PICK_WAITING);
    peer_reply("MSTATE,1,1,1,320,240,1",0);vision_update(clock_ms);CHECK(vision_material_state()==VISION_PICK_READY);
    puts("PASS J sessions: request-delivery gate, manual R, repeated local emergency cancel retains one R, stale terminal cannot restart");
}

int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"cancel")){cancel_ready_test();return 0;}
    unordered_result_test();cancel_ready_test();state_protocol_tests();session_cancel_tests();
    printf("PASS H/J protocol: %u assertions; host only\n",assertions);return 0;
}
