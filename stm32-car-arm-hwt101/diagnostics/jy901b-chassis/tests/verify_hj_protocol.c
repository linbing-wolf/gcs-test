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
        "MSTATE,1,2,3,0,0,1,1,0,0,1","MSTATE,1,1,1,320,240"};
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
    CHECK(!arm_positions && !velocity_batches && vision_tx_count>=6 && vision_tx_count<=12);
    send_line("vision j");tick(10);CHECK(!vision_material_state_result(samples));
    peer_reply("MSTATE,1,1,2,-32768,32767,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_material_state_result(samples)==1);
    CHECK(samples[0].x==-32768 && samples[0].y==32767);
    puts("PASS J: exact fields/colors/bounds/CRC,58-byte frame, indefinite wait and no motion; valid zero arms repeats");
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

static void repeat_zero_tests(void)
{
    unsigned sent;
    j_begin();peer_reply("MSTATE,0,1,1,491,32,0",0);vision_update(clock_ms);
    tick(499);CHECK(vision_tx_count==6);tick(10);
    CHECK(vision_tx_count==12 && vision_tx_bytes[9]=='J' && vision_material_state()==VISION_PICK_WAITING);
    peer_reply("BUSY",0);tick(500);CHECK(vision_tx_count==18 && vision_material_state()==VISION_PICK_WAITING);
    peer_reply("MSTATE,1,1,1,491,32,1",0);vision_update(clock_ms);sent=vision_tx_count;tick(1600);
    CHECK(vision_material_state()==VISION_PICK_READY && vision_tx_count==sent && !arm_positions);
    j_begin();peer_reply("MSTATE,1,1,1,491,32,0",0);vision_update(clock_ms);tick(600);
    CHECK(vision_tx_count==6 && vision_material_state()==VISION_PICK_WAITING);
    peer_reply("MSTATE,0,1,1,491,32,2",0);vision_update(clock_ms);
    vision_tx_ready=0;tick(700);CHECK(vision_tx_count==6);
    peer_reply("MSTATE,1,1,1,491,32,1",0);vision_update(clock_ms);
    CHECK(vision_material_state()==VISION_PICK_READY);vision_tx_ready=1;tick(1000);
    CHECK(vision_tx_count==6);
    j_begin();peer_reply("MSTATE,0,0",0);vision_update(clock_ms);
    for(unsigned i=0;i<10;i++){peer_reply("MSTATE,0,0",0);tick(100);}
    tick(10);CHECK(vision_tx_count==18 && vision_material_state()==VISION_PICK_WAITING);
    queue_byte('!');firmware_step();tick(1000);
    CHECK(vision_tx_count==24 && vision_tx_bytes[21]=='R' && vision_material_state()==VISION_PICK_IDLE);
    puts("PASS J zero repeats every500ms, BUSY keeps waiting, done1 stops repeats");
    puts("PASS invalid terminal cannot arm retry; backpressure does not drop done1; continuous zeros do not postpone retries; emergency cancels repeats with R");
}

int main(void)
{
    repeat_zero_tests();state_protocol_tests();session_cancel_tests();
    printf("PASS H/J protocol: %u assertions; host only\n",assertions);return 0;
}
