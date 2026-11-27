/* 比赛入口联合模型：颜色顺序固定，J更新下一次固定抓点。 */
#define RAW_PLATE_TEST_MAIN inherited_raw_plate_tests
#include "verify_raw_plate.c"

static void changed_position_regression(void)
{
    uint32_t limit;unsigned grips=0;float previous=ARM_CLAW_OPEN_DEG;
    prepare_raw_plate_task(0,"task 213");
    peer_reply("HDISC2,0,0",0);limit=clock_ms+60000;
    while(grips<2 && clock_ms<limit) {
        float yaw,claw;plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
        if(claw==ARM_CLAW_CLOSE_DEG && previous!=claw) {
            CHECK(yaw==(grips ? ARM_RAW_PAST_YAW_DEG : ARM_RAW_NEXT_YAW_DEG));grips++;
        }
        previous=claw;
        if(vision_tx_count==12 && vision_material_state()==VISION_PICK_WAITING)
            peer_reply("MSTATE,1,1,2,136,44,1",0);
        if(vision_tx_count==18 && vision_material_state()==VISION_PICK_WAITING)
            peer_reply("MSTATE,1,2,1,491,32,1,3,310,315,1",0);
    }
    CHECK(clock_ms<limit && grips==2);stop_request();tick(20);
    puts("PASS production task213: J1 yellow left, J2 red right; task/cup order preserved");
}
static void await_j(void)
{
    uint32_t limit=clock_ms+60000;
    while(vision_tx_count<12u && clock_ms<limit)plant_step();
    CHECK(clock_ms<limit && vision_material_state()==VISION_PICK_WAITING);
}
static void start_target_j(void)
{
    prepare_raw_plate_task(0,"task 314");peer_reply("HDISC2,0,0",0);await_j();
}
static void region_integration_tests(void)
{
    const int xs[]={136,310,491,313,314,0,639};
    const int ys[]={44,315,32,179,179,180,479};
    const float yaw_expected[]={ARM_RAW_NEXT_YAW_DEG,ARM_RAW_CURRENT_YAW_DEG,ARM_RAW_PAST_YAW_DEG,
        ARM_RAW_NEXT_YAW_DEG,ARM_RAW_PAST_YAW_DEG,ARM_RAW_CURRENT_YAW_DEG,ARM_RAW_CURRENT_YAW_DEG};
    unsigned i;
    for(i=0;i<7u;i++) {
        char packet[80];float yaw,claw;uint32_t limit;
        start_target_j();snprintf(packet,sizeof(packet),"MSTATE,1,1,3,%d,%d,1",xs[i],ys[i]);peer_reply(packet,0);
        limit=clock_ms+30000;
        do {plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);}while(claw!=ARM_CLAW_CLOSE_DEG && clock_ms<limit);
        CHECK(clock_ms<limit && yaw==yaw_expected[i]);stop_request();tick(20);
    }
    {const char *bad[]={"MSTATE,1,1,4,491,32,1","MSTATE,1,1,3,-1,32,1","MSTATE,1,1,3,640,32,1"};
     for(i=0;i<3u;i++) {
        unsigned positions;uint32_t limit;float yaw,claw;
        start_target_j();positions=arm_positions;peer_reply(bad[i],0);tick(1500);
        CHECK(station_active() && arm_positions==positions && vision_tx_count==12);
        send_line("raw observe");tick(20);CHECK(vision_tx_count==18 && vision_tx_bytes[15]=='J');
        send_line("raw observe");CHECK(strstr(output,"rejected") && vision_tx_count==18);
        peer_reply("MSTATE,1,1,3,491,32,1",0);limit=clock_ms+30000;
        do {plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);}while(claw!=ARM_CLAW_CLOSE_DEG && clock_ms<limit);
        CHECK(clock_ms<limit && yaw==ARM_RAW_PAST_YAW_DEG);stop_request();tick(20);
     }}
    for(i=0;i<2u;i++) {
        unsigned positions;
        CHECK(station_raw_set_region(0,400,500,0,100));
        CHECK(station_raw_set_region(1,200,350,250,400));
        CHECK(station_raw_set_region(2,i ? 450 : 0,i ? 510 : 180,0,100));
        start_target_j();positions=arm_positions;
        peer_reply(i ? "MSTATE,1,1,3,491,32,1" : "MSTATE,1,1,3,250,150,1",0);tick(1500);
        CHECK(station_active() && arm_positions==positions && vision_tx_count==12);
        send_line("raw region left 0 180 0 100");CHECK(strstr(output,"before changing grip regions"));
        stop_request();tick(20);
    }
    CHECK(station_raw_set_region(0,314,639,0,179));
    CHECK(station_raw_set_region(1,0,639,180,479));
    CHECK(station_raw_set_region(2,0,313,0,179));
    send_line("raw region left 180 0 0 179");CHECK(strstr(output,"rejected"));
    send_line("raw region left 0 313 0 179");send_line("raw region status");
    CHECK(strstr(output,"left xmin=0 xmax=313 ymin=0 ymax=179"));
    send_line("raw observe");CHECK(strstr(output,"rejected"));
    puts("PASS production regions: measured points/boundaries, missing/invalid/gap/overlap wait; manual observe/CLI ownership and fixed poses");
}
int main(void)
{
    changed_position_regression();
    region_integration_tests();
    return 0;
}
