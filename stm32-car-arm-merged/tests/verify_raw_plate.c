/* 原料H与真实工位/底盘闭环联合模拟；不访问相机或电机。 */
#define STATION_TEST_MAIN retained_raw_station_suite
#include "verify_station_vision.c"

static void prepare_raw_plate(unsigned batch)
{
    RoadPoint point;uint32_t deadline;
    prepare_arm_auto(0);send_line("task 134+123+314+231");
    CHECK(road_point(NODE_F,&point));
    field_x=planned_x=point.x;field_y=planned_y=point.y;
    initial_field_heading=HEAD_X_NEG;physical_yaw=origin_yaw=0;sample_yaw(0);
    run.imu_mode=run.station_enabled=run.station_phase=1;run.state=RUN_STATION;
    run.job=checked_job=(uint8_t)(batch ? 4 : 1);run.edge=(uint8_t)(batch ? 9 : 3);
    run.heading=run.target_heading=HEAD_X_NEG;run.yaw_zero_cdeg=9000;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
    CHECK(station_start(NODE_F,run.job,clock_ms)==1);deadline=clock_ms+5000;
    while(vision_tx_count<6 && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
    CHECK(clock_ms<deadline && vision_tx_count==6 && vision_tx_bytes[2]==1 && vision_tx_bytes[3]=='H');
    CHECK(vision_tx_bytes[4]==peer_crc(vision_tx_bytes+2,2) && vision_align_state()==VISION_PICK_WAITING);
    CHECK(station_chassis_pose_safe() && !arm_positions && !run_active);
}

static void plate_motion_tests(void)
{
    unsigned batch,variant;
    for(batch=0;batch<2;batch++)for(variant=0;variant<6;variant++) {
        uint32_t deadline;double x,y;int expected_forward=0,expected_left=0;
        unsigned saw_moving=0,saw_concurrent=0,paused=0;
        prepare_raw_plate(batch);x=field_x;y=field_y;
        if(variant==0)peer_reply("MOVEXY,0,0",0);
        else if(variant==2) {
            peer_reply("MOVEXY,-18,+11",0);expected_forward=18;expected_left=11;
        } else if(variant==3) {
            peer_reply("MOVEXY,+18,-11",0);peer_reply("MOVE,-7",0);peer_reply("MOVEY,+5",0);
            expected_forward=-11;expected_left=-6;
        } else {
            peer_reply("MOVEXY,+18,-11",0);expected_forward=-18;expected_left=-11;
        }
        /* 终包中的动作前坐标/残差不生成第二次修正。 */
        peer_reply("DISC,331,250,20,1",0);tick(1);
        CHECK(vision_align_state()==VISION_PICK_READY);
        peer_reply("MOVEXY,99,99",0); /* 终包之后不再接收运动。 */
        deadline=clock_ms+30000;
        while(vision_tx_count<14 && run.state!=RUN_FAULT && clock_ms<deadline) {
            if(station_chassis_busy())CHECK(vision_tx_count==6);
            CHECK(!arm_positions && !arm_busy());
            if(run.station_shift && run_active) {
                int slide,lift;float yaw;
                arm_pose_now(&slide,&lift,&yaw,NULL);
                CHECK(slide==0 && lift==135 && yaw==ARM_PICK_YAW_DEG);
                saw_moving=1;
                if(run.forward_cmd_rpm && run.left_cmd_rpm)saw_concurrent=1;
                if(variant==4 && !paused && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>=3) {
                    missing_feedback_motor=4;tick(1800);paused=1;
                    CHECK(station_active() && !arm_failed() && vision_tx_count==6 && !arm_positions);
                    missing_feedback_motor=0;
                }
                if(variant==5 && !paused && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>=3) {
                    frozen_imu=1;heading_sensor_invalidate(&heading_sensor);tick(1800);paused=1;
                    CHECK(station_active() && !arm_failed() && vision_tx_count==6 && !arm_positions);
                    frozen_imu=0;
                }
            }
            plant_step();
        }
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !station_chassis_busy());
        CHECK(vision_tx_count==14 && !memcmp(vision_tx_bytes+9,batch ? "C,3" : "C,1",3));
        CHECK(vision_pick_state()==VISION_PICK_WAITING && !arm_positions);
        CHECK(fabs(field_x-x+expected_forward)<3.0 && fabs(field_y-y-expected_left)<3.0);
        CHECK(variant==0 ? !saw_moving : saw_moving && saw_concurrent);
        CHECK(variant<4 || paused);
        /* 首块PICK之后才开始抓取；剩余两块不再请求H。 */
        {
            unsigned offset=6,replies=0;deadline=clock_ms+140000;
            while(station_active() && run.job==(batch ? 4u : 1u) && run.state!=RUN_FAULT && clock_ms<deadline) {
                replies+=serve_peer(&offset);plant_step();
            }
            CHECK(clock_ms<deadline && run.state!=RUN_FAULT && replies==3);
            CHECK(vision_tx_count==30 && peer_area_count[batch ? 4 : 1]==0);
        }
        stop_request();
    }
    puts("PASS raw H: both batches; zero, both signed vectors, mixed legacy FIFO, early DISC, independent displacement, wheel/IMU loss and recovery; C/grip wait for completion; exactly H,C,C,C per visit");
}

static void plate_wait_and_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<5;variant++) {
        uint32_t deadline;
        prepare_raw_plate(variant&1u);
        if(variant==0) {
            peer_reply("BUSY",0);peer_reply("NONE,H",0);peer_reply("DISC,311,267,0,1",1);
            peer_reply("DISC,311,267,0,0",0);peer_reply("PICK,1,311,267",0);
            tick(61000);CHECK(station_active() && !arm_failed() && !arm_positions && vision_tx_count==6);
            peer_reply("DISC,311,267,0,1",0);tick(50);
            CHECK(vision_tx_count==14 && vision_pick_state()==VISION_PICK_WAITING);
        } else {
            if(variant>=3) {
                peer_reply("MOVEXY,+40,-30",0);peer_reply("DISC,351,237,40,1",0);
                deadline=clock_ms+10000;
                while((!run_active || run.velocity_phase!=VELOCITY_FORWARD || run.progress_mm<3) && clock_ms<deadline)plant_step();
                CHECK(clock_ms<deadline && run.station_shift && run_active);
            }
            if(variant==2 || variant==4) {motor_flags[2]=11;tick(700);}
            else {send_line("!");tick(10);}
            CHECK(!run_active && !station_active() && arm_failed());
            CHECK(!vision_motion_peek_xy(NULL,NULL,NULL) && vision_tx_count==6 && !arm_positions);
            peer_reply("DISC,311,267,0,1",0);peer_reply("MOVEXY,10,10",0);tick(200);
            CHECK(!run_active && !arm_positions && vision_tx_count==6);
        }
        stop_request();
    }
    puts("PASS raw H waits/cancel: missing target, bad CRC, unsuccessful terminal and >60s retain request; late valid DISC resumes; emergency and driver fault cancel waiting/moving without C/grip");
}

int main(void)
{
    plate_motion_tests();plate_wait_and_cancel_tests();
    automatic_route_tests();raw_last_return_tests();
    printf("PASS raw plate integration: %u assertions; host only\n",assertions);
    return 0;
}
