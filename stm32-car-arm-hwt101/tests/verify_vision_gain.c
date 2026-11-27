/* 非生产比例下核验真实UART帧到编码器目标，复用现有工位模拟器。 */
#define STATION_TEST_MAIN station_full_suite_main
#include "verify_station_vision.c"

int main(void)
{
    unsigned area,direction,axis;
    CHECK(STN_VISION_MM_PER_PX_X1000==500);
    for(area=0;area<2;area++)for(direction=0;direction<2;direction++)for(axis=0;axis<2;axis++) {
        uint32_t deadline;unsigned positions;
        if(area)prepare_storage(0,"task 134+231+314+123");
        else prepare_rough(0,"task 134+231+314+123");
        deadline=clock_ms+30000;
        while((vision_align_state()!=VISION_PICK_WAITING || vision_tx_count<6) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline);tick(150);positions=arm_positions;
        peer_reply(axis ? (direction ? "MOVEY,-40" : "MOVEY,+40") : (direction ? "MOVE,-40" : "MOVE,+40"),0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.remaining_mm==20);
        CHECK(run.travel_axis==(axis ? BODY_LEFT : BODY_FORWARD) && run.travel_sign==(axis ? (direction ? -1 : 1) : (direction ? 1 : -1)));
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            CHECK(arm_positions==positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && vision_tx_count==6);
        stop_request();
    }
    for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned offset=0,positions;
        prepare_storage(1,"task 134+231+314+123");peer_stack_hold=1;deadline=clock_ms+30000;
        while(!peer_stack_count && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(clock_ms<deadline && peer_stack_count==1);positions=arm_positions;
        peer_reply(direction ? "MOVE,-40" : "MOVE,+40",0);
        peer_reply("STACK,600,100,620,1",0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.remaining_mm==20);
        CHECK(run.travel_sign==(direction ? 1 : -1));
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            check_correction_arm(positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && vision_tx_count==8);
        stop_request();
    }
    for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned offset=0,positions;
        prepare_storage(1,"task 134+231+314+123");peer_stack_hold=1;deadline=clock_ms+30000;
        while(!peer_stack_count && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(clock_ms<deadline && peer_stack_count==1);positions=arm_positions;
        peer_reply(direction ? "MOVEY,-40" : "MOVEY,+40",0);
        peer_reply("STACK,600,100,620,1",0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.travel_axis==BODY_LEFT && run.remaining_mm==20);
        CHECK(run.travel_sign==(direction ? -1 : 1));
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            check_correction_arm(positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && vision_tx_count==8);stop_request();
    }
    for(area=0;area<3;area++) {
        double x,y;uint32_t deadline;unsigned concurrent=0;
        prepare_vision_debug();send_line(area==2 ? "vision debug F 2" : area==1 ? "vision debug E" : "vision debug D");
        wait_debug_request(area==2);x=field_x;y=field_y;
        peer_reply("MOVEXY,+40,-30",0);peer_reply(area==2 ? "STACK,700,-50,620,1" : "DISC,700,-50,620,1",0);
        deadline=clock_ms+30000;
        while(run.vision_debug && run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();
            if(run.vector_move) {
                CHECK(run.vector_target_mm[0]== -20 && run.vector_target_mm[1]== -15);
                if(abs(run.forward_cmd_rpm)>1 && abs(run.wheel_rpm[0]-run.wheel_rpm[1])>2)concurrent++;
            }
        }
        CHECK(clock_ms<deadline && run.state==RUN_IDLE && concurrent);
        CHECK(fabs(field_x-x+15)<3.0 && fabs(field_y-y+20)<3.0);
    }
    puts("PASS MOVEXY half gain: signed40/-30 pixels converted once to20/-15mm for D/E/F simultaneous endpoints");
    printf("PASS configurable pixel gain: %u assertions; test-only0.5mm/px maps signed40px to20mm for D/E MOVE/MOVEY and F MOVE/MOVEY (STACK report alone never moves), not production calibration\n",assertions);
    return 0;
}
