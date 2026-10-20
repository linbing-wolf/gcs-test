/* 非生产比例下核验真实UART帧到编码器目标，复用现有工位模拟器。 */
#define STATION_TEST_MAIN station_full_suite_main
#include "verify_station_vision.c"

int main(void)
{
    unsigned area,direction;
    CHECK(STN_VISION_MM_PER_PX_X1000==500);
    for(area=0;area<2;area++)for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned positions;
        if(area)prepare_storage(0,"task 134+231+314+123");
        else prepare_rough(0,"task 134+231+314+123");
        deadline=clock_ms+30000;
        while((vision_align_state()!=VISION_PICK_WAITING || vision_tx_count<6) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline);tick(150);positions=arm_positions;
        peer_reply(direction ? "MOVE,-40" : "MOVE,+40",0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.remaining_mm==20);
        CHECK(run.travel_sign==(direction ? 1 : -1));
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
        peer_reply(direction ? "STACK,280,240" : "STACK,360,240",0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.remaining_mm==20);
        CHECK(run.travel_sign==(direction ? 1 : -1));
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            CHECK(arm_positions==positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && vision_tx_count==14);
        stop_request();
    }
    for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned offset=0,positions;
        prepare_storage(1,"task 134+231+314+123");peer_stack_hold=1;deadline=clock_ms+30000;
        while(!peer_stack_count && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(clock_ms<deadline && peer_stack_count==1);positions=arm_positions;
        peer_reply(direction ? "STACK,320,200" : "STACK,320,280",0);tick(1);
        while(!run.station_shift && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.state!=RUN_FAULT && run.station_shift && run.travel_axis==BODY_LEFT && run.remaining_mm==20);
        CHECK(run.travel_sign==(direction ? 1 : -1));
        while(station_chassis_busy() && run.state!=RUN_FAULT && clock_ms<deadline) {
            CHECK(arm_positions==positions);plant_step();
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && vision_tx_count==14);stop_request();
    }
    printf("PASS configurable pixel gain: %u assertions; test-only0.5mm/px maps signed40px to20mm for D/E MOVE and both F STACK axes, not production calibration\n",assertions);
    return 0;
}
