/* 同一夹具分别链接改动前后源码，只比较模型墙钟时间，不表示实车完成时间。 */
#define STATION_TEST_MAIN retained_station_suite
#include <verify_station_vision.c>
int main(void)
{
    unsigned plan;
    for(plan=0;plan<2u;plan++) {
        uint32_t start,deadline;unsigned offset=0,last_job=0,i,total=0;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=route_timing.last_ms;deadline=clock_ms+300000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        run_timing_poll();
        for(i=0;i<6u;i++)total+=route_timing.total[i];
        CHECK(total==clock_ms-start);
        printf("MODEL_TIME plan=%u total_ms=%u road_ms=%lu station_move_ms=%lu arm_ms=%lu vision_ms=%lu other_ms=%lu\n",
            plan,total,(unsigned long)route_timing.total[0],(unsigned long)route_timing.total[2],
            (unsigned long)route_timing.total[3],(unsigned long)route_timing.total[4],(unsigned long)route_timing.total[5]);
        stop_request();
    }
    return 0;
}
