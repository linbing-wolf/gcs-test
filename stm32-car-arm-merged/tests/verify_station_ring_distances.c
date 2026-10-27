/* 检查实际工位编排的每一段环距、离站前缀，并运行四个工位批次的执行模型。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"
#include "../src/station_job.c"

static void ring_program_tests(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    unsigned storage,batch,order;
    for(storage=0;storage<2;storage++)for(batch=0;batch<2;batch++)for(order=0;order<6;order++) {
        char task[48];int positions[3];unsigned i,ring=2,next=0,count=0,moves=0;
        unsigned targets[7];
        snprintf(task,sizeof(task),"task 134+%s+314+%s",orders[order],orders[order]);
        if(storage)prepare_storage(batch,task);else prepare_rough(batch,task);
        CHECK(station_start(storage ? NODE_H : NODE_D,run.job,clock_ms)==1);
        positions[0]=storage ? -STN_STORAGE_INDEX_12_MM : -STN_ROUGH_INDEX_12_MM;
        positions[1]=0;positions[2]=storage ? STN_STORAGE_INDEX_23_MM : STN_ROUGH_INDEX_23_MM;
        for(i=0;i<(storage ? 3u : 6u);i++) {
            unsigned slot=storage && batch && i<2u ? 1u-i : i%3u;
            targets[count++]=(unsigned)(orders[order][slot]-'0');
        }
        targets[count++]=2;
        for(i=0;i<stn.count;i++) {
            StationStep *step=&program[i];
            if(step->kind!=SS_INDEX_BEGIN && step->kind!=SS_CHASSIS && step->kind!=SS_DEPART_BEGIN)continue;
            if(step->kind!=SS_DEPART_BEGIN)while(next<count && targets[next]==ring)next++;
            CHECK(next<count);
            {unsigned target=targets[next++];
             int expected=(positions[target-1u]-positions[ring-1u])*(storage ? STN_STORAGE_INDEX_SIGN : STN_ROUGH_INDEX_SIGN);
             CHECK(step->value==expected);
             CHECK(step->kind==SS_DEPART_BEGIN ? target==2 : step->ring==target);
             ring=target;moves++;}
        }
        while(next<count && targets[next]==ring)next++;
        CHECK(next==count && ring==2 && moves>=2);
    }
    printf("PASS ring programs: rough12=%d rough23=%d storage12=%d storage23=%d; 24 permutations/batches; parallel=%d through=%d\n",
        STN_ROUGH_INDEX_12_MM,STN_ROUGH_INDEX_23_MM,STN_STORAGE_INDEX_12_MM,STN_STORAGE_INDEX_23_MM,
        CAR_ARM_TRANSLATION_OVERLAP_ENABLED,STN_ROUGH_THROUGH_ENABLED);
}

static void ring_execution_tests(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    unsigned storage,batch,order;
    for(storage=0;storage<2;storage++)for(batch=0;batch<2;batch++)for(order=0;order<6;order++) {
        char task[48];unsigned offset=0,placed=0,gripped=0;float previous=ARM_CLAW_OPEN_DEG;
        uint32_t deadline;uint8_t job;RoadPoint point;
        snprintf(task,sizeof(task),"task 134+%s+314+%s",orders[order],orders[order]);
        if(storage)prepare_storage(batch,task);else prepare_rough(batch,task);
        CHECK(road_point(storage ? NODE_H : NODE_D,&point));job=run.job;deadline=clock_ms+180000u;
        while((run.job==job || run.departure_overlap) && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if(claw!=previous && slide==(storage ? STN_STORAGE_SLIDE_MM : STN_ROUGH_SLIDE_MM)
                && yaw==(storage ? STN_STORAGE_YAW_DEG : STN_ROUGH_YAW_DEG)) {
                unsigned target;double actual;
                if(claw==ARM_CLAW_OPEN_DEG) {
                    unsigned slot=storage && batch && placed<2u ? 1u-placed : placed;
                    CHECK(placed<3);target=(unsigned)(orders[order][slot]-'0');placed++;
                    CHECK(lift==(storage ? STN_STORAGE_LIFT_MM+(batch ? STN_STORAGE_LAYER_LIFT_MM : 0) : STN_ROUGH_PLACE_LIFT_MM));
                } else if(!storage && claw==ARM_CLAW_CLOSE_DEG) {
                    CHECK(gripped<3);target=(unsigned)(orders[order][gripped]-'0');gripped++;
                    CHECK(lift==STN_ROUGH_LIFT_MM);
                } else {previous=claw;continue;}
                actual=storage ? field_y-point.y : field_x-point.x;
                CHECK(fabs(actual-test_station_offset_mm(storage,target))<15.0);
            }
            previous=claw;
        }
        CHECK(clock_ms<deadline && placed==3 && gripped==(storage ? 0u : 3u));
        CHECK(run.job==job+1u && !arm_failed());
        CHECK(peer_stack_count==(storage && batch ? 3u : 0u));
    }
    puts("PASS ring execution: both batches/areas, all six orders, actual modeled ground positions and departure completion");
}

int main(void)
{
    ring_program_tests();ring_execution_tests();
    printf("PASS independent ring distances: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
