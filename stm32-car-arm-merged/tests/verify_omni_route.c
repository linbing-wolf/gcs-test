/* 复用真实入口/车辆模型，以独立场地坐标核对道路方向；不连接硬件。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void omni_projection_tests(void)
{
    static const int forward_x[4]={0,-1,0,1},forward_y[4]={1,0,-1,0};
    static const int left_x[4]={1,0,-1,0},left_y[4]={0,1,0,-1};
    unsigned direction,part,heading,history,plan,job,total=0;
    if(!CAR_RUN_OMNI_ENABLED)return;
    for(direction=0;direction<ROAD_DIRECTION_COUNT;direction++) {
        const RoadPlan *road=&road_motion_table[direction];
        for(part=0;part<road->count;part++)for(heading=0;heading<4;heading++)
        for(history=0;history<3;history++)for(plan=0;plan<2;plan++)for(job=0;job<8;job++) {
            RunLeg leg;
            MotionParameters stored=road->segments[part];
            int want_x=stored.axis==BODY_LEFT ? -stored.mm : 0;
            int want_y=stored.axis==BODY_FORWARD ? stored.mm : 0;
            int dx,dy,mm;
            uint16_t completed=history==0 ? 0 : history==1 ? (uint16_t)(1u<<road->road) : 0xffffu;
            memset(&run,0,sizeof(run));run.plan=(uint8_t)plan;run.job=(uint8_t)job;
            CHECK(run_resolve_leg(road,(uint8_t)part,job ? 13 : 0,(RoadHeading)heading,completed,&leg));
            CHECK(leg.heading==(RoadHeading)heading && leg.mm==abs(stored.mm));
            mm=leg.mm*leg.sign;
            dx=mm*(leg.axis==BODY_FORWARD ? forward_x[heading] : left_x[heading]);
            dy=mm*(leg.axis==BODY_FORWARD ? forward_y[heading] : left_y[heading]);
            CHECK(dx==want_x && dy==want_y);total++;
        }
    }
    /* 首次正后方也直接后退，首次侧方也直接横移；不伪造已走道路。 */
    {
        RoadPlan road={0};RunLeg leg;
        memset(&run,0,sizeof(run));road.count=1;road.segments[0].axis=BODY_FORWARD;
        road.segments[0].mm=-150;run.heading=HEAD_Y_POS;
        CHECK(run_resolve_leg(&road,0,0,run.heading,0,&leg));
        CHECK(leg.heading==HEAD_Y_POS && leg.axis==BODY_FORWARD && leg.sign==-1);
        road.segments[0].axis=BODY_LEFT;road.segments[0].mm=150;
        CHECK(run_resolve_leg(&road,0,0,run.heading,0,&leg));
        CHECK(leg.heading==HEAD_Y_POS && leg.axis==BODY_LEFT && leg.sign==-1);
        CHECK(!run.completed_roads);
    }
    printf("PASS omni projection: %u real directed-road/part/heading/history/plan/job cases; independent drawing displacement, heading retained, first backward/lateral without history\n",total);
}

int main(void)
{
    omni_projection_tests();map_reverse_and_postcheck_tests();continuous_route_tests();
    automatic_route_tests();departure_overlap_tests();translation_overlap_tests();
    printf("PASS omni route: mode=%u assertions=%u; full A/C geometry, scanner/work headings, arm/vision and departure; host simulation only\n",
           (unsigned)CAR_RUN_OMNI_ENABLED,assertions);
    return 0;
}
