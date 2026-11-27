#include <stdio.h>
#include <stdlib.h>
#include "road_map.h"
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);return 1;}} while(0)
int main(void)
{
    static const RoadPoint expected[9]={{2190+CAR_START_INSET_MM,170},{2190,1200-CAR_MAP_SCAN_TOWARD_A_MM},{2190+CAR_START_INSET_MM,2230},{1200,2060},
                                      {1200,1200},{1200,290},{340,290},{420,1200},{340,2060}};
    RoadPoint p;RoadPlan road;unsigned i,j;
    for(i=0;i<9;i++){CHECK(road_point((uint8_t)i,&p));CHECK(p.x==expected[i].x && p.y==expected[i].y);}
    CHECK(road_plan(NODE_E,NODE_F,&road) && road.length==910 && road.segments[0].mm==-910);
    CHECK(road_plan(NODE_F,NODE_E,&road) && road.length==910 && road.segments[0].mm==910);
    CHECK(road_plan(NODE_G,NODE_F,&road) && road.length==860 && road.count==1);
    CHECK(road_plan(NODE_A,NODE_F,&road) && road.length==1180);
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        int x=expected[road_motion_table[i].from].x,y=expected[road_motion_table[i].from].y,length=0;
        road=road_motion_table[i];
        for(j=0;j<road.count;j++) {
            if(road.segments[j].axis==BODY_FORWARD)y+=road.segments[j].mm;else x-=road.segments[j].mm;
            length+=abs(road.segments[j].mm);
        }
        CHECK(x==expected[road.to].x && y==expected[road.to].y && length==road.length);
    }
    puts("PASS raw map: F1200,290; EF910/GF860/AF1180; all24 directed roads close");
    return 0;
}
