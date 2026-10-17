/* 电脑端：从实际编译的节点/动作导出地图，并验证24个方向的几何闭合。 */
#include <stdio.h>
#include <stdlib.h>
#include "road_map.h"
#define REQUIRE(condition) do {if(!(condition)){fprintf(stderr,"Map geometry failed at line %d\n",__LINE__);return 1;}} while(0)
int main(void)
{
    static const RoadPoint original[9]={{2250,170},{2190,1200},{2250,2230},{1200,2060},
                                       {1200,1200},{1200,200},{340,200},{340,1200},{340,2060}};
    RoadPoint points[9];
    unsigned road_counts[ROAD_COUNT]={0};
    unsigned i,j;
    for(i=0;i<9;i++) {
        RoadPoint expected=original[i];
        if(i==NODE_F || i==NODE_G)expected.y+=CAR_MAP_TOP_INSET_MM;
        if(i==NODE_F)expected.y+=CAR_MAP_RAW_EXTRA_INSET_MM;
        if(i==NODE_H)expected.x+=CAR_MAP_STORAGE_INSET_MM;
        if(i==NODE_D || i==NODE_I)expected.y-=CAR_MAP_BOTTOM_INSET_MM;
        REQUIRE(road_point((uint8_t)i,&points[i]));
        REQUIRE(points[i].x==expected.x && points[i].y==expected.y);
    }
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        const RoadPlan *p=&road_motion_table[i];
        RoadPlan reverse;
        int x=points[p->from].x,y=points[p->from].y,length=0;
        REQUIRE(p->road<ROAD_COUNT && p->count>0 && p->count<=ROAD_MAX_SEGMENTS);
        REQUIRE(road_plan(p->to,p->from,&reverse) && reverse.road==p->road && reverse.count==p->count);
        for(j=0;j<p->count;j++) {
            MotionParameters step=p->segments[j],back=reverse.segments[p->count-1u-j];
            REQUIRE(step.mm && (step.axis==BODY_FORWARD || step.axis==BODY_LEFT));
            REQUIRE(back.axis==step.axis && back.mm==-step.mm);
            if(step.axis==BODY_FORWARD)y+=step.mm;else x-=step.mm;
            length+=abs(step.mm);
        }
        REQUIRE(x==points[p->to].x && y==points[p->to].y && length==p->length);
        REQUIRE(length==abs(points[p->to].x-points[p->from].x)+abs(points[p->to].y-points[p->from].y));
        if(p->from==NODE_G && p->to==NODE_F && CAR_MAP_RAW_EXTRA_INSET_MM) {
            REQUIRE(p->count==2 && p->segments[0].axis==BODY_FORWARD);
            REQUIRE(p->segments[0].mm==CAR_MAP_RAW_EXTRA_INSET_MM);
        }
        if(p->to==NODE_H && (p->from==NODE_G || p->from==NODE_I) && CAR_MAP_STORAGE_INSET_MM) {
            REQUIRE(p->count==2 && p->segments[0].axis==BODY_LEFT);
            REQUIRE(p->segments[0].mm==-CAR_MAP_STORAGE_INSET_MM);
        }
        REQUIRE(reverse.length==p->length);road_counts[p->road]++;
    }
    for(i=0;i<ROAD_COUNT;i++)REQUIRE(road_counts[i]==2);
    printf("{\"geometryValidationPassed\":true,\"parameters\":{\"topInsetMm\":%d,\"bottomInsetMm\":%d,\"rawExtraInsetMm\":%d,\"storageInsetMm\":%d},\"nodes\":[",CAR_MAP_TOP_INSET_MM,CAR_MAP_BOTTOM_INSET_MM,CAR_MAP_RAW_EXTRA_INSET_MM,CAR_MAP_STORAGE_INSET_MM);
    for(i=0;i<9;i++)printf("%s[%d,%d]",i?",":"",points[i].x,points[i].y);
    printf("],\"roads\":[");
    for(i=0;i<ROAD_COUNT;i++) {
        for(j=0;j<ROAD_DIRECTION_COUNT;j++)if(road_motion_table[j].road==i)break;
        printf("%s[%u,%u,%u]",i?",":"",road_motion_table[j].from,road_motion_table[j].to,road_motion_table[j].length);
    }
    puts("]}");return 0;
}
