#ifndef NINE_ROAD_MAP_H
#define NINE_ROAD_MAP_H
#include "structure.h"
int road_point(uint8_t node, RoadPoint *out);
RoadPoint road_scan_waypoint(void); /* 首次扫码的运行点，不占地图节点。 */
int road_index(uint8_t from, uint8_t to);
int road_plan(uint8_t from, uint8_t to, RoadPlan *out);
int road_body_segment(RoadSegment segment, RoadHeading heading, BodySegment *out);
int road_motion_parameters(MotionParameters stored, RoadHeading heading, MotionParameters *out);
#endif
