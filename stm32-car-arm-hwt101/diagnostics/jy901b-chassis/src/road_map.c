#define ROAD_STRUCTURE_DEFINE
#include "road_map.h"
/* 以官方图为坐标基准，F/G、D/I是内移后的车辆中心行驶/停靠点。
 * A右上启停1、C右下启停2保持原位；X向右、Y向下，启动朝扫码区B。
 */
static const RoadPoint nodes[9] =
{
    {
        2250, 170
    },
    {
        2190, 1200
    },
    {
        2250, 2230
    },
    {
        1200, 2060 - CAR_MAP_BOTTOM_INSET_MM
    },
    {
        1200, 1200
    },
    {
        1200, 200 + CAR_MAP_TOP_INSET_MM + CAR_MAP_RAW_EXTRA_INSET_MM
    },
    {
        340, 200 + CAR_MAP_TOP_INSET_MM
    },
    {
        340 + CAR_MAP_STORAGE_INSET_MM, 1200
    },
    {
        340, 2060 - CAR_MAP_BOTTOM_INSET_MM
    }
};
static int road_entry(uint8_t from, uint8_t to)
{
    uint8_t i;
    for (i = 0; i < ROAD_DIRECTION_COUNT; i++)
        if (road_motion_table[i].from == from && road_motion_table[i].to == to)
            return i;
    return -1;
}

int road_point(uint8_t node, RoadPoint *out)
{
    if (node >= 9 || !out)
        return 0;
    * out = nodes[node];
    return 1;
}

int road_index(uint8_t from, uint8_t to)
{
    int entry = road_entry(from, to);
    return entry < 0 ? -1 : road_motion_table[entry].road;
}

int road_plan(uint8_t from, uint8_t to, RoadPlan *out)
{
    int entry = road_entry(from, to);
    if (!out || entry < 0)
        return 0;
    *out = road_motion_table[entry];
    return 1;
}

int road_motion_parameters(MotionParameters stored, RoadHeading heading, MotionParameters *out)
{
    RoadSegment field;
    BodySegment body;
    if (!out || (unsigned) stored.axis > BODY_LEFT)
        return 0;
    /* 旧结构表的横向正值编码图面-X；先还原场地位移，再换算真实车身左右。 */
    field.dx = stored.axis == BODY_LEFT ? (int16_t)-stored.mm : 0;
    field.dy = stored.axis == BODY_FORWARD ? stored.mm : 0;
    if (!road_body_segment(field, heading, &body))
        return 0;
    *out = stored;
    out->axis = body.axis;
    out->mm = body.mm;
    return 1;
}

int road_body_segment(RoadSegment segment, RoadHeading heading, BodySegment *out)
{
    int forward, left;
    if (!out || (unsigned) heading > HEAD_X_POS || (!segment.dx && !segment.dy) || (segment.dx
        && segment.dy))
        return 0;
    /* 图面Y向下：朝下时，车身左侧朝图面+X。 */
    switch (heading)
    {
    case HEAD_Y_POS:
        forward = segment.dy;
        left = segment.dx;
        break;
    case HEAD_X_NEG:
        forward = -segment.dx;
        left = segment.dy;
        break;
    case HEAD_Y_NEG:
        forward = -segment.dy;
        left = -segment.dx;
        break;
    case HEAD_X_POS:
        forward = segment.dx;
        left = -segment.dy;
        break;
    default:
        return 0;
    }
    out->axis = forward ? BODY_FORWARD : BODY_LEFT;
    out->mm = (int16_t)(forward ? forward : left);
    return 1;
}
