#ifndef NINE_STRUCTURE_H
#define NINE_STRUCTURE_H
#include <stdint.h>
#include "car_config.h"
#define ROAD_COUNT 12
#define ROAD_DIRECTION_COUNT 24
#define ROAD_MAX_SEGMENTS 3
/* 由原图基准距离和内移量推导，去程/回程共用，防止单向改距。 */
#define ROAD_TOP_LEG_MM          (1000 - CAR_MAP_TOP_INSET_MM)
#define ROAD_BOTTOM_LEG_MM       (860 - CAR_MAP_BOTTOM_INSET_MM)
#define ROAD_RAW_LEG_MM          (ROAD_TOP_LEG_MM - CAR_MAP_RAW_EXTRA_INSET_MM)
#define ROAD_STORAGE_LEG_MM      (860 - CAR_MAP_STORAGE_INSET_MM)
#define ROAD_A_TOP_LINK_MM       (30 + CAR_MAP_TOP_INSET_MM + CAR_MAP_RAW_EXTRA_INSET_MM)
#define ROAD_C_BOTTOM_LINK_MM    (170 + CAR_MAP_BOTTOM_INSET_MM)
typedef enum
{
    NODE_A, NODE_B, NODE_C, NODE_D, NODE_E, NODE_F, NODE_G, NODE_H, NODE_I
} RoadNode;
typedef struct
{
    int16_t x, y;
} RoadPoint;
typedef struct
{
    int16_t dx, dy;
} RoadSegment;
/* 图面X向右、Y向下：+Y、-X、-Y、+X按图面顺时针递增。
 * runimu转换为左转为正的IMU角度时使用CAR_MAP_YAW_SIGN。
 */
typedef enum
{
    HEAD_Y_POS, HEAD_X_NEG, HEAD_Y_NEG, HEAD_X_POS
} RoadHeading;
typedef enum
{
    BODY_FORWARD, BODY_LEFT
} BodyAxis;
typedef struct
{
    BodyAxis axis;
    int16_t mm;
} BodySegment;
/* motion_start() 的运动参数：前/左方向、带符号毫米、rpm、驱动器加速度。
 * mm>0 前进/左移，mm<0 后退/右移；acceleration=0 沿用驱动器的0档含义。
 * Motion对象与当前时间由执行层传入，不作为固定路段参数。
 */
typedef struct
{
    BodyAxis axis;
    int16_t mm;
    uint16_t rpm;
    uint8_t acceleration;
} MotionParameters;
/* 一条有向道路对应一个结构体；segments按实际执行顺序填写。
 * 保留旧道路编码：BODY_FORWARD正值=图面+Y，BODY_LEFT正值=图面-X。
 * 该表的横向符号不是实际车身左右；必须经road_motion_parameters换算。
 */
typedef struct
{
    uint8_t road, from, to, count;
    uint16_t length;
    MotionParameters segments[ROAD_MAX_SEGMENTS];
} RoadPlan;
extern const RoadPlan road_motion_table[ROAD_DIRECTION_COUNT];
/* 只由road_map.c定义一次；其他源文件直接include本头文件即可。 */
#ifdef ROAD_STRUCTURE_DEFINE
const RoadPlan road_motion_table[ROAD_DIRECTION_COUNT] =
{
    /* A → B：道路 0，2 个运动段，理论总长 1090 mm。 */
    {
        0, NODE_A, NODE_B, 2, 1090,
        {
            /* 第 1 段：左移 60 mm。 */
            {BODY_LEFT, 60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：前进 1030 mm。 */
            {BODY_FORWARD, 1030, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* B → A：道路 0，2 个运动段，理论总长 1090 mm。 */
    {
        0, NODE_B, NODE_A, 2, 1090,
        {
            /* 第 1 段：后退 1030 mm。 */
            {BODY_FORWARD, -1030, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：右移 60 mm。 */
            {BODY_LEFT, -60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* B → C：道路 1，2 个运动段，理论总长 1090 mm。 */
    {
        1, NODE_B, NODE_C, 2, 1090,
        {
            /* 第 1 段：前进 1030 mm。 */
            {BODY_FORWARD, 1030, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：右移 60 mm。 */
            {BODY_LEFT, -60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* C → B：道路 1，2 个运动段，理论总长 1090 mm。 */
    {
        1, NODE_C, NODE_B, 2, 1090,
        {
            /* 第 1 段：左移 60 mm。 */
            {BODY_LEFT, 60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：后退 1030 mm。 */
            {BODY_FORWARD, -1030, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* C → D：道路 2，3 个运动段；启停点固定，连接内移后的下沿。 */
    {
        2, NODE_C, NODE_D, 3, 1050 + ROAD_C_BOTTOM_LINK_MM,
        {
            /* 第 1 段：左移 60 mm。 */
            {BODY_LEFT, 60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：从固定启停位置进入下沿中心线。 */
            {BODY_FORWARD, -ROAD_C_BOTTOM_LINK_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 3 段：左移 990 mm。 */
            {BODY_LEFT, 990, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* D → C：道路 2，3 个运动段，返回原启停点。 */
    {
        2, NODE_D, NODE_C, 3, 1050 + ROAD_C_BOTTOM_LINK_MM,
        {
            /* 第 1 段：右移 990 mm。 */
            {BODY_LEFT, -990, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：从下沿中心线返回固定启停位置。 */
            {BODY_FORWARD, ROAD_C_BOTTOM_LINK_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 3 段：右移 60 mm。 */
            {BODY_LEFT, -60, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* D → I：道路 3，1 个运动段，理论总长 860 mm。 */
    {
        3, NODE_D, NODE_I, 1, 860,
        {
            /* 第 1 段：左移 860 mm。 */
            {BODY_LEFT, 860, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* I → D：道路 3，1 个运动段，理论总长 860 mm。 */
    {
        3, NODE_I, NODE_D, 1, 860,
        {
            /* 第 1 段：右移 860 mm。 */
            {BODY_LEFT, -860, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* I → H：在G/I端连接H的新X中心线，避免经过旧H停靠点。 */
    {
        4, NODE_I, NODE_H, 1 + (CAR_MAP_STORAGE_INSET_MM > 0), ROAD_BOTTOM_LEG_MM + CAR_MAP_STORAGE_INSET_MM,
        {
#if CAR_MAP_STORAGE_INSET_MM > 0
            {BODY_LEFT, -CAR_MAP_STORAGE_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#endif
            {BODY_FORWARD, -ROAD_BOTTOM_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* H → I：在G/I端连接H的新X中心线，避免经过旧H停靠点。 */
    {
        4, NODE_H, NODE_I, 1 + (CAR_MAP_STORAGE_INSET_MM > 0), ROAD_BOTTOM_LEG_MM + CAR_MAP_STORAGE_INSET_MM,
        {
            {BODY_FORWARD, ROAD_BOTTOM_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#if CAR_MAP_STORAGE_INSET_MM > 0
            {BODY_LEFT, CAR_MAP_STORAGE_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
#endif
        }
    },

    /* H → G：在G/I端连接H的新X中心线，避免经过旧H停靠点。 */
    {
        5, NODE_H, NODE_G, 1 + (CAR_MAP_STORAGE_INSET_MM > 0), ROAD_TOP_LEG_MM + CAR_MAP_STORAGE_INSET_MM,
        {
            {BODY_FORWARD, -ROAD_TOP_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#if CAR_MAP_STORAGE_INSET_MM > 0
            {BODY_LEFT, CAR_MAP_STORAGE_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
#endif
        }
    },

    /* G → H：在G/I端连接H的新X中心线，避免经过旧H停靠点。 */
    {
        5, NODE_G, NODE_H, 1 + (CAR_MAP_STORAGE_INSET_MM > 0), ROAD_TOP_LEG_MM + CAR_MAP_STORAGE_INSET_MM,
        {
#if CAR_MAP_STORAGE_INSET_MM > 0
            {BODY_LEFT, -CAR_MAP_STORAGE_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#endif
            {BODY_FORWARD, ROAD_TOP_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* G↔F沿F的新停靠线横走；拐点在G侧，避免经过旧F碰撞位置。 */
    {
        6, NODE_G, NODE_F, 1 + (CAR_MAP_RAW_EXTRA_INSET_MM > 0), 860 + CAR_MAP_RAW_EXTRA_INSET_MM,
        {
#if CAR_MAP_RAW_EXTRA_INSET_MM > 0
            {BODY_FORWARD, CAR_MAP_RAW_EXTRA_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#endif
            {BODY_LEFT, -860, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    {
        6, NODE_F, NODE_G, 1 + (CAR_MAP_RAW_EXTRA_INSET_MM > 0), 860 + CAR_MAP_RAW_EXTRA_INSET_MM,
        {
            {BODY_LEFT, 860, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
#if CAR_MAP_RAW_EXTRA_INSET_MM > 0
            {BODY_FORWARD, -CAR_MAP_RAW_EXTRA_INSET_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
#endif
        }
    },

    /* F → A：道路 7，2 个运动段，返回固定启停点。 */
    {
        7, NODE_F, NODE_A, 2, 1050 + ROAD_A_TOP_LINK_MM,
        {
            /* 第 1 段：右移 1050 mm。 */
            {BODY_LEFT, -1050, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            {BODY_FORWARD, -ROAD_A_TOP_LINK_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* A → F：道路 7，2 个运动段，从固定启停点接入上沿。 */
    {
        7, NODE_A, NODE_F, 2, 1050 + ROAD_A_TOP_LINK_MM,
        {
            {BODY_FORWARD, ROAD_A_TOP_LINK_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION},
            /* 第 2 段：左移 1050 mm。 */
            {BODY_LEFT, 1050, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* F → E：道路 8，内移后的原料停靠点到中线。 */
    {
        8, NODE_F, NODE_E, 1, ROAD_RAW_LEG_MM,
        {
            {BODY_FORWARD, ROAD_RAW_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* E → F：道路 8，中线到内移后的原料停靠点。 */
    {
        8, NODE_E, NODE_F, 1, ROAD_RAW_LEG_MM,
        {
            {BODY_FORWARD, -ROAD_RAW_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* E → D：道路 9，中线到内移后的粗加工停靠点。 */
    {
        9, NODE_E, NODE_D, 1, ROAD_BOTTOM_LEG_MM,
        {
            {BODY_FORWARD, ROAD_BOTTOM_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* D → E：道路 9，粗加工停靠点到中线。 */
    {
        9, NODE_D, NODE_E, 1, ROAD_BOTTOM_LEG_MM,
        {
            {BODY_FORWARD, -ROAD_BOTTOM_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* H → E：道路 10，1 个运动段，理论总长 860 mm。 */
    {
        10, NODE_H, NODE_E, 1, ROAD_STORAGE_LEG_MM,
        {
            /* 第 1 段：右移 860 mm。 */
            {BODY_LEFT, -ROAD_STORAGE_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* E → H：道路 10，1 个运动段，理论总长 860 mm。 */
    {
        10, NODE_E, NODE_H, 1, ROAD_STORAGE_LEG_MM,
        {
            /* 第 1 段：左移 860 mm。 */
            {BODY_LEFT, ROAD_STORAGE_LEG_MM, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* E → B：道路 11，1 个运动段，理论总长 990 mm。 */
    {
        11, NODE_E, NODE_B, 1, 990,
        {
            /* 第 1 段：右移 990 mm。 */
            {BODY_LEFT, -990, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

    /* B → E：道路 11，1 个运动段，理论总长 990 mm。 */
    {
        11, NODE_B, NODE_E, 1, 990,
        {
            /* 第 1 段：左移 990 mm。 */
            {BODY_LEFT, 990, CAR_ROAD_DEFAULT_RPM, CAR_MOTION_ACCELERATION}
        }
    },

};
#endif
#endif
