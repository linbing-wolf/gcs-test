#ifndef FULL_ROUTE_H
#define FULL_ROUTE_H
#include "road_map.h"
/* 无障碍、固定作业顺序、返回原启停区的最短距离路线。
 * 在电脑端校验最优性；板上只读取这些节点和既有路段结构体。
 */
#define FULL_ROUTE_NODES 17u
#define FULL_ROUTE_JOBS 8u
static const uint8_t full_route_nodes[2][FULL_ROUTE_NODES] =
{
    {
        NODE_A, NODE_B, NODE_E, NODE_F, NODE_E, NODE_D, NODE_E, NODE_H, NODE_E, NODE_F, NODE_E, NODE_D,
        NODE_E, NODE_H, NODE_E, NODE_B, NODE_A
    },
    {
        NODE_C, NODE_B, NODE_E, NODE_F, NODE_E, NODE_D, NODE_E, NODE_H, NODE_E, NODE_F, NODE_E, NODE_D,
        NODE_E, NODE_H, NODE_E, NODE_B, NODE_C
    }
};
static const uint8_t full_route_jobs[2][FULL_ROUTE_JOBS] =
{
    {
        NODE_B, NODE_F, NODE_D, NODE_H, NODE_F, NODE_D, NODE_H, NODE_A
    },
    {
        NODE_B, NODE_F, NODE_D, NODE_H, NODE_F, NODE_D, NODE_H, NODE_C
    }
};
#endif
