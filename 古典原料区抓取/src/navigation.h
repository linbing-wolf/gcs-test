#ifndef NINE_NAVIGATION_H
#define NINE_NAVIGATION_H
#include "road_map.h"
typedef enum
{
    NAV_IDLE,         /* 等待 go 命令。 */
    NAV_PLAN,         /* 下一轮主循环查表，选择下一条道路。 */
    NAV_MOVING,       /* 道路中的某一运动段正在执行。 */
    NAV_WAIT_NODE,    /* 道路各段已确认，等待 node 确认实际到达。 */
    NAV_WAIT_RETURN,  /* 道路已封禁，等待实际回到道路起点的确认。 */
    NAV_ARRIVED,      /* 人工导航目标已确认到达。 */
    NAV_STOPPED,      /* 已请求停车。 */
    NAV_FAULT         /* 无路/配置/提交失败的未解决状态，不自动发停车命令。 */
} NavState;
typedef enum
{
    NAV_OK, NAV_NO_ROUTE, NAV_BAD_CONFIG, NAV_MOTION_FAILED
} NavError;
typedef enum
{
    NAV_MOTION_BUSY, NAV_MOTION_DONE, NAV_MOTION_FAULT
} NavMotionEvent;
typedef struct
{
    int(*start)(void *user, MotionParameters parameters, uint32_t now_ms);
    NavMotionEvent(*poll)(void *user, uint32_t now_ms);
    void(*stop)(void *user);
    void *user;
} NavMotionPort;
typedef struct
{
    NavMotionPort motion;
    NavState state;
    NavError error;
    RoadHeading heading;
    uint8_t current;             /* 最近确认的真实节点。 */
    uint8_t target;              /* go 命令指定的最终目标。 */
    uint8_t next;                /* 当前道路的终点。 */
    uint8_t position_confirmed;  /* 不能由估时自动置为 1。 */
    uint8_t segment_index;       /* 当前道路的段下标。 */
    uint16_t blocked;            /* 12 条道路的封禁位图。 */
    uint16_t passed;             /* 人工确认完整通过后才记录的道路位图。 */
    uint16_t rpm;                /* 本次导航速度覆盖，0 为使用道路表速度。 */
    uint16_t progress_mm;        /* 人工/外部报告的段内实测进度。 */
    uint8_t progress_valid;
    RoadPlan road;               /* 从 structure.h 取出的当前道路。 */
} Navigation;
/* initial_node 必须由外部实际定位/人工摆放确认。 */
int nav_init(Navigation *navigation, uint8_t initial_node, RoadHeading heading, const NavMotionPort *port);
/* rpm=0使用structure.h各段的速度；20..300为本次任务的人工速度覆盖。 */
int nav_start(Navigation *navigation, uint8_t target, uint16_t rpm);
void nav_update(Navigation *navigation, uint32_t now_ms);
void nav_stop(Navigation *navigation);
int nav_confirm_node(Navigation *navigation, uint8_t actual_node);
int nav_set_blocked(Navigation *navigation, uint8_t road, int blocked);
int nav_obstacle(Navigation *navigation);
int nav_confirm_return(Navigation *navigation, uint8_t actual_node);
int nav_confirm_pose(Navigation *navigation, uint8_t actual_node);
int nav_set_heading(Navigation *navigation, RoadHeading heading);
/* 只记录实测进度；不从等待时间推算位置，不据此自动标记到达。 */
int nav_report_progress(Navigation *navigation, uint16_t actual_mm);
#endif
