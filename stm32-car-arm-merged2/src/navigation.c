#include "navigation.h"
#include "route_table.h"
#include <string.h>
static int stationary(NavState state)
{
    return state == NAV_IDLE || state == NAV_ARRIVED || state == NAV_STOPPED || state == NAV_FAULT;
}

static void fault(Navigation *navigation, NavError error)
{
    navigation->motion.stop(navigation->motion.user);
    navigation->error = error;
    navigation->state = NAV_FAULT;
}

static int start_segment(Navigation *navigation, uint32_t now_ms)
{
    MotionParameters parameters;
    navigation->progress_mm = 0;
    navigation->progress_valid = 0;
    if (!road_motion_parameters(navigation->road.segments[navigation->segment_index], navigation->heading, &parameters))
    {
        fault(navigation, NAV_BAD_CONFIG);
        return 0;
    }
    /* 提交失败也可能已有部分轮子收到命令，因此位置不再视为已确认。 */
    navigation->position_confirmed = 0;
    if (navigation->rpm)
        parameters.rpm = navigation->rpm;
    if (!navigation->motion.start(navigation->motion.user, parameters, now_ms))
    {
        fault(navigation, NAV_MOTION_FAILED);
        return 0;
    }
    navigation->state = NAV_MOVING;
    return 1;
}

int nav_init(Navigation *navigation, uint8_t initial_node, RoadHeading heading, const NavMotionPort *port)
{
    if (!navigation || initial_node >= 9 || (unsigned) heading > HEAD_X_POS || !port || !port->start
        || !port->poll || !port->stop)
        return 0;
    memset(navigation, 0, sizeof(*navigation));
    navigation->motion = *port;
    navigation->current = initial_node;
    navigation->target = initial_node;
    navigation->next = initial_node;
    navigation->heading = heading;
    navigation->position_confirmed = 1;
    return 1;
}

int nav_start(Navigation *navigation, uint8_t target, uint16_t rpm)
{
    if (!navigation || target >= 9 || (rpm && (rpm < 20 || rpm > 300)) || !navigation->position_confirmed
        || !stationary(navigation->state) || navigation->state == NAV_FAULT)
        return 0;
    navigation->target = target;
    navigation->rpm = rpm;
    navigation->error = NAV_OK;
    navigation->state = NAV_PLAN;
    return 1;
}

void nav_update(Navigation *navigation, uint32_t now_ms)
{
    if (!navigation)
        return;
    if (navigation->state == NAV_PLAN)
    {
        RouteTablePath route;
        if (!route_table_lookup(navigation->current, navigation->target, navigation->blocked, navigation->passed, &route))
        {
            fault(navigation, NAV_NO_ROUTE);
            return;
        }
        if (route.count == 1)
        {
            navigation->state = NAV_ARRIVED;
            return;
        }
        navigation->next = route.nodes[1];
        if (!road_plan(navigation->current, navigation->next, &navigation->road))
        {
            fault(navigation, NAV_BAD_CONFIG);
            return;
        }
        navigation->segment_index = 0;
        start_segment(navigation, now_ms);
    }
    else if (navigation->state == NAV_MOVING)
    {
        NavMotionEvent event = navigation->motion.poll(navigation->motion.user, now_ms);
        if (event == NAV_MOTION_FAULT)
        {
            fault(navigation, NAV_MOTION_FAILED);
            return;
        }
        if (event == NAV_MOTION_DONE)
        {
            navigation->segment_index++;
            navigation->progress_mm = 0;
            navigation->progress_valid = 0;
            if (navigation->segment_index == navigation->road.count)
                navigation->state = NAV_WAIT_NODE;
            else
                start_segment(navigation, now_ms);
        }
    }
}

void nav_stop(Navigation *navigation)
{
    if (!navigation)
        return;
    navigation->motion.stop(navigation->motion.user);
    navigation->state = NAV_STOPPED;
    /* 道路中段/等待到达确认时仍保持 position_confirmed=0。 */
}

int nav_confirm_node(Navigation *navigation, uint8_t actual_node)
{
    if (!navigation || navigation->state != NAV_WAIT_NODE || actual_node != navigation->next || (navigation->blocked & (1u << navigation->road.road)))
        return 0;
    navigation->current = navigation->next;
    navigation->position_confirmed = 1;
    navigation->passed |=(uint16_t)(1u << navigation->road.road);
    navigation->state = NAV_PLAN;
    return 1;
}

int nav_obstacle(Navigation *navigation)
{
    if (!navigation || (navigation->state != NAV_MOVING && navigation->state != NAV_WAIT_NODE))
        return 0;
    navigation->motion.stop(navigation->motion.user);
    navigation->blocked |=(uint16_t)(1u << navigation->road.road);
    navigation->passed &=(uint16_t) ~(1u << navigation->road.road);
    navigation->position_confirmed = 0;
    navigation->state = NAV_WAIT_RETURN;
    return 1;
}

int nav_set_blocked(Navigation *navigation, uint8_t road, int blocked)
{
    if (!navigation || road >= ROAD_COUNT)
        return 0;
    if (blocked)
    {
        navigation->blocked |=(uint16_t)(1u << road);
        navigation->passed &=(uint16_t) ~(1u << road);
        if ((navigation->state == NAV_MOVING || navigation->state == NAV_WAIT_NODE) && navigation->road.road == road)
            nav_obstacle(navigation);
    }
    else
        navigation->blocked &=(uint16_t) ~(1u << road);
    return 1;
}

int nav_confirm_return(Navigation *navigation, uint8_t actual_node)
{
    if (!navigation || navigation->state != NAV_WAIT_RETURN || actual_node != navigation->current)
        return 0;
    navigation->position_confirmed = 1;
    navigation->progress_valid = 0;
    navigation->state = NAV_PLAN;
    return 1;
}

int nav_confirm_pose(Navigation *navigation, uint8_t actual_node)
{
    if (!navigation || actual_node >= 9 || !stationary(navigation->state))
        return 0;
    navigation->current = navigation->target = navigation->next = actual_node;
    navigation->position_confirmed = 1;
    navigation->progress_mm = 0;
    navigation->progress_valid = 0;
    navigation->error = NAV_OK;
    navigation->state = NAV_IDLE;
    return 1;
}

int nav_set_heading(Navigation *navigation, RoadHeading heading)
{
    if (!navigation || (unsigned) heading > HEAD_X_POS || !stationary(navigation->state) || !navigation->position_confirmed)
        return 0;
    navigation->heading = heading;
    return 1;
}

int nav_report_progress(Navigation *navigation, uint16_t actual_mm)
{
    MotionParameters s;
    uint16_t length;
    if (!navigation || navigation->state != NAV_MOVING)
        return 0;
    s = navigation->road.segments[navigation->segment_index];
    length = (uint16_t)(s.mm < 0 ? -s.mm : s.mm);
    if (actual_mm > length || (navigation->progress_valid && actual_mm < navigation->progress_mm))
        return 0;
    navigation->progress_mm = actual_mm;
    navigation->progress_valid = 1;
    return 1;
}
