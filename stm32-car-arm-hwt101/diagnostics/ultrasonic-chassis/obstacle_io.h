#ifndef CHASSIS_OBSTACLE_IO_H
#define CHASSIS_OBSTACLE_IO_H
#include <stdint.h>
#define OBSTACLE_CLOSE_MM 300u /* 严格小于此距离才计入近障碍，mm。 */
#define OBSTACLE_EARLY_MM 550u /* 普通道路沿用提前检测，包含550mm。 */
#define OBSTACLE_ENTRY_SAMPLES 2u
/* 选路代价初值，均为ms；不是实车计时结果。未知代价有限，避免无限绕远。 */
#define OBSTACLE_TURN_COST_MS 1500u /* 每次转向统一估时；用户实测约1~2秒，90/180度均计一次。 */
#define OBSTACLE_UNKNOWN_COST_MS 1000u
#define OBSTACLE_ACTION_COST_MS 200u /* 每次实际起停动作的反馈交接估计，不按道路节点重复收取。 */
#define OBSTACLE_ENTRY_COST_MS 280u
/* 已确认的探头安装坐标，车体前/左为正，mm。 */
#define OBSTACLE_FRONT_FORWARD_MM 145
#define OBSTACLE_FRONT_LEFT_MM 0
#define OBSTACLE_LEFT_FORWARD_MM 0
#define OBSTACLE_LEFT_LEFT_MM 135
#define OBSTACLE_OBSERVE_MAX_MM 1000
#define OBSTACLE_OBSERVE_LINE_MM 60
#define OBSTACLE_OBSERVE_ENDPOINT_MM 200
#define OBSTACLE_OBSERVE_MATCH_MM 100
#define OBSTACLE_OBSERVE_MAX_AGE_MS 500u
#define OBSTACLE_EDGE_ALLOWANCE_MM 30
#define OBSTACLE_REQUIRED_HITS 2u
#ifdef CAR_START_INSET_MM
#define OBSTACLE_START_INSET_MM CAR_START_INSET_MM
#else
#define OBSTACLE_START_INSET_MM 60 /* 独立旧诊断快照的原道路连接距离，mm。 */
#endif
void obstacle_io_init(void);
void obstacle_io_poll(void);
int obstacle_io_range_enabled(void);
void obstacle_io_send(uint8_t byte);
void obstacle_io_receive(uint8_t byte);
void obstacle_io_shot(uint8_t sensor);
/* FRONT=0, LEFT=1. Invalid or missing echo is -1. */
void obstacle_io_result(uint8_t sensor,int mm);
#endif
