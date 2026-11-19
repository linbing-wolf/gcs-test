#ifndef CHASSIS_OBSTACLE_IO_H
#define CHASSIS_OBSTACLE_IO_H
#include <stdint.h>
#define OBSTACLE_WHEEL_RPM 60u
#define OBSTACLE_CLOSE_MM 550u
#define OBSTACLE_EDGE_ALLOWANCE_MM 30
#define OBSTACLE_REQUIRED_HITS 2u
void obstacle_io_init(void);
void obstacle_io_poll(void);
void obstacle_io_send(uint8_t byte);
void obstacle_io_receive(uint8_t byte);
void obstacle_io_shot(uint8_t sensor);
/* FRONT=0, LEFT=1. Invalid or missing echo is -1. */
void obstacle_io_result(uint8_t sensor,int mm);
#endif
