#ifndef NINE_DISTANCE_ROUTE_H
#define NINE_DISTANCE_ROUTE_H
#include <stdint.h>
typedef struct { uint8_t a,b; uint16_t length; } Road;
typedef struct { uint16_t distance,unknown; uint8_t count,nodes[9]; } Route;
/* A-I=0-8；roads按当前道路位序传入。仅电脑验证，返回1可达/0不可达。 */
int route_distance(uint8_t from,uint8_t to,uint16_t blocked,uint16_t passed,const Road roads[12],Route *out);
#endif
