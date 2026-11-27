#ifndef NINE_ROUTE_TABLE_H
#define NINE_ROUTE_TABLE_H
#include <stdint.h>

/* A-I=0-8；道路位序见 README.md。距离单位 mm。 */
typedef struct {
    uint16_t distance;
    uint8_t count;
    uint8_t nodes[9];
} RouteTablePath;

/* 1=可达，0=不可达或参数无效。封路优先于已通过记录。
 * 只查常量表并复制路线，不运行寻路或候选路线评分。
 * count=1 表示已经在目标节点；count>=2 时 nodes[1] 为下一节点。
 */
int route_table_lookup(uint8_t from, uint8_t to, uint16_t blocked,
                       uint16_t passed, RouteTablePath *out);
#endif
