#include "route_table.h"
#include "car_config.h"
#include "table_data.h"
#if CAR_MAP_TOP_INSET_MM != TABLE_MAP_TOP_INSET_MM || \
    CAR_MAP_BOTTOM_INSET_MM != TABLE_MAP_BOTTOM_INSET_MM || \
    CAR_MAP_RAW_EXTRA_INSET_MM != TABLE_MAP_RAW_EXTRA_INSET_MM || \
    CAR_MAP_STORAGE_INSET_MM != TABLE_MAP_STORAGE_INSET_MM
#error Map_inset_changed_rebuild_route_table_first
#endif

#ifdef ROUTE_TABLE_VERIFY_STORAGE
/* 仅电脑验证构建启用；不进入车辆模块的公开接口。 */
uint32_t route_table_storage_bytes(void)
{
    return (uint32_t)(sizeof(table_primary) + sizeof(table_secondary) +
                     sizeof(table_path_base) + sizeof(table_decision_base) +
                     sizeof(table_decisions) + sizeof(table_paths));
}
#endif

int route_table_lookup(uint8_t from, uint8_t to, uint16_t blocked,
                       uint16_t passed, RouteTablePath *out)
{
    uint32_t pair, index, packed, bit;
    uint8_t code, i;
    const TableDecision *decision;
    const RouteTablePath *path;
    if (!out) return 0;
    out->distance = 65535u;
    out->count = 0;
    for (i = 0; i < 9; i++) out->nodes[i] = 255;
    if (from >= 9 || to >= 9) return 0;

    blocked &= 4095u;
    passed &= (uint16_t)(4095u & ~blocked);
    pair = (uint32_t)from * 9u + to;
    code = table_primary[(uint32_t)blocked * 81u + pair];
    if (code == 255) return 0;
    if (code >= 16) {
        decision = &table_decisions[table_decision_base[pair] + code - 16u];
        packed = 0;
        index = 0;
        for (bit = 0; bit < 12; bit++) {
            if (decision->mask & (1u << bit)) {
                if (passed & (1u << bit)) packed |= 1u << index;
                index++;
            }
        }
        code = table_secondary[decision->offset + packed];
    }
    path = &table_paths[table_path_base[pair] + code];
    *out = *path;
    return 1;
}
