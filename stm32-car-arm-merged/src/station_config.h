#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
/* B点二维码板位于+X；扫码模块在车尾，车头须朝-X，让车尾朝+X。 */
#define STN_SCAN_HEADING 1u
/* 粗加工区 / 暂存区的取放姿态标定表。
 *
 * 粗加工使用操作者2026-10-19指定参数；暂存及其平台取料仍未标定。
 * 未启用对应READY或粗加工姿态全零时，拒绝动作并原地停车。
 *
 * 每项含义：slide=平推轴伸出量(mm)，lift=升降高度(mm)，yaw=云台角(°)。
 * 也就是“爪子正好在环位/杯位正上方、下降到能夹住物块”的那一组数。
 * 标定方法：手动 arm on → arm pose → arm platform pose 后，用
 *   arm slide N / arm lift N / arm yaw D / claw angle D
 * 一点一点试到位，记下四个数，再回填到下面，把 READY 改成 1。
 */
typedef struct {
    int slide_mm;
    int lift_mm;
    float yaw_deg;
} StationPose;

/* ===== 粗加工区 ===== 到站/离站均为2号环基准；只允许1/2/3的排列。
 * 操作者确认：前1、中2、后3；先视觉确认2，再按1/2/3放下和抓回。
 * 2026-10-19指定粗加工复用原料平推/升降，云台125度；不等于实车验收。
 * 姿态全零即使READY=1也拒绝。下面#ifndef仅供电脑测试覆盖，不是标定值。 */
#ifndef STN_ROUGH_PLACE_READY
#define STN_ROUGH_PLACE_READY   1
#endif
#define STN_ROUGH_INDEX_MM    150
#define STN_ROUGH_INDEX_SIGN   -1
#define STN_ROUGH_INDEX_RPM    60
#define STN_ROUGH_ALIGN_STEP_MM 3 /* 只采用MOVE偏差方向，每次3mm；不把像素当毫米。 */
#define STN_ROUGH_ALIGN_MOVE_SIGN 1 /* MOVE正号向前；若现场相机镜像应验证此方向。 */
#define STN_ROUGH_ALIGN_TRAVEL_MM 75 /* 每个环的累计修正路程上限，mm。 */
#define STN_ROUGH_ALIGN_SAMPLES 3 /* 停稳后连续独立请求确认到位。 */
#define STN_ROUGH_ALIGN_TIMEOUT_MS 60000u
#ifndef STN_ROUGH_SLIDE_MM
#define STN_ROUGH_SLIDE_MM     55
#endif
#ifndef STN_ROUGH_LIFT_MM
#define STN_ROUGH_LIFT_MM     105
#endif
#ifndef STN_ROUGH_YAW_DEG
#define STN_ROUGH_YAW_DEG     125.0f
#endif
static const StationPose stn_rough_place = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_LIFT_MM, STN_ROUGH_YAW_DEG
};
/* 粗加工专用平台取料由操作者确认复用ARM_PLACE_*，不放行暂存旧取料表。 */
#ifndef STN_ROUGH_PLATFORM_TAKE_READY
#define STN_ROUGH_PLATFORM_TAKE_READY 1
#endif
#if (STN_ROUGH_INDEX_SIGN != 1 && STN_ROUGH_INDEX_SIGN != -1) || \
    STN_ROUGH_INDEX_MM < 1 || STN_ROUGH_INDEX_MM > 300 || \
    STN_ROUGH_INDEX_RPM < 20 || STN_ROUGH_INDEX_RPM > 60 || \
    STN_ROUGH_ALIGN_STEP_MM < 3 || STN_ROUGH_ALIGN_STEP_MM > 20 || \
    (STN_ROUGH_ALIGN_MOVE_SIGN != 1 && STN_ROUGH_ALIGN_MOVE_SIGN != -1)
#error Invalid_rough_station_index_configuration
#endif

/* ===== 暂存区 ===== 数组下标 0~4 对应 1~5 号环位。 */
#ifndef STN_STORAGE_PLACE_READY
#define STN_STORAGE_PLACE_READY 0
#endif
static const StationPose stn_storage_place[5] = {
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f}
};

/* 从车上载物台把物块夹起来的姿态，沿用已标定的载物台放置参数即可；
 * 若实测发现爪子下不到杯里，只改这三项。 */
#ifndef STN_PLATFORM_TAKE_READY
#define STN_PLATFORM_TAKE_READY 0
#endif
#define STN_PLATFORM_TAKE_SLIDE 30
#define STN_PLATFORM_TAKE_LIFT 110
#define STN_PLATFORM_TAKE_YAW  254.0f

#endif
