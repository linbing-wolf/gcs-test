#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
/* B点二维码板位于+X；扫码模块在车尾，车头须朝-X，让车尾朝+X。 */
#define STN_SCAN_HEADING 1u
/* 粗加工区 / 暂存区的取放姿态标定表。
 *
 * 这三张表没填之前，自动流程走到对应工位会报
 * [station fault] ... not calibrated 并原地停车，不会猜角度乱动。
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
 * 每次前后索引150mm；1↔3经2号拆两段。INDEX_SIGN=1表示3号在车头方向。
 * 只需标定同一组侧向抓放姿态，放下和抓回都按任务码物块顺序进行。
 * 姿态全零即使READY=1也拒绝。下面#ifndef仅供电脑测试覆盖，不是标定值。 */
#ifndef STN_ROUGH_PLACE_READY
#define STN_ROUGH_PLACE_READY   0
#endif
#define STN_ROUGH_INDEX_MM    150
#define STN_ROUGH_INDEX_SIGN    1
#define STN_ROUGH_INDEX_RPM    60
#ifndef STN_ROUGH_SLIDE_MM
#define STN_ROUGH_SLIDE_MM      0
#endif
#ifndef STN_ROUGH_LIFT_MM
#define STN_ROUGH_LIFT_MM       0
#endif
#ifndef STN_ROUGH_YAW_DEG
#define STN_ROUGH_YAW_DEG       0.0f
#endif
static const StationPose stn_rough_place = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_LIFT_MM, STN_ROUGH_YAW_DEG
};
#if (STN_ROUGH_INDEX_SIGN != 1 && STN_ROUGH_INDEX_SIGN != -1) || \
    STN_ROUGH_INDEX_MM < 1 || STN_ROUGH_INDEX_MM > 300 || \
    STN_ROUGH_INDEX_RPM < 20 || STN_ROUGH_INDEX_RPM > 60
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
