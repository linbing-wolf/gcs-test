#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
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

/* ===== 粗加工区 ===== 数组下标 0~4 对应 1~5 号环位。
 * 任务码 G2（第一批）和 G4（第二批）决定车上第 1/2/3 块各放到哪个环位；
 * 放完三块后程序按 1、2、3 号环位顺序抓回车上。 */
#define STN_ROUGH_PLACE_READY   0
static const StationPose stn_rough_place[5] = {
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f}
};

/* ===== 暂存区 ===== 数组下标 0~4 对应 1~5 号环位。 */
#define STN_STORAGE_PLACE_READY 0
static const StationPose stn_storage_place[5] = {
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f},
    {0, 0, 0.0f}
};

/* 从车上载物台把物块夹起来的姿态，沿用已标定的载物台放置参数即可；
 * 若实测发现爪子下不到杯里，只改这三项。 */
#define STN_PLATFORM_TAKE_READY 0
#define STN_PLATFORM_TAKE_SLIDE 30
#define STN_PLATFORM_TAKE_LIFT 110
#define STN_PLATFORM_TAKE_YAW  254.0f

#endif
