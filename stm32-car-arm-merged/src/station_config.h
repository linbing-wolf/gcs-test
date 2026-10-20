#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
/* B点二维码板位于+X；扫码模块在车尾，车头须朝-X，让车尾朝+X。 */
#define STN_SCAN_HEADING 1u
/* 粗加工区 / 暂存区的取放姿态标定表。
 *
 * 粗加工使用操作者2026-10-19指定参数；暂存第一层取放沿用粗加工参数，第二层在同一放料姿态上增加物块高度。
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
/* 两区共用现有MaixCAM D/MOVE/DISC对准：只用像素偏差的方向，每次移动3mm。
 * 每次进站仅对准2号一次（粗加工D、暂存E），连续MOVE纠偏，成功DISC结束；移动期间不排队执行偏差。 */
#define STN_ALIGN_STEP_MM 3
#define STN_ALIGN_MOVE_SIGN -1 /* 2026-10-19按操作者要求反转：正偏差后退，负偏差前进；实车待核验。 */
#define STN_ALIGN_TRAVEL_MM 75
#define STN_ALIGN_TIMEOUT_MS 60000u

/* ===== 粗加工区 ===== 到站/离站均为2号环基准；只允许1/2/3的排列。
 * 操作者确认：前1、中2、后3；按盘位顺序查任务靶位，放下后同序抓回。
 * 进站先视觉对准2号基准一次，此后按150mm索引进行取放。
 * 2026-10-19指定粗加工平推55mm、放料高度5mm、抓回高度5mm、平台取料高度100mm。
 * 作业内云台只在125/256度间直转，离站回0度；不等于实车验收。
 * 姿态全零即使READY=1也拒绝。下面#ifndef仅供电脑测试覆盖，不是标定值。 */
#ifndef STN_ROUGH_PLACE_READY
#define STN_ROUGH_PLACE_READY   1
#endif
#define STN_ROUGH_INDEX_MM    150
#define STN_ROUGH_INDEX_SIGN   -1
#define STN_ROUGH_INDEX_RPM    60
#ifndef STN_ROUGH_SLIDE_MM
#define STN_ROUGH_SLIDE_MM     55
#endif
#ifndef STN_ROUGH_PLACE_LIFT_MM
#define STN_ROUGH_PLACE_LIFT_MM 5 /* 放料松爪高度，mm；抓回高度同为5mm。 */
#endif
#ifndef STN_ROUGH_LIFT_MM
#define STN_ROUGH_LIFT_MM       5
#endif
#ifndef STN_ROUGH_YAW_DEG
#define STN_ROUGH_YAW_DEG     125.0f
#endif
static const StationPose stn_rough_place = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_PLACE_LIFT_MM, STN_ROUGH_YAW_DEG
};
static const StationPose stn_rough_grip = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_LIFT_MM, STN_ROUGH_YAW_DEG
};
/* 粗加工仅覆盖平台取料高度，平推/云台和放回平台高度仍用ARM_PLACE_*。 */
#define STN_ROUGH_PLATFORM_TAKE_LIFT_MM 100
#ifndef STN_ROUGH_PLATFORM_TAKE_READY
#define STN_ROUGH_PLATFORM_TAKE_READY 1
#endif
#if (STN_ROUGH_INDEX_SIGN != 1 && STN_ROUGH_INDEX_SIGN != -1) || \
    STN_ROUGH_INDEX_MM < 1 || STN_ROUGH_INDEX_MM > 300 || \
    STN_ROUGH_INDEX_RPM < 20 || STN_ROUGH_INDEX_RPM > 60
#error Invalid_rough_station_index_configuration
#endif

/* ===== 暂存区 ===== 同一侧向姿态，通过底盘索引覆盖1/2/3号靶位。
 * 方向由操作者2026-10-19确认：前1、中2、后3，不能由工位航向推断。
 * 第二层松爪高度=单层放料高度+物块高度；0表示层高未测，拒绝第二批暂存。
 * 第一层沿用粗加工55mm/5mm/125度；物块60mm，第二层lift=5+60=65mm。
 * 配置值来自作业方案，并不等于实车标定或叠放验收。 */
#ifndef STN_STORAGE_PLACE_READY
#define STN_STORAGE_PLACE_READY 1
#endif
#define STN_STORAGE_INDEX_MM 150
#define STN_STORAGE_INDEX_SIGN -1
#define STN_STORAGE_INDEX_RPM 60
#define STN_STORAGE_SLIDE_MM 55
#define STN_STORAGE_LIFT_MM 5
#define STN_STORAGE_YAW_DEG 125.0f
#define STN_STORAGE_ALIGN_YAW_DEG 125.0f /* 观察/换盘/微调角；放料角同为125度。 */
#ifndef STN_STORAGE_LAYER_LIFT_MM
#define STN_STORAGE_LAYER_LIFT_MM 60
#endif
static const StationPose stn_storage_place = {
    STN_STORAGE_SLIDE_MM, STN_STORAGE_LIFT_MM, STN_STORAGE_YAW_DEG
};

/* 暂存两批平台取料均沿用粗加工100mm；叠层只改变地面放料高度。 */
#ifndef STN_PLATFORM_TAKE_READY
#define STN_PLATFORM_TAKE_READY 1
#endif
#if (STN_STORAGE_INDEX_SIGN != 1 && STN_STORAGE_INDEX_SIGN != -1) || \
    STN_STORAGE_INDEX_MM != 150 || STN_STORAGE_INDEX_RPM < 20 || STN_STORAGE_INDEX_RPM > 60 || \
    STN_STORAGE_LAYER_LIFT_MM < 0
#error Invalid_storage_station_configuration
#endif
#if STN_ALIGN_STEP_MM < 3 || STN_ALIGN_STEP_MM > 20 || \
    (STN_ALIGN_MOVE_SIGN != 1 && STN_ALIGN_MOVE_SIGN != -1)
#error Invalid_station_alignment_configuration
#endif

#endif
