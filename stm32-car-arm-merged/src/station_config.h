#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
/* 复用现有连续道路和工位并行开关；关闭其中之一即保留原回2环停车路径。 */
#define STN_ROUGH_THROUGH_ENABLED (CAR_ARM_TRANSLATION_OVERLAP_ENABLED && CAR_RUN_CORNER_ENABLED && CAR_RUN_OMNI_ENABLED)
#define STN_STORAGE_FIRST_THROUGH_ENABLED STN_ROUGH_THROUGH_ENABLED /* 首次暂存回2环并入离站道路。 */
#define STN_STORAGE_SECOND_THROUGH_ENABLED STN_ROUGH_THROUGH_ENABLED /* 第二次暂存放完后同样连续离站，保留逐物块F/STACK。 */
/* B点二维码板位于+X；扫码模块在车尾，车头须朝-X，让车尾朝+X。 */
#define STN_SCAN_HEADING 1u /* B点扫码车头地图朝向编号；1=向左，使车尾扫码器朝二维码板。 */
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
/* 两区D/MOVE/DISC对准：MOVE是像素偏差，按完整幅值换算毫米。
 * 粗加工及首次暂存进站对准2号一次（D/E），连续MOVE纠偏；第二次暂存直接F定位已有物块。 */
#define STN_ALIGN_MOVE_SIGN -1 /* 正像素偏差后退、负偏差前进；实车待核验。 */
/* 操作者假定1px=1mm，非实测；H沿用D/E/F的换算与方向，原料机位仍需实测核验。 */
#ifndef STN_VISION_MM_PER_PX_X1000
#define STN_VISION_MM_PER_PX_X1000 1000 /* 像素到毫米比例×1000；1000=假定1px/1mm，H/D/E/F两轴统一换算。 */
#endif
#define STN_ALIGN_Y_MOVE_SIGN       1  /* 实车反馈反向：MOVEY正值向左、负值向右；接口正值向左。 */
#define STN_ALIGN_TIMEOUT_MS 60000u /* 工位视觉等待告警周期，ms；不是每次发D/E的间隔。 */

/* 视觉终包且FIFO清空或固定索引的末段，允许下一靠近动作与低速修正并行。
 * mm为每轴剩余距离，RPM为每轴已提交速度；夹爪闭合仍按原顺序等待底盘完成。 */
#define STN_APPROACH_OVERLAP_MM 5
#define STN_APPROACH_OVERLAP_RPM 6
#define STN_APPROACH_OVERLAP_YAW_CDEG 300 /* 最后小角度修正窗口，0.01度；也用于进站准备姿态。 */

/* ===== 粗加工区 ===== 到站/离站均为2号环基准；只允许1/2/3的排列。
 * 操作者确认：前1、中2、后3；按盘位顺序查任务靶位，放下后同序抓回。
 * 进站先视觉对准2号基准一次，1↔2、2↔3环距分别可调；1↔3一次走两段之和。
 * 2026-10-19指定粗加工平推55mm、放料高度5mm、抓回高度5mm、平台取料高度100mm。
 * 作业内云台只在126.5/256度间直转，离站回0度；不等于实车验收。
 * 姿态全零即使READY=1也拒绝。下面#ifndef仅供电脑测试覆盖，不是标定值。 */
#ifndef STN_ROUGH_PLACE_READY
#define STN_ROUGH_PLACE_READY   1 /* 粗加工地面姿态启用标记，1启用/0未标定；不是速度开关。 */
#endif
#ifndef STN_ROUGH_INDEX_12_MM
#define STN_ROUGH_INDEX_12_MM 145 /* 粗加工1↔2号环中心距，mm；双向共用。 */
#endif
#ifndef STN_ROUGH_INDEX_23_MM
#define STN_ROUGH_INDEX_23_MM 145 /* 粗加工2↔3号环中心距，mm；双向共用。 */
#endif
#define STN_ROUGH_INDEX_SIGN   -1 /* 环编号增加时的移车方向，-1=后退；前1、中2、后3。 */
#define STN_ROUGH_INDEX_RPM    60 /* 粗加工环位索引/视觉微调速度请求上限，RPM；允许20~60。 */
#ifndef STN_ROUGH_SLIDE_MM
#define STN_ROUGH_SLIDE_MM     55 /* 粗加工地面放料/抓回的平推目标，mm；不是车载杯位参数。 */
#endif
#ifndef STN_ROUGH_PLACE_LIFT_MM
#define STN_ROUGH_PLACE_LIFT_MM 5 /* 放料松爪高度，mm；抓回高度同为5mm。 */
#endif
#ifndef STN_ROUGH_LIFT_MM
#define STN_ROUGH_LIFT_MM       5 /* 粗加工地面抓回高度，mm；绝对坐标，放料高度由上一项定义。 */
#endif
#ifndef STN_ROUGH_YAW_DEG
#define STN_ROUGH_YAW_DEG     126.5f /* 粗加工观察/放料/抓回角，度；车载杯取放256度在arm_config.h。 */
#endif
static const StationPose stn_rough_place = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_PLACE_LIFT_MM, STN_ROUGH_YAW_DEG
};
static const StationPose stn_rough_grip = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_LIFT_MM, STN_ROUGH_YAW_DEG
};
/* 粗加工仅覆盖平台取料高度，平推/云台和放回平台高度仍用ARM_PLACE_*。 */
#define STN_ROUGH_PLATFORM_TAKE_LIFT_MM 100 /* 从车载杯取出物块的高度，mm；粗加工和两批暂存共用100。 */
#ifndef STN_ROUGH_PLATFORM_TAKE_READY
#define STN_ROUGH_PLATFORM_TAKE_READY 1 /* 粗加工平台取料标定启用，1启用/0未标定。 */
#endif
#if (STN_ROUGH_INDEX_SIGN != 1 && STN_ROUGH_INDEX_SIGN != -1) || \
    STN_ROUGH_INDEX_12_MM < 1 || STN_ROUGH_INDEX_12_MM > 300 || \
    STN_ROUGH_INDEX_23_MM < 1 || STN_ROUGH_INDEX_23_MM > 300 || \
    STN_ROUGH_INDEX_RPM < 20 || STN_ROUGH_INDEX_RPM > 60
#error Invalid_rough_station_index_configuration
#endif

/* ===== 暂存区 ===== 同一侧向姿态，通过底盘索引覆盖1/2/3号靶位。
 * 方向由操作者2026-10-19确认：前1、中2、后3，不能由工位航向推断。
 * 第二层松爪高度=单层放料高度+物块高度；0表示层高未测，拒绝第二批暂存。
 * 第一层沿用粗加工55mm/5mm/126.5度；物块60mm，第二层lift=5+60=65mm。
 * 配置值来自作业方案，并不等于实车标定或叠放验收。 */
#ifndef STN_STORAGE_PLACE_READY
#define STN_STORAGE_PLACE_READY 1 /* 暂存放料姿态启用，1启用/0未标定。 */
#endif
#ifndef STN_STORAGE_INDEX_12_MM
#define STN_STORAGE_INDEX_12_MM 150 /* 暂存1↔2号环中心距，mm；两批作业及双向共用。 */
#endif
#ifndef STN_STORAGE_INDEX_23_MM
#define STN_STORAGE_INDEX_23_MM 150 /* 暂存2↔3号环中心距，mm；两批作业及双向共用。 */
#endif
#define STN_STORAGE_INDEX_SIGN -1 /* 环编号增加时的移车方向，-1=后退；前1、中2、后3。 */
#define STN_STORAGE_INDEX_RPM 60 /* 暂存索引/视觉微调速度请求上限，RPM；允许20~60。 */
#define STN_STORAGE_SLIDE_MM 55 /* 暂存地面放料平推目标，mm；第一层与第二层共用。 */
#define STN_STORAGE_LIFT_MM 5 /* 暂存第一层松爪绝对高度，mm；第二层在此基础上加层高。 */
#define STN_STORAGE_YAW_DEG 126.5f /* 暂存地面放料云台角，度；需与观察角配合。 */
#define STN_STORAGE_ALIGN_YAW_DEG 126.5f /* 观察/换盘/微调角；放料角同为126.5度。 */
#ifndef STN_STORAGE_LAYER_LIFT_MM
#define STN_STORAGE_LAYER_LIFT_MM 60 /* 物块/层高增量，mm；第二层=5+60=65，非从车上取料高度。 */
#endif
static const StationPose stn_storage_place = {
    STN_STORAGE_SLIDE_MM, STN_STORAGE_LIFT_MM, STN_STORAGE_YAW_DEG
};

/* 暂存两批平台取料均沿用粗加工100mm；叠层只改变地面放料高度。 */
#ifndef STN_PLATFORM_TAKE_READY
#define STN_PLATFORM_TAKE_READY 1 /* 暂存平台取料标定启用，1启用/0未标定。 */
#endif
#if (STN_STORAGE_INDEX_SIGN != 1 && STN_STORAGE_INDEX_SIGN != -1) || \
    STN_STORAGE_INDEX_12_MM < 1 || STN_STORAGE_INDEX_12_MM > 300 || \
    STN_STORAGE_INDEX_23_MM < 1 || STN_STORAGE_INDEX_23_MM > 300 || \
    STN_STORAGE_INDEX_RPM < 20 || STN_STORAGE_INDEX_RPM > 60 || \
    STN_STORAGE_LAYER_LIFT_MM < 0
#error Invalid_storage_station_configuration
#endif
#if STN_ALIGN_MOVE_SIGN != 1 && STN_ALIGN_MOVE_SIGN != -1
#error Invalid_station_alignment_configuration
#endif

#endif
