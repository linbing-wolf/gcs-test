#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H

/* 比赛工位参数入口。机械坐标为绝对坐标，平推伸出、升降上升为正。
 * 视觉标注与装置变动后先按文件末尾实测。#ifndef 包住的默认值可由已有构建选项覆盖。
 */

/* 1. 原料区 J 像素分区
 * 使用相机回包原始坐标，原点在左下。区域顺序为 {X最小, X最大, Y最小, Y最大}，端点包含，单位 px。
 * 初始边界由左(136,44)、中(310,315)、右(491,32) 三个样点估算，完整可抓范围须复测。
 * 三块区域应互不重叠，超出或命中多个区域会等待；-1 可表示未标定。姿态在 arm_config.h 的 ARM_RAW_* 标定。
 */
#define RAW_J_LEFT_REGION   {   0, 313,   0, 179 } /* J 目标命中此区后选左侧 NEXT 抓取姿态；初始边界从样点推导。 */
#define RAW_J_MIDDLE_REGION {   0, 639, 180, 479 } /* J 目标命中此区后选中间 CURRENT 抓取姿态；初始边界从样点推导。 */
#define RAW_J_RIGHT_REGION  { 314, 639,   0, 179 } /* J 目标命中此区后选右侧 PAST 抓取姿态；初始边界从样点推导。 */

/* 2. 粗加工区 D
 * 两批均先对准 2 号环，再按任务环位移车，放完后逐环抓回。
 * 移车索引时三个环共用工作角，平推按目标环号配置。固定车身另用独立姿态。
 */
#ifndef STN_ROUGH_PLACE_READY
#define STN_ROUGH_PLACE_READY    1 /* 地面姿态启用标记，1 已标定/0 未标定；姿态全零仍会拒绝作业。 */
#endif
#ifndef STN_ROUGH_RING1_SLIDE_MM
#define STN_ROUGH_RING1_SLIDE_MM (132) /* 固定车身时 1 号环平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_ROUGH_RING1_YAW_DEG
#define STN_ROUGH_RING1_YAW_DEG  (96.0f) /* 固定车身时 1 号环云台目标角，度。 */
#endif
#ifndef STN_ROUGH_SLIDE_MM
#define STN_ROUGH_SLIDE_MM       80 /* 固定车身 2 号环平推绝对坐标，mm；与移车索引独立标定。 */
#endif
#ifndef STN_ROUGH_YAW_DEG
#define STN_ROUGH_YAW_DEG        133.0f /* 2 号环及移车索引模式的地面云台角，度；也用于粗加工观察。 */
#endif
#ifndef STN_ROUGH_RING3_SLIDE_MM
#define STN_ROUGH_RING3_SLIDE_MM (115) /* 固定车身时 3 号环平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_ROUGH_RING3_YAW_DEG
#define STN_ROUGH_RING3_YAW_DEG  (170.0f) /* 固定车身时 3 号环云台目标角，度。 */
#endif
#ifndef STN_ROUGH_PLACE_LIFT_MM
#define STN_ROUGH_PLACE_LIFT_MM  0 /* 各环放料松爪的绝对高度，mm；增大向上，可填写行程内的负值。 */
#endif
#ifndef STN_ROUGH_LIFT_MM
#define STN_ROUGH_LIFT_MM        0 /* 各环抓回时的绝对高度，mm；增大向上，可独立于放料高度标定。 */
#endif
#ifndef STN_ROUGH_INDEX_RING1_SLIDE_MM
#define STN_ROUGH_INDEX_RING1_SLIDE_MM 35 /* 粗加工两批 1 号环放料和抓回共用，填平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_ROUGH_INDEX_RING2_SLIDE_MM
#define STN_ROUGH_INDEX_RING2_SLIDE_MM 35 /* 粗加工两批 2 号环放料和抓回共用，填平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_ROUGH_INDEX_RING3_SLIDE_MM
#define STN_ROUGH_INDEX_RING3_SLIDE_MM 35 /* 粗加工两批 3 号环放料和抓回共用，填平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_ROUGH_INDEX_12_MM
#define STN_ROUGH_INDEX_12_MM    150 /* 移车索引时 1↔2 号环中心距，mm；双向共用，范围 1～300。 */
#endif
#ifndef STN_ROUGH_INDEX_23_MM
#define STN_ROUGH_INDEX_23_MM    145 /* 移车索引时 2↔3 号环中心距，mm；双向共用，1↔3 走两段之和。 */
#endif
#define STN_ROUGH_INDEX_SIGN     -1 /* 环号增加时的移车方向，-1 后退/+1 前进；以 2 号环为基准。 */
#define STN_ROUGH_INDEX_RPM      60 /* 粗加工环位索引与视觉纠偏的速度请求上限，RPM；范围 20～60。 */

/* 3. 暂存区 H
 * 第一批先对准 2 号环再逐环移车放料。第二批逐环移车、F 视觉定位和叠放，环距与索引方向两批均生效。
 */
#ifndef STN_STORAGE_PLACE_READY
#define STN_STORAGE_PLACE_READY    1 /* 地面放料姿态启用标记，1 已标定/0 未标定。 */
#endif
#ifndef STN_STORAGE_RING1_SLIDE_MM
#define STN_STORAGE_RING1_SLIDE_MM (132) /* 第一批固定车身时 1 号环的平推绝对坐标，mm。 */
#endif
#ifndef STN_STORAGE_RING1_YAW_DEG
#define STN_STORAGE_RING1_YAW_DEG  (96.0f) /* 第一批固定车身时 1 号环的云台目标角，度。 */
#endif
#define STN_STORAGE_SLIDE_MM       80 /* 固定车身 2 号环平推绝对坐标，mm；与第一批移车索引独立标定。 */
#define STN_STORAGE_YAW_DEG        133.0f /* 2 号环及逐环移车作业的地面放料云台角，度。 */
#ifndef STN_STORAGE_RING3_SLIDE_MM
#define STN_STORAGE_RING3_SLIDE_MM (115) /* 第一批固定车身时 3 号环的平推绝对坐标，mm。 */
#endif
#ifndef STN_STORAGE_RING3_YAW_DEG
#define STN_STORAGE_RING3_YAW_DEG  (170.0f) /* 第一批固定车身时 3 号环的云台目标角，度。 */
#endif
#ifndef STN_STORAGE_LIFT_MM
#define STN_STORAGE_LIFT_MM        0 /* 第一层松爪的绝对高度，mm；第二层从这一高度向上加层高。 */
#endif
#ifndef STN_STORAGE_LAYER_LIFT_MM
#define STN_STORAGE_LAYER_LIFT_MM  60 /* 第二层相对第一层的高度增量，mm；0 表示未标定，总高度须在升降行程内。 */
#endif
#define STN_STORAGE_ALIGN_YAW_DEG  133.0f /* 暂存观察、换盘或视觉微调姿态的云台角，度。 */
#ifndef STN_STORAGE_INDEX_RING1_SLIDE_MM
#define STN_STORAGE_INDEX_RING1_SLIDE_MM 35 /* 第一次暂存 1 号环放料的平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_STORAGE_INDEX_RING2_SLIDE_MM
#define STN_STORAGE_INDEX_RING2_SLIDE_MM 35 /* 第一次暂存 2 号环放料的平推绝对坐标，mm；增大向外伸。 */
#endif
#ifndef STN_STORAGE_INDEX_RING3_SLIDE_MM
#define STN_STORAGE_INDEX_RING3_SLIDE_MM 35 /* 第一次暂存 3 号环放料的平推绝对坐标，mm；增大向外伸。 */
#endif
#define STN_STORAGE_STACK_SLIDE_MM 80 /* 第二批 F 视觉定位后叠放所用的已标定平推坐标，mm，改第一批环位不影响此值。 */
#ifndef STN_STORAGE_INDEX_12_MM
#define STN_STORAGE_INDEX_12_MM    150 /* 移车索引时 1↔2 号环中心距，mm；双向共用，范围 1～300。 */
#endif
#ifndef STN_STORAGE_INDEX_23_MM
#define STN_STORAGE_INDEX_23_MM    145 /* 移车索引时 2↔3 号环中心距，mm；双向共用，1↔3 走两段之和。 */
#endif
#define STN_STORAGE_INDEX_SIGN     -1 /* 环号增加时的移车方向，-1 后退/+1 前进；以 2 号环为基准。 */
#define STN_STORAGE_INDEX_RPM      60 /* 暂存环位索引与视觉纠偏的速度请求上限，RPM；范围 20～60。 */

/* 4. 车载杯取料与工位模式 */
#define STN_ROUGH_PLATFORM_TAKE_LIFT_MM    100 /* 从车载杯取料的绝对升降高度，mm；两批粗加工及两批暂存共用。 */
#ifndef STN_ROUGH_PLATFORM_TAKE_READY
#define STN_ROUGH_PLATFORM_TAKE_READY      1 /* 粗加工车载杯取料标定启用，1 已标定/0 未标定。 */
#endif
#ifndef STN_PLATFORM_TAKE_READY
#define STN_PLATFORM_TAKE_READY            1 /* 暂存车载杯取料标定启用，1 已标定/0 未标定。 */
#endif
#ifndef STN_FIXED_RING_ARM_ENABLED
#define STN_FIXED_RING_ARM_ENABLED         0 /* 0 复用原底盘环位索引；1 两批粗加工和第一批暂存固定车身。第二批仍逐环移车。 */
#endif
#define STN_ROUGH_THROUGH_ENABLED          (CAR_ARM_TRANSLATION_OVERLAP_ENABLED && CAR_RUN_CORNER_ENABLED && CAR_RUN_OMNI_ENABLED) /* 派生开关；工位底盘并行、圆角、全向均启用才允许索引回 2 号环连续接离站道路。 */
#define STN_STORAGE_FIRST_THROUGH_ENABLED  STN_ROUGH_THROUGH_ENABLED /* 派生开关；第一批暂存的索引回 2 号环接离站道路沿用粗加工的开关条件。 */
#define STN_STORAGE_SECOND_THROUGH_ENABLED STN_ROUGH_THROUGH_ENABLED /* 派生开关；第二批暂存的索引回 2 号环接离站道路沿用同一开关条件。 */
#define STN_SCAN_HEADING                   1u /* 保留的 B 点扫码地图朝向编号，1 为车头向左；当前主控未引用此宏。 */

/* 5. 视觉纠偏与工位等待
 * MOVE 的正方向见纵向符号，MOVEY 的正方向见横向符号，MOVEXY 分别复用两轴。底盘航向增益、末段控制和脉冲/mm 见 car_config.h。
 */
#ifndef STN_VISION_MM_PER_PX_X1000
#define STN_VISION_MM_PER_PX_X1000    1000 /* 像素到毫米比例×1000；1000 对应 1 mm/px，两轴共用，须用实移标定。 */
#endif
#define STN_ALIGN_MOVE_SIGN           -1 /* 视觉纵向位移方向，-1 正值后退/+1 正值前进。 */
#define STN_ALIGN_Y_MOVE_SIGN         1 /* 视觉横向位移方向，+1 正值向左/-1 正值向右。 */
#define STN_APPROACH_OVERLAP_MM       5 /* 允许提前靠近的位移剩余窗口，mm；每个有效轴均须满足。 */
#define STN_APPROACH_OVERLAP_RPM      6 /* 允许提前靠近的速度窗口，RPM；检查已提交的纵向、横向和转向分量。 */
#define STN_APPROACH_OVERLAP_YAW_CDEG 300 /* 允许工位末段提前靠近的航向误差窗，0.01 度。 */
#define STN_ALIGN_TIMEOUT_MS          60000u /* 处理对准步骤时的日志告警周期，ms；继续等真实结果。 */
#define STN_VISION_TIMEOUT_MS         25000u /* J 等待与 F/STACK 定位的告警时间，ms；继续等待真实终包或反馈。 */
#define STN_STEP_TIMEOUT_MS           40000u /* 普通机构或工位索引执行过久的告警周期，ms；到位仍由实际反馈决定。 */

/* 由上方参数生成的工位姿态 */
typedef struct {
    int slide_mm;
    int lift_mm;
    float yaw_deg;
} StationPose;
static const StationPose stn_rough_place = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_PLACE_LIFT_MM, STN_ROUGH_YAW_DEG
};
static const StationPose stn_rough_grip = {
    STN_ROUGH_SLIDE_MM, STN_ROUGH_LIFT_MM, STN_ROUGH_YAW_DEG
};
static const StationPose stn_storage_place = {
    STN_STORAGE_STACK_SLIDE_MM, STN_STORAGE_LIFT_MM, STN_STORAGE_YAW_DEG
};

/* 编译期工位约束 */
#if (STN_ROUGH_INDEX_SIGN != 1 && STN_ROUGH_INDEX_SIGN != -1) || \
    STN_ROUGH_INDEX_12_MM < 1 || STN_ROUGH_INDEX_12_MM > 300 || \
    STN_ROUGH_INDEX_23_MM < 1 || STN_ROUGH_INDEX_23_MM > 300 || \
    STN_ROUGH_INDEX_RPM < 20 || STN_ROUGH_INDEX_RPM > 60
#error Invalid_rough_station_index_configuration
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

/*
 * 需要联动的调整
 *
 * 1. 工位实际落点
 *    先按 arm_config.h 文件末尾的方法声明真实姿态。固定车身模式先让底盘对准 2 号环，
 *    再分别对 1/2/3 号环标定绝对平推、云台角和地面高度。将各项回填到本文件，
 *    确认每个环的下降、夹放、抬升、收回净空后再启用对应 READY。暂存二层目标为
 *    STN_STORAGE_LIFT_MM + STN_STORAGE_LAYER_LIFT_MM。车载杯平推和云台在 arm_config.h。
 *    比赛默认移车索引时，分别回填 STN_ROUGH_INDEX_RING1/2/3_SLIDE_MM 与
 *    STN_STORAGE_INDEX_RING1/2/3_SLIDE_MM。按实际地面环号选择对应行。
 *    这些数是平推绝对坐标，单位 mm。0 为完全收回。夹爪离环中心还差 4mm 才够到，就把
 *    35 加到 39。伸过环中心 3mm，就把 35 减到 32。改完重新编译后复测该环。
 *    暂存两批用不同平推来源，第二批的 F 定位及已标定 STN_STORAGE_STACK_SLIDE_MM 保留。
 *    若整片工位停车点偏移，查 car_config.h 的地图说明；环距、机械臂姿态和视觉比例
 *    应分别按环中心距、机械落点与实际移动量标定。
 *
 * 2. 原料 J 像素分区及相机联调
 *    固定相机位置和三点姿态，记录三处物块的 J 原始坐标。在空闲时用
 *    raw region left/middle/right XMIN XMAX YMIN YMAX 逐区试填（以单个区名替换）。
 *    raw region status 可查看当前区域。用中心、边界点核对同一目标色命中的抓点；
 *    若已完成的 J 因目标位置不可用而等待，可用 raw observe 再观察。运行时修改只
 *    保存在本次运行，验证后回填本文件的 RAW_J_*_REGION。
 *    如果变更相机输出分辨率或坐标原点，先同步相机输出和 raw_region_config.h 中
 *    的图像尺寸，再重测三个区域和视觉毫米/像素比例。相机装位变化后也要复测。
 *
 * 3. 视觉接口与作业流程
 *    视觉位移比例和两个方向符号可在本文件调整；短距离用 vision debug D、vision debug E
 *    或 vision debug F 色号试运行并实测每轴位移。新位置验证至少覆盖一个正偏差和
 *    一个负偏差。角度/轮速参数见 car_config.h 的闭环与末段分组。
 *    模拟 I2C 接线、地址、邮箱大小在 vision_soft_i2c.h，半帧间隔与解析在 vision_link.c。
 *    更换接口或邮箱时两端同步；主机 SCL 要在 MaixCAM 端调并实测。J 跨片间隙先核对
 *    相机整帧连续写入和 vision status 中的接收诊断，见 docs/I2C视觉通信与联调.md
 *    及 docs/J长帧跨片间隙定位与上位机修复_20261124.md。
 *    作业顺序和工位选择见 station_job.c 的 build_raw/build_rough/build_storage；
 *    工位每一处位置与等待参数可以在上面的分组调整。串口 station auto on/off 用于
 *    本次运行的自动作业开关。
 */

#endif
