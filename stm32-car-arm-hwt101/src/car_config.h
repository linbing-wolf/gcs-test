#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include <stdint.h>

/* 比赛底盘参数入口。先按症状找速度、四向标定或闭环分组。
 * 改动编译期默认值后重新构建并下载。#ifndef 包住的默认值可由已有构建选项覆盖。
 */

/* 1. 速度、加减速与停车 */
#define CAR_RUN_RPM                    250 /* 普通道路前进/后退速度上限，RPM；提高后需复测制动距离。 */
#define CAR_LATERAL_SHORT_MM           1200 /* 连续同向横移总长的分档阈值，mm；≤此值用短档，途中不按剩余距离切档。 */
#define CAR_LATERAL_SHORT_RPM          250 /* 短档横移速度上限，RPM。 */
#define CAR_LATERAL_LONG_RPM           200 /* 长档横移速度上限，RPM。 */
#define CAR_TEST_RPM                   60 /* 手动 test 上电默认速度，RPM；可用 rpm 临时改。 */
#define CAR_TEST_DISTANCE_MM           1000 /* 手动 test 上电默认距离，mm；可用 dist 临时改。 */
#define CAR_TURN_RPM                   30 /* 手动定角转向上电默认速度上限，RPM；进入 turn 后可用 rpm 临时改。 */
#define CAR_TURN_CLOSED_LOOP_RPM       300 /* 比赛原地转向速度上限，RPM；单轮最终仍受当前运动限幅。 */
#ifndef CAR_TURN_ACCELERATION
#define CAR_TURN_ACCELERATION         0 /* 比赛空车和手动原地转向的驱动器加速度档位，0～255；0 为直接启动。 */
#endif
#ifndef CAR_LOADED_TURN_RPM
#define CAR_LOADED_TURN_RPM           250 /* 比赛载料原地转向的单轮速度上限，RPM；物料易甩出时调低。 */
#endif
#ifndef CAR_LOADED_TURN_ACCELERATION
#define CAR_LOADED_TURN_ACCELERATION  225 /* 比赛载料原地转向的驱动器加减速档位，0～255；0 为直接启动。 */
#endif
#define CAR_DRIVE_ACCEL_RPM_PER_S      150u /* 工位平移等共用的基础软件升速率，RPM/s；普通横移也引用此值。 */
#define CAR_LATERAL_ACCEL_RPM_PER_S    CAR_DRIVE_ACCEL_RPM_PER_S /* 普通道路/手动横移的软件升速率，RPM/s；默认随上项变化。 */
#define CAR_ROUTE_ACCEL_RPM_PER_S      240u /* 前后道路与非纯横移路径的软件升速率，RPM/s。 */
#define CAR_DRIVE_DECEL_RPM_PER_S      200u /* 通用软件减速率，RPM/s；增大可缩短模型制动距离，须测实车。 */
#define CAR_DRIVE_BRAKE_LOOKAHEAD_MS   200u /* 通用制动的延迟预留，ms；增大使规划更早减速。 */
#define CAR_LATERAL_DECEL_RPM_PER_S    120u /* 普通左右横移的软件减速率，RPM/s；同时影响横移制动规划。 */
#define CAR_LATERAL_BRAKE_LOOKAHEAD_MS 400u /* 普通左右横移的制动延迟预留，ms；增大使横移更早减速。 */
#define CAR_ROUGH_BRAKE_LOOKAHEAD_MS   200u /* D 站制动延迟预留，ms；范围 100～1000，与通用减速率配合。 */
#define CAR_ROUTE_NODE_STOP_MS           0u /* 四轮停稳后的固定停留，ms；0 为直接进入下一动作。 */
#define CAR_OBSTACLE_RETURN_RPM        60 /* 外部 obstacle 请求后沿原路退回的速度上限，RPM；至少 20。 */
#define CAR_ROAD_DEFAULT_RPM           100 /* 道路表的默认速度，RPM；人工导航 rpm 0 时使用。 */
#define CAR_MOTION_ACCELERATION        200 /* 平移、圆弧和位置模式默认的驱动器加速度档位；与软件 RPM/s 分开。 */
#define CAR_STOP_ACCELERATION          200 /* 全局停车下发非同步零速命令时的驱动器加速度档位。 */
#define CAR_LATERAL_MAX_RPM            ((CAR_LATERAL_SHORT_RPM > CAR_LATERAL_LONG_RPM) ? CAR_LATERAL_SHORT_RPM : CAR_LATERAL_LONG_RPM) /* 派生值，RPM；由左右横移的短档、长档取较大值。 */
#define CAR_TRANSLATION_MAX_RPM        ((CAR_RUN_RPM > CAR_LATERAL_MAX_RPM) ? CAR_RUN_RPM : CAR_LATERAL_MAX_RPM) /* 派生值，RPM；由前后与横移上限取较大值。 */
#define CAR_WHEEL_MAX_RPM              ((CAR_TRANSLATION_MAX_RPM > CAR_TURN_CLOSED_LOOP_RPM) ? CAR_TRANSLATION_MAX_RPM : CAR_TURN_CLOSED_LOOP_RPM) /* 派生值，RPM；由平移与原地转向上限取较大值，另有当前动作限幅。 */

/* 2. 四向距离标定与偏移补偿
 * 先用已知距离做单方向重复测量。实走偏短时提高对应脉冲/mm，实走偏长时降低。
 * 按比例试调可用 旧值×目标距离÷实走距离，改后复测。当前左值来自 2000 mm 目标实走 1900 mm；
 * 右值由 10.986（实走 1950 mm）和 11.316（实走 2060 mm）两点插值得到。
 */
#define CALIBRATED_PPM_FORWARD      10.00f /* 普通前进距离标定，脉冲/mm；工位前后微调用 CALIBRATED_PPM_Y。 */
#define CALIBRATED_PPM_BACKWARD     10.00f /* 普通后退距离标定，脉冲/mm；工位前后微调用 CALIBRATED_PPM_Y。 */
#define CALIBRATED_PPM_LEFT         11.000f /* 普通左移距离标定，脉冲/mm；比赛、手动 l、避障共用，按该段目标方向锁定。 */
#define CALIBRATED_PPM_RIGHT        11.000f /* 普通右移距离标定，脉冲/mm；比赛、手动 r、避障共用，末端反向回拉不切换。 */
#define CALIBRATED_PPM_X            11.00f /* 共用横向标定，脉冲/mm；工位横移、多轴微调、普通直行侧偏和圆角横向换算使用。 */
#define CALIBRATED_PPM_Y            10.00f /* 共用纵向标定，脉冲/mm；工位前后微调、双轴纵向和横移前后补偿使用。 */
#define LEFT_FORWARD_MM_PER_M       0 /* 左移时前后固定补偿，前移 mm/横移 m；正向前、负向后，0 关闭。 */
#define RIGHT_FORWARD_MM_PER_M      0 /* 右移时前后固定补偿，前移 mm/横移 m；正向前、负向后，0 关闭。 */
#define CAR_FORWARD_LEFT_MM_PER_M   0 /* 前进时左右固定补偿，左移 mm/行驶 m；正向左、负向右，0 关闭。 */
#define CAR_BACKWARD_LEFT_MM_PER_M  0 /* 后退时左右固定补偿，左移 mm/行驶 m；正向左、负向右，0 关闭。 */
#define CAR_POSITION_PULSES_PER_REV 3200u /* 位置指令脉冲/圈；当前对应驱动器 16 细分，须与硬件设置一致。 */
#define CAR_ENCODER_COUNTS_PER_REV  65536u /* CPOS 编码器计数/圈；须与驱动器反馈协议一致。 */

/* 3. 闭环增益与到位
 * 各组增益对应的误差和速度单位见行尾。P 加大后纠偏更强，D 加大后对变化速度的阻尼更强。
 * 实际输出仍受各组限幅、滤波和死区约束。当前航向和正交位置使用 PD，沿程距离使用 P。
 */

/* 原地定角和普通末段航向 */
#define CAR_TURN_CCW_DIR                    1u /* 正向（左转）的驱动器方向位，0/1；四轮转向混控也用此值。 */
#define CAR_HEADING_KP_X100                 250 /* 原地定角和普通平移末段的航向 P，RPM/度×100；250 对应 2.50 RPM/度。 */
#define CAR_HEADING_KD_X100                 30 /* 通用航向 D，也用于 90° 转向最后几度，RPM/(度/秒)×100。 */
#ifndef CAR_TURN_90_KD_X100
#define CAR_TURN_90_KD_X100                 24 /* 左右 90° 原地转向前中段的 D，RPM/(度/秒)×100；减小会减少提前制动，设为通用 D 可恢复原行为。 */
#endif
#ifndef CAR_TURN_90_KD_BLEND_START_CDEG
#define CAR_TURN_90_KD_BLEND_START_CDEG    501 /* 90° 转向剩余该角度时开始恢复通用 D，0.01 度；1500 为 15°。 */
#endif
#ifndef CAR_TURN_90_KD_BLEND_END_CDEG
#define CAR_TURN_90_KD_BLEND_END_CDEG       500 /* 90° 转向剩余该角度时已恢复通用 D，0.01 度；500 为 5°，须小于恢复起点。 */
#endif
#define CAR_HEADING_MIN_RPM                 5 /* 原地定角超出角度容差时的最低修正转速，RPM。 */
#ifndef CAR_TURN_BRAKE_START_CDEG
#define CAR_TURN_BRAKE_START_CDEG          100 /* 原地转向还剩多少角度开始压低轮速上限，0.01 度；1500 为 15°，数值越大越早减速，0 关闭。 */
#endif
#ifndef CAR_TURN_BRAKE_CURVE_X100
#define CAR_TURN_BRAKE_CURVE_X100            20 /* 原地转向减速曲线陡度，0～100；0 为线性，100 为二次曲线。增大使前段降得更快、末端更缓。 */
#endif
#define CAR_MANUAL_TURN_90_OFFSET_CDEG      0 /* 手动与比赛转向每 90° 的补偿，0.01 度；180° 用两倍，正值左右均多转，0 关闭。 */
#define CAR_MANUAL_TURN_90_TOLERANCE_CDEG   20 /* left90/right90 独立到位角差，0.01 度；20 为 ±0.20°，调小可能延长收尾。 */
#define CAR_MANUAL_TURN_90_FINE_RPM          2 /* 手动 90° 进入通用到位窗后的低速补转，RPM；减小可减少来回补转。 */
#define CAR_HEADING_TOLERANCE_CDEG          80 /* 航向到位误差窗口，0.01 度；增大会放宽角度要求。 */
#define CAR_HEADING_STABLE_RATE_CDEG_S      100 /* 航向判为稳定所允许的最大角速度，0.01 度/秒。 */
#define CAR_HEADING_STABLE_SAMPLES          1u /* 航向判稳所需的连续合格新样本数，至少 1。 */
#define CAR_HEADING_STABLE_MS               0u /* 合格角度和角速度需连续保持的时间，ms；0 不加时间门槛。 */
#define CAR_HEADING_SETTLE_MS               0u /* 发零速指令后额外保持的时间，ms；0 仍须等待规定的新反馈。 */
#ifndef CAR_ROUTE_TURN_HANDOFF_CDEG
#define CAR_ROUTE_TURN_HANDOFF_CDEG        500 /* 比赛转向后接道路平移的余角上限，0.01 度；300=3°，0 关闭。 */
#endif
#ifndef CAR_ROUTE_STOP_RPM
#define CAR_ROUTE_STOP_RPM                  2 /* 比赛道路动作交接的四轮停稳阈值，RPM；2 表示 ±2 RPM。 */
#endif
#define CAR_ROUTE_TURN_HANDOFF_RATE_CDEG_S 500 /* 接道路平移前的角速度上限，0.01 度/秒；仍须四轮新反馈停稳。 */
#define CAR_HEADING_TIMEOUT_MS              20000u /* 转向或准备对齐的等待告警时间，ms；到位仍看实际反馈。 */

/* 行驶与视觉平移的固定航向增益 */
#ifndef CAR_GYRO_HEADING_DEFAULT_ON
#define CAR_GYRO_HEADING_DEFAULT_ON         1 /* 平移航向纠偏上电开关，1 开/0 关；运行时用 loop gyro on/off。 */
#endif
#define CAR_DRIVE_HEADING_KP_X100           150 /* 固定行驶航向 P，RPM/度×100；增益调度关闭时、工位视觉平移及圆角等共用。 */
#define CAR_DRIVE_HEADING_KD_X100           30 /* 固定行驶航向 D，RPM/(度/秒)×100；工位视觉平移末端也使用此组。 */
#define CAR_DRIVE_HEADING_QUIET_CDEG        20 /* 行驶航向输出归零的角度窗，0.01 度；须同时满足下项角速度窗。 */
#define CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S 50 /* 行驶航向输出归零的角速度窗，0.01 度/秒。 */
#define CAR_DRIVE_HEADING_MIN_RPM           1 /* 行驶航向非静稳区的最小非零纠偏，RPM；还需经过各模式限幅。 */
#define CAR_HEADING_DRIVE_TRIM_RPM          20 /* 行驶航向纠偏上限，RPM；低速时还受当前平移量约束。 */

/* 普通道路/手动单段的航向增益调度 */
#ifndef CAR_DRIVE_GAIN_SCHEDULE_ENABLED
#define CAR_DRIVE_GAIN_SCHEDULE_ENABLED     1 /* 普通道路/手动 f/b/l/r 的行驶航向分档，1 启用/0 用固定增益。 */
#endif
#define CAR_DRIVE_GAIN_LOW_RPM              60u /* 分档的低速结点，RPM；按主轴指令斜坡幅值插值。 */
#define CAR_DRIVE_GAIN_MID_RPM              120u /* 分档的中速结点，RPM；须介于低速与高速结点之间。 */
#define CAR_DRIVE_GAIN_HIGH_RPM             200u /* 分档的高速结点，RPM；超过此值继续使用高速档增益。 */
#define CAR_FORWARD_HEADING_KP_LOW_X100     130 /* 前后行驶低速航向 P，RPM/度×100。 */
#define CAR_FORWARD_HEADING_KD_LOW_X100     30 /* 前后行驶低速航向 D，RPM/(度/秒)×100。 */
#define CAR_FORWARD_HEADING_KP_MID_X100     130 /* 前后行驶中速航向 P，RPM/度×100。 */
#define CAR_FORWARD_HEADING_KD_MID_X100     30 /* 前后行驶中速航向 D，RPM/(度/秒)×100。 */
#define CAR_FORWARD_HEADING_KP_HIGH_X100    130 /* 前后行驶高速航向 P，RPM/度×100。 */
#define CAR_FORWARD_HEADING_KD_HIGH_X100    30 /* 前后行驶高速航向 D，RPM/(度/秒)×100。 */
#define CAR_LATERAL_HEADING_KP_LOW_X100     130 /* 左右横移低速航向 P，RPM/度×100。 */
#define CAR_LATERAL_HEADING_KD_LOW_X100     30 /* 左右横移低速航向 D，RPM/(度/秒)×100。 */
#define CAR_LATERAL_HEADING_KP_MID_X100     160 /* 左右横移中速航向 P，RPM/度×100。 */
#define CAR_LATERAL_HEADING_KD_MID_X100     30 /* 左右横移中速航向 D，RPM/(度/秒)×100。 */
#define CAR_LATERAL_HEADING_KP_HIGH_X100    180 /* 左右横移高速航向 P，RPM/度×100。 */
#define CAR_LATERAL_HEADING_KD_HIGH_X100    28 /* 左右横移高速航向 D，RPM/(度/秒)×100。 */

/* 前后行驶的左右位置纠偏 */
#ifndef CAR_STRAIGHT_HOLD_ENABLED
#define CAR_STRAIGHT_HOLD_ENABLED           1 /* 普通前进/后退的左右位置纠偏，1 编入/0 关闭此支路。 */
#endif
#ifndef CAR_ENCODER_DRIFT_DEFAULT_ON
#define CAR_ENCODER_DRIFT_DEFAULT_ON        1 /* 前后行驶的编码器横偏反馈上电开关，1 开/0 关；运行时用 loop encoder on/off。 */
#endif
#define CAR_STRAIGHT_HOLD_KP_X100           50 /* 前后行驶的左右位置 P，RPM/mm×100。 */
#define CAR_STRAIGHT_HOLD_KD_X100           10 /* 前后行驶的左右速度阻尼 D，RPM/(mm/s)×100；0 关闭 D。 */
#define CAR_STRAIGHT_HOLD_DEADBAND_MM       2 /* 正交位置纠偏死区，mm；也是独立平移到位和道路横移残差交接的阈值。 */
#define CAR_STRAIGHT_HOLD_MAX_RPM           20 /* 前后行驶的左右纠偏上限，RPM。 */

/* 左右横移的前后位置纠偏 */
#define CAR_LATERAL_HOLD_DEFAULT_ON         1 /* 普通左右横移的前后位置反馈上电开关，1 开/0 关；运行时用 loop forward on/off。 */
#define CAR_LATERAL_HOLD_KP_X100            40 /* 左右横移的前后位置 P，RPM/mm×100。 */
#define CAR_LATERAL_HOLD_KD_X100            10 /* 左右横移的前后速度阻尼 D，RPM/(mm/s)×100；0 关闭 D。 */
#define CAR_LATERAL_HOLD_RATE_FILTER_MS     100u /* 正交位置差分速度的一阶滤波时间常数，ms；增大更平滑、滞后也更大。 */
#define CAR_LATERAL_HOLD_MAX_RPM            8 /* 左右横移的前后纠偏上限，RPM。 */
#define CAR_LATERAL_HOLD_SLEW_RPM_S         60u /* 左右横移的前后纠偏变化率，RPM/s；减小会使纠偏变化更缓。 */

/* 工位驻车航向保持 */
#ifndef CAR_STATION_HOLD_DEFAULT_ON
#define CAR_STATION_HOLD_DEFAULT_ON         1 /* 驻车作业的航向保持上电开关，1 开/0 关；运行时用 loop station on/off。 */
#endif
#define CAR_STATION_HOLD_KP_X100            150 /* 驻车保持的航向 P，RPM/度×100；使用发车时建立的地图参考。 */
#define CAR_STATION_HOLD_KD_X100            30 /* 驻车保持的航向 D，RPM/(度/秒)×100。 */
#define CAR_STATION_HOLD_MAX_RPM            5 /* 驻车旋转纠偏上限，RPM；须不大于行驶航向纠偏上限。 */
#define CAR_STATION_HOLD_QUIET_CDEG         20 /* 驻车纠偏归零的角度窗，0.01 度；须同时满足下项角速度窗。 */
#define CAR_STATION_HOLD_RATE_CDEG_S        50 /* 驻车纠偏归零的角速度窗，0.01 度/秒。 */
#define CAR_STATION_HOLD_SLEW_RPM_S         20u /* 驻车旋转纠偏的指令变化率，RPM/s。 */

/* 沿程停车、末段与四轮停稳 */
#define CAR_DISTANCE_KP_X100                50 /* 沿程距离 P，RPM/mm×100；增大会提高近目标请求速度，仍受制动限速。 */
#define CAR_DISTANCE_TOLERANCE_MM           1 /* 主轴编码器到位窗口，mm；增大会放宽沿程到位要求。 */
#define CAR_DISTANCE_MIN_RPM                3 /* 常规非零平移的最低修正速度，RPM；增大会加快接近，也可能增加过冲。 */
#define CAR_LATERAL_SETTLE_MIN_RPM          1 /* 普通横移末段和回拉的最低修正速度，RPM；不大于常规最低速度。 */
#define CAR_SETTLE_DISTANCE_MM              10 /* 普通平移及视觉纠偏进入末段航向控制的剩余距离，mm；须大于到位窗口。 */
#define CAR_SETTLE_TURN_RPM_PER_S           60u /* 平移末段航向纠偏的变化率，RPM/s。 */
#define CAR_MOVEXY_FINE_MAX_RPM             6 /* 双轴视觉微调中两轴均处于低速时的门槛，RPM；小于 1 RPM 的航向修正可归零。 */
#define CAR_MOVEXY_MIN_RPM                  1 /* 双轴视觉纠偏末段允许的最低非零轴速，RPM；不大于常规最低速度。 */
#define CAR_VELOCITY_STOP_RPM               1 /* 四轮新速度反馈的停稳阈值，RPM；增大会放宽停稳判断。 */

#ifndef CAR_OBSTACLE_DEFAULT_ON
#define CAR_OBSTACLE_DEFAULT_ON 1 /* 比赛复位默认启用双超声避障；停车时用 obstacle on/off 切换。 */
#endif

/* 4. 道路模式、工位方向与地图
 * 地图 X 向右、Y 向下。地图航向 0=下(+Y)、1=左(-X)、2=上(-Y)、3=右(+X)。改地图内移或道路几何需要重生成离线路表。
 */
#ifndef CAR_RUN_OMNI_ENABLED
#define CAR_RUN_OMNI_ENABLED                1 /* 1 优先保持车头走前后/横移，0 使用下方首次/已走道路策略；工位另对齐。 */
#endif
#ifndef CAR_RUN_CORNER_ENABLED
#ifdef CHASSIS_OBSTACLE_TEST
#define CAR_RUN_CORNER_ENABLED              0
#else
#define CAR_RUN_CORNER_ENABLED              1 /* 比赛的路径换轴采用连续圆角及起终点渐变。 */
#endif
#endif
#ifndef CAR_RUN_CORNER_YAW_BLEND_ENABLED
#define CAR_RUN_CORNER_YAW_BLEND_ENABLED    0 /* 0 停稳后原地转头，1 允许平移时切换目标航向。 */
#endif
#ifndef CAR_ARM_TRANSLATION_OVERLAP_ENABLED
#define CAR_ARM_TRANSLATION_OVERLAP_ENABLED 1 /* 1 允许工位与底盘在既有净空和收拢条件下并行，0 等待原串行交接。 */
#endif
#define CAR_RUN_REVERSE_ENABLED             1 /* 全向关闭时生效；1 允许本轮已完整走过的道路直接后退，0 禁用。 */
#define CAR_RUN_REVISIT_LATERAL_ENABLED     1 /* 全向关闭时生效；1 允许已走道路用横移代替转头，0 禁用。 */
#define CAR_RUN_START_LATERAL_ENABLED       1 /* 全向关闭时生效；1 允许 A/C 发车首段保持初始航向横移，0 禁用。 */
#define CAR_RUN_RETURN_LATERAL_ENABLED      1 /* 全向关闭时生效；1 允许最终经 H→E→B 返回时保持离站航向，0 禁用。 */
#define CAR_MAP_CENTER_X_MM                1180 /* D/E/F 共用的图面 X 坐标，mm；X 向 B 增大。 */
#define CAR_MAP_LEFT_X_MM                   360 /* G/H/I 的左侧基准 X 坐标，mm；X 向 B 增大。 */
#define CAR_MAP_TOP_INSET_MM                90 /* 上侧 F/G 基础坐标沿 +Y 内移，mm；范围 0～999。 */
#define CAR_MAP_BOTTOM_INSET_MM           0 /* 下侧 D/I 进站地图沿 −Y 的内移量，mm；改动后重生成离线路表。 */
#define CAR_MAP_RAW_EXTRA_INSET_MM            0 /* F 在上侧基础偏移上的有符号 Y 增量，mm；负值向 −Y，F 的总内移须在 0～999。 */
#define CAR_MAP_STORAGE_INSET_MM             0 /* H 相对 CAR_MAP_LEFT_X_MM 沿 +X 内移，mm；避障 I/G 通行线随 H 对齐，0 时 I/H/G 共线。 */
#define CAR_START_INSET_MM                  70 /* A/C 实际摆车位置到内侧行驶线的横向距离，mm；发车和最终回程并入轨迹。 */
#ifndef CAR_MAP_SCAN_TOWARD_A_MM
#define CAR_MAP_SCAN_TOWARD_A_MM          120 /* 扫码位置沿右侧通道相对 B 中点朝 A 的距离，mm；不移动地图节点。 */
#endif
#define CAR_RAW_DEPARTURE_EXTRA_MM          80 /* 原料视觉定位后，F→E 比赛离站在地图距离上追加的 mm。 */
#define CAR_ROUGH_DEPARTURE_EXTRA_MM        45 /* 粗加工两批使用 35mm 平推时，离站沿 −Y 补回的视觉靠环偏移，mm。0 关闭。 */
#define CAR_STORAGE_DEPARTURE_EXTRA_MM      50 /* 两批暂存 H→E 原有离站横移补偿，mm，沿 +X。 */
#define CAR_STORAGE_FIRST_DEPARTURE_EXTRA_MM 45 /* 第一次暂存使用 35mm 平推时，离站沿 +X 额外补回的靠环偏移，mm。叠加在原 50mm 上。 */
#define CAR_MAP_YAW_SIGN                    -1 /* 地图航向增加映射到 IMU 的角度方向，±1；当前与左转角方向相反。 */
#define CAR_STATION_RAW_HEADING             1u /* 原料站车头地图方向编号，0～3；同时决定右侧机械臂朝向。 */
#define CAR_STATION_ROUGH_HEADING           3u /* 粗加工站车头地图方向编号，0～3；同时决定右侧机械臂朝向。 */
#define CAR_STATION_STORAGE_HEADING         0u /* 暂存站车头地图方向编号，0～3；同时决定右侧机械臂朝向。 */
#define CAR_ROUTE_VISUAL_WINDOW_MM          50 /* 路段末端交给视觉前允许的近站残差窗口，mm；仍有路径和反馈门槛。 */

/* 5. 连续圆角与过弯
 * 以下圆角参数在启用连续道路时使用。现场空间、轮速饱和和实际净空需单独核对。
 */
#define CAR_CORNER_RADIUS_MM                150 /* 圆角半径上限，mm；范围 50～300，短边会限制实际半径。 */
#define CAR_ROUGH_DETOUR_RING2_RADIUS_MM      6 /* 已走 D→I 绕行回 2 环的首个圆角半径上限，mm；145 mm 回程在约 140 mm 开始横移。 */
#define CAR_CORNER_SPEED_MM_S               180 /* 同步转头圆弧的参考速度上限，mm/s；范围 30～300。 */
#define CAR_CORNER_FIXED_YAW_SPEED_MM_S     250 /* 车头保持不变的圆弧参考速度上限，mm/s；范围 30～300。 */
#define CAR_CORNER_WHEEL_ACCEL_RPM_PER_S    300u /* 同步转头圆弧内单轮升速率，RPM/s。 */
#define CAR_CORNER_WHEEL_DECEL_RPM_PER_S    300u /* 同步转头圆弧内单轮降速或过零率，RPM/s；终点制动另沿用道路参数。 */
#ifndef CAR_CORNER_EXIT_BLEND_ENABLED
#define CAR_CORNER_EXIT_BLEND_ENABLED       1 /* 1 出弯按实测角差逐步恢复升速，0 关闭该衔接。 */
#endif
#define CAR_CORNER_EXIT_YAW_CDEG            1500 /* 进入出弯恢复允许的实测航向残差上界，0.01 度。 */
#define CAR_CORNER_EXIT_ALIGNED_CDEG        800 /* 出弯恢复的接近对正角差，0.01 度；也用于几何剩余转角和恢复阶段比例折速。 */
#define CAR_CORNER_EXIT_ACCEL_RPM_PER_S     150u /* 出弯衔接起始升速率，RPM/s。 */
#define CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S 450u /* 出弯接近对正时的最大升速率，RPM/s；不得小于起始值。 */
#define CAR_CORNER_YAW_GAIN_X100            115 /* 圆弧转向前馈换算，(度/秒)/RPM×100；增大系数会降低同一参考角速度对应的前馈 RPM。 */
#define CAR_CORNER_YAW_MAX_RPM              100 /* 圆弧转向分量限幅，RPM；仍受动作档位和变化率约束。 */
#define CAR_CORNER_TRACK_GAIN_X100          250 /* 圆弧路径位置反馈与末段比例减速的增益，(mm/s)/mm×100。 */
#define CAR_CORNER_TRACK_MAX_MM_S           80 /* 圆弧位置纠偏的速度上限，mm/s。 */
#define CAR_CORNER_MIN_MM_S                 5.2f /* 圆弧末端仍需推进时的最低非零参考速度，mm/s。 */
#define CAR_CORNER_SLOW_YAW_CDEG            300 /* 圆弧常规航向误差的比例折速阈值，0.01 度；越大越晚开始折速。 */
#define CAR_CORNER_RETURN_MM_S              50 /* obstacle 后沿曲线路径退回的参考速度，mm/s；也受返回轮速上限约束。 */
#define CAR_START_BLEND_MM                  600 /* 比赛发车和 B→E 离开扫码区时的并行移动长度，mm。从起步叠加偏移，增大使偏移分布到更长的主段。 */
#define CAR_RETURN_BLEND_MM                 600 /* 比赛经 B、F 或 D 回 A/C 时，入区偏移并入主段尾部的渐变长度，mm，含避障开启。 */

/* 6. IMU、轮反馈与诊断 */
#ifndef CAR_IMU_UART_ENABLED
#define CAR_IMU_UART_ENABLED         1 /* 1 启用 USART2 被动接收 HWT101 数据，0 关闭；可由构建覆盖。 */
#endif
#define CAR_IMU_UART_BAUD            230400u /* HWT101 的 USART2 波特率，bit/s；模块沿用 50 Hz 输出设置。 */
#define CAR_IMU_YAW_ONLY             1 /* HWT101 沿用旧程序，仅取 0x53 Yaw，角速度用相邻 Yaw 直接差分。 */
#define CAR_IMU_YAW_SIGN             1 /* IMU 角度及角速度的方向符号，±1；车身左转时逻辑角应增加。 */
#define CAR_IMU_MAX_AGE_MS           250u /* IMU 航向样本允许的最大年龄，ms；增大允许使用更早的样本。 */
#define CAR_IMU_RATE_ENABLED         (!CAR_IMU_YAW_ONLY) /* 仅多轴模块使用 0x52 作 D 项。 */
#define CAR_IMU_RATE_MAX_AGE_MS      100u /* 0x52 角速度样本允许的最大年龄，ms；须不大于航向样本年龄上限。 */
#define CAR_IMU_RATE_FILTER_MS       30u /* IMU 角速度 D 项的一阶滤波时间常数，ms；增大更平滑、滞后也更大。 */
#ifndef CHASSIS_OBSTACLE_TEST
/* 横移追加距离实验。 */
#define CAR_ACCEL_EXTRA_DEFAULT_ON      0u /* HWT101 只取 Yaw，横移追加距离实验关闭。 */
#define CAR_ACCEL_EXTRA_CAL_SAMPLES     8u
#define CAR_ACCEL_EXTRA_CAL_TIMEOUT_MS  600u
#define CAR_ACCEL_EXTRA_MAX_AGE_MS      100u
#define CAR_ACCEL_EXTRA_TILT_CDEG       150 /* 相对静止校零姿态允许的横滚/俯仰变化。 */
#define CAR_ACCEL_EXTRA_NOISE_MG        20
#define CAR_ACCEL_EXTRA_V_LEAK_MS       300u
#define CAR_ACCEL_EXTRA_POS_LEAK_MS     6000u
#define CAR_ACCEL_EXTRA_MAX_SPEED_MM_S  500.0f /* 残差积分速度限幅。 */
#define CAR_ACCEL_EXTRA_DEADBAND_MM     80
#define CAR_ACCEL_EXTRA_GAIN            1.60f
#define CAR_ACCEL_EXTRA_LIMIT_MM        300u
#endif
#define CAR_IMU_MAX_TILT_CDEG        3000 /* 姿态诊断的倾斜阈值，0.01 度；倾斜过大时 D 项使用 Yaw 差分。 */
#define CAR_DRIVE_SLEW_MAX_DT_MS     50u /* 单次速度斜坡计算允许计入的最大时间步长，ms；避免调度延迟造成指令突变。 */
#define CAR_VELOCITY_CONTROL_MS      25u /* 轮速控制和发送的目标更新周期，ms；至少 10。 */
#define CAR_VELOCITY_QUERY_MS        5u /* 四轮位置、转速、状态的串行查询间隔，ms；减小会增加 CAN 流量。 */
#define CAR_VELOCITY_REPLY_MS        150u /* 单次查询的应答等待窗口，ms。 */
#define CAR_VELOCITY_FEEDBACK_MS     300u /* 轮反馈允许的最大年龄，ms；须大于单次查询应答窗口。 */
#define CAR_VELOCITY_ACK_MS          150u /* 轮速命令 ACK 等待窗口，ms；也用于驻车航向保持的 ACK 检查。 */
#define CAR_VELOCITY_STALL_MS        1500u /* 非零轮速指令发出且位置长期无变化时的告警阈值，ms。 */
#define CAR_ENCODER_LEARN_MM         2 /* 学习 CPOS 极性所需的最小同向位移幅度，mm；还要有连续新样本。 */
#define CAR_ENCODER_LEARN_RPM        20 /* 四轮方向未确认时的平移速度上限，RPM。 */
#define CAR_ENCODER_LEARN_TIMEOUT_MS 1500u /* ms；当前仅用于编译期约束，运行中没有使用此值判断超时。 */
#define CAR_DISTANCE_OVERRUN_MM      500 /* 旧距离超程阈值，mm；当前主控未引用此宏，现场调整不会生效。 */
#define CAR_DRIVE_TIMEOUT_MS         30000u /* 普通平移段的等待告警周期，ms；动作完成仍看实际反馈。 */
#define CAR_HEADING_MAX_DRIFT_CDEG   1500 /* 平移过程中航向偏差告警阈值，0.01 度。 */
#define CAR_CONTROL_LOG_MS           500u /* 底盘控制日志输出周期，ms；减小会增加打印量。 */
#define CAR_MOTION_MIN_ESTIMATE_MS   200u /* 人工道路位置模式估时下限，ms；估时不作到位证据。 */
#define CAR_MOTION_TIMEOUT_FACTOR    3u /* 人工道路位置模式的超时估算倍率，无单位。 */
#define CAR_MOTION_TIMEOUT_EXTRA_MS  5000u /* 人工道路位置模式的超时估算额外余量，ms。 */

/* 7. 启动按钮、调试串口与扫码 */
#define CAR_START_BUTTON_GPIO          GPIOC /* 启动按钮端口；换口须同时改 GPIO 时钟、引脚及接线。 */
#define CAR_START_BUTTON_GPIO_CLOCK    RCC_AHB1Periph_GPIOC /* 启动按钮所在 GPIO 端口的 AHB 时钟位，与端口同步修改。 */
#define CAR_START_BUTTON_PIN           GPIO_Pin_9 /* 启动按钮引脚掩码；当前 PC9，内部上拉，按钮接 GND，低电平按下。 */
#define CAR_START_BUTTON_DEBOUNCE_MS   30u /* 按下和松开各自需要维持的消抖时间，ms。 */
#define CAR_START_BUTTON_LONG_PRESS_MS 1000u /* 短按 C、长按 A 的分界，ms；两者均在松开消抖后启动。 */
#define CAR_DEBUG_UART_BAUD            115200 /* 无线调试软件串口 U6 的波特率，bit/s；PB0 TX、PB1 RX，须匹配无线模块。 */
#define CAR_SCANNER_UART_BAUD          9600u /* XM1601 的 USART1 波特率，bit/s；PB6 TX、PB7 RX，须匹配扫码器。 */
#define CAR_SCANNER_BOOT_MS            1000u /* 扫码器初始化前的启动等待，ms；增大让扫码器有更多上电准备时间。 */
#define CAR_SCANNER_TX_MS              1000u /* 单条扫码配置发送的最长等待，ms。 */
#define CAR_SCANNER_ACK_MS             1000u /* 单条扫码配置应答的最长等待，ms。 */

/* 派生换算与日志使用的内部辅助。现场直接改上方参数。 */
static inline uint16_t car_lateral_limit_rpm(uint32_t distance_mm)
{
    return distance_mm <= CAR_LATERAL_SHORT_MM ? CAR_LATERAL_SHORT_RPM : CAR_LATERAL_LONG_RPM;
}

static inline float car_road_ppm_x(float left)
{
    return left < 0.0f ? CALIBRATED_PPM_RIGHT : CALIBRATED_PPM_LEFT;
}

static inline float car_road_ppm_y(float forward)
{
    return forward < 0.0f ? CALIBRATED_PPM_BACKWARD : CALIBRATED_PPM_FORWARD;
}

static inline float car_lateral_forward_pulses(float left_mm)
{
    int16_t per_m=left_mm>0 ? LEFT_FORWARD_MM_PER_M : RIGHT_FORWARD_MM_PER_M;
    return (left_mm<0 ? -left_mm : left_mm)*per_m*CALIBRATED_PPM_Y/1000.0f;
}

static inline float car_straight_left_trim(float forward)
{
    return forward < 0.0f ? -forward * CAR_BACKWARD_LEFT_MM_PER_M / 1000.0f
                          : forward * CAR_FORWARD_LEFT_MM_PER_M / 1000.0f;
}

#define CAR_STRING_VALUE_INNER(value) #value
#define CAR_STRING_VALUE(value) CAR_STRING_VALUE_INNER(value)

/* 编译期配置约束 */
#if CAR_ROUGH_BRAKE_LOOKAHEAD_MS < 100 || CAR_ROUGH_BRAKE_LOOKAHEAD_MS > 1000
#error Invalid_rough_brake_lookahead
#endif

#if CAR_MAP_TOP_INSET_MM < 0 || CAR_MAP_TOP_INSET_MM >= 1000 || \
    CAR_MAP_BOTTOM_INSET_MM <= -170 || CAR_MAP_BOTTOM_INSET_MM >= 860 || \
    CAR_MAP_RAW_EXTRA_INSET_MM < -CAR_MAP_TOP_INSET_MM || CAR_MAP_TOP_INSET_MM + CAR_MAP_RAW_EXTRA_INSET_MM >= 1000 || \
    CAR_MAP_STORAGE_INSET_MM < 0 || CAR_MAP_STORAGE_INSET_MM >= CAR_MAP_CENTER_X_MM - CAR_MAP_LEFT_X_MM || \
    CAR_ROUGH_DEPARTURE_EXTRA_MM < 0 || CAR_STORAGE_FIRST_DEPARTURE_EXTRA_MM < 0 || \
    CAR_MAP_SCAN_TOWARD_A_MM < 0 || CAR_MAP_SCAN_TOWARD_A_MM >= 1030
#error Invalid_map_inset
#endif

#if CAR_TURN_ACCELERATION < 0 || CAR_TURN_ACCELERATION > 255
#error Invalid_turn_acceleration
#endif
#if CAR_LOADED_TURN_RPM < 20 || CAR_LOADED_TURN_RPM > CAR_TURN_CLOSED_LOOP_RPM || \
    CAR_LOADED_TURN_ACCELERATION < 0 || CAR_LOADED_TURN_ACCELERATION > 255
#error Invalid_loaded_turn_configuration
#endif
#if CAR_TURN_BRAKE_START_CDEG < 0 || CAR_TURN_BRAKE_START_CDEG > 18000 || \
    CAR_TURN_BRAKE_CURVE_X100 < 0 || CAR_TURN_BRAKE_CURVE_X100 > 100
#error Invalid_turn_braking_configuration
#endif
#if CAR_TURN_90_KD_X100 < 0 || CAR_TURN_90_KD_BLEND_END_CDEG < 0 || \
    CAR_TURN_90_KD_BLEND_START_CDEG <= CAR_TURN_90_KD_BLEND_END_CDEG || CAR_TURN_90_KD_BLEND_START_CDEG > 9000
#error Invalid_90_degree_turn_damping_configuration
#endif

#if (CAR_IMU_YAW_SIGN != 1 && CAR_IMU_YAW_SIGN != -1) || \
    (CAR_MAP_YAW_SIGN != 1 && CAR_MAP_YAW_SIGN != -1) || \
    (CAR_RUN_REVERSE_ENABLED != 0 && CAR_RUN_REVERSE_ENABLED != 1) || \
    (CAR_RUN_REVISIT_LATERAL_ENABLED != 0 && CAR_RUN_REVISIT_LATERAL_ENABLED != 1) || \
    (CAR_RUN_START_LATERAL_ENABLED != 0 && CAR_RUN_START_LATERAL_ENABLED != 1) || \
    (CAR_RUN_RETURN_LATERAL_ENABLED != 0 && CAR_RUN_RETURN_LATERAL_ENABLED != 1) || \
    CAR_LATERAL_MAX_RPM < 20 || CAR_LATERAL_MAX_RPM > CAR_WHEEL_MAX_RPM || \
    CAR_OBSTACLE_RETURN_RPM < 20 || CAR_OBSTACLE_RETURN_RPM > CAR_WHEEL_MAX_RPM || \
    CAR_HEADING_TOLERANCE_CDEG < 1 || CAR_HEADING_STABLE_SAMPLES < 1 || \
    CAR_MANUAL_TURN_90_OFFSET_CDEG <= -9000 || CAR_MANUAL_TURN_90_OFFSET_CDEG >= 9000 || \
    CAR_MANUAL_TURN_90_TOLERANCE_CDEG < 1 || CAR_MANUAL_TURN_90_TOLERANCE_CDEG > CAR_HEADING_TOLERANCE_CDEG || \
    CAR_MANUAL_TURN_90_FINE_RPM < 1 || CAR_MANUAL_TURN_90_FINE_RPM > CAR_HEADING_MIN_RPM || \
    CAR_ROUTE_TURN_HANDOFF_CDEG < 0 || CAR_ROUTE_TURN_HANDOFF_CDEG > 500 || CAR_ROUTE_TURN_HANDOFF_RATE_CDEG_S < 1 || \
    CAR_ROUTE_STOP_RPM < 0 || \
    (CAR_IMU_RATE_ENABLED != 0 && CAR_IMU_RATE_ENABLED != 1) || \
    CAR_IMU_RATE_MAX_AGE_MS < 1 || CAR_IMU_RATE_MAX_AGE_MS > CAR_IMU_MAX_AGE_MS || \
    CAR_IMU_RATE_FILTER_MS < 1 || \
    CAR_IMU_MAX_TILT_CDEG < 100 || CAR_IMU_MAX_TILT_CDEG >= 9000 || \
    CAR_HEADING_KP_X100 < 1 || CAR_VELOCITY_CONTROL_MS < 10 || \
    CAR_DRIVE_HEADING_KP_X100 < 1 || CAR_DRIVE_HEADING_KD_X100 < 0 || \
    CAR_DRIVE_HEADING_QUIET_CDEG < 1 || CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S < 1 || \
    CAR_DRIVE_HEADING_MIN_RPM < 1 || CAR_DRIVE_HEADING_MIN_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_SETTLE_TURN_RPM_PER_S < 1 || CAR_SETTLE_DISTANCE_MM <= CAR_DISTANCE_TOLERANCE_MM || \
    CAR_LATERAL_SETTLE_MIN_RPM < 1 || CAR_LATERAL_SETTLE_MIN_RPM > CAR_DISTANCE_MIN_RPM || \
    CAR_MOVEXY_FINE_MAX_RPM < 1 || CAR_MOVEXY_FINE_MAX_RPM > CAR_ENCODER_LEARN_RPM || \
    CAR_MOVEXY_MIN_RPM < 1 || CAR_MOVEXY_MIN_RPM > CAR_DISTANCE_MIN_RPM || \
    CAR_DRIVE_ACCEL_RPM_PER_S < 1 || CAR_LATERAL_ACCEL_RPM_PER_S < 1 || CAR_ROUTE_ACCEL_RPM_PER_S < 1 || CAR_DRIVE_DECEL_RPM_PER_S < 1 || \
    CAR_DRIVE_SLEW_MAX_DT_MS < CAR_VELOCITY_CONTROL_MS || CAR_DRIVE_SLEW_MAX_DT_MS > CAR_VELOCITY_REPLY_MS || \
    CAR_VELOCITY_QUERY_MS < 1 || CAR_VELOCITY_FEEDBACK_MS <= CAR_VELOCITY_REPLY_MS || \
    CAR_POSITION_PULSES_PER_REV < 1 || CAR_DISTANCE_TOLERANCE_MM < 1 || \
    CAR_ENCODER_LEARN_MM < 1 || CAR_ENCODER_LEARN_RPM < CAR_DISTANCE_MIN_RPM || \
    CAR_ENCODER_LEARN_RPM > CAR_LATERAL_MAX_RPM || CAR_ENCODER_LEARN_TIMEOUT_MS < CAR_VELOCITY_FEEDBACK_MS
#error Invalid_heading_configuration
#endif

#if (CAR_DRIVE_GAIN_SCHEDULE_ENABLED != 0 && CAR_DRIVE_GAIN_SCHEDULE_ENABLED != 1) || \
    CAR_DRIVE_GAIN_LOW_RPM < 1 || CAR_DRIVE_GAIN_LOW_RPM >= CAR_DRIVE_GAIN_MID_RPM || \
    CAR_DRIVE_GAIN_MID_RPM >= CAR_DRIVE_GAIN_HIGH_RPM || \
    CAR_FORWARD_HEADING_KP_LOW_X100 < 1 || CAR_FORWARD_HEADING_KP_MID_X100 < 1 || CAR_FORWARD_HEADING_KP_HIGH_X100 < 1 || \
    CAR_LATERAL_HEADING_KP_LOW_X100 < 1 || CAR_LATERAL_HEADING_KP_MID_X100 < 1 || CAR_LATERAL_HEADING_KP_HIGH_X100 < 1 || \
    CAR_FORWARD_HEADING_KD_LOW_X100 < 0 || CAR_FORWARD_HEADING_KD_MID_X100 < 0 || CAR_FORWARD_HEADING_KD_HIGH_X100 < 0 || \
    CAR_LATERAL_HEADING_KD_LOW_X100 < 0 || CAR_LATERAL_HEADING_KD_MID_X100 < 0 || CAR_LATERAL_HEADING_KD_HIGH_X100 < 0
#error Invalid_drive_gain_schedule_configuration
#endif

#if CAR_LATERAL_DECEL_RPM_PER_S < 1 || CAR_LATERAL_BRAKE_LOOKAHEAD_MS < 1
#error Invalid_lateral_braking_configuration
#endif

#if CAR_STRAIGHT_HOLD_ENABLED < 0 || CAR_STRAIGHT_HOLD_ENABLED > 1 || \
    CAR_ENCODER_DRIFT_DEFAULT_ON < 0 || CAR_ENCODER_DRIFT_DEFAULT_ON > 1 || \
    CAR_GYRO_HEADING_DEFAULT_ON < 0 || CAR_GYRO_HEADING_DEFAULT_ON > 1 || \
    CAR_STRAIGHT_HOLD_KP_X100 < 1 || CAR_STRAIGHT_HOLD_KD_X100 < 0 || CAR_STRAIGHT_HOLD_DEADBAND_MM < 1 || \
    CAR_STRAIGHT_HOLD_MAX_RPM < 1 || CAR_STRAIGHT_HOLD_MAX_RPM >= CAR_WHEEL_MAX_RPM
#error Invalid_straight_hold_configuration
#endif

#if CAR_LATERAL_HOLD_DEFAULT_ON < 0 || CAR_LATERAL_HOLD_DEFAULT_ON > 1 || \
    CAR_LATERAL_HOLD_KP_X100 < 1 || CAR_LATERAL_HOLD_SLEW_RPM_S < 1 || \
    CAR_LATERAL_HOLD_KD_X100 < 0 || CAR_LATERAL_HOLD_RATE_FILTER_MS < 1 || \
    CAR_LATERAL_HOLD_MAX_RPM < 1 || CAR_LATERAL_HOLD_MAX_RPM >= CAR_WHEEL_MAX_RPM
#error Invalid_lateral_hold_configuration
#endif

#if CAR_STATION_HOLD_DEFAULT_ON < 0 || CAR_STATION_HOLD_DEFAULT_ON > 1 || \
    CAR_STATION_HOLD_KP_X100 < 1 || CAR_STATION_HOLD_KD_X100 < 0 || \
    CAR_STATION_HOLD_MAX_RPM < 1 || CAR_STATION_HOLD_MAX_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_STATION_HOLD_QUIET_CDEG < 1 || CAR_STATION_HOLD_RATE_CDEG_S < 1 || CAR_STATION_HOLD_SLEW_RPM_S < 1
#error Invalid_station_hold_configuration
#endif

#if CAR_STATION_RAW_HEADING < 0 || CAR_STATION_RAW_HEADING > 3 || \
    CAR_STATION_ROUGH_HEADING < 0 || CAR_STATION_ROUGH_HEADING > 3 || \
    CAR_STATION_STORAGE_HEADING < 0 || CAR_STATION_STORAGE_HEADING > 3
#error Invalid_station_heading
#endif

#if CAR_RUN_CORNER_ENABLED < 0 || CAR_RUN_CORNER_ENABLED > 1 || \
    CAR_RUN_CORNER_YAW_BLEND_ENABLED < 0 || CAR_RUN_CORNER_YAW_BLEND_ENABLED > 1 || \
    CAR_RETURN_BLEND_MM < 1 || CAR_START_INSET_MM < 1 || CAR_START_BLEND_MM < 1 || CAR_ROUTE_VISUAL_WINDOW_MM < 1 || \
    CAR_CORNER_RADIUS_MM < 50 || CAR_CORNER_RADIUS_MM > 300 || \
    CAR_ROUGH_DETOUR_RING2_RADIUS_MM < 1 || CAR_ROUGH_DETOUR_RING2_RADIUS_MM > CAR_CORNER_RADIUS_MM || \
    CAR_CORNER_SPEED_MM_S < 30 || CAR_CORNER_SPEED_MM_S > 300 || \
    CAR_CORNER_WHEEL_ACCEL_RPM_PER_S < 1 || CAR_CORNER_WHEEL_DECEL_RPM_PER_S < 1 || \
    CAR_CORNER_EXIT_BLEND_ENABLED < 0 || CAR_CORNER_EXIT_BLEND_ENABLED > 1 || \
    CAR_CORNER_EXIT_ALIGNED_CDEG < 1 || CAR_CORNER_EXIT_YAW_CDEG <= CAR_CORNER_EXIT_ALIGNED_CDEG || \
    CAR_CORNER_EXIT_YAW_CDEG > 9000 || CAR_CORNER_EXIT_ACCEL_RPM_PER_S < 1 || \
    CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S < CAR_CORNER_EXIT_ACCEL_RPM_PER_S || \
    CAR_CORNER_FIXED_YAW_SPEED_MM_S < 30 || CAR_CORNER_FIXED_YAW_SPEED_MM_S > 300 || \
    CAR_CORNER_YAW_GAIN_X100 < 1 || CAR_CORNER_YAW_MAX_RPM < 1 || \
    CAR_CORNER_YAW_MAX_RPM > CAR_WHEEL_MAX_RPM || CAR_CORNER_TRACK_GAIN_X100 < 1 || \
    CAR_CORNER_TRACK_MAX_MM_S < 1
#error Invalid_corner_configuration
#endif

/*
 * 需要联动的调整
 *
 * 1. 地图和路线
 *    改单条道路长度、分段或方向时，编辑 structure.h 的 road_motion_table，并核对正向、
 *    反向和总长；涉及节点位置时，同步检查 road_map.c 的 nodes。A/C 全程节点与作业顺序
 *    编辑 full_route.h 的 full_route_nodes/full_route_jobs，并核对节点相邻、作业对应正确。
 *    改 CAR_MAP_* 或道路几何后，从工程根运行 .\tools\rebuild-route-table.ps1，再重新编译。
 *    生成脚本会校验几何并验证离线路表；生成文件 lookup/table_data.h 无需手填。
 *    只改速度、闭环增益、距离脉冲系数或工位姿态时，重新编译即可。
 *
 * 2. 现场底盘标定
 *    手动用 test、dist N、rpm N 与 f/b/l/r 逐方向测距离；用 loop status 查看开关。
 *    核对轮位和反馈后，先调四向脉冲/mm，再调固定偏移补偿和相应的闭环。
 *    比赛与 diagnostics/ultrasonic-chassis 共用本配置；各自固件都需要重新构建。
 *    diagnostics/jy901b-chassis 有自己的 src/car_config.h，使用该固件时在那份单独回填。
 *    若更换/重编号电机，先在驱动器设置地址与细分，再核对 motion.c 的 wheels、
 *    main_table_nav.c 的四轮发令/反馈地址和混控方向。驱动器内部控制参数按对应型号
 *    配置方法调整，随后同一方向低速复测指令和 CPOS 反馈。当前 runimu 使用本文件的
 *    闭环增益；vendor/stm32/Hardware/pid.c 是旧控制实现。
 *
 * 3. 外设接口
 *    IMU 波特率同时改模块和 CAR_IMU_UART_BAUD；改接线时检查 vendor/stm32/imu/imu.c
 *    的 imu_init（USART2、PD5 TX/PD6 RX）并用 imu 和 feedback 日志确认新鲜数据和转向符号。
 *    扫码器换接口时核对 main_table_nav.c 的 scanner_init/scanner_commands；扫码器与
 *    串口波特率一致，用 qr init 和 qr 查配置应答，再扫一组有效任务码。
 *    B 点当前沿路线航向继续扫码，见 main_table_nav.c 的 run_advance_segment。
 *    屏幕刷新与轮位映射在 hmi_status.h；更换串口屏时同步核对 hmi_status.c 和屏工程。
 *    超声起步扫描的探头位置、外廓和采样参数在 start_scan.c；重新量测装车尺寸，
 *    用已知距离和角度核查采样，再核对道路候选。自动避障的测距门槛、探头位置和
 *    路线估时在 diagnostics/ultrasonic-chassis/obstacle_io.h，比赛与避障诊断共用。
 *    改探头接线须同时核对比赛的 start_scan.c 和诊断的 ultrasonic-dual-wireless/main.c。
 */

#endif
