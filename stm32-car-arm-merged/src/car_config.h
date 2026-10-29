#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include <stdint.h>

/*
 * 小车调试参数入口：日常调速、距离标定、横移预补偿优先改本文件。
 * 改完重新编译、下载、复位运行。
 * 单条道路的距离/方向在 structure.h，全程节点顺序在 full_route.h。
 */

/* 1. 常用速度与加速度，RPM 为电机转速，驱动器加速度档位为200（与下面RPM/s速度斜坡不同）。 */
#define CAR_TEST_RPM                60       /* test 模式上电默认速度；可用 rpm 命令改。 */
#define CAR_RUN_RPM                 250      /* 前进/后退上限，RPM；不按距离分档。 */
#define CAR_LATERAL_SHORT_MM       1200      /* 连续同向横移总长不超过此值使用短档，mm；不随剩余距离切档。 */
#define CAR_LATERAL_SHORT_RPM        250      /* 短距离左右横移上限，RPM。 */
#define CAR_LATERAL_LONG_RPM         200      /* 长距离左右横移上限，RPM。 */
#ifndef CAR_ARM_TRANSLATION_OVERLAP_ENABLED
#define CAR_ARM_TRANSLATION_OVERLAP_ENABLED 1 /* 1=工位离物净空后索引与车载搬运并行，收拢后末尾旋转可与道路并行；0=原串行取放/平移等旋转。 */
#endif
#ifndef CAR_RUN_OMNI_ENABLED
#define CAR_RUN_OMNI_ENABLED          1       /* 无障碍路线：所有道路保持当前车头，直接前后/横移；工位和扫码朝向另行对准。 */
#endif
#ifndef CAR_RUN_CORNER_ENABLED
#define CAR_RUN_CORNER_ENABLED        1       /* 非作业拐角连续通过；到下一作业区需改车头时在弧中转向。0恢复分段走法。 */
#endif
#define CAR_CORNER_RADIUS_MM        150       /* 圆角半径上限，mm；短边自动取至相邻边各一半，允许切角。 */
#define CAR_CORNER_SPEED_MM_S       180       /* 同步转动车头的圆弧参考上限，mm/s；与固定车头及直线分开调。 */
#define CAR_CORNER_WHEEL_ACCEL_RPM_PER_S 300u /* 仅同步转头圆弧内的轮速上升率，RPM/s。 */
#define CAR_CORNER_WHEEL_DECEL_RPM_PER_S 300u /* 仅同步转头圆弧内的轮速下降/过零率，RPM/s；终点制动仍用道路减速率。 */
#ifndef CAR_CORNER_EXIT_BLEND_ENABLED
#define CAR_CORNER_EXIT_BLEND_ENABLED 1     /* 转头尾段先温和升速，再随车头对正恢复原加速度；0恢复r94。 */
#endif
#define CAR_CORNER_EXIT_YAW_CDEG   1500      /* 圆弧结束后，实际车头距最终方向不超过15度时进入出弯衔接。 */
#define CAR_CORNER_EXIT_ALIGNED_CDEG 800     /* 几何转向剩5度起恢复；实际偏差≤5度时用出弯最大升速率并解除比例折速。 */
#define CAR_CORNER_EXIT_ACCEL_RPM_PER_S 150u /* 出弯尚未对正时平移升速率起值，RPM/s。 */
#define CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S 450u /* 车头基本对正后的出弯升速率，RPM/s；不改变其他直线及停车减速率。 */
#define CAR_CORNER_FIXED_YAW_SPEED_MM_S 250  /* 固定车头的前后/横移换向弧段参考上限，mm/s。 */
#define CAR_CORNER_YAW_GAIN_X100    115       /* 转向前馈(度/秒)/RPM×100，来自主机模型，尚待实测。 */
#define CAR_CORNER_YAW_MAX_RPM      100       /* 圆角转向分量上限，RPM；仍受当前运动档位及变化率约束。 */
#define CAR_CORNER_TRACK_GAIN_X100  250       /* 路径横向/终点位置反馈，(mm/s)/mm×100。 */
#define CAR_CORNER_TRACK_MAX_MM_S    80       /* 路径位置修正速度上限，mm/s。 */
#define CAR_CORNER_MIN_MM_S        5.2f       /* 末端非零修正约1RPM；避免整数RPM量化后停在距离窗外。 */
#define CAR_CORNER_SLOW_YAW_CDEG    300       /* 跟踪误差超过3度时按比例降低平移速度；不是停机门槛。 */
#define CAR_CORNER_RETURN_MM_S       50       /* 外部obstacle请求后的曲线退回速度，mm/s；单轮仍不超过既有返回RPM上限。 */
#define CAR_C_RETURN_BLEND_MM       600       /* C回程：出B弯后，在此前进距离内平滑并入原60mm侧移，余下路程只直行。 */
#define CAR_START_BLEND_MM          600       /* A/C发车：在首段1030mm的前600mm内平滑完成60mm内移，B->E保持990mm。 */
#define CAR_ROUTE_VISUAL_WINDOW_MM   50       /* 自动工位近到站窗口初值，mm；进入后残差交视觉，不是实测视野或地面精度。 */
#define CAR_ROUGH_BRAKE_LOOKAHEAD_MS 200u     /* 自动D进站制动预留，ms；减速度沿用CAR_DRIVE_DECEL_RPM_PER_S，无固定低速段。 */
#if CAR_ROUGH_BRAKE_LOOKAHEAD_MS < 100 || CAR_ROUGH_BRAKE_LOOKAHEAD_MS > 1000
#error Invalid_rough_brake_lookahead
#endif
/* 以下四项仅在CAR_RUN_OMNI_ENABLED=0时控制旧的首次/已走道路策略。 */
#define CAR_RUN_REVERSE_ENABLED      1       /* runimu仅在本轮完整走过的道路上允许正后方直接后退。 */
#define CAR_RUN_REVISIT_LATERAL_ENABLED 1    /* 本轮完整走过的道路保持当前车头，以左右横移代替90度转向。 */
#define CAR_RUN_START_LATERAL_ENABLED 1      /* 仅发车A/C->B首段60mm保持初始航向横移。 */
#define CAR_RUN_RETURN_LATERAL_ENABLED 1     /* 最终H->E->B->A/C保持暂存离站航向，以前后/横移返回。 */
#define CAR_LATERAL_MAX_RPM          ((CAR_LATERAL_SHORT_RPM > CAR_LATERAL_LONG_RPM) ? CAR_LATERAL_SHORT_RPM : CAR_LATERAL_LONG_RPM)
static inline uint16_t car_lateral_limit_rpm(uint32_t distance_mm)
{
    return distance_mm <= CAR_LATERAL_SHORT_MM ? CAR_LATERAL_SHORT_RPM : CAR_LATERAL_LONG_RPM;
}
#define CAR_OBSTACLE_RETURN_RPM      60      /* obstacle触发后沿已走道路退回的速度上限，RPM。 */
/* 行驶中心线内移：上沿F/G向+Y，下沿D/I向-Y，暂存H向+X；A/C及B/E不移动。
 * 单位mm。修改后先运行tools/rebuild-route-table.ps1重生成并验证查表，再编译。 */
#define CAR_MAP_TOP_INSET_MM         90 /* 上侧工位向场内内移距离，mm；改后须重生成离线路表。 */
#define CAR_MAP_BOTTOM_INSET_MM      0 /* 粗加工保留原50mm内移，按操作者纠正不撤回；改后须重生成离线路表。 */
#define CAR_MAP_RAW_EXTRA_INSET_MM  100      /* 仅原料F额外内移，原50mm再加50mm；G点不随动，单位mm。 */
#define CAR_MAP_STORAGE_INSET_MM     80      /* 撤回暂存新增50mm，保留原80mm内移，单位mm。 */
#if CAR_MAP_TOP_INSET_MM < 0 || CAR_MAP_TOP_INSET_MM >= 1000 || \
    CAR_MAP_BOTTOM_INSET_MM < 0 || CAR_MAP_BOTTOM_INSET_MM >= 860 || \
    CAR_MAP_RAW_EXTRA_INSET_MM < 0 || CAR_MAP_TOP_INSET_MM + CAR_MAP_RAW_EXTRA_INSET_MM >= 1000 || \
    CAR_MAP_STORAGE_INSET_MM < 0 || CAR_MAP_STORAGE_INSET_MM >= 860
#error Invalid_map_inset
#endif
#define CAR_ROAD_DEFAULT_RPM        100       /* 人工导航 rpm 0 时，道路表的默认速度。 */
#define CAR_MOTION_ACCELERATION    200       /* 单段标定和道路表的默认加速度档位。 */

/* 2. 距离换算，单位：脉冲/mm；左右共用 X，前后共用 Y。 */
#define CALIBRATED_PPM_X     11.052632f       /* 原 10.5；60 RPM 下 1000/950 比例修正。 */
#define CALIBRATED_PPM_Y        10.36f       /* 前后系数，保留队友的值。 */
#define CAR_TEST_DISTANCE_MM    1000       /* test 模式上电默认距离；可用 dist 命令改。 */

/* 3. 横移往后偏的预补偿，单位：前移 mm/横移米。正值向前，0 关闭。
 * 30/10 为当前调参；横移闭环叠加该平移前馈，IMU负责车头航向。
 */
#define LEFT_FORWARD_MM_PER_M     10 /* 向左横移时的前移预补偿，mm/横移米；增大更向前，0关闭。 */
#define RIGHT_FORWARD_MM_PER_M    10 /* 向右横移时的前移预补偿，mm/横移米；增大更向前，0关闭。 */

/* 原地转向只使用IMU闭环；方向位同时供四轮速度混控使用，不能随旧入口删除。 */
#define CAR_TURN_RPM                  30       /* turn模式手动定角转向默认速度上限，RPM。 */
#define CAR_TURN_CLOSED_LOOP_RPM      300      /* 原地转向上限，RPM；保留本轮开始时的现值，与平移档位解耦。 */
#define CAR_TURN_CCW_DIR               1u      /* 正向旋转的驱动器dir位；IMU左转角应增加。 */

/* 6. runimu航向闭环。现场日志已有新角度，沿用当前开启的USART2接收。
 * 置1只开启USART2被动接收，不自动扫描波特率或向模块写配置。
 * 当前硬件适配在vendor/stm32/imu/imu.c：PD5 TX / PD6 RX。
 */
#ifndef CAR_IMU_UART_ENABLED
#define CAR_IMU_UART_ENABLED           1
#endif
#define CAR_IMU_UART_BAUD         230400u
#define CAR_IMU_YAW_SIGN               1       /* 实车左转时角度应增加；反向改为-1。 */
#define CAR_MAP_YAW_SIGN              -1       /* 场地图X向右、Y向下；地图方向递增与左转角相反。 */
#define CAR_IMU_MAX_AGE_MS           250u
#define CAR_HEADING_TOLERANCE_CDEG    80       /* 0.80度；实际精度仍需实测。 */
#define CAR_HEADING_STABLE_MS        300u
#define CAR_HEADING_STABLE_SAMPLES     3u
#define CAR_HEADING_SETTLE_MS        100u
#define CAR_HEADING_TIMEOUT_MS    20000u       /* 每次航向对齐的总时限。 */
#define CAR_HEADING_KP_X100          250       /* 2.00 RPM/度；提高转向后段速度，过冲沿误差反向修正。 */
#define CAR_HEADING_KD_X100           30       /* 0.30 RPM/(度/秒)，测量角速度阻尼。 */
#define CAR_HEADING_MIN_RPM           5       /* 误差超容差时的最低转向修正，RPM。 */
#define CAR_HEADING_STABLE_RATE_CDEG_S 100     /* 稳定时角速度不超过1度/秒。 */
#define CAR_HEADING_DRIVE_TRIM_RPM    20       /* 直行时旋转修正上限，不允许轮子反转。 */
#define CAR_TRANSLATION_MAX_RPM      ((CAR_RUN_RPM > CAR_LATERAL_MAX_RPM) ? CAR_RUN_RPM : CAR_LATERAL_MAX_RPM)
#define CAR_WHEEL_MAX_RPM            ((CAR_TRANSLATION_MAX_RPM > CAR_TURN_CLOSED_LOOP_RPM) ? CAR_TRANSLATION_MAX_RPM : CAR_TURN_CLOSED_LOOP_RPM) /* 硬上限随各档取最大；最终输出另受当前运动档位限制。 */
#ifndef CAR_STRAIGHT_HOLD_ENABLED
#define CAR_STRAIGHT_HOLD_ENABLED      1      /* 普通前进/后退按编码器估计横向位移补偿；不含工位、横移和obstacle。 */
#endif
#define CAR_STRAIGHT_HOLD_KP_X100     50      /* 横向误差比例增益，RPM/mm x100；初值须实车标定。 */
#define CAR_STRAIGHT_HOLD_DEADBAND_MM  2      /* 编码器估计横向位移死区，mm；不是地面精度保证。 */
#define CAR_STRAIGHT_HOLD_MAX_RPM     20      /* 横向补偿上限，RPM；低速时进一步限制，保持四轮原行驶方向。 */
/* 直行保持航向独立于原地转向：沿用PD增益初值，小角度仍保留角速度阻尼。 */
#define CAR_DRIVE_HEADING_KP_X100    120       /* RPM/度 ×100，尚待本车实测调节。 */
#define CAR_DRIVE_HEADING_KD_X100     30       /* RPM/(度/秒) ×100。 */
#define CAR_DRIVE_HEADING_QUIET_CDEG  20       /* 同时满足±0.20度和下面角速度时才输出0。 */
#define CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S 50 /* 0.50度/秒；不是新的到站精度承诺。 */
#define CAR_DRIVE_HEADING_MIN_RPM      1       /* 非静稳区的最小纠偏，避免沿用原地转向的最低转速跳变。 */
#define CAR_SETTLE_TURN_RPM_PER_S     60u      /* 普通平移末段的航向纠偏变化率，RPM/s；保留PD增益，取消10RPM下限，不影响正常定角转向。 */
#define CAR_SETTLE_DISTANCE_MM        10      /* 道路/单测最后10mm进入同一末端控制，回拉与回正不反复切换；不是到位误差窗口。 */
#define CAR_MOVEXY_FINE_MAX_RPM        6       /* 仅工位双轴低速微调：两轴均不超过此RPM时，不放大小于1RPM的航向控制量。 */
#define CAR_MOVEXY_MIN_RPM             1       /* 双轴最后几毫米允许降到1RPM；不沿用普通平移3RPM下限覆盖末端制动结果。 */
#define CAR_DRIVE_ACCEL_RPM_PER_S    150u      /* 速度指令上升率RPM/s；原120，起步提高25%；减速及制动距离沿用120。 */
#define CAR_ROUTE_ACCEL_RPM_PER_S    240u      /* 普通道路及连续路径的起步/再加速，RPM/s；工位微调与返回障碍起点不使用。 */
#define CAR_DRIVE_DECEL_RPM_PER_S    200u      /* 回退r91时按操作者确认保留减速200 RPM/s；正常减速和制动距离计算共用。 */
#define CAR_DRIVE_BRAKE_LOOKAHEAD_MS 200u      /* 减速计算预留控制/反馈延迟的初值，需现场确认。 */
#define CAR_DRIVE_SLEW_MAX_DT_MS      50u      /* 吸收正常日志开销；长时间延迟最多推进50ms的斜坡。 */
#define CAR_VELOCITY_CONTROL_MS      25u      /* 目标控制周期；只采用新IMU样本更新角度。 */
#define CAR_VELOCITY_QUERY_MS         5u      /* 串行轮询四轮位置/转速/状态，约60ms一轮。 */
#define CAR_VELOCITY_REPLY_MS       150u
#define CAR_VELOCITY_FEEDBACK_MS    300u
#define CAR_VELOCITY_ACK_MS         150u
#define CAR_VELOCITY_STALL_MS      1500u      /* 轮子被命令运动但位置长期不变时停止。 */
#define CAR_VELOCITY_STOP_RPM         1       /* 四轮新速度反馈的停稳阈值。 */
#define CAR_ENCODER_COUNTS_PER_REV 65536u     /* 0x36编码器位置计数/圈。 */
#define CAR_POSITION_PULSES_PER_REV 3200u     /* 既有距离标定的脉冲/圈，16细分。 */
#define CAR_ENCODER_LEARN_MM          2       /* 净位移至少2mm且连续两份新位置沿同向增长，才确认符号。 */
#define CAR_ENCODER_LEARN_RPM        20       /* 四轮方向全部确认前的平移速度上限，RPM。 */
#define CAR_ENCODER_LEARN_TIMEOUT_MS 1500u    /* 起步方向无法确认时停车；不带着未知里程继续加速。 */
#define CAR_DISTANCE_KP_X100          50       /* 剩余1mm对应0.50 RPM，近终点减速。 */
#define CAR_DISTANCE_MIN_RPM           3 /* 终点附近非零平移速度下限，RPM；增大会更快接近但更易过冲。 */
#define CAR_LATERAL_SETTLE_MIN_RPM     1 /* 道路/单测四轮方向确认后的横移末段与回拉下限，RPM；工位微调仍用原参数。 */
#define CAR_DISTANCE_TOLERANCE_MM      1 /* 编码器到站距离窗口，mm；增大降低停车距离精度。 */
#define CAR_DISTANCE_OVERRUN_MM      500       /* 操作者指定：距离超程阈值500mm；不改变目标距离与到位精度。 */
#define CAR_DRIVE_TIMEOUT_MS      30000u      /* 单个几何直行段总时限。 */
#define CAR_CONTROL_LOG_MS          500u
#define CAR_HEADING_MAX_DRIFT_CDEG  1500       /* 直行偏航15度请求停车。 */

/* 右侧夹爪朝工位：地图航向0=下(+Y)、1=左(-X)、2=上(-Y)、3=右(+X)。 */
#define CAR_STATION_RAW_HEADING       1u      /* 原料在上：车头向左，右侧向上。 */
#define CAR_STATION_ROUGH_HEADING     3u      /* 粗加工在下：车头向右，右侧向下。 */
#define CAR_STATION_STORAGE_HEADING   0u      /* 暂存在左：车头向下，右侧向左。 */

#if (CAR_IMU_YAW_SIGN != 1 && CAR_IMU_YAW_SIGN != -1) || \
    (CAR_MAP_YAW_SIGN != 1 && CAR_MAP_YAW_SIGN != -1) || \
    (CAR_RUN_REVERSE_ENABLED != 0 && CAR_RUN_REVERSE_ENABLED != 1) || \
    (CAR_RUN_REVISIT_LATERAL_ENABLED != 0 && CAR_RUN_REVISIT_LATERAL_ENABLED != 1) || \
    (CAR_RUN_START_LATERAL_ENABLED != 0 && CAR_RUN_START_LATERAL_ENABLED != 1) || \
    (CAR_RUN_RETURN_LATERAL_ENABLED != 0 && CAR_RUN_RETURN_LATERAL_ENABLED != 1) || \
    CAR_LATERAL_MAX_RPM < 20 || CAR_LATERAL_MAX_RPM > CAR_WHEEL_MAX_RPM || \
    CAR_OBSTACLE_RETURN_RPM < 20 || CAR_OBSTACLE_RETURN_RPM > CAR_WHEEL_MAX_RPM || \
    CAR_HEADING_TOLERANCE_CDEG < 1 || CAR_HEADING_STABLE_SAMPLES < 2 || \
    CAR_HEADING_KP_X100 < 1 || CAR_VELOCITY_CONTROL_MS < 10 || \
    CAR_DRIVE_HEADING_KP_X100 < 1 || CAR_DRIVE_HEADING_KD_X100 < 0 || \
    CAR_DRIVE_HEADING_QUIET_CDEG < 1 || CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S < 1 || \
    CAR_DRIVE_HEADING_MIN_RPM < 1 || CAR_DRIVE_HEADING_MIN_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_SETTLE_TURN_RPM_PER_S < 1 || CAR_SETTLE_DISTANCE_MM <= CAR_DISTANCE_TOLERANCE_MM || \
    CAR_LATERAL_SETTLE_MIN_RPM < 1 || CAR_LATERAL_SETTLE_MIN_RPM > CAR_DISTANCE_MIN_RPM || \
    CAR_MOVEXY_FINE_MAX_RPM < 1 || CAR_MOVEXY_FINE_MAX_RPM > CAR_ENCODER_LEARN_RPM || \
    CAR_MOVEXY_MIN_RPM < 1 || CAR_MOVEXY_MIN_RPM > CAR_DISTANCE_MIN_RPM || \
    CAR_DRIVE_ACCEL_RPM_PER_S < 1 || CAR_ROUTE_ACCEL_RPM_PER_S < 1 || CAR_DRIVE_DECEL_RPM_PER_S < 1 || \
    CAR_DRIVE_SLEW_MAX_DT_MS < CAR_VELOCITY_CONTROL_MS || CAR_DRIVE_SLEW_MAX_DT_MS > CAR_VELOCITY_REPLY_MS || \
    CAR_VELOCITY_QUERY_MS < 1 || CAR_VELOCITY_FEEDBACK_MS <= CAR_VELOCITY_REPLY_MS || \
    CAR_POSITION_PULSES_PER_REV < 1 || CAR_DISTANCE_TOLERANCE_MM < 1 || \
    CAR_ENCODER_LEARN_MM < 1 || CAR_ENCODER_LEARN_RPM < CAR_DISTANCE_MIN_RPM || \
    CAR_ENCODER_LEARN_RPM > CAR_LATERAL_MAX_RPM || CAR_ENCODER_LEARN_TIMEOUT_MS < CAR_VELOCITY_FEEDBACK_MS
#error Invalid_heading_configuration
#endif
#if CAR_STRAIGHT_HOLD_ENABLED < 0 || CAR_STRAIGHT_HOLD_ENABLED > 1 || \
    CAR_STRAIGHT_HOLD_KP_X100 < 1 || CAR_STRAIGHT_HOLD_DEADBAND_MM < 1 || \
    CAR_STRAIGHT_HOLD_MAX_RPM < 1 || CAR_STRAIGHT_HOLD_MAX_RPM >= CAR_WHEEL_MAX_RPM
#error Invalid_straight_hold_configuration
#endif
#if CAR_STATION_RAW_HEADING < 0 || CAR_STATION_RAW_HEADING > 3 || \
    CAR_STATION_ROUGH_HEADING < 0 || CAR_STATION_ROUGH_HEADING > 3 || \
    CAR_STATION_STORAGE_HEADING < 0 || CAR_STATION_STORAGE_HEADING > 3
#error Invalid_station_heading
#endif
#if CAR_RUN_CORNER_ENABLED < 0 || CAR_RUN_CORNER_ENABLED > 1 || \
    CAR_C_RETURN_BLEND_MM < 1 || CAR_START_BLEND_MM < 1 || CAR_ROUTE_VISUAL_WINDOW_MM < 1 || \
    CAR_CORNER_RADIUS_MM < 50 || CAR_CORNER_RADIUS_MM > 300 || \
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

/* 4. 通信与等待参数。通常先看故障回传，不靠加大超时掩盖通信问题。 */
#define CAR_DEBUG_UART_BAUD              115200
/* 开始按钮：PC9内部上拉，按钮接GND，按下低电平；长按不重复，上电按住不启动。 */
#define CAR_START_BUTTON_GPIO                GPIOC
#define CAR_START_BUTTON_GPIO_CLOCK          RCC_AHB1Periph_GPIOC
#define CAR_START_BUTTON_PIN                 GPIO_Pin_9
#define CAR_START_BUTTON_DEBOUNCE_MS          30u /* 按下/松开消抖时长，ms；与串口runimu C共用启动条件。 */
/* XM1601：USART1 PB6 TX/PB7 RX，PC0低电平触发脚保持释放。
 * 9600是待现场确认的初值；qr baud命令只改STM32串口，不写模块波特率。 */
#define CAR_SCANNER_UART_BAUD              9600u
#define CAR_SCANNER_BOOT_MS                1000u
#define CAR_SCANNER_TX_MS                  1000u /* 单条配置发送限时；只影响扫码配置状态，不新增底盘停机。 */
#define CAR_SCANNER_ACK_MS                 1000u

/* 5. 运动估时/超时。估时不确认到位；全程仍需四轮反馈，导航仍需确认。 */
#define CAR_MOTION_MIN_ESTIMATE_MS    200u
#define CAR_MOTION_TIMEOUT_FACTOR      3u
#define CAR_MOTION_TIMEOUT_EXTRA_MS  5000u
#define CAR_STOP_ACCELERATION        200       /* 停车时的速度零命令，独立于运动参数。 */

/* 只供日志显示宏值，日常不用修改。 */
#define CAR_STRING_VALUE_INNER(value) #value
#define CAR_STRING_VALUE(value) CAR_STRING_VALUE_INNER(value)

#endif
