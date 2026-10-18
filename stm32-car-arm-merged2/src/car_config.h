#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include <stdint.h>

/*
 * 小车调试参数入口：日常调速、距离标定、横移预补偿优先改本文件。
 * 改完重新编译、下载、复位运行。
 * 单条道路的距离/方向在 structure.h，全程节点顺序在 full_route.h。
 */

/* 1. 常用速度与加速度，RPM 为电机转速，150 为驱动器加速度档位。 */
#define CAR_TEST_RPM                60       /* test 模式上电默认速度；可用 rpm 命令改。 */
#define CAR_RUN_RPM                 3000      /* runimu 全程直行速度；覆盖道路表速度。 */
#define CAR_RUN_REVERSE_ENABLED      1       /* runimu仅在本轮完整走过的道路上允许正后方直接后退。 */
#define CAR_RUN_START_LATERAL_ENABLED 1      /* 仅发车A/C->B首段60mm保持初始航向横移。 */
#define CAR_RUN_RETURN_LATERAL_ENABLED 1     /* 已走返回路：恢复发车朝向后后退，末段60mm横移归位。 */
#define CAR_LATERAL_MAX_RPM          60      /* 横移速度上限；先沿用已标定的60RPM范围。 */
#define CAR_OBSTACLE_RETURN_RPM      60      /* obstacle触发后沿已走道路退回的速度上限，RPM。 */
/* 行驶中心线内移：上沿F/G向+Y，下沿D/I向-Y，暂存H向+X；A/C及B/E不移动。
 * 单位mm。修改后先运行tools/rebuild-route-table.ps1重生成并验证查表，再编译。 */
#define CAR_MAP_TOP_INSET_MM         90
#define CAR_MAP_BOTTOM_INSET_MM      50
#define CAR_MAP_RAW_EXTRA_INSET_MM   50      /* 仅原料F再向场内内移；G点不随动，单位mm。 */
#define CAR_MAP_STORAGE_INSET_MM     80      /* 仅暂存H向地图中部(+X)内移，单位mm。 */
#if CAR_MAP_TOP_INSET_MM < 0 || CAR_MAP_TOP_INSET_MM >= 1000 || \
    CAR_MAP_BOTTOM_INSET_MM < 0 || CAR_MAP_BOTTOM_INSET_MM >= 860 || \
    CAR_MAP_RAW_EXTRA_INSET_MM < 0 || CAR_MAP_TOP_INSET_MM + CAR_MAP_RAW_EXTRA_INSET_MM >= 1000 || \
    CAR_MAP_STORAGE_INSET_MM < 0 || CAR_MAP_STORAGE_INSET_MM >= 860
#error Invalid_map_inset
#endif
#define CAR_ROAD_DEFAULT_RPM        100       /* 人工导航 rpm 0 时，道路表的默认速度。 */
#define CAR_MOTION_ACCELERATION    150       /* 单段标定和道路表的默认加速度档位。 */

/* 2. 距离换算，单位：脉冲/mm；左右共用 X，前后共用 Y。 */
#define CALIBRATED_PPM_X     11.052632f       /* 原 10.5；60 RPM 下 1000/950 比例修正。 */
#define CALIBRATED_PPM_Y        10.36f       /* 前后系数，保留队友的值。 */
#define CAR_TEST_DISTANCE_MM    1000       /* test 模式上电默认距离；可用 dist 命令改。 */

/* 3. 横移往后偏的预补偿，单位：前移 mm/横移米。正值向前，0 关闭。
 * 20/8 是待实测的初值；横移闭环仍叠加该平移前馈，IMU仅负责车头航向。
 */
#define LEFT_FORWARD_MM_PER_M     20
#define RIGHT_FORWARD_MM_PER_M     8

/* 原地转向只使用IMU闭环；方向位同时供四轮速度混控使用，不能随旧入口删除。 */
#define CAR_TURN_RPM                  30       /* turn模式手动定角转向默认速度上限，RPM。 */
#define CAR_TURN_CLOSED_LOOP_RPM      1000       /* runimu连续转向速度上限；不靠固定脉冲确认角度。 */
#define CAR_TURN_CCW_DIR               1u      /* 正向旋转的驱动器dir位；IMU左转角应增加。 */

/* 6. runimu航向闭环。现场日志已有新角度，沿用当前开启的USART2接收。
 * 置1只开启USART2被动接收，不自动扫描波特率或向模块写配置。
 * 当前硬件适配在vendor/stm32/imu/imu.c：PD5 TX / PD6 RX。
 */
#ifndef CAR_IMU_UART_ENABLED
#define CAR_IMU_UART_ENABLED           1
#endif
#define CAR_IMU_UART_BAUD         115200u
#define CAR_IMU_YAW_SIGN               1       /* 实车左转时角度应增加；反向改为-1。 */
#define CAR_MAP_YAW_SIGN              -1       /* 场地图X向右、Y向下；地图方向递增与左转角相反。 */
#define CAR_IMU_MAX_AGE_MS           250u
#define CAR_HEADING_TOLERANCE_CDEG    80       /* 0.80度；实际精度仍需实测。 */
#define CAR_HEADING_STABLE_MS        300u
#define CAR_HEADING_STABLE_SAMPLES     3u
#define CAR_HEADING_SETTLE_MS        100u
#define CAR_HEADING_TIMEOUT_MS    20000u       /* 每次航向对齐的总时限。 */
#define CAR_HEADING_KP_X100          120       /* 1.20 RPM/度，角度误差到电机速度。 */
#define CAR_HEADING_KD_X100           30       /* 0.30 RPM/(度/秒)，测量角速度阻尼。 */
#define CAR_HEADING_MIN_RPM           3       /* 误差超容差时的低速修正初值。 */
#define CAR_HEADING_STABLE_RATE_CDEG_S 100     /* 稳定时角速度不超过1度/秒。 */
#define CAR_HEADING_DRIVE_TRIM_RPM    20       /* 直行时旋转修正上限，不允许轮子反转。 */
/* 直行保持航向独立于原地转向：沿用PD增益初值，小角度仍保留角速度阻尼。 */
#define CAR_DRIVE_HEADING_KP_X100    120       /* RPM/度 ×100，尚待本车实测调节。 */
#define CAR_DRIVE_HEADING_KD_X100     30       /* RPM/(度/秒) ×100。 */
#define CAR_DRIVE_HEADING_QUIET_CDEG  20       /* 同时满足±0.20度和下面角速度时才输出0。 */
#define CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S 50 /* 0.50度/秒；不是新的到站精度承诺。 */
#define CAR_DRIVE_HEADING_MIN_RPM      1       /* 非静稳区的最小纠偏，避免沿用原地转向3RPM跳变。 */
#define CAR_DRIVE_ACCEL_RPM_PER_S    120u      /* 前进/后退速度指令上升率，0到90RPM至少750ms。 */
#define CAR_DRIVE_DECEL_RPM_PER_S    120u      /* 正常减速的指令变化率；急停/故障不经过斜坡。 */
#define CAR_DRIVE_BRAKE_LOOKAHEAD_MS 100u      /* 减速计算预留控制/反馈延迟的初值，需现场确认。 */
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
#define CAR_DISTANCE_KP_X100          50       /* 剩余1mm对应0.50 RPM，近终点减速。 */
#define CAR_DISTANCE_MIN_RPM           3
#define CAR_DISTANCE_TOLERANCE_MM      1
#define CAR_DISTANCE_OVERRUN_MM       15       /* 反馈距离超过目标15mm则故障，不放行。 */
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
    (CAR_RUN_START_LATERAL_ENABLED != 0 && CAR_RUN_START_LATERAL_ENABLED != 1) || \
    (CAR_RUN_RETURN_LATERAL_ENABLED != 0 && CAR_RUN_RETURN_LATERAL_ENABLED != 1) || \
    CAR_LATERAL_MAX_RPM < 20 || CAR_LATERAL_MAX_RPM > 300 || \
    CAR_OBSTACLE_RETURN_RPM < 20 || CAR_OBSTACLE_RETURN_RPM > 300 || \
    CAR_HEADING_TOLERANCE_CDEG < 1 || CAR_HEADING_STABLE_SAMPLES < 2 || \
    CAR_HEADING_KP_X100 < 1 || CAR_VELOCITY_CONTROL_MS < 10 || \
    CAR_DRIVE_HEADING_KP_X100 < 1 || CAR_DRIVE_HEADING_KD_X100 < 0 || \
    CAR_DRIVE_HEADING_QUIET_CDEG < 1 || CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S < 1 || \
    CAR_DRIVE_HEADING_MIN_RPM < 1 || CAR_DRIVE_HEADING_MIN_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_DRIVE_ACCEL_RPM_PER_S < 1 || CAR_DRIVE_DECEL_RPM_PER_S < 1 || \
    CAR_DRIVE_SLEW_MAX_DT_MS < CAR_VELOCITY_CONTROL_MS || CAR_DRIVE_SLEW_MAX_DT_MS > CAR_VELOCITY_REPLY_MS || \
    CAR_VELOCITY_QUERY_MS < 1 || CAR_VELOCITY_FEEDBACK_MS <= CAR_VELOCITY_REPLY_MS || \
    CAR_POSITION_PULSES_PER_REV < 1 || CAR_DISTANCE_TOLERANCE_MM < 1
#error Invalid_heading_configuration
#endif
#if CAR_STATION_RAW_HEADING < 0 || CAR_STATION_RAW_HEADING > 3 || \
    CAR_STATION_ROUGH_HEADING < 0 || CAR_STATION_ROUGH_HEADING > 3 || \
    CAR_STATION_STORAGE_HEADING < 0 || CAR_STATION_STORAGE_HEADING > 3
#error Invalid_station_heading
#endif

/* 4. 通信与等待参数。通常先看故障回传，不靠加大超时掩盖通信问题。 */
#define CAR_DEBUG_UART_BAUD              115200
/* 开始按钮：PC9，按下高电平；仅发送串口提示，不触发运动。 */
#define CAR_START_BUTTON_GPIO                GPIOC
#define CAR_START_BUTTON_GPIO_CLOCK          RCC_AHB1Periph_GPIOC
#define CAR_START_BUTTON_PIN                 GPIO_Pin_9
#define CAR_START_BUTTON_DEBOUNCE_MS          30u
/* XM1601：USART1 PB6 TX/PB7 RX，PC0低电平触发脚保持释放。
 * 9600是待现场确认的初值；qr baud命令只改STM32串口，不写模块波特率。 */
#define CAR_SCANNER_UART_BAUD              9600u
#define CAR_SCANNER_BOOT_MS                1000u
#define CAR_SCANNER_ACK_MS                 1000u

/* 5. 运动估时/超时。估时不确认到位；全程仍需四轮反馈，导航仍需确认。 */
#define CAR_MOTION_MIN_ESTIMATE_MS    200u
#define CAR_MOTION_TIMEOUT_FACTOR      3u
#define CAR_MOTION_TIMEOUT_EXTRA_MS  5000u
#define CAR_STOP_ACCELERATION        150       /* 停车时的速度零命令，独立于运动参数。 */

/* 只供日志显示宏值，日常不用修改。 */
#define CAR_STRING_VALUE_INNER(value) #value
#define CAR_STRING_VALUE(value) CAR_STRING_VALUE_INNER(value)

#endif
