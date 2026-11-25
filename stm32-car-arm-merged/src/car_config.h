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
#define CAR_MOTION_ACCELERATION    200       /* 单段标定和道路表的默认加速度档位。 */

/* 2. 距离换算，单位：脉冲/mm；左右共用 X，前后共用 Y。 */
#define CALIBRATED_PPM_X     10.889293f       /* 100 RPM 右移指令 2000mm、实测 2030mm，按 2000/2030 修正。 */
#define CALIBRATED_PPM_Y        10.5445f     /* 100RPM实测指令2000mm、实际约1965mm，按短35mm修正。 */
#define CAR_TEST_DISTANCE_MM    1000       /* test 模式上电默认距离；可用 dist 命令改。 */

/* 3. 横移往后偏的预补偿，单位：前移 mm/横移米。正值向前，0 关闭。
 * 20/8 是待实测的初值；横移闭环仍叠加该平移前馈，IMU仅负责车头航向。
 */
#define LEFT_FORWARD_MM_PER_M     20
#define RIGHT_FORWARD_MM_PER_M     8
/* 实验横移专用前后前馈（不影响旧位置模块和人工nav）。
 * 实测证明打滑时大前馈会引入旋转耦合，先保持与旧模块一致的小值。 */
#define CAR_EXPERIMENT_LEFT_FORWARD_MM_PER_M   20
#define CAR_EXPERIMENT_RIGHT_FORWARD_MM_PER_M   8
/* 加速度计侧向速度闭环：一次积分+泄漏，只做有限补偿，不积分位移。 */
#define CAR_ACCEL_SIDE_DIV_MM_S       20   /* 每 20 mm/s 侧向速度补 1 RPM。 */
#define CAR_ACCEL_SIDE_MAX_RPM         6   /* 速度环补偿上限；过大只会抢走横移牵引。 */
#define CAR_ACCEL_FORWARD_MAX_RPM      6   /* 纵轴/前后修正单独限幅；实测加大无收益。 */
#define CAR_ACCEL_SIDE_LEAK_MS       300u  /* 速度泄漏；抑制零偏漂移。 */
#define CAR_ACCEL_SIDE_MAX_MM_S      500   /* 侧向速度观测上限。 */
#define CAR_ACCEL_SIDE_DEADBAND_MG    20   /* 小于该加速度视为噪声/残余重力投影。 */
#define CAR_ACCEL_LPF_MS             120u  /* 加速度低通，滤掉麦轮高频振动。 */
#define CAR_ACCEL_CALIB_SAMPLES        8u  /* 起步前控制周期做零偏平均。 */
#define CAR_ACCEL_POS_LEAK_MS       6000u  /* 位移估计慢泄漏，只压零偏漂移。 */
#define CAR_ACCEL_POS_LIMIT_MM       400   /* 位移观测量限幅，避免异常积分。 */
#define CAR_ACCEL_DISTANCE_GAIN_X100  160  /* 加速度估出的缺口按1.60倍追加到目标距离。 */
#define CAR_ACCEL_DISTANCE_MAX_MM     300  /* 实验距离追加上限。 */
#define CAR_ACCEL_DISTANCE_DEADBAND_MM 80  /* 估出的偏移低于该值视为积分漂移，不追加距离。 */
/* 横移位移误差改用轻微车头偏置修正：不抢横移牵引，靠旋转力矢量把轨迹掰回。 */
#define CAR_LATERAL_YAW_GAIN_X1000      0  /* 暂时关闭；实测终点回退，先做距离补偿。 */
#define CAR_LATERAL_YAW_MAX_CDEG      550  /* 最大 5.5 度。 */
#define CAR_LATERAL_YAW_TAPER_MM      400  /* 最后 400mm 渐退，避免停车甩尾。 */
#define CAR_LATERAL_YAW_SIGN            1  /* 实测正偏置使后偏增大，用反号修正。 */
#define CAR_EXPERIMENT_DISTANCE_OVERRUN_MM 200 /* 实验段允许末端滑行后再停。 */
#define CAR_PROJECTED_FORWARD_MM_PER_RPM 10.0f
#define CAR_ACCEL_AXIS_MM_PER_RPM        10.0f /* 加速度双轴位移换算轮速；放大后实测会抢横移主轴。 */

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
#define CAR_HEADING_TOLERANCE_CDEG    40       /* 0.40度；转向例程实测总少转约0.5度，收紧到此值。 */
#define CAR_HEADING_STABLE_MS        300u
#define CAR_HEADING_STABLE_SAMPLES     3u
#define CAR_HEADING_SETTLE_MS        100u
#define CAR_HEADING_TIMEOUT_MS    20000u       /* 每次航向对齐的总时限。 */
#define CAR_HEADING_KP_X100          200       /* 2.00 RPM/度；提高转向后段速度，过冲沿误差反向修正。 */
#define CAR_HEADING_KD_X100           30       /* 0.30 RPM/(度/秒)，测量角速度阻尼。 */
#define CAR_HEADING_MIN_RPM           5       /* 误差超容差时的最低转向修正，RPM。 */
#define CAR_HEADING_STABLE_RATE_CDEG_S 100     /* 稳定时角速度不超过1度/秒。 */
#define CAR_HEADING_DRIVE_TRIM_RPM    20       /* 直行时旋转修正上限，不允许轮子反转。 */
/* 直行保持航向独立于原地转向：沿用PD增益初值，小角度仍保留角速度阻尼。 */
#define CAR_DRIVE_HEADING_KP_X100    120       /* RPM/度 ×100，尚待本车实测调节。 */
#define CAR_DRIVE_HEADING_KD_X100     30       /* RPM/(度/秒) ×100。 */
#define CAR_DRIVE_HEADING_QUIET_CDEG  20       /* 同时满足±0.20度和下面角速度时才输出0。 */
#define CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S 50 /* 0.50度/秒；不是新的到站精度承诺。 */
#define CAR_DRIVE_HEADING_MIN_RPM      1       /* 非静稳区的最小纠偏，避免沿用原地转向的最低转速跳变。 */
#define CAR_DRIVE_ACCEL_RPM_PER_S    120u      /* 速度指令上升率RPM/s；0到90RPM至少750ms，沿用可跟随斜坡，与驱动器200档位不是同一单位。 */
#define CAR_DRIVE_DECEL_RPM_PER_S    120u      /* 正常减速及制动距离计算共用RPM/s；沿用120的可跟随制动斜坡，不能由驱动器200档位推定制动能力。 */
#define CAR_DRIVE_BRAKE_LOOKAHEAD_MS 100u      /* 减速计算预留控制/反馈延迟的初值，需现场确认。 */
#define CAR_LATERAL_BRAKE_LOOKAHEAD_MS 300u    /* 横移高速末端提前制动，避免滑行超差。 */
#define CAR_LATERAL_ACCEL_RPM_PER_S    80u     /* 横移轮胎侧向抓地弱于直行，降低起步冲击。 */
#define CAR_LATERAL_DECEL_RPM_PER_S    80u     /* 横移制动与制动距离计算共用，减少末端甩尾。 */
#define CAR_LATERAL_NEAR_MM           200      /* 最后200mm先降速，减少侧向动量。 */
#define CAR_LATERAL_NEAR_RPM           45
#define CAR_LATERAL_CLOSE_MM          100
#define CAR_LATERAL_CLOSE_RPM          20
#define CAR_LATERAL_FINAL_MM           50
#define CAR_LATERAL_FINAL_RPM           8
#define CAR_LATERAL_TRACTION_MIN_RPM   60   /* 达到该速度后检查四轮牵引一致性。 */
#define CAR_LATERAL_TRACTION_SPREAD_PCT 60  /* 最慢轮低于最快轮60%时进入牵引同步。 */
#define CAR_LATERAL_TRACTION_MARGIN_RPM 8  /* 同步目标比最慢轮高出的余量。 */
#define CAR_LATERAL_TRACTION_FLOOR_RPM 20  /* 同步时保留的最低横移速度。 */
#define CAR_COURSE_HOLD_MAX_RPM        20   /* 航向误差引起的世界轨迹交叉轴前馈上限。 */
#define CAR_LATERAL_HEADING_TRIGGER_CDEG 150 /* 横移航向偏差超过1.5度才启用实时三角投影。 */
#define CAR_LATERAL_HEADING_RELEASE_CDEG  80 /* 回到0.8度以内释放投影，形成迟滞。 */
#define CAR_LATERAL_PROJECTION_MAX_RPM    20 /* 投影叠加到单轴的速度上限。 */
#define CAR_ACCEL_SLIP_GAIN_X100          200 /* 加速度位移比理论偏小，等效滑移角按2倍放大。 */
#define CAR_ACCEL_SLIP_MAX_CDEG          5000 /* 实验：允许等效滑移角到50度。 */
#define CAR_ACCEL_SLIP_MIN_MM              30 /* 位移噪声死区，低于30mm不算滑移。 */
#define CAR_ACCEL_SLIP_START_MM           500 /* 走到500mm后才允许加速度等效角介入。 */
#define CAR_SLIP_FB_GAIN_X100         100   /* 四轮合成交叉轴速度误差的反馈增益。 */
#define CAR_SLIP_FB_MAX_RPM            12   /* 轮速交叉轴反馈修正上限。 */
#define CAR_SLIP_FILTER_MS             80u  /* 轮速反馈低通，吸收60ms轮询采样差。 */
#define CAR_LATERAL_SETTLE_MIN_RPM       5  /* 末端仍有残余轮速时保留航向阻尼。 */
#define CAR_LATERAL_SETTLE_TRIM_RPM      8  /* 末端阻尼不超过该转向修正。 */
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
#define CAR_DISTANCE_MIN_RPM           8        /* 低于8RPM克服不了静摩擦，末端会原地卡死。 */
#define CAR_DISTANCE_TOLERANCE_MM      6        /* 6mm；1mm要求会把车逼进3RPM爬行卡住。 */
#define CAR_DISTANCE_OVERRUN_MM       15       /* 反馈距离超过目标15mm则故障，不放行。 */
#define CAR_DRIVE_TIMEOUT_MS      30000u      /* 单个几何直行段总时限。 */
#define CAR_CONTROL_LOG_MS          500u
#define CAR_HEADING_MAX_DRIFT_CDEG  1500       /* 直行偏航15度请求停车。 */

#define IMUTEST_MAX_TRANSLATION_RPM 200
#if CHASSIS_IMU_EXPERIMENT
#define IMUTEST_DISTANCE_MM 1000
#define IMUTEST_FORWARD_RPM 40
#define IMUTEST_TURN_RPM 30
#define IMUTEST_GYRO_MAX_AGE_MS 100u
#define IMUTEST_GYRO_TAU_MS 50.0f
#define IMUTEST_GYRO_BIAS_CDEG_S 0
#define IMUTEST_I_KI 0.25f
#define IMUTEST_I_TAU_S 2.0f
#define IMUTEST_I_MAX_RPM 3.0f
#define IMUTEST_I_MIN_SPEED_RPM 10
#endif

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
    CAR_LATERAL_BRAKE_LOOKAHEAD_MS < CAR_DRIVE_BRAKE_LOOKAHEAD_MS || CAR_LATERAL_BRAKE_LOOKAHEAD_MS > 500u || \
    CAR_LATERAL_ACCEL_RPM_PER_S < 1 || CAR_LATERAL_DECEL_RPM_PER_S < 1 || \
    CAR_LATERAL_NEAR_MM <= CAR_LATERAL_CLOSE_MM || CAR_LATERAL_CLOSE_MM <= CAR_LATERAL_FINAL_MM || \
    CAR_LATERAL_FINAL_RPM < CAR_DISTANCE_MIN_RPM || \
    CAR_LATERAL_FINAL_RPM >= CAR_LATERAL_CLOSE_RPM || CAR_LATERAL_CLOSE_RPM >= CAR_LATERAL_NEAR_RPM || \
    CAR_LATERAL_TRACTION_MIN_RPM <= CAR_LATERAL_TRACTION_FLOOR_RPM || \
    CAR_LATERAL_TRACTION_SPREAD_PCT < 1 || CAR_LATERAL_TRACTION_SPREAD_PCT > 99 || \
    CAR_LATERAL_TRACTION_MARGIN_RPM < 1 || \
    CAR_COURSE_HOLD_MAX_RPM < 1 || CAR_COURSE_HOLD_MAX_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_SLIP_FB_GAIN_X100 < 1 || CAR_SLIP_FB_GAIN_X100 > 300 || \
    CAR_SLIP_FB_MAX_RPM < 1 || CAR_SLIP_FB_MAX_RPM > CAR_HEADING_DRIVE_TRIM_RPM || \
    CAR_SLIP_FILTER_MS < CAR_VELOCITY_CONTROL_MS || \
    CAR_LATERAL_SETTLE_MIN_RPM < CAR_VELOCITY_STOP_RPM || \
    CAR_LATERAL_SETTLE_TRIM_RPM < CAR_DRIVE_HEADING_MIN_RPM || \
    CAR_VELOCITY_QUERY_MS < 1 || CAR_VELOCITY_FEEDBACK_MS <= CAR_VELOCITY_REPLY_MS || \
    CAR_POSITION_PULSES_PER_REV < 1 || CAR_DISTANCE_TOLERANCE_MM < 1 || \
    CAR_ENCODER_LEARN_MM < 1 || CAR_ENCODER_LEARN_RPM < CAR_DISTANCE_MIN_RPM || \
    CAR_ENCODER_LEARN_RPM > CAR_LATERAL_MAX_RPM || CAR_ENCODER_LEARN_TIMEOUT_MS < CAR_VELOCITY_FEEDBACK_MS
#error Invalid_heading_configuration
#endif
#if CAR_STATION_RAW_HEADING < 0 || CAR_STATION_RAW_HEADING > 3 || \
    CAR_STATION_ROUGH_HEADING < 0 || CAR_STATION_ROUGH_HEADING > 3 || \
    CAR_STATION_STORAGE_HEADING < 0 || CAR_STATION_STORAGE_HEADING > 3
#error Invalid_station_heading
#endif

/* 4. 通信与等待参数。通常先看故障回传，不靠加大超时掩盖通信问题。 */
#define CAR_DEBUG_UART_BAUD              115200
/* 本地调试转接板：PB0=软件TX、PB1=软件RX。设为0恢复USART3 PB10/PB11。 */
#define CAR_DEBUG_UART_SOFT_PB0_PB1          1
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
#define CAR_STOP_ACCELERATION        200       /* 停车时的速度零命令，独立于运动参数。 */

/* 只供日志显示宏值，日常不用修改。 */
#define CAR_STRING_VALUE_INNER(value) #value
#define CAR_STRING_VALUE(value) CAR_STRING_VALUE_INNER(value)

#endif
