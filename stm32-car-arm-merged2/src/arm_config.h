#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
/* 参数来源：gcs-test/test-code/stm32/main_armclaw.c，96f65d8；合并版待实车验证。 */
#define ARM_DEBUG_UART_BAUD       115200u
#define ARM_LIFT_ADDRESS               5u
#define ARM_SLIDE_ADDRESS              7u
#define ARM_LIFT_PULSES_PER_MM        80.0f
#define ARM_SLIDE_PULSES_PER_MM       25.465f
#define ARM_LIFT_MIN_MM                0
#define ARM_LIFT_MAX_MM              135
#define ARM_LIFT_HOME_MM             135 /* 最高处；0是最低处，不在上电时自动认定。 */
#define ARM_SLIDE_MIN_MM               0 /* 齿条完全收回的人工坐标原点。 */
#define ARM_SLIDE_MAX_MM             187 /* 队友程序的理论行程上限，尚未实测全行程。 */
#define ARM_SLIDE_HOME_MM              0
#define ARM_LIFT_DIR_INVERT            1 /* 沿用队友调试结果：反转位置命令的方向位。 */
#define ARM_SLIDE_DIR_INVERT           1
#define ARM_AXIS_RPM                 120u /* 先降低调试速度，旧代码为1000/2000RPM。 */
#define ARM_AXIS_ACCEL               150u
#define ARM_ENCODER_COUNTS_PER_REV 65536u
#define ARM_PULSES_PER_REV          3200u /* 驱动器必须使用对应16细分。 */
#define ARM_POSITION_TOLERANCE_MM      0.5f /* 到位误差窗口，不能把1mm指令的零位移算完成。 */
#define ARM_LIFT_ENCODER_SIGN          1  /* dir=0时CPOS增加为+1，相反为-1。 */
#define ARM_SLIDE_ENCODER_SIGN         1
#define ARM_YAW_MIN_DEG            (-94.5f)
#define ARM_YAW_MAX_DEG              265.5f
#define ARM_YAW_ZERO_DEG             273.4f
#define ARM_YAW_DIRECTION             (-1.0f)
#define ARM_YAW_HOME_DEG                0.0f /* 1975us，远程当前标定的初始角。 */
#define ARM_YAW_OFFSET_DEG             7.9f
#define ARM_CLAW_MIN_DEG               0.0f
#define ARM_CLAW_MAX_DEG             270.0f
#define ARM_CLAW_OFFSET_DEG            1.0f
#define ARM_CLAW_OPEN_DEG             45
#define ARM_CLAW_CLOSE_DEG             0
#define ARM_PICK_SLIDE_MM             65
#define ARM_PICK_LIFT_MM             110
#define ARM_PICK_YAW_DEG             126.0f /* 1275us，采用远程当前抓取标定。 */
#define ARM_PLACE_SLIDE_MM            30
#define ARM_PLACE_LIFT_MM            110
#define ARM_PLACE_YAW_DEG            254.0f /* 564us，放上载物台。 */
#define ARM_PLATFORM_FIRST_DEG        20.0f /* PA2，沿用随包PWM.c三个档位。 */
#define ARM_PLATFORM_SECOND_DEG      139.0f
#define ARM_PLATFORM_THIRD_DEG       256.0f
#define ARM_CHASSIS_STOP_HOLD_MS     200u
#define ARM_CHASSIS_LOCK_TIMEOUT_MS 2000u
#define ARM_SERVO_TIME_MS           2000u /* 插值时间；舵机无到位反馈。 */
#define ARM_YAW_TEST_TIME_MS        8000u /* 全行程诊断单次插值时间，不改变夹取动作速度。 */
#define ARM_POLL_MS                   10u
#define ARM_FEEDBACK_TIMEOUT_MS      500u
#define ARM_ACK_TIMEOUT_MS          1500u
#define ARM_ACTION_TIMEOUT_MS      15000u
#endif
