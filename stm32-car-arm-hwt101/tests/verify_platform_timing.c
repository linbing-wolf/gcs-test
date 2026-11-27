/* 复用既有平台/云台并行及各通道独立计时测试，不运行无关旧故障策略断言。 */
#define ARM_INTEGRATION_TEST_MAIN retained_arm_integration_tests
#include "verify_arm_integration.c"

int main(void)
{
    CHECK(ARM_SERVO_TIME_MS==200u && ARM_YAW_TIME_MS==100u && ARM_CLAW_TIME_MS==30u);
    parallel_platform_yaw_tests();
    independent_servo_and_grip_hold_tests();
    yaw_settle_tests();
    puts("PASS platform200ms: platform/yaw parallel interpolation, cup slots, independent yaw100/claw30, grip hold and manual emergency");
    return 0;
}
