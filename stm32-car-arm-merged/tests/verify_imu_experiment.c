#define HEADING_TEST_MAIN heading_regression_main
#include "verify_heading.c"

static void experiment_sensor_test(void)
{
    HeadingSensor sensor;
    HeadingSample sample;
    HeadingControl control = {0};
    heading_sensor_reset(&sensor);
    wit_packet(&sensor, 0x53, 0, 10, 0);
    wit_packet(&sensor, 0x52, -16384, 20, 0);
    wit_packet(&sensor, 0x51, 2048, 21, 0);
    wit_packet(&sensor, 0x52, 16384, 22, 1);
    sample = sensor.sample;
    CHECK(sample.valid && sample.sequence == 1 && sample.received_ms == 10);
    CHECK(sample.gyro_valid && sample.gyro_sequence == 1 && sample.gyro_ms == 20);
    CHECK(sample.gyro_z_cdeg_s == -100000);
    CHECK(sample.accel_valid && sample.accel_ms == 21 && sample.accel_sequence == 1
          && sample.accel_mg[2] == 1000);
    control.experiment_mode = IMUTEST_GYRO;
    CHECK(heading_experiment_gyro(&control, sample, 20) && control.rate_cdeg_s == -100000);
    CHECK(heading_experiment_gyro(&control, sample, 120) && control.rate_cdeg_s == -100000);
    CHECK(!heading_experiment_gyro(&control, sample, 121));
    wit_packet(&sensor, 0x52, 0, 70, 0);
    sample = sensor.sample;
    CHECK(heading_experiment_gyro(&control, sample, 70));
    CHECK(control.rate_cdeg_s > -36800 && control.rate_cdeg_s < -36770);
    heading_experiment_reset(&control);
    CHECK(heading_experiment_gyro(&control, sample, 70) && control.rate_cdeg_s == 0);
    heading_sensor_invalidate(&sensor);
    CHECK(!sensor.sample.valid && !sensor.sample.gyro_valid && !sensor.sample.accel_valid);
}

static void experiment_course_slip_test(void)
{
    unsigned iteration;
    int16_t observed_course = 0, observed_slip = 0;
    reset_plant(0, 1.0);
    send_line("imutest pid");
    wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
    send_line("imutest dist 600");
    send_line("imutest rpm 80");
    send_line("imutest l");
    while (run_active && run.progress_mm < 200) {
        if ((clock_ms % 10u) == 0) wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        plant_step();
    }
    CHECK(run_active && run.state != RUN_FAULT);

    /* 向左横移时车身被扰动为左偏：世界轨迹要留在横向，
     * 车体必须临时补一个向前速度，不能只等航向转回来。 */
    physical_yaw += 300;
    sample_yaw(wrapped_yaw(physical_yaw));
    for (iteration = 0; iteration < 40u && run_active; iteration++) {
        wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        plant_step();
        if (run.course_cross_rpm > observed_course) observed_course = run.course_cross_rpm;
    }
    CHECK(run_active && run.state != RUN_FAULT);
    CHECK(observed_course > 0);

    /* 轮2反馈被压低，四轮合成出现交叉轴分量；修正应在线叠加，
     * 任务保持运行，不能以停车/FAULT换精度。 */
    wheel_feedback_bias[1] = -20;
    for (iteration = 0; iteration < 40u && run_active; iteration++) {
        wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        plant_step();
        if (run.slip_cross_rpm > observed_slip) observed_slip = run.slip_cross_rpm;
        if (-run.slip_cross_rpm > observed_slip) observed_slip = (int16_t)-run.slip_cross_rpm;
    }
    CHECK(run_active && run.state != RUN_FAULT);
    CHECK(observed_slip > 0);
    wheel_feedback_bias[1] = 0;
    {
        uint32_t deadline = clock_ms + 30000u;
        while (run_active && clock_ms < deadline) {
            if ((clock_ms % 10u) == 0) wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
            plant_step();
        }
    }
    CHECK(run.state == RUN_DONE && !run_active);
    send_line("imutest dist 1000");
    send_line("imutest rpm 40");
}

static void experiment_controller_test(void)
{
    HeadingControl control = {0};
    unsigned iteration;
    control.error_cdeg = 200;
    control.rate_cdeg_s = 1500;
    CHECK(heading_control_rpm(&control, 30) == 5);
    control.experiment_mode = IMUTEST_PD;
    CHECK(heading_control_rpm(&control, 30) == 0);
    control.error_cdeg = -200;
    control.rate_cdeg_s = -1500;
    CHECK(heading_control_rpm(&control, 30) == 0);
    control.experiment_mode = IMUTEST_PID;
    control.error_cdeg = 300;
    control.rate_cdeg_s = 0;
    for (iteration = 0; iteration < 301; iteration++) {
        control.sample_sequence++;
        control.sample_ms += 20;
        CHECK(heading_experiment_drive(&control, -40, 1) <= 20);
    }
    CHECK(control.integral_rpm > 1.4f && control.integral_rpm < 1.5f);
    {
        float previous = control.integral_rpm;
        (void)heading_experiment_drive(&control, 40, 1);
        CHECK(control.integral_rpm == previous);
        control.error_cdeg = 2000;
        control.sample_sequence++;
        control.sample_ms += 20;
        CHECK(heading_experiment_drive(&control, 10, 1) == 9);
        CHECK(control.integral_rpm < previous);
        previous = control.integral_rpm;
        control.error_cdeg = 0;
        control.sample_sequence++;
        control.sample_ms += 20;
        (void)heading_experiment_drive(&control, 40, 1);
        CHECK(control.integral_rpm < previous);
    }
    (void)heading_experiment_drive(&control, 40, 0);
    CHECK(control.integral_rpm == 0.0f && !control.integral_seeded);
    (void)heading_experiment_drive(&control, 9, 1);
    CHECK(!control.integral_seeded);
}

static void experiment_entry_test(void)
{
    static const char *const modes[] = {"imutest pd", "imutest gyro", "imutest pid"};
    static const char *const actions[] = {"imutest b", "imutest l", "imutest r"};
    unsigned iteration, wheel, action;
    send_line("imu baud 460800");
    CHECK(strstr(output, "host test only"));
    send_line("imu baud 12345");
    CHECK(strstr(output, "baud rejected"));
    reset_plant(0, 1.0);
    send_line("imutest");
    CHECK(strstr(output, "[imutest v2]") && strstr(output, " | l | r | "));
    send_line("imutest pid");
    send_line("imutest f");
    CHECK(!run_active && strstr(output, "fresh gyro 0x52 required"));
    wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
    send_line("imutest f");
    CHECK(run_active && run.travel_axis == BODY_FORWARD && run.travel_sign == 1);
    CHECK(run.remaining_mm == IMUTEST_DISTANCE_MM && run.heading_control.experiment_mode == IMUTEST_PID);
    send_line("imutest pd");
    CHECK(strstr(output, "chassis busy") && run.heading_control.experiment_mode == IMUTEST_PID);
    for (iteration = 0; iteration < 900; iteration++) {
        if (iteration % 10u == 0) wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        plant_step();
    }
    CHECK(run_active && run.progress_mm > 10 && run.forward_cmd_rpm > 0);
    CHECK(run.forward_cmd_rpm <= IMUTEST_FORWARD_RPM);
    for (iteration = 0; iteration < 250; iteration++) plant_step();
    CHECK(!run_active && run.state == RUN_FAULT && run.heading_control.integral_rpm == 0.0f);
    for (wheel = 0; wheel < 4; wheel++) CHECK(!simulated_wheel_rpm[wheel]);
    wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
    plant_step();
    CHECK(!run_active && run.state == RUN_FAULT);
    send_line("imutest f");
    CHECK(run_active && run.state == RUN_PREFLIGHT);
    stop_request();
    CHECK(!run_active);
    for (wheel = 0; wheel < 4; wheel++) CHECK(!simulated_wheel_rpm[wheel]);
    for (iteration = 0; iteration < 9u; iteration++) {
        uint32_t start;
        double last_yaw;
        unsigned mode = iteration / 3u;
        action = iteration % 3u;
        int direction = action == 1u ? 1 : -1;
        uint16_t target_mm = IMUTEST_DISTANCE_MM;
        int lateral_seen = 0, integral_seen = 0;
        reset_plant(0, 1.0);
        send_line(modes[mode]);
        wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        send_line(actions[action]);
        CHECK(run_active && run.travel_sign == direction);
        CHECK(run.travel_axis == (action ? BODY_LEFT : BODY_FORWARD));
        start = clock_ms;
        last_yaw = physical_yaw;
        while (run_active && clock_ms - start < 25000u) {
            if ((clock_ms - start) % 10u == 0 && clock_ms != start) {
                int16_t raw = (int16_t)lround((physical_yaw - last_yaw) * 100.0 * 512.0 / 3125.0);
                wit_packet(&heading_sensor, 0x52, raw, clock_ms, 0);
                last_yaw = physical_yaw;
            }
            plant_step();
            if (run.heading_control.integral_rpm != 0.0f) integral_seen = 1;
            if (action && run.progress_mm < target_mm / 2) {
                int lateral = -simulated_wheel_rpm[0] - simulated_wheel_rpm[1]
                    + simulated_wheel_rpm[2] + simulated_wheel_rpm[3];
                if (lateral) {
                    CHECK(lateral * direction > 0 && abs(lateral) <= 4 * IMUTEST_FORWARD_RPM);
                    lateral_seen = 1;
                }
            }
        }
        CHECK(!action || lateral_seen);
        CHECK(mode == 2u ? integral_seen : !integral_seen);
        CHECK(run.state == RUN_DONE && !run_active);
        CHECK(abs(run.progress_mm - target_mm) <= CAR_DISTANCE_TOLERANCE_MM);
    }
    for (action = 1; action < 3u; action++) {
        reset_plant(0, 1.0);
        send_line("imutest pid");
        wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
        send_line(actions[action]);
        for (iteration = 0; iteration < 900u; iteration++) {
            if (iteration % 10u == 0) wit_packet(&heading_sensor, 0x52, 0, clock_ms, 0);
            plant_step();
        }
        CHECK(run_active && run.progress_mm > 10);
        if (action == 1u) {
            for (iteration = 0; iteration < 250u; iteration++) plant_step();
            CHECK(run.state == RUN_FAULT);
        } else {
            stop_request();
            CHECK(run.state == RUN_STOPPED);
        }
        CHECK(!run_active && run.heading_control.integral_rpm == 0.0f);
        for (wheel = 0; wheel < 4; wheel++) CHECK(!simulated_wheel_rpm[wheel]);
    }
    reset_plant(0, 1.0);
    send_line("imutest pd");
    send_line("imutest turnl");
    CHECK(run_active && run.turn_only && run.heading_control.target_cdeg == 9000);
    finish_plant(15000);
    reset_plant(0, 1.0);
    send_line("imutest dist 250");
    CHECK(strstr(output, "dist_mm=250"));
    send_line("imutest rpm 15");
    CHECK(strstr(output, "rpm_limit=15"));
    send_line("imutest dist 2001");
    CHECK(strstr(output, "rejected"));
    send_line("imutest rpm 9");
    CHECK(strstr(output, "rejected"));
    send_line("imutest");
    CHECK(strstr(output, "dist_mm=250 rpm_limit=15"));
    send_line("imutest f");
    CHECK(run_active && run.remaining_mm == 250 && run.translation_rpm_limit == 15);
    send_line("imutest rpm 60");
    CHECK(strstr(output, "chassis busy"));
    for (iteration = 0; iteration < 900u; iteration++) {
        plant_step();
        CHECK(abs(run.forward_cmd_rpm) <= 15);
    }
    CHECK(run.progress_mm > 10 && run.forward_cmd_rpm > 0);
    finish_plant(15000);
    CHECK(run.state == RUN_DONE && abs(run.progress_mm - 250) <= CAR_DISTANCE_TOLERANCE_MM);
    send_line("imutest r");
    CHECK(run_active && run.remaining_mm == 250 && run.translation_rpm_limit == 15
          && run.travel_axis == BODY_LEFT && run.travel_sign == -1);
    finish_plant(15000);
    CHECK(run.state == RUN_DONE && abs(run.progress_mm - 250) <= CAR_DISTANCE_TOLERANCE_MM);
    reset_plant(0, 1.0);
    send_line("imutest dist 2000");
    send_line("imutest rpm 200");
    CHECK(strstr(output, "rpm_limit=200"));
    send_line("imutest clear");
    CHECK(strstr(output, "drivers re-enabled") && clog_reset_count == 4);
    send_line("imutest rpm 201");
    CHECK(strstr(output, "rejected"));
    send_line("imutest l");
    CHECK(run_active && run.translation_rpm_limit == 200);
    for (iteration = 0; iteration < 900u; iteration++) plant_step();
    CHECK(run_active && run.progress_mm > 0 && run.forward_mrpm > 0);
    CHECK(run.forward_mrpm <= (int32_t)(900u * CAR_DRIVE_ACCEL_RPM_PER_S));
    for (; iteration < 4000u && run_active; iteration++) {
        plant_step();
        CHECK(run.forward_mrpm <= 200000);
    }
    CHECK(run.progress_mm > 1000);
    stop_request();
    CHECK(!run_active);
}

int main(void)
{
    experiment_sensor_test();
    experiment_course_slip_test();
    experiment_controller_test();
    experiment_entry_test();
    puts("PASS IMU experiment: serial entry, forward/backward, six lateral mode/direction cases, integral, stale gyro and emergency stop");
    return 0;
}
