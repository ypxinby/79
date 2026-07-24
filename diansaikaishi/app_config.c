#include "app_config.h"
#include "app_features.h"
#include "track_sensor.h"

AppConfig g_appConfig;

static int16_t clamp_i16(int16_t value, int16_t minValue, int16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static uint8_t clamp_u8(uint8_t value, uint8_t minValue, uint8_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

void AppConfig_InitDefault(void)
{
    g_appConfig.target_laps = 1;
    g_appConfig.min_target_laps = 1;
    g_appConfig.max_target_laps = 99;

    g_appConfig.base_speed = 160;
    g_appConfig.min_base_speed = 100;
    g_appConfig.max_base_speed = 700;

    g_appConfig.search_speed = 100;
    g_appConfig.min_search_speed = 50;
    g_appConfig.max_search_speed = 300;

    g_appConfig.recover_speed = 100;
    g_appConfig.min_recover_speed = 50;
    g_appConfig.max_recover_speed = 300;

    g_appConfig.turn_speed = 130;
    g_appConfig.min_turn_speed = 50;
    g_appConfig.max_turn_speed = 300;

    g_appConfig.max_correction = 160;
    g_appConfig.min_max_correction = 50;
    g_appConfig.max_max_correction = 500;

    g_appConfig.track_kp = 100;
    g_appConfig.track_kd = 50;
    g_appConfig.track_scale = 100;

    g_appConfig.start_line_threshold = 5;
    g_appConfig.lost_line_threshold = 15;
    g_appConfig.lap_cooldown_ms = 2000;
    g_appConfig.lost_recover_max_ms = 1400;
    g_appConfig.turn_min_ms = 160;
    g_appConfig.turn_max_ms = 1400;
    g_appConfig.yaw_turn_timeout_ms = YAW_TURN_TIMEOUT_MS_DEFAULT;
    g_appConfig.yaw_turn_slow_threshold_deg =
        YAW_TURN_SLOW_THRESHOLD_DEG_DEFAULT;
    g_appConfig.yaw_turn_done_tolerance_deg =
        YAW_TURN_DONE_TOLERANCE_DEG_DEFAULT;
    g_appConfig.yaw_turn_settle_gyro_dps =
        YAW_TURN_SETTLE_GYRO_DPS_DEFAULT;
    g_appConfig.yaw_turn_settle_ms = YAW_TURN_SETTLE_MS_DEFAULT;
    g_appConfig.yaw_turn_min_slow_command =
        YAW_TURN_MIN_SLOW_COMMAND_DEFAULT;
    g_appConfig.heading_imu_invalid_grace_ms =
        HEADING_IMU_INVALID_GRACE_MS_DEFAULT;
    g_appConfig.gyro_deadband_dps = 1.5f;
    g_appConfig.imu_dt_min_ms = IMU_DT_MIN_MS_DEFAULT;
    g_appConfig.imu_dt_max_ms = IMU_DT_MAX_MS_DEFAULT;
    g_appConfig.imu_short_gap_max_ms = IMU_SHORT_GAP_MAX_MS_DEFAULT;
    g_appConfig.imu_stale_timeout_ms = IMU_STALE_TIMEOUT_MS_DEFAULT;
    g_appConfig.imu_max_abs_gyro_dps =
        IMU_MAX_ABS_GYRO_DPS_DEFAULT;
    g_appConfig.imu_yaw_axis_sign = IMU_YAW_AXIS_SIGN_DEFAULT;

    g_appConfig.heading_kp = 20;
    g_appConfig.heading_kd = 5;
    g_appConfig.heading_scale = 10;
    g_appConfig.heading_max_correction = 30;

    g_appConfig.servo_angle_deg = 90;
    g_appConfig.min_servo_angle_deg = 35;
    g_appConfig.max_servo_angle_deg = 145;

    g_appConfig.avoid_turn_out_deg = 35;
    g_appConfig.avoid_drive_out_ms = 1400;
    g_appConfig.avoid_turn_to_line_deg = -55;
    g_appConfig.avoid_wait_before_ms = 2000;
    g_appConfig.avoid_resume_grace_ms = 200;
    g_appConfig.avoid_reacquire_settle_ms = 300;
    g_appConfig.avoid_reacquire_timeout_ms = 5000U;

    /* P3 defaults use the physically measured encoder and chassis geometry. */
    g_appConfig.encoder_ppr_x2 = WHEEL_ENCODER_PPR_X2_DEFAULT;
    g_appConfig.wheel_diameter_cm = WHEEL_DIAMETER_CM_DEFAULT;
    g_appConfig.wheel_track_cm = WHEEL_TRACK_CM_DEFAULT;
    g_appConfig.left_encoder_direction =
        LEFT_ENCODER_DIRECTION_DEFAULT;
    g_appConfig.right_encoder_direction =
        RIGHT_ENCODER_DIRECTION_DEFAULT;

    /* P4 defaults include the 30 cm/s fast-settling vehicle-test update. */
    g_appConfig.wheel_control_max_speed_cmps =
        WHEEL_CONTROL_MAX_SPEED_CMPS_DEFAULT;
    g_appConfig.wheel_control_left_kp = WHEEL_CONTROL_LEFT_KP_DEFAULT;
    g_appConfig.wheel_control_left_ki = WHEEL_CONTROL_LEFT_KI_DEFAULT;
    g_appConfig.wheel_control_right_kp = WHEEL_CONTROL_RIGHT_KP_DEFAULT;
    g_appConfig.wheel_control_right_ki = WHEEL_CONTROL_RIGHT_KI_DEFAULT;
    g_appConfig.wheel_control_left_kp_overspeed =
        WHEEL_CONTROL_LEFT_KP_OVERSPEED_DEFAULT;
    g_appConfig.wheel_control_right_kp_overspeed =
        WHEEL_CONTROL_RIGHT_KP_OVERSPEED_DEFAULT;
#if FEATURE_MOTOR_CONTROL_FAST_SETTLING
    g_appConfig.wheel_control_left_feedforward_gain =
        WHEEL_CONTROL_LEFT_FF_GAIN_DEFAULT;
    g_appConfig.wheel_control_right_feedforward_gain =
        WHEEL_CONTROL_RIGHT_FF_GAIN_DEFAULT;
#else
    g_appConfig.wheel_control_left_feedforward_gain =
        WHEEL_CONTROL_LEFT_FF_GAIN_LEGACY;
    g_appConfig.wheel_control_right_feedforward_gain =
        WHEEL_CONTROL_RIGHT_FF_GAIN_LEGACY;
#endif
    g_appConfig.wheel_control_integral_limit =
        WHEEL_CONTROL_INTEGRAL_LIMIT_DEFAULT;
    g_appConfig.wheel_control_integral_release_multiplier =
        WHEEL_CONTROL_INTEGRAL_RELEASE_MULTIPLIER_DEFAULT;
    g_appConfig.wheel_control_max_accel_cmps2 =
        WHEEL_CONTROL_MAX_ACCEL_CMPS2_DEFAULT;
    g_appConfig.wheel_control_max_decel_cmps2 =
        WHEEL_CONTROL_MAX_DECEL_CMPS2_DEFAULT;
    g_appConfig.wheel_control_target_timeout_ms =
        WHEEL_CONTROL_TARGET_TIMEOUT_MS_DEFAULT;

    /* P5.1 uses its own fixed normalized base command and filtered PD loop. */
    g_appConfig.line_control_v2_base_command =
        LINE_CONTROL_V2_BASE_COMMAND_DEFAULT;
    /* P5.2A fixed wheel command and bounded all-white search time. */
    g_appConfig.line_control_v2_lost_turn_command =
        LINE_CONTROL_V2_LOST_TURN_COMMAND_DEFAULT;
    g_appConfig.line_lost_search_max_ms =
        LINE_LOST_SEARCH_MAX_MS_DEFAULT;
    g_appConfig.line_control_v2_error_filter_alpha =
        LINE_CONTROL_V2_ERROR_FILTER_ALPHA_DEFAULT;
    g_appConfig.line_control_v2_derivative_filter_alpha =
        LINE_CONTROL_V2_DERIV_FILTER_ALPHA_DEFAULT;
    g_appConfig.line_control_v2_kp = LINE_CONTROL_V2_KP_DEFAULT;
    g_appConfig.line_control_v2_kd = LINE_CONTROL_V2_KD_DEFAULT;
    g_appConfig.line_control_v2_max_correction =
        LINE_CONTROL_V2_MAX_CORRECTION_DEFAULT;
    g_appConfig.line_control_v2_min_running_command =
        LINE_CONTROL_V2_MIN_RUNNING_COMMAND_DEFAULT;
    g_appConfig.line_control_v2_min_dt_ms =
        LINE_CONTROL_V2_MIN_DT_MS_DEFAULT;
    g_appConfig.line_control_v2_max_dt_ms =
        LINE_CONTROL_V2_MAX_DT_MS_DEFAULT;
}

void AppConfig_LimitAll(void)
{
    g_appConfig.target_laps = clamp_u8(g_appConfig.target_laps,
        g_appConfig.min_target_laps, g_appConfig.max_target_laps);

    g_appConfig.base_speed = clamp_i16(g_appConfig.base_speed,
        g_appConfig.min_base_speed, g_appConfig.max_base_speed);

    g_appConfig.search_speed = clamp_i16(g_appConfig.search_speed,
        g_appConfig.min_search_speed, g_appConfig.max_search_speed);

    g_appConfig.recover_speed = clamp_i16(g_appConfig.recover_speed,
        g_appConfig.min_recover_speed, g_appConfig.max_recover_speed);

    g_appConfig.turn_speed = clamp_i16(g_appConfig.turn_speed,
        g_appConfig.min_turn_speed, g_appConfig.max_turn_speed);

    g_appConfig.max_correction = clamp_i16(g_appConfig.max_correction,
        g_appConfig.min_max_correction, g_appConfig.max_max_correction);

    g_appConfig.track_kp = clamp_i16(g_appConfig.track_kp, 0, 1000);
    g_appConfig.track_kd = clamp_i16(g_appConfig.track_kd, 0, 1000);
    g_appConfig.track_scale = clamp_i16(g_appConfig.track_scale, 1, 1000);
    g_appConfig.start_line_threshold =
        clamp_u8(g_appConfig.start_line_threshold, 1,
            (uint8_t)TRACK_SENSOR_COUNT);
    g_appConfig.lost_line_threshold =
        clamp_u8(g_appConfig.lost_line_threshold, 1, 100);
    if (g_appConfig.lap_cooldown_ms < 200U) {
        g_appConfig.lap_cooldown_ms = 200U;
    }
    if (g_appConfig.lap_cooldown_ms > 5000U) {
        g_appConfig.lap_cooldown_ms = 5000U;
    }
    if (g_appConfig.lost_recover_max_ms < 100U) {
        g_appConfig.lost_recover_max_ms = 100U;
    }
    if (g_appConfig.lost_recover_max_ms > 2000U) {
        g_appConfig.lost_recover_max_ms = 2000U;
    }
    if (g_appConfig.turn_min_ms < 40U) {
        g_appConfig.turn_min_ms = 40U;
    }
    if (g_appConfig.turn_min_ms > 1000U) {
        g_appConfig.turn_min_ms = 1000U;
    }
    if (g_appConfig.turn_max_ms < g_appConfig.turn_min_ms) {
        g_appConfig.turn_max_ms = g_appConfig.turn_min_ms;
    }
    if (g_appConfig.turn_max_ms > 2000U) {
        g_appConfig.turn_max_ms = 2000U;
    }
    if (g_appConfig.yaw_turn_timeout_ms < 200U) {
        g_appConfig.yaw_turn_timeout_ms = 200U;
    }
    if (g_appConfig.yaw_turn_timeout_ms > 10000U) {
        g_appConfig.yaw_turn_timeout_ms = 10000U;
    }
    if ((g_appConfig.yaw_turn_slow_threshold_deg <= 0.0f) ||
        (g_appConfig.yaw_turn_slow_threshold_deg > 90.0f)) {
        g_appConfig.yaw_turn_slow_threshold_deg =
            YAW_TURN_SLOW_THRESHOLD_DEG_DEFAULT;
    }
    if ((g_appConfig.yaw_turn_done_tolerance_deg <= 0.0f) ||
        (g_appConfig.yaw_turn_done_tolerance_deg >=
            g_appConfig.yaw_turn_slow_threshold_deg)) {
        g_appConfig.yaw_turn_done_tolerance_deg =
            YAW_TURN_DONE_TOLERANCE_DEG_DEFAULT;
    }
    if ((g_appConfig.yaw_turn_settle_gyro_dps <= 0.0f) ||
        (g_appConfig.yaw_turn_settle_gyro_dps > 100.0f)) {
        g_appConfig.yaw_turn_settle_gyro_dps =
            YAW_TURN_SETTLE_GYRO_DPS_DEFAULT;
    }
    if ((g_appConfig.yaw_turn_settle_ms == 0U) ||
        (g_appConfig.yaw_turn_settle_ms > 2000U)) {
        g_appConfig.yaw_turn_settle_ms = YAW_TURN_SETTLE_MS_DEFAULT;
    }
    g_appConfig.yaw_turn_min_slow_command = clamp_i16(
        g_appConfig.yaw_turn_min_slow_command, 0, 1000);
    if ((g_appConfig.heading_imu_invalid_grace_ms < 20U) ||
        (g_appConfig.heading_imu_invalid_grace_ms > 1000U)) {
        g_appConfig.heading_imu_invalid_grace_ms =
            HEADING_IMU_INVALID_GRACE_MS_DEFAULT;
    }
    if ((g_appConfig.imu_dt_min_ms == 0U) ||
        (g_appConfig.imu_dt_min_ms > 20U)) {
        g_appConfig.imu_dt_min_ms = IMU_DT_MIN_MS_DEFAULT;
    }
    if ((g_appConfig.imu_dt_max_ms < g_appConfig.imu_dt_min_ms) ||
        (g_appConfig.imu_dt_max_ms > 100U)) {
        g_appConfig.imu_dt_max_ms = IMU_DT_MAX_MS_DEFAULT;
    }
    if ((g_appConfig.imu_short_gap_max_ms == 0U) ||
        (g_appConfig.imu_short_gap_max_ms > 500U)) {
        g_appConfig.imu_short_gap_max_ms =
            IMU_SHORT_GAP_MAX_MS_DEFAULT;
    }
    if ((g_appConfig.imu_stale_timeout_ms <=
            g_appConfig.imu_short_gap_max_ms) ||
        (g_appConfig.imu_stale_timeout_ms > 1000U)) {
        g_appConfig.imu_stale_timeout_ms =
            IMU_STALE_TIMEOUT_MS_DEFAULT;
    }
    if ((g_appConfig.imu_max_abs_gyro_dps <= 0.0f) ||
        (g_appConfig.imu_max_abs_gyro_dps > 500.0f)) {
        g_appConfig.imu_max_abs_gyro_dps =
            IMU_MAX_ABS_GYRO_DPS_DEFAULT;
    }
    if ((g_appConfig.imu_yaw_axis_sign != -1) &&
        (g_appConfig.imu_yaw_axis_sign != 1)) {
        g_appConfig.imu_yaw_axis_sign = IMU_YAW_AXIS_SIGN_DEFAULT;
    }
    g_appConfig.heading_kp = clamp_i16(g_appConfig.heading_kp, 0, 1000);
    g_appConfig.heading_kd = clamp_i16(g_appConfig.heading_kd, 0, 1000);
    g_appConfig.heading_scale =
        clamp_i16(g_appConfig.heading_scale, 1, 1000);
    g_appConfig.heading_max_correction =
        clamp_i16(g_appConfig.heading_max_correction, 0, 200);
    g_appConfig.servo_angle_deg =
        clamp_i16(g_appConfig.servo_angle_deg,
            g_appConfig.min_servo_angle_deg,
            g_appConfig.max_servo_angle_deg);
    g_appConfig.avoid_turn_out_deg =
        clamp_i16(g_appConfig.avoid_turn_out_deg, -90, 90);
    g_appConfig.avoid_turn_to_line_deg =
        clamp_i16(g_appConfig.avoid_turn_to_line_deg, -120, 120);
    if (g_appConfig.avoid_drive_out_ms < 100U) {
        g_appConfig.avoid_drive_out_ms = 100U;
    }
    if (g_appConfig.avoid_drive_out_ms > 5000U) {
        g_appConfig.avoid_drive_out_ms = 5000U;
    }
    if (g_appConfig.avoid_wait_before_ms > 5000U) {
        g_appConfig.avoid_wait_before_ms = 5000U;
    }
    if (g_appConfig.avoid_resume_grace_ms > 2000U) {
        g_appConfig.avoid_resume_grace_ms = 2000U;
    }
    if (g_appConfig.avoid_reacquire_settle_ms < 100U) {
        g_appConfig.avoid_reacquire_settle_ms = 100U;
    }
    if (g_appConfig.avoid_reacquire_settle_ms > 2000U) {
        g_appConfig.avoid_reacquire_settle_ms = 2000U;
    }
    if (g_appConfig.avoid_reacquire_timeout_ms < 500U) {
        g_appConfig.avoid_reacquire_timeout_ms = 500U;
    }
    if (g_appConfig.avoid_reacquire_timeout_ms > 15000U) {
        g_appConfig.avoid_reacquire_timeout_ms = 15000U;
    }

    g_appConfig.line_control_v2_base_command = clamp_i16(
        g_appConfig.line_control_v2_base_command, 0, 1000);
    g_appConfig.line_control_v2_lost_turn_command = clamp_i16(
        g_appConfig.line_control_v2_lost_turn_command, 0, 1000);
    if ((g_appConfig.line_lost_search_max_ms == 0U) ||
        (g_appConfig.line_lost_search_max_ms > 10000U)) {
        g_appConfig.line_lost_search_max_ms =
            LINE_LOST_SEARCH_MAX_MS_DEFAULT;
    }
    if ((g_appConfig.line_control_v2_error_filter_alpha <= 0.0f) ||
        (g_appConfig.line_control_v2_error_filter_alpha > 1.0f)) {
        g_appConfig.line_control_v2_error_filter_alpha =
            LINE_CONTROL_V2_ERROR_FILTER_ALPHA_DEFAULT;
    }
    if ((g_appConfig.line_control_v2_derivative_filter_alpha <= 0.0f) ||
        (g_appConfig.line_control_v2_derivative_filter_alpha > 1.0f)) {
        g_appConfig.line_control_v2_derivative_filter_alpha =
            LINE_CONTROL_V2_DERIV_FILTER_ALPHA_DEFAULT;
    }
    if ((g_appConfig.line_control_v2_kp < 0.0f) ||
        (g_appConfig.line_control_v2_kp > 10.0f)) {
        g_appConfig.line_control_v2_kp = LINE_CONTROL_V2_KP_DEFAULT;
    }
    if ((g_appConfig.line_control_v2_kd < 0.0f) ||
        (g_appConfig.line_control_v2_kd > 1.0f)) {
        g_appConfig.line_control_v2_kd = LINE_CONTROL_V2_KD_DEFAULT;
    }
    g_appConfig.line_control_v2_max_correction = clamp_i16(
        g_appConfig.line_control_v2_max_correction, 0, 1000);
    g_appConfig.line_control_v2_min_running_command = clamp_i16(
        g_appConfig.line_control_v2_min_running_command, 0, 1000);
    if (g_appConfig.line_control_v2_min_dt_ms == 0U) {
        g_appConfig.line_control_v2_min_dt_ms =
            LINE_CONTROL_V2_MIN_DT_MS_DEFAULT;
    }
    if (g_appConfig.line_control_v2_max_dt_ms <
        g_appConfig.line_control_v2_min_dt_ms) {
        g_appConfig.line_control_v2_max_dt_ms =
            g_appConfig.line_control_v2_min_dt_ms;
    }
}

void AppConfig_IncreaseTargetLap(void)
{
    if (g_appConfig.target_laps < g_appConfig.max_target_laps) {
        g_appConfig.target_laps++;
    }
}

void AppConfig_DecreaseTargetLap(void)
{
    if (g_appConfig.target_laps > g_appConfig.min_target_laps) {
        g_appConfig.target_laps--;
    }
}
