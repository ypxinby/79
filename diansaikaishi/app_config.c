#include "app_config.h"
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
    g_appConfig.gyro_deadband_dps = 1.5f;

    g_appConfig.heading_kp = 20;
    g_appConfig.heading_kd = 5;
    g_appConfig.heading_scale = 10;
    g_appConfig.heading_max_correction = 30;
    g_appConfig.heading_enable_error = 50;
    g_appConfig.heading_enable_derivative = 100;
    g_appConfig.heading_lock_delay_ms = 100;
    g_appConfig.seek_heading_offset_deg = -1;
    g_appConfig.second_seek_angle_deg = 215;

    g_appConfig.servo_angle_deg = 90;
    g_appConfig.min_servo_angle_deg = 35;
    g_appConfig.max_servo_angle_deg = 145;
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
    g_appConfig.heading_kp = clamp_i16(g_appConfig.heading_kp, 0, 1000);
    g_appConfig.heading_kd = clamp_i16(g_appConfig.heading_kd, 0, 1000);
    g_appConfig.heading_scale =
        clamp_i16(g_appConfig.heading_scale, 1, 1000);
    g_appConfig.heading_max_correction =
        clamp_i16(g_appConfig.heading_max_correction, 0, 200);
    g_appConfig.heading_enable_error =
        clamp_i16(g_appConfig.heading_enable_error, 0, 300);
    g_appConfig.heading_enable_derivative =
        clamp_i16(g_appConfig.heading_enable_derivative, 0, 300);
    if (g_appConfig.heading_lock_delay_ms > 2000U) {
        g_appConfig.heading_lock_delay_ms = 2000U;
    }
    g_appConfig.seek_heading_offset_deg =
        clamp_i16(g_appConfig.seek_heading_offset_deg, -45, 45);
    g_appConfig.second_seek_angle_deg =
        clamp_i16(g_appConfig.second_seek_angle_deg, 120, 220);
    g_appConfig.servo_angle_deg =
        clamp_i16(g_appConfig.servo_angle_deg,
            g_appConfig.min_servo_angle_deg,
            g_appConfig.max_servo_angle_deg);
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
