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

    g_appConfig.max_correction = 160;
    g_appConfig.min_max_correction = 50;
    g_appConfig.max_max_correction = 500;

    g_appConfig.track_kp = 120;
    g_appConfig.track_kd = 35;
    g_appConfig.track_scale = 100;

    g_appConfig.start_line_threshold = 5;
    g_appConfig.lost_line_threshold = 15;
    g_appConfig.lap_cooldown_ms = 2000;
}

void AppConfig_LimitAll(void)
{
    g_appConfig.target_laps = clamp_u8(g_appConfig.target_laps,
        g_appConfig.min_target_laps, g_appConfig.max_target_laps);

    g_appConfig.base_speed = clamp_i16(g_appConfig.base_speed,
        g_appConfig.min_base_speed, g_appConfig.max_base_speed);

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
