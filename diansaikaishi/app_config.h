#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

typedef struct {
    uint8_t target_laps;
    uint8_t min_target_laps;
    uint8_t max_target_laps;

    int16_t base_speed;
    int16_t min_base_speed;
    int16_t max_base_speed;

    int16_t max_correction;
    int16_t min_max_correction;
    int16_t max_max_correction;

    int16_t track_kp;
    int16_t track_kd;
    int16_t track_scale;

    uint8_t start_line_threshold;
    uint8_t lost_line_threshold;
    uint16_t lap_cooldown_ms;
} AppConfig;

extern AppConfig g_appConfig;

void AppConfig_InitDefault(void);
void AppConfig_LimitAll(void);
void AppConfig_IncreaseTargetLap(void);
void AppConfig_DecreaseTargetLap(void);

#endif
