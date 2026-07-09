#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdint.h>

typedef struct {
    uint8_t current_lap;
    uint8_t sensor_raw;
    uint8_t black_count;

    int16_t line_error;
    int16_t last_error;
    int16_t correction;

    int16_t left_speed;
    int16_t right_speed;

    uint8_t lost_count;
    uint16_t lap_cooldown_ms;
} AppRuntime;

extern AppRuntime g_appRuntime;

void CarController_Init(void);
void CarController_ResetRuntime(void);
void CarController_Update_20ms(void);

#endif
