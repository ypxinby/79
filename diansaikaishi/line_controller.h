#ifndef LINE_CONTROLLER_H
#define LINE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_TURN_DIRECTION_UNKNOWN = 0,
    LINE_TURN_DIRECTION_LEFT = -1,
    LINE_TURN_DIRECTION_RIGHT = 1
} LineTurnDirection;

typedef struct {
    bool enabled;
    bool config_valid;
    bool dt_valid;
    bool initialized;
    bool line_valid;
    uint8_t sensor_pattern;
    uint8_t active_count;
    int16_t raw_error;
    float filtered_error;
    float raw_derivative;
    float filtered_derivative;
    float correction_raw;
    int16_t correction;
    int16_t base_command;
    int16_t left_target_command;
    int16_t right_target_command;
    bool left_low_speed_zeroed;
    bool right_low_speed_zeroed;

    LineTurnDirection last_turn_direction;
    bool direction_valid;
    uint8_t last_valid_pattern;
    int16_t last_valid_error;
    uint32_t direction_update_count;
    uint32_t update_count;
} LineControllerRuntime;

void LineController_Init(void);
void LineController_Reset(void);
void LineController_ResetControlState(void);
void LineController_ObserveSensors(uint8_t sensor_pattern);
void LineController_Update(uint32_t elapsed_ms, uint8_t sensor_pattern,
    int16_t base_command, int16_t *left_command, int16_t *right_command);
const LineControllerRuntime *LineController_GetRuntime(void);

#endif
