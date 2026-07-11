#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdint.h>

typedef enum {
    TRACK_MODE_SEEK_LINE = 0,
    TRACK_MODE_FOLLOW_LINE,
    TRACK_MODE_TURN_LEFT_90,
    TRACK_MODE_TURN_RIGHT_90,
    TRACK_MODE_LOST_RECOVER
} TrackRunMode;

typedef enum {
    SEEK_HEADING_CURRENT = 0,
    SEEK_HEADING_TARGET
} SeekHeadingMode;

typedef struct {
    uint8_t current_lap;
    uint8_t sensor_raw;
    uint8_t black_count;
    TrackRunMode run_mode;
    uint8_t has_seen_line;

    int16_t line_error;
    int16_t last_error;
    int16_t last_valid_error;
    SeekHeadingMode seek_heading_mode;
    float seek_target_yaw_deg;
    int8_t recover_direction;
    int16_t correction;
    int16_t heading_correction;

    int16_t left_speed;
    int16_t right_speed;

    uint8_t lost_count;
    uint16_t lost_elapsed_ms;
    uint16_t turn_elapsed_ms;
    uint16_t heading_straight_elapsed_ms;
    uint16_t lap_cooldown_ms;
} AppRuntime;

extern AppRuntime g_appRuntime;

void CarController_Init(void);
void CarController_ResetRuntime(void);
void CarController_ResetTransientState(void);
void CarController_Update_20ms(void);
void CarController_Stop(void);
void CarController_StartSeekLine(float target_yaw_deg);
void CarController_StartFollowLine(void);
void CarController_StartTurnLeft90(void);
void CarController_StartTurnRight90(void);
void CarController_UseCurrentHeadingForSeek(void);
void CarController_SetSeekTargetYaw(float target_yaw_deg);
TrackRunMode CarController_GetRunMode(void);
const char *CarController_RunModeToString(TrackRunMode mode);

#endif
