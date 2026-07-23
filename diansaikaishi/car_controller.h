#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"
#include "track_sensor.h"

typedef enum {
    TRACK_MODE_SEEK_LINE = 0,
    TRACK_MODE_FOLLOW_LINE,
    TRACK_MODE_TURN_LEFT_90,
    TRACK_MODE_TURN_RIGHT_90,
    TRACK_MODE_TURN_TO_YAW,
    TRACK_MODE_DRIVE_HEADING,
    TRACK_MODE_LOST_RECOVER
} TrackRunMode;

typedef enum {
    SEEK_HEADING_CURRENT = 0,
    SEEK_HEADING_TARGET
} SeekHeadingMode;

typedef enum {
    CAR_TURN_POLICY_AUTO = 0,
    CAR_TURN_POLICY_REPORT_ONLY,
    CAR_TURN_POLICY_IGNORE
} CarTurnHandlingPolicy;

typedef enum {
    CAR_CONTROLLER_ERROR_NONE = 0,
    CAR_CONTROLLER_ERROR_IMU_NOT_READY,
    CAR_CONTROLLER_ERROR_YAW_TURN_TIMEOUT,
    CAR_CONTROLLER_ERROR_INVALID_MODE
} CarControllerErrorCode;

typedef enum {
    HEADING_ACTION_MODE_IDLE = 0,
    HEADING_ACTION_MODE_HOLD,
    HEADING_ACTION_MODE_TURN_FAST,
    HEADING_ACTION_MODE_TURN_SLOW,
    HEADING_ACTION_MODE_TURN_SETTLE
} HeadingActionMode;

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
    uint32_t turn_elapsed_ms;
    uint16_t yaw_turn_stable_ms;
    uint16_t heading_straight_elapsed_ms;
    uint16_t drive_heading_duration_ms;
    uint16_t lap_cooldown_ms;

    float yaw_turn_target_deg;
    float yaw_turn_error_deg;
    uint32_t yaw_turn_timeout_ms;
    float drive_heading_target_yaw_deg;
    HeadingActionMode heading_action_mode;
    MotionActionResult heading_action_result;
    CarControllerErrorCode heading_action_error_code;
} AppRuntime;

typedef struct {
    TrackRunMode run_mode;
    uint8_t sensor_raw;
    uint8_t black_count;
    int16_t line_error;

    bool line_found;
    bool line_lost;
    bool center_detected;

    TrackTurnType detected_turn;
    bool turn_completed;
    bool operation_failed;
    CarControllerErrorCode error_code;
} CarControllerFeedback;

extern AppRuntime g_appRuntime;

void CarController_Init(void);
void CarController_ResetRuntime(void);
void CarController_ResetTransientState(void);
void CarController_Update_20ms(uint32_t elapsed_ms);
void CarController_Stop(void);
void CarController_StartSeekLine(float target_yaw_deg);
void CarController_StartFollowLine(CarTurnHandlingPolicy turn_policy);
void CarController_StartTurnLeft90(void);
void CarController_StartTurnRight90(void);
void CarController_StartTurnToYawRelative(float angle_deg,
    uint32_t timeout_ms);
void CarController_StartDriveHeading(uint32_t duration_ms);
void CarController_UseCurrentHeadingForSeek(void);
void CarController_SetSeekTargetYaw(float target_yaw_deg);
void CarController_SetSafetyHold(bool enable);
bool CarController_IsSafetyHoldActive(void);
TrackRunMode CarController_GetRunMode(void);
const CarControllerFeedback *CarController_GetFeedback(void);
const char *CarController_RunModeToString(TrackRunMode mode);

#endif
