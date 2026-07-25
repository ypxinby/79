#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "track_sensor.h"

typedef enum {
    TRACK_MODE_SEEK_LINE = 0,
    TRACK_MODE_FOLLOW_LINE,
    TRACK_MODE_TURN_LEFT_90,
    TRACK_MODE_TURN_RIGHT_90,
    TRACK_MODE_TURN_TO_YAW,
    TRACK_MODE_DRIVE_HEADING,
    TRACK_MODE_LOST_RECOVER,
    TRACK_MODE_IDLE,
    TRACK_MODE_DRIVE_DISTANCE,
    TRACK_MODE_DRIVE_DISTANCE_HEADING
} TrackRunMode;

typedef enum {
    DRIVE_DISTANCE_STATE_IDLE = 0,
    DRIVE_DISTANCE_STATE_DRIVE,
    DRIVE_DISTANCE_STATE_SLOW,
    DRIVE_DISTANCE_STATE_SETTLE,
    DRIVE_DISTANCE_STATE_DONE,
    DRIVE_DISTANCE_STATE_ABORTED,
    DRIVE_DISTANCE_STATE_ERROR
} DriveDistanceState;

typedef enum {
    CAR_TURN_POLICY_AUTO = 0,
    CAR_TURN_POLICY_REPORT_ONLY,
    CAR_TURN_POLICY_IGNORE
} CarTurnHandlingPolicy;

typedef enum {
    CAR_CONTROLLER_ERROR_NONE = 0,
    CAR_CONTROLLER_ERROR_IMU_NOT_READY,
    CAR_CONTROLLER_ERROR_YAW_TURN_TIMEOUT,
    CAR_CONTROLLER_ERROR_INVALID_MODE,
    CAR_CONTROLLER_ERROR_ENCODER_NOT_READY,
    CAR_CONTROLLER_ERROR_HEADING_START_MISMATCH
} CarControllerErrorCode;

typedef struct {
    uint8_t current_lap;
    uint8_t sensor_raw;
    uint8_t black_count;
    TrackRunMode run_mode;
    uint8_t has_seen_line;

    int16_t line_error;
    int16_t last_error;
    int16_t last_valid_error;
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
    uint16_t heading_imu_invalid_elapsed_ms;
    uint16_t drive_distance_settle_elapsed_ms;
    uint16_t lap_cooldown_ms;

    float yaw_turn_target_deg;
    float yaw_turn_error_deg;
    uint32_t yaw_turn_timeout_ms;
    float drive_heading_target_yaw_deg;
    DriveDistanceState drive_distance_state;
    float drive_distance_start_center_cm;
    float drive_distance_target_cm;
    float drive_distance_travelled_cm;
    float drive_distance_remaining_cm;
    int16_t drive_distance_command;
    bool drive_distance_heading_enabled;
    bool drive_distance_heading_start_mismatch;
    float drive_distance_target_yaw_deg;
    float drive_distance_heading_error_deg;
    int16_t drive_distance_heading_correction;
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
    bool distance_completed;
    bool operation_failed;
    CarControllerErrorCode error_code;
} CarControllerFeedback;

extern AppRuntime g_appRuntime;

void CarController_Init(void);
void CarController_ResetRuntime(void);
void CarController_ResetTransientState(void);
void CarController_Update_20ms(uint32_t elapsed_ms);
void CarController_Stop(void);
void CarController_StartSeekLine(void);
void CarController_StartFollowLine(CarTurnHandlingPolicy turn_policy);
void CarController_StartTurnLeft90(void);
void CarController_StartTurnRight90(void);
void CarController_StartTurnToYawRelative(float angle_deg,
    uint32_t timeout_ms);
void CarController_StartDriveHeading(float target_yaw_deg,
    uint32_t duration_ms);
void CarController_StartDriveDistance(float distance_cm,
    int16_t normalized_command);
void CarController_StartDriveDistanceAtYaw(float distance_cm,
    float target_yaw_deg, int16_t normalized_command);
void CarController_SetSafetyHold(bool enable);
bool CarController_IsSafetyHoldActive(void);
TrackRunMode CarController_GetRunMode(void);
const CarControllerFeedback *CarController_GetFeedback(void);
const char *CarController_RunModeToString(TrackRunMode mode);

#endif
