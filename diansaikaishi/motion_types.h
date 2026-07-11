#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

#include <stdint.h>

#define MOTION_USE_GLOBAL_SPEED     ((int16_t)-1)

typedef enum {
    MOTION_ACTION_SEEK_LINE = 0,
    MOTION_ACTION_FOLLOW_LINE,
    MOTION_ACTION_TURN_LEFT_90,
    MOTION_ACTION_TURN_RIGHT_90,
    MOTION_ACTION_WAIT,
    MOTION_ACTION_STOP
} MotionActionType;

typedef enum {
    MOTION_RESULT_IDLE = 0,
    MOTION_RESULT_RUNNING,
    MOTION_RESULT_SUCCESS,
    MOTION_RESULT_FAILED,
    MOTION_RESULT_TIMEOUT,
    MOTION_RESULT_CANCELLED
} MotionActionResult;

typedef enum {
    FOLLOW_END_NEVER = 0,
    FOLLOW_END_LEFT_90_DETECTED,
    FOLLOW_END_RIGHT_90_DETECTED,
    FOLLOW_END_ANY_90_DETECTED,
    FOLLOW_END_START_LINE_DETECTED,
    FOLLOW_END_LINE_LOST,
    FOLLOW_END_DURATION,
    FOLLOW_END_LAP_COUNT
} FollowEndCondition;

typedef enum {
    TURN_POLICY_AUTO = 0,
    TURN_POLICY_REPORT_ONLY,
    TURN_POLICY_IGNORE
} TurnHandlingPolicy;

typedef enum {
    YAW_REFERENCE_CURRENT = 0,
    YAW_REFERENCE_MISSION_START,
    YAW_REFERENCE_ABSOLUTE
} YawReferenceType;

typedef enum {
    MOTION_ERROR_NONE = 0,
    MOTION_ERROR_SEEK_TIMEOUT,
    MOTION_ERROR_FOLLOW_TIMEOUT,
    MOTION_ERROR_TURN_TIMEOUT,
    MOTION_ERROR_LINE_LOST,
    MOTION_ERROR_IMU_NOT_READY,
    MOTION_ERROR_INVALID_ACTION,
    MOTION_ERROR_INVALID_MISSION
} MotionErrorCode;

typedef struct {
    YawReferenceType reference;
    float angle_deg;
} MotionYawTarget;

typedef struct {
    MotionActionType type;
    uint32_t timeout_ms;
    uint8_t max_retries;

    union {
        struct {
            MotionYawTarget yaw;
            int16_t speed_override;
        } seek_line;

        struct {
            FollowEndCondition end_condition;
            TurnHandlingPolicy turn_policy;
            uint32_t duration_ms;
            uint8_t target_laps;
            int16_t speed_override;
        } follow_line;

        struct {
            int16_t speed_override;
            uint16_t min_turn_ms;
        } turn_90;

        struct {
            uint32_t wait_ms;
        } wait;
    } params;
} MotionAction;

#endif
