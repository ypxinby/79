#ifndef MISSION_LIBRARY_H
#define MISSION_LIBRARY_H

#include <stdbool.h>
#include <stdint.h>

#include "app_features.h"
#include "motion_types.h"

#define MISSION_MAX_ACTIONS             (32U)

/* 0~9: legacy debug IDs; newer development-only tests use 100 and above. */
#define MISSION_ID_LEGACY               (0U)
#define MISSION_ID_TEST_SF              (1U)
#define MISSION_ID_TEST_R90             (2U)
#define MISSION_ID_TEST_RSTOP           (3U)
#define MISSION_ID_TEST_SEEK_FOLLOW     (4U)
#define MISSION_ID_TEST_SEEK_STOP       (5U)
#define MISSION_ID_TEST_YAW             (6U)
#define MISSION_ID_TEST_HEAD            (7U)
#define MISSION_ID_TEST_OBSTACLE_FIXED  (8U)
#define MISSION_ID_TEST_STOP_ONLY       (9U)
/* P6.3 development-only IDs live above the competition-map range. */
#define MISSION_ID_TEST_DISTANCE_20     (100U)

/* 10~99: competition missions. */
#define MISSION_ID_TASK1_LAP            (10U)
#define MISSION_ID_H3_BALL_50           (11U)
#define MISSION_ID_H4_AB_CENTER         (12U)
#define MISSION_ID_H5_LAP_CENTER        (13U)
#define MISSION_ID_H6_LAP_LOCK          (14U)

/* Kept as a source-compatible alias for older callers. */
#define MISSION_ID_COMPETITION_MAIN     MISSION_ID_TASK1_LAP

#define ACTION_STOP() \
    { \
        .type = MOTION_ACTION_STOP, \
        .timeout_ms = 0U, \
        .max_retries = 0U \
    }

#define ACTION_WAIT_MS(wait_time_ms) \
    { \
        .type = MOTION_ACTION_WAIT, \
        .timeout_ms = (uint32_t)(wait_time_ms), \
        .max_retries = 0U, \
        .params.wait = { \
            .wait_ms = (uint32_t)(wait_time_ms) \
        } \
    }

#define ACTION_FOLLOW_TO_FINISH(distance_cm, decel_pct, high_pct, low_pct, \
    black_min, confirm_frames, stop_delay_ms, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE_TO_FINISH, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 0U, \
        .params.follow_line_to_finish = { \
            .expected_distance_cm = (float)(distance_cm), \
            .decel_percent = (uint8_t)(decel_pct), \
            .high_speed_percent = (uint16_t)(high_pct), \
            .low_speed_percent = (uint16_t)(low_pct), \
            .finish_black_min = (uint8_t)(black_min), \
            .finish_confirm_frames = (uint8_t)(confirm_frames), \
            .finish_stop_delay_ms = (uint16_t)(stop_delay_ms) \
        } \
    }

#define ACTION_H_TASK(id) \
    { \
        .type = MOTION_ACTION_H_TASK, \
        .timeout_ms = 0U, \
        .max_retries = 0U, \
        .params.h_task = { \
            .task_id = (uint8_t)(id) \
        } \
    }

#if FEATURE_LEGACY_MOTION_CONTROL
#define ACTION_SEEK_LINE(timeout) \
    { \
        .type = MOTION_ACTION_SEEK_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.seek_line = { \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }
#endif

#define ACTION_FOLLOW_FOREVER(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_NEVER, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_FOREVER_WITH_OBSTACLE(policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_NEVER, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_FOR_TIME(duration, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_DURATION, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = (uint32_t)(duration), \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_FOR_TIME_WITH_OBSTACLE(duration, policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_DURATION, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = (uint32_t)(duration), \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_LINE_LOST(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_LINE_LOST, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_LINE_LOST_WITH_OBSTACLE(policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_LINE_LOST, \
            .turn_policy = TURN_POLICY_AUTO, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_LEFT_90(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_LEFT_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_LEFT_90_WITH_OBSTACLE(policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_LEFT_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_RIGHT_90(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_RIGHT_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_RIGHT_90_WITH_OBSTACLE(policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_RIGHT_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_ANY_90(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_ANY_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = OBSTACLE_POLICY_FIXED_BYPASS, \
            .bypass_direction = BYPASS_DIRECTION_RIGHT, \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_UNTIL_ANY_90_WITH_OBSTACLE(policy, direction, timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_ANY_90_DETECTED, \
            .turn_policy = TURN_POLICY_REPORT_ONLY, \
            .obstacle_policy = (policy), \
            .bypass_direction = (direction), \
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#if FEATURE_LEGACY_MOTION_CONTROL
#define ACTION_TURN_LEFT_90(timeout) \
    { \
        .type = MOTION_ACTION_TURN_LEFT_90, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.turn_90 = { \
            .speed_override = MOTION_USE_GLOBAL_SPEED, \
            .min_turn_ms = 0U \
        } \
    }

#define ACTION_TURN_RIGHT_90(timeout) \
    { \
        .type = MOTION_ACTION_TURN_RIGHT_90, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.turn_90 = { \
            .speed_override = MOTION_USE_GLOBAL_SPEED, \
            .min_turn_ms = 0U \
        } \
    }

#endif

#define ACTION_TURN_RELATIVE_YAW(angle, timeout) \
    { \
        .type = MOTION_ACTION_TURN_TO_YAW, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.turn_to_yaw = { \
            .angle_deg = (float)(angle), \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#if FEATURE_LEGACY_MOTION_CONTROL
#define ACTION_DRIVE_HEADING_YAW(target_yaw, duration, timeout) \
    { \
        .type = MOTION_ACTION_DRIVE_HEADING_TIME, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.drive_heading_time = { \
            .target_yaw_deg = (float)(target_yaw), \
            .duration_ms = (uint32_t)(duration), \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }
#endif

#define ACTION_DRIVE_HEADING_UNTIL_LINE(target_yaw, command, timeout) \
    { \
        .type = MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 0U, \
        .params.drive_heading_until_line = { \
            .target_yaw_deg = (float)(target_yaw), \
            .normalized_command = (int16_t)(command) \
        } \
    }

#define ACTION_DRIVE_DISTANCE_FORWARD(distance, command, timeout) \
    { \
        .type = MOTION_ACTION_DRIVE_DISTANCE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 0U, \
        .params.drive_distance = { \
            .distance_cm = (float)(distance), \
            .normalized_command = (int16_t)(command) \
        } \
    }

#define ACTION_DRIVE_DISTANCE_AT_YAW(distance, target_yaw, command, timeout) \
    { \
        .type = MOTION_ACTION_DRIVE_DISTANCE_HEADING, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 0U, \
        .params.drive_distance_heading = { \
            .distance_cm = (float)(distance), \
            .target_yaw_deg = (float)(target_yaw), \
            .normalized_command = (int16_t)(command), \
            .lock_current_yaw_on_start = false \
        } \
    }

#define ACTION_DRIVE_DISTANCE_HOLD_CURRENT_YAW(distance, command, timeout) \
    { \
        .type = MOTION_ACTION_DRIVE_DISTANCE_HEADING, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 0U, \
        .params.drive_distance_heading = { \
            .distance_cm = (float)(distance), \
            .target_yaw_deg = 0.0f, \
            .normalized_command = (int16_t)(command), \
            .lock_current_yaw_on_start = true \
        } \
    }

typedef struct {
    uint8_t mission_id;
    const char *name;
    const MotionAction *actions;
    uint16_t action_count;
    uint8_t control_profile_id;
} MissionDefinition;

uint16_t MissionLibrary_GetCount(void);
const MissionDefinition *MissionLibrary_GetByIndex(uint16_t index);
const MissionDefinition *MissionLibrary_FindById(uint8_t mission_id);
bool MissionLibrary_Validate(const MissionDefinition *mission,
    uint16_t *error_code);

#endif
