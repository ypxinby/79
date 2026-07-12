#ifndef MISSION_LIBRARY_H
#define MISSION_LIBRARY_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"

#define MISSION_MAX_ACTIONS     (32U)
#define MISSION_ID_LEGACY       (0U)
#define MISSION_ID_TEST_SF      (1U)
#define MISSION_ID_TEST_R90     (2U)
#define MISSION_ID_TEST_RSTOP   (3U)

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

#define ACTION_SEEK_CURRENT_YAW(yaw_deg, timeout) \
    { \
        .type = MOTION_ACTION_SEEK_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.seek_line = { \
            .yaw = { \
                .reference = YAW_REFERENCE_CURRENT, \
                .angle_deg = (float)(yaw_deg) \
            }, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_SEEK_MISSION_YAW(yaw_deg, timeout) \
    { \
        .type = MOTION_ACTION_SEEK_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.seek_line = { \
            .yaw = { \
                .reference = YAW_REFERENCE_MISSION_START, \
                .angle_deg = (float)(yaw_deg) \
            }, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_SEEK_SECOND_CONFIG(timeout) \
    { \
        .type = MOTION_ACTION_SEEK_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.seek_line = { \
            .yaw = { \
                .reference = YAW_REFERENCE_SECOND_SEEK_CONFIG, \
                .angle_deg = 0.0f \
            }, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

#define ACTION_FOLLOW_FOREVER(timeout) \
    { \
        .type = MOTION_ACTION_FOLLOW_LINE, \
        .timeout_ms = (uint32_t)(timeout), \
        .max_retries = 1U, \
        .params.follow_line = { \
            .end_condition = FOLLOW_END_NEVER, \
            .turn_policy = TURN_POLICY_AUTO, \
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
            .duration_ms = 0U, \
            .target_laps = 0U, \
            .speed_override = MOTION_USE_GLOBAL_SPEED \
        } \
    }

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
