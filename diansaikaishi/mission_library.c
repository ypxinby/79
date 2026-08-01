#include "mission_library.h"

#include "app_features.h"
#include "h_task_controller.h"

#define ARRAY_SIZE(array) \
    ((uint16_t)(sizeof(array) / sizeof((array)[0])))

#if (APP_PROFILE != APP_PROFILE_DEVELOPMENT) && \
    (APP_PROFILE != APP_PROFILE_COMPETITION)
#error "APP_PROFILE must be APP_PROFILE_DEVELOPMENT or APP_PROFILE_COMPETITION"
#endif

typedef enum {
    MISSION_VALIDATE_OK = 0,
    MISSION_VALIDATE_NULL,
    MISSION_VALIDATE_EMPTY,
    MISSION_VALIDATE_TOO_LONG,
    MISSION_VALIDATE_INVALID_ACTION,
    MISSION_VALIDATE_MISSING_TIMEOUT,
    MISSION_VALIDATE_MISSING_STOP,
    MISSION_VALIDATE_TOO_MANY_RETRIES
} MissionValidateError;

/*
 * H-task chassis Task 1.  The 300 command remains the OLED tuning reference.
 * At mission start the action resolves HIGH=250% (750) and LOW=50% (150),
 * while the shared FF/KP/KD/MAX defaults are intentionally biased toward
 * high-speed cornering and remain live-tunable during the run.
 * Distance is deliberately centralized here for the final track measurement.
 */
#define TASK1_EXPECTED_DISTANCE_CM       (614.0f)
#define TASK1_DECEL_PERCENT              (90U)
#define TASK1_HIGH_SPEED_PERCENT         (250U)
#define TASK1_LOW_SPEED_PERCENT          (50U)
#define TASK1_FINISH_BLACK_MIN           (5U)
#define TASK1_FINISH_CONFIRM_FRAMES      (2U)
#define TASK1_TIMEOUT_MS                 (35000U)

static const MotionAction g_missionTask1[] = {
    ACTION_FOLLOW_TO_FINISH(TASK1_EXPECTED_DISTANCE_CM,
        TASK1_DECEL_PERCENT, TASK1_HIGH_SPEED_PERCENT,
        TASK1_LOW_SPEED_PERCENT, TASK1_FINISH_BLACK_MIN,
        TASK1_FINISH_CONFIRM_FRAMES, TASK1_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionH3[] = {
    ACTION_H_TASK(H_TASK_ID_H3),
    ACTION_STOP()
};

static const MotionAction g_missionH4[] = {
    ACTION_H_TASK(H_TASK_ID_H4),
    ACTION_STOP()
};

static const MotionAction g_missionH5[] = {
    ACTION_H_TASK(H_TASK_ID_H5),
    ACTION_STOP()
};

static const MotionAction g_missionH6[] = {
    ACTION_H_TASK(H_TASK_ID_H6),
    ACTION_STOP()
};

/*
 * Mission registry.
 *
 * OLED shows exactly the five field tasks.  mission_id values remain stable
 * internal identifiers so development and competition profiles select the
 * same task ordering.
 */
static const MissionDefinition g_missionRegistry[] = {
    {
        .mission_id = MISSION_ID_TASK1_LAP,
        .name = "TASK1-LAP",
        .actions = g_missionTask1,
        .action_count = ARRAY_SIZE(g_missionTask1),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_H3_BALL_50,
        .name = "H3-BALL50",
        .actions = g_missionH3,
        .action_count = ARRAY_SIZE(g_missionH3),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_H4_AB_CENTER,
        .name = "H4-AB-CENTER",
        .actions = g_missionH4,
        .action_count = ARRAY_SIZE(g_missionH4),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_H5_LAP_CENTER,
        .name = "H5-LAP-CENTER",
        .actions = g_missionH5,
        .action_count = ARRAY_SIZE(g_missionH5),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_H6_LAP_LOCK,
        .name = "H6-LAP-LOCK",
        .actions = g_missionH6,
        .action_count = ARRAY_SIZE(g_missionH6),
        .control_profile_id = 0U
    }
};

static bool action_type_is_valid(MotionActionType type)
{
    return (type == MOTION_ACTION_FOLLOW_LINE) ||
        (type == MOTION_ACTION_FOLLOW_LINE_TO_FINISH) ||
        (type == MOTION_ACTION_TURN_TO_YAW) ||
        (type == MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) ||
        (type == MOTION_ACTION_DRIVE_DISTANCE) ||
        (type == MOTION_ACTION_DRIVE_DISTANCE_HEADING) ||
        (type == MOTION_ACTION_WAIT) ||
        (type == MOTION_ACTION_STOP) ||
        (type == MOTION_ACTION_H_TASK)
#if FEATURE_LEGACY_MOTION_CONTROL
        || (type == MOTION_ACTION_SEEK_LINE)
        || (type == MOTION_ACTION_TURN_LEFT_90)
        || (type == MOTION_ACTION_TURN_RIGHT_90)
        || (type == MOTION_ACTION_DRIVE_HEADING_TIME)
#endif
        ;
}

static bool action_requires_timeout(const MotionAction *action)
{
#if APP_PROFILE == APP_PROFILE_COMPETITION
    if (action == (const MotionAction *)0) {
        return false;
    }

    switch (action->type) {
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_SEEK_LINE:
        case MOTION_ACTION_TURN_LEFT_90:
        case MOTION_ACTION_TURN_RIGHT_90:
#endif
        case MOTION_ACTION_TURN_TO_YAW:
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_DRIVE_HEADING_TIME:
#endif
        case MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE:
        case MOTION_ACTION_DRIVE_DISTANCE:
        case MOTION_ACTION_DRIVE_DISTANCE_HEADING:
        case MOTION_ACTION_FOLLOW_LINE_TO_FINISH:
            return true;
        case MOTION_ACTION_FOLLOW_LINE:
            return action->params.follow_line.end_condition !=
                FOLLOW_END_NEVER;
        default:
            return false;
    }
#else
    (void)action;
    return false;
#endif
}

static void set_error(uint16_t *error_code, MissionValidateError error)
{
    if (error_code != (uint16_t *)0) {
        *error_code = (uint16_t)error;
    }
}

uint16_t MissionLibrary_GetCount(void)
{
    return ARRAY_SIZE(g_missionRegistry);
}

const MissionDefinition *MissionLibrary_GetByIndex(uint16_t index)
{
    if (index >= MissionLibrary_GetCount()) {
        return (const MissionDefinition *)0;
    }

    return &g_missionRegistry[index];
}

const MissionDefinition *MissionLibrary_FindById(uint8_t mission_id)
{
    for (uint16_t i = 0; i < MissionLibrary_GetCount(); i++) {
        if (g_missionRegistry[i].mission_id == mission_id) {
            return &g_missionRegistry[i];
        }
    }

    return (const MissionDefinition *)0;
}

bool MissionLibrary_Validate(const MissionDefinition *mission,
    uint16_t *error_code)
{
    if (mission == (const MissionDefinition *)0) {
        set_error(error_code, MISSION_VALIDATE_NULL);
        return false;
    }
    if ((mission->actions == (const MotionAction *)0) ||
        (mission->action_count == 0U)) {
        set_error(error_code, MISSION_VALIDATE_EMPTY);
        return false;
    }
    if (mission->action_count > MISSION_MAX_ACTIONS) {
        set_error(error_code, MISSION_VALIDATE_TOO_LONG);
        return false;
    }

    for (uint16_t i = 0; i < mission->action_count; i++) {
        const MotionAction *action = &mission->actions[i];

        if (!action_type_is_valid(action->type)) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
#if !FEATURE_LEGACY_MOTION_CONTROL
        if ((action->type == MOTION_ACTION_FOLLOW_LINE) &&
            (action->params.follow_line.obstacle_policy !=
                OBSTACLE_POLICY_STOP_ONLY)) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
#endif
        if ((action->type == MOTION_ACTION_TURN_TO_YAW) &&
            ((action->params.turn_to_yaw.angle_deg !=
                action->params.turn_to_yaw.angle_deg) ||
             (action->params.turn_to_yaw.angle_deg < -180.0f) ||
             (action->params.turn_to_yaw.angle_deg > 180.0f))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_DRIVE_DISTANCE) &&
            ((action->params.drive_distance.distance_cm !=
                action->params.drive_distance.distance_cm) ||
             (action->params.drive_distance.distance_cm == 0.0f) ||
             (action->params.drive_distance.normalized_command <= 0) ||
             (action->params.drive_distance.normalized_command >
                MOTION_NORMALIZED_COMMAND_MAX))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_DRIVE_DISTANCE_HEADING) &&
            ((action->params.drive_distance_heading.distance_cm !=
                action->params.drive_distance_heading.distance_cm) ||
             (action->params.drive_distance_heading.distance_cm == 0.0f) ||
             (!action->params.drive_distance_heading.lock_current_yaw_on_start &&
              ((action->params.drive_distance_heading.target_yaw_deg !=
                    action->params.drive_distance_heading.target_yaw_deg) ||
               (action->params.drive_distance_heading.target_yaw_deg <
                    -180.0f) ||
               (action->params.drive_distance_heading.target_yaw_deg >
                    180.0f))) ||
             (action->params.drive_distance_heading.normalized_command <= 0) ||
             (action->params.drive_distance_heading.normalized_command >
                MOTION_NORMALIZED_COMMAND_MAX))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) &&
            ((action->params.drive_heading_until_line.target_yaw_deg !=
                action->params.drive_heading_until_line.target_yaw_deg) ||
             (action->params.drive_heading_until_line.target_yaw_deg <
                -180.0f) ||
             (action->params.drive_heading_until_line.target_yaw_deg >
                180.0f) ||
             (action->params.drive_heading_until_line.normalized_command <=
                0) ||
             (action->params.drive_heading_until_line.normalized_command >
                MOTION_NORMALIZED_COMMAND_MAX))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_FOLLOW_LINE_TO_FINISH) &&
            ((action->params.follow_line_to_finish.expected_distance_cm !=
                action->params.follow_line_to_finish.expected_distance_cm) ||
             (action->params.follow_line_to_finish.expected_distance_cm <=
                0.0f) ||
             (action->params.follow_line_to_finish.decel_percent == 0U) ||
             (action->params.follow_line_to_finish.decel_percent > 100U) ||
             (action->params.follow_line_to_finish.high_speed_percent == 0U) ||
             (action->params.follow_line_to_finish.low_speed_percent == 0U) ||
             (action->params.follow_line_to_finish.low_speed_percent >
                action->params.follow_line_to_finish.high_speed_percent) ||
             (action->params.follow_line_to_finish.finish_black_min == 0U) ||
             (action->params.follow_line_to_finish.finish_black_min > 7U) ||
             (action->params.follow_line_to_finish.finish_confirm_frames ==
                0U))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_H_TASK) &&
            ((action->params.h_task.task_id < H_TASK_ID_H3) ||
             (action->params.h_task.task_id > H_TASK_ID_H6))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if (action_requires_timeout(action) && (action->timeout_ms == 0U)) {
            set_error(error_code, MISSION_VALIDATE_MISSING_TIMEOUT);
            return false;
        }
        if (action->max_retries > 3U) {
            set_error(error_code, MISSION_VALIDATE_TOO_MANY_RETRIES);
            return false;
        }
    }

    if (mission->actions[mission->action_count - 1U].type !=
        MOTION_ACTION_STOP) {
        set_error(error_code, MISSION_VALIDATE_MISSING_STOP);
        return false;
    }

    set_error(error_code, MISSION_VALIDATE_OK);
    return true;
}
