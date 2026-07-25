#include "mission_library.h"

#include "app_features.h"

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

/* Three isolated new-base development tests. */
#define TEST_TURN_ANGLE_DEG             (90.0f)
#define TEST_TURN_TIMEOUT_MS            (4000U)
#define TEST_DISTANCE_TARGET_CM         (20.0f)
#define TEST_DISTANCE_TARGET_YAW_DEG    (45.0f)
#define TEST_DISTANCE_COMMAND           (200)
#define TEST_DISTANCE_TIMEOUT_MS        (5000U)

static const MotionAction g_missionTestTurn[] = {
    ACTION_WAIT_MS(500U),
    ACTION_TURN_RELATIVE_YAW(TEST_TURN_ANGLE_DEG, TEST_TURN_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionTestLine[] = {
    /* Start on a visible line and exercise only the P5 -> P4 chain. */
    ACTION_WAIT_MS(500U),
    ACTION_FOLLOW_FOREVER_WITH_OBSTACLE(OBSTACLE_POLICY_STOP_ONLY,
        BYPASS_DIRECTION_RIGHT, 0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestDistance20[] = {
    ACTION_WAIT_MS(500U),
    ACTION_DRIVE_DISTANCE_AT_YAW(TEST_DISTANCE_TARGET_CM,
        TEST_DISTANCE_TARGET_YAW_DEG, TEST_DISTANCE_COMMAND,
        TEST_DISTANCE_TIMEOUT_MS),
    ACTION_STOP()
};

/*
 * The first formal competition task uses only the new motion primitives:
 * P6 heading hold drives straight from boot yaw zero until any black line is
 * detected, then P5 takes over and follows continuously through P4.
 */
#define COMPETITION_ENTRY_YAW_DEG       (0.0f)
#define COMPETITION_ENTRY_COMMAND       (200)
#define COMPETITION_FIND_LINE_TIMEOUT_MS (5000U)

static const MotionAction g_missionCompetitionMain[] = {
    ACTION_DRIVE_HEADING_UNTIL_LINE(COMPETITION_ENTRY_YAW_DEG,
        COMPETITION_ENTRY_COMMAND, COMPETITION_FIND_LINE_TIMEOUT_MS),
    ACTION_FOLLOW_FOREVER_WITH_OBSTACLE(OBSTACLE_POLICY_STOP_ONLY,
        BYPASS_DIRECTION_RIGHT, 0U),
    ACTION_STOP()
};

/*
 * Mission registry.
 *
 * OLED shows visible list indexes. mission_id values remain stable internal
 * identifiers. Development builds expose the three isolated new-base tests
 * plus RACE-1 for integration testing. Competition builds expose only RACE-1.
 */
static const MissionDefinition g_missionRegistry[] = {
#if APP_PROFILE == APP_PROFILE_DEVELOPMENT
    {
        .mission_id = MISSION_ID_LEGACY,
        .name = "TEST-LINE",
        .actions = g_missionTestLine,
        .action_count = ARRAY_SIZE(g_missionTestLine),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_YAW,
        .name = "TEST-TURN",
        .actions = g_missionTestTurn,
        .action_count = ARRAY_SIZE(g_missionTestTurn),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_DISTANCE_20,
        .name = "TEST-DIST",
        .actions = g_missionTestDistance20,
        .action_count = ARRAY_SIZE(g_missionTestDistance20),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_COMPETITION_MAIN,
        .name = "RACE-1",
        .actions = g_missionCompetitionMain,
        .action_count = ARRAY_SIZE(g_missionCompetitionMain),
        .control_profile_id = 0U
    }
#else
    {
        .mission_id = MISSION_ID_COMPETITION_MAIN,
        .name = "RACE-1",
        .actions = g_missionCompetitionMain,
        .action_count = ARRAY_SIZE(g_missionCompetitionMain),
        .control_profile_id = 0U
    }
#endif
};

static bool action_type_is_valid(MotionActionType type)
{
    return (type == MOTION_ACTION_FOLLOW_LINE) ||
        (type == MOTION_ACTION_TURN_TO_YAW) ||
        (type == MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) ||
        (type == MOTION_ACTION_DRIVE_DISTANCE) ||
        (type == MOTION_ACTION_DRIVE_DISTANCE_HEADING) ||
        (type == MOTION_ACTION_WAIT) ||
        (type == MOTION_ACTION_STOP)
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
        if ((action->type == MOTION_ACTION_DRIVE_DISTANCE) &&
            (!(action->params.drive_distance.distance_cm > 0.0f) ||
             (action->params.drive_distance.normalized_command <= 0) ||
             (action->params.drive_distance.normalized_command >
                MOTION_NORMALIZED_COMMAND_MAX))) {
            set_error(error_code, MISSION_VALIDATE_INVALID_ACTION);
            return false;
        }
        if ((action->type == MOTION_ACTION_DRIVE_DISTANCE_HEADING) &&
            (!(action->params.drive_distance_heading.distance_cm > 0.0f) ||
             (action->params.drive_distance_heading.target_yaw_deg !=
                action->params.drive_distance_heading.target_yaw_deg) ||
             (action->params.drive_distance_heading.target_yaw_deg <
                -180.0f) ||
             (action->params.drive_distance_heading.target_yaw_deg >
                180.0f) ||
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
