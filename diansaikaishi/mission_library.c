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

/*
 * Test missions.
 *
 * Keep these missions for validating one behavior at a time.
 * Development registry below exposes only the common regression set:
 * TEST-BASIC, TEST-R90, TEST-TURN, TEST-HEAD, TEST-OBS-F, and TEST-STOP.
 *
 * Retained diagnostic arrays:
 * - LEGACY: close to the old run mode.
 * - TEST-SF: seek, follow until line lost, reverse seek, follow until lost.
 * - TEST-R90 / TEST-RSTOP: same right-90 detection, different decisions.
 * - TEST-SK-L / TEST-SK-S: same seek-line event, different decisions.
 * - TEST-TURN: one relative IMU yaw turn from the current pose.
 * - TEST-HEAD: drive along an absolute yaw referenced to boot yaw zero.
 * - TEST-STOP: follow with stop-only obstacle policy.
 */
static const MotionAction g_missionLegacy[] = {
    ACTION_SEEK_LINE(0U),
    /* Follow forever after the line is found. */
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekFollow[] = {
    ACTION_SEEK_LINE(0U),
    /* Leave the first line segment. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_SEEK_LINE(0U),
    /* Leave the second line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestRight90Turn[] = {
    /* Find line, then follow until a right-90 event is reported. */
    ACTION_SEEK_LINE(0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    /* The task layer decides to turn right after the event. */
    ACTION_TURN_RIGHT_90(0U),
    /* Continue line following after the turn. */
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestRight90Stop[] = {
    /* Same right-90 event as TEST-R90, but this task stops instead. */
    ACTION_SEEK_LINE(0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekThenFollow[] = {
    /* Seek until a line is found, then continue following it. */
    ACTION_SEEK_LINE(0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekThenStop[] = {
    /* Seek until a line is found, then stop. */
    ACTION_SEEK_LINE(0U),
    ACTION_STOP()
};

/* Minimal P6 development tests. Change only these values for bench tests. */
#define TEST_TURN_ANGLE_DEG             (90.0f)
#define TEST_TURN_TIMEOUT_MS            (4000U)
#define TEST_HEADING_TARGET_YAW_DEG     (0.0f)
#define TEST_HEADING_DRIVE_MS           (3000U)
#define TEST_HEADING_TIMEOUT_MS         (4000U)

static const MotionAction g_missionTestTurn[] = {
    ACTION_WAIT_MS(500U),
    ACTION_TURN_RELATIVE_YAW(TEST_TURN_ANGLE_DEG, TEST_TURN_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionTestHeadingDrive[] = {
    ACTION_WAIT_MS(500U),
    ACTION_DRIVE_HEADING_YAW(TEST_HEADING_TARGET_YAW_DEG,
        TEST_HEADING_DRIVE_MS, TEST_HEADING_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionTestObstacleFixed[] = {
    /* Place an obstacle while following to validate safety and fixed bypass. */
    ACTION_SEEK_LINE(0U),
    ACTION_FOLLOW_FOREVER_WITH_OBSTACLE(OBSTACLE_POLICY_FIXED_BYPASS,
        BYPASS_DIRECTION_RIGHT, 0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestStopOnly[] = {
    /* Follow normally, but obstacle handling only stops and waits. */
    ACTION_SEEK_LINE(0U),
    ACTION_FOLLOW_FOREVER_WITH_OBSTACLE(OBSTACLE_POLICY_STOP_ONLY,
        BYPASS_DIRECTION_RIGHT, 0U),
    ACTION_STOP()
};

/*
 * Competition missions.
 *
 * Add formal map tasks here. New map work should normally only add:
 * 1. A static MotionAction array in this section.
 * 2. One entry in g_missionRegistry below.
 */
#define MAP_SEEK_TIMEOUT_MS        (4000U)
#define MAP_FOLLOW_TIMEOUT_MS      (8000U)
#define MAP_TURN_TIMEOUT_MS        (2500U)
#define MAP_FINAL_TIMEOUT_MS       (5000U)

static const MotionAction g_missionMapA[] = {
    ACTION_SEEK_LINE(MAP_SEEK_TIMEOUT_MS),
    /* Follow until the right-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_RIGHT_90(MAP_FOLLOW_TIMEOUT_MS),
    /* The task explicitly decides to turn right. */
    ACTION_TURN_RIGHT_90(MAP_TURN_TIMEOUT_MS),
    /* Follow until the left-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_LEFT_90(MAP_FOLLOW_TIMEOUT_MS),
    /* The task explicitly decides to turn left. */
    ACTION_TURN_LEFT_90(MAP_TURN_TIMEOUT_MS),
    /* Follow until leaving the final line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(MAP_FINAL_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionMapB[] = {
    ACTION_SEEK_LINE(MAP_SEEK_TIMEOUT_MS),
    /* Follow until the left-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_LEFT_90(MAP_FOLLOW_TIMEOUT_MS),
    /* The task explicitly decides to turn left. */
    ACTION_TURN_LEFT_90(MAP_TURN_TIMEOUT_MS),
    /* Follow until the right-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_RIGHT_90(MAP_FOLLOW_TIMEOUT_MS),
    /* The task explicitly decides to turn right. */
    ACTION_TURN_RIGHT_90(MAP_TURN_TIMEOUT_MS),
    /* Follow until leaving the final line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(MAP_FINAL_TIMEOUT_MS),
    ACTION_STOP()
};

static const MotionAction g_missionMapC[] = {
    ACTION_SEEK_LINE(MAP_SEEK_TIMEOUT_MS),
    /* Leave the first line segment. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(MAP_FINAL_TIMEOUT_MS),
    ACTION_SEEK_LINE(MAP_SEEK_TIMEOUT_MS),
    /* Leave the second line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(MAP_FINAL_TIMEOUT_MS),
    ACTION_STOP()
};

/*
 * Mission registry.
 *
 * OLED shows visible list indexes. mission_id values remain stable internal
 * identifiers. Development builds expose test missions plus maps. Competition
 * builds expose maps only.
 */
static const MissionDefinition g_missionRegistry[] = {
#if APP_PROFILE == APP_PROFILE_DEVELOPMENT
    {
        .mission_id = MISSION_ID_LEGACY,
        .name = "TEST-BASIC",
        .actions = g_missionTestSeekThenFollow,
        .action_count = ARRAY_SIZE(g_missionTestSeekThenFollow),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_R90,
        .name = "TEST-R90",
        .actions = g_missionTestRight90Turn,
        .action_count = ARRAY_SIZE(g_missionTestRight90Turn),
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
        .mission_id = MISSION_ID_TEST_HEAD,
        .name = "TEST-HEAD",
        .actions = g_missionTestHeadingDrive,
        .action_count = ARRAY_SIZE(g_missionTestHeadingDrive),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_OBSTACLE_FIXED,
        .name = "TEST-OBS-F",
        .actions = g_missionTestObstacleFixed,
        .action_count = ARRAY_SIZE(g_missionTestObstacleFixed),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_STOP_ONLY,
        .name = "TEST-STOP",
        .actions = g_missionTestStopOnly,
        .action_count = ARRAY_SIZE(g_missionTestStopOnly),
        .control_profile_id = 0U
    },
#endif
    {
        .mission_id = MISSION_ID_MAP_A,
        .name = "MAP-A",
        .actions = g_missionMapA,
        .action_count = ARRAY_SIZE(g_missionMapA),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_MAP_B,
        .name = "MAP-B",
        .actions = g_missionMapB,
        .action_count = ARRAY_SIZE(g_missionMapB),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_MAP_C,
        .name = "MAP-C",
        .actions = g_missionMapC,
        .action_count = ARRAY_SIZE(g_missionMapC),
        .control_profile_id = 0U
    }
};

static bool action_type_is_valid(MotionActionType type)
{
    return (type == MOTION_ACTION_SEEK_LINE) ||
        (type == MOTION_ACTION_FOLLOW_LINE) ||
        (type == MOTION_ACTION_TURN_LEFT_90) ||
        (type == MOTION_ACTION_TURN_RIGHT_90) ||
        (type == MOTION_ACTION_TURN_TO_YAW) ||
        (type == MOTION_ACTION_DRIVE_HEADING_TIME) ||
        (type == MOTION_ACTION_WAIT) ||
        (type == MOTION_ACTION_STOP);
}

static bool action_requires_timeout(const MotionAction *action)
{
#if APP_PROFILE == APP_PROFILE_COMPETITION
    if (action == (const MotionAction *)0) {
        return false;
    }

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
        case MOTION_ACTION_TURN_LEFT_90:
        case MOTION_ACTION_TURN_RIGHT_90:
        case MOTION_ACTION_TURN_TO_YAW:
        case MOTION_ACTION_DRIVE_HEADING_TIME:
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
