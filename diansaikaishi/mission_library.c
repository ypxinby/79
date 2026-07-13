#include "mission_library.h"

#define ARRAY_SIZE(array) \
    ((uint16_t)(sizeof(array) / sizeof((array)[0])))

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
 * Keep these missions for validating one behavior at a time:
 * - LEGACY: close to the old run mode.
 * - TEST-SF: seek, follow until line lost, reverse seek, follow until lost.
 * - TEST-R90 / TEST-RSTOP: same right-90 detection, different decisions.
 * - TEST-SK-L / TEST-SK-S: same seek-line event, different decisions.
 * - TEST-YAW: relative IMU yaw turn validation.
 * - TEST-HEAD: yaw heading drive validation.
 * - TEST-REQ: line reacquire validation.
 */
static const MotionAction g_missionLegacy[] = {
    /* Find the first line using the mission start yaw. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    /* Follow forever after the line is found. */
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekFollow[] = {
    /* First straight seek. Target = mission_start_yaw + 0 + YAW. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    /* Leave the first line segment. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    /* Second straight seek. Target = mission_start_yaw + REV + YAW. */
    ACTION_SEEK_SECOND_CONFIG(0U),
    /* Leave the second line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestRight90Turn[] = {
    /* Find line, then follow until a right-90 event is reported. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    /* The task layer decides to turn right after the event. */
    ACTION_TURN_RIGHT_90(0U),
    /* Continue line following after the turn. */
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestRight90Stop[] = {
    /* Same right-90 event as TEST-R90, but this task stops instead. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekThenFollow[] = {
    /* Seek until a line is found, then continue following it. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekThenStop[] = {
    /* Seek until a line is found, then stop. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestYaw[] = {
    /* Turn relative to the current yaw, wait, then turn back. */
    ACTION_TURN_RELATIVE_YAW(40.0f, 2000U),
    ACTION_WAIT_MS(300U),
    ACTION_TURN_RELATIVE_YAW(-40.0f, 2000U),
    ACTION_STOP()
};

static const MotionAction g_missionTestHeadingDrive[] = {
    /* Turn away from the current heading, then drive along that yaw. */
    ACTION_TURN_RELATIVE_YAW(40.0f, 2000U),
    ACTION_DRIVE_HEADING_TIME(700U, 1200U),
    ACTION_STOP()
};

static const MotionAction g_missionTestReacquire[] = {
    /* Leave the line, turn back toward it, then require stable reacquire. */
    ACTION_TURN_RELATIVE_YAW(35.0f, 2500U),
    ACTION_DRIVE_HEADING_TIME(1400U, 2000U),
    ACTION_TURN_RELATIVE_YAW(-55.0f, 2500U),
    ACTION_REACQUIRE_LINE(3000U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

/*
 * Competition missions.
 *
 * Add formal map tasks here. New map work should normally only add:
 * 1. A static MotionAction array in this section.
 * 2. One entry in g_missionRegistry below.
 */
static const MotionAction g_missionMapA[] = {
    /* Find the first line using mission_start_yaw + 0 + YAW. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    /* Follow until the right-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    /* The task explicitly decides to turn right. */
    ACTION_TURN_RIGHT_90(0U),
    /* Follow until the left-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_LEFT_90(0U),
    /* The task explicitly decides to turn left. */
    ACTION_TURN_LEFT_90(0U),
    /* Follow until leaving the final line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

static const MotionAction g_missionMapB[] = {
    /* Find the first line using mission_start_yaw + 0 + YAW. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    /* Follow until the left-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_LEFT_90(0U),
    /* The task explicitly decides to turn left. */
    ACTION_TURN_LEFT_90(0U),
    /* Follow until the right-90 event is reported. This does not auto-turn. */
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    /* The task explicitly decides to turn right. */
    ACTION_TURN_RIGHT_90(0U),
    /* Follow until leaving the final line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

static const MotionAction g_missionMapC[] = {
    /* First straight seek. Target = mission_start_yaw + 0 + YAW. */
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    /* Leave the first line segment. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    /* Second straight seek. Target = mission_start_yaw + REV + YAW. */
    ACTION_SEEK_SECOND_CONFIG(0U),
    /* Leave the second line segment, then stop. */
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

/*
 * Mission registry.
 *
 * TASK values shown on OLED are mission_id values. Keep test missions in
 * 0~9 and competition missions in 10~99.
 */
static const MissionDefinition g_missionRegistry[] = {
    {
        .mission_id = MISSION_ID_LEGACY,
        .name = "LEGACY",
        .actions = g_missionLegacy,
        .action_count = ARRAY_SIZE(g_missionLegacy),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_SF,
        .name = "TEST-SF",
        .actions = g_missionTestSeekFollow,
        .action_count = ARRAY_SIZE(g_missionTestSeekFollow),
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
        .mission_id = MISSION_ID_TEST_RSTOP,
        .name = "TEST-RSTOP",
        .actions = g_missionTestRight90Stop,
        .action_count = ARRAY_SIZE(g_missionTestRight90Stop),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_SEEK_FOLLOW,
        .name = "TEST-SK-L",
        .actions = g_missionTestSeekThenFollow,
        .action_count = ARRAY_SIZE(g_missionTestSeekThenFollow),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_SEEK_STOP,
        .name = "TEST-SK-S",
        .actions = g_missionTestSeekThenStop,
        .action_count = ARRAY_SIZE(g_missionTestSeekThenStop),
        .control_profile_id = 0U
    },
    {
        .mission_id = MISSION_ID_TEST_YAW,
        .name = "TEST-YAW",
        .actions = g_missionTestYaw,
        .action_count = ARRAY_SIZE(g_missionTestYaw),
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
        .mission_id = MISSION_ID_TEST_REACQUIRE,
        .name = "TEST-REQ",
        .actions = g_missionTestReacquire,
        .action_count = ARRAY_SIZE(g_missionTestReacquire),
        .control_profile_id = 0U
    },
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
        (type == MOTION_ACTION_REACQUIRE_LINE) ||
        (type == MOTION_ACTION_WAIT) ||
        (type == MOTION_ACTION_STOP);
}

static bool action_requires_timeout(const MotionAction *action)
{
    (void)action;
    return false;
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
