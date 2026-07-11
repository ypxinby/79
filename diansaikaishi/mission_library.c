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

static const MotionAction g_missionLegacy[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};

static const MotionAction g_missionTestSeekFollow[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_SEEK_SECOND_CONFIG(0U),
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};

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
    }
};

static bool action_type_is_valid(MotionActionType type)
{
    return (type == MOTION_ACTION_SEEK_LINE) ||
        (type == MOTION_ACTION_FOLLOW_LINE) ||
        (type == MOTION_ACTION_TURN_LEFT_90) ||
        (type == MOTION_ACTION_TURN_RIGHT_90) ||
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
