#include "mission_manager.h"

#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "motion_action.h"
#include "watchdog_monitor.h"

static MissionRuntime g_missionRuntime;

static void mission_stop_outputs(void)
{
    CarController_Stop();
}

static void mission_reset_runtime_counters(void)
{
    g_missionRuntime.current_action_index = 0U;
    g_missionRuntime.current_retry_count = 0U;
    g_missionRuntime.mission_elapsed_ms = 0U;
    g_missionRuntime.action_elapsed_ms = 0U;
    g_missionRuntime.last_action_result = MOTION_RESULT_IDLE;
    g_missionRuntime.external_hold = false;
}

static uint16_t mission_get_current_registry_index(void)
{
    uint16_t count = MissionLibrary_GetCount();

    for (uint16_t i = 0; i < count; i++) {
        if (MissionLibrary_GetByIndex(i) == g_missionRuntime.definition) {
            return i;
        }
    }

    return 0U;
}

static void mission_finish_success(void)
{
    mission_stop_outputs();
    g_missionRuntime.status = MISSION_STATUS_DONE;
    g_missionRuntime.last_action_result = MOTION_RESULT_SUCCESS;
    CarState_Set(CAR_STATE_FINISHED);
}

static void mission_finish_error(uint16_t error_code)
{
    mission_stop_outputs();
    g_missionRuntime.status = MISSION_STATUS_ERROR;
    g_missionRuntime.last_action_result = MOTION_RESULT_FAILED;
    g_missionRuntime.last_error_code = error_code;
    CarState_Set(CAR_STATE_ERROR);
}

void MissionManager_Init(void)
{
    MotionAction_Init();

    g_missionRuntime.definition = MissionLibrary_GetByIndex(0U);
    g_missionRuntime.status = MISSION_STATUS_IDLE;
    g_missionRuntime.last_error_code = 0U;
    mission_reset_runtime_counters();

    if (MissionLibrary_Validate(g_missionRuntime.definition,
            &g_missionRuntime.last_error_code)) {
        g_missionRuntime.status = MISSION_STATUS_READY;
    } else {
        g_missionRuntime.status = MISSION_STATUS_ERROR;
    }
}

bool MissionManager_Select(uint8_t mission_id)
{
    const MissionDefinition *mission;
    uint16_t error_code = 0U;

    if ((g_missionRuntime.status == MISSION_STATUS_RUNNING) ||
        (g_missionRuntime.status == MISSION_STATUS_PAUSED)) {
        return false;
    }

    mission = MissionLibrary_FindById(mission_id);
    if (!MissionLibrary_Validate(mission, &error_code)) {
        g_missionRuntime.last_error_code = error_code;
        g_missionRuntime.status = MISSION_STATUS_ERROR;
        return false;
    }

    g_missionRuntime.definition = mission;
    g_missionRuntime.last_error_code = 0U;
    mission_reset_runtime_counters();
    g_missionRuntime.status = MISSION_STATUS_READY;
    return true;
}

bool MissionManager_SelectNext(void)
{
    uint16_t count = MissionLibrary_GetCount();
    uint16_t index;
    const MissionDefinition *mission;

    if (count == 0U) {
        return false;
    }
    if ((g_missionRuntime.status == MISSION_STATUS_RUNNING) ||
        (g_missionRuntime.status == MISSION_STATUS_PAUSED)) {
        return false;
    }

    index = (uint16_t)(mission_get_current_registry_index() + 1U);
    if (index >= count) {
        index = 0U;
    }

    mission = MissionLibrary_GetByIndex(index);
    if (mission == (const MissionDefinition *)0) {
        return false;
    }

    return MissionManager_Select(mission->mission_id);
}

bool MissionManager_SelectPrevious(void)
{
    uint16_t count = MissionLibrary_GetCount();
    uint16_t index;
    const MissionDefinition *mission;

    if (count == 0U) {
        return false;
    }
    if ((g_missionRuntime.status == MISSION_STATUS_RUNNING) ||
        (g_missionRuntime.status == MISSION_STATUS_PAUSED)) {
        return false;
    }

    index = mission_get_current_registry_index();
    if (index == 0U) {
        index = (uint16_t)(count - 1U);
    } else {
        index--;
    }

    mission = MissionLibrary_GetByIndex(index);
    if (mission == (const MissionDefinition *)0) {
        return false;
    }

    return MissionManager_Select(mission->mission_id);
}

bool MissionManager_Start(void)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        return false;
    }
    if ((g_missionRuntime.definition == (const MissionDefinition *)0) ||
        (g_missionRuntime.status == MISSION_STATUS_ERROR)) {
        return false;
    }

    mission_reset_runtime_counters();
    g_missionRuntime.status = MISSION_STATUS_RUNNING;
    g_missionRuntime.last_action_result = MOTION_RESULT_RUNNING;
    CarState_Set(CAR_STATE_RUNNING);
    return true;
}

void MissionManager_Pause(void)
{
    if (g_missionRuntime.status != MISSION_STATUS_RUNNING) {
        return;
    }

    mission_stop_outputs();
    MotionAction_Init();
    g_missionRuntime.status = MISSION_STATUS_PAUSED;
    CarState_Set(CAR_STATE_PAUSED);
}

void MissionManager_Resume(void)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        return;
    }
    if (g_missionRuntime.status != MISSION_STATUS_PAUSED) {
        return;
    }

    MotionAction_Init();
    g_missionRuntime.status = MISSION_STATUS_RUNNING;
    g_missionRuntime.last_action_result = MOTION_RESULT_RUNNING;
    CarState_Set(CAR_STATE_RUNNING);
}

void MissionManager_Cancel(void)
{
    if (g_missionRuntime.status == MISSION_STATUS_ERROR) {
        return;
    }

    MotionAction_Cancel();
    g_missionRuntime.status = MISSION_STATUS_READY;
    mission_reset_runtime_counters();
    g_missionRuntime.last_action_result = MOTION_RESULT_CANCELLED;
    CarState_Set(CAR_STATE_READY);
}

void MissionManager_Reset(void)
{
    mission_stop_outputs();
    MotionAction_Init();
    g_missionRuntime.last_error_code = 0U;
    mission_reset_runtime_counters();
    g_missionRuntime.status = MISSION_STATUS_READY;
    CarState_Set(CAR_STATE_READY);
}

void MissionManager_Update_20ms(uint32_t elapsed_ms)
{
    const MotionAction *action;
    const MotionActionRuntime *action_runtime;
    MotionActionResult result;

    if ((g_missionRuntime.status != MISSION_STATUS_RUNNING) ||
        EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        return;
    }
    if (g_missionRuntime.external_hold) {
        return;
    }

    if ((g_missionRuntime.definition == (const MissionDefinition *)0) ||
        (g_missionRuntime.current_action_index >=
            g_missionRuntime.definition->action_count)) {
        mission_finish_error((uint16_t)MOTION_ERROR_INVALID_MISSION);
        return;
    }

    action =
        &g_missionRuntime.definition->actions[g_missionRuntime.current_action_index];

    if (g_missionRuntime.mission_elapsed_ms > UINT32_MAX - elapsed_ms) {
        g_missionRuntime.mission_elapsed_ms = UINT32_MAX;
    } else {
        g_missionRuntime.mission_elapsed_ms += elapsed_ms;
    }

    action_runtime = MotionAction_GetRuntime();
    if ((action_runtime->action != action) || !action_runtime->started) {
        if (!MotionAction_Start(action)) {
            action_runtime = MotionAction_GetRuntime();
            mission_finish_error(action_runtime->error_code);
            return;
        }
    }

    result = MotionAction_Update_20ms(elapsed_ms);
    action_runtime = MotionAction_GetRuntime();
    g_missionRuntime.action_elapsed_ms = action_runtime->elapsed_ms;
    g_missionRuntime.last_action_result = result;

    if (result == MOTION_RESULT_RUNNING) {
        return;
    }

    if (result == MOTION_RESULT_SUCCESS) {
        if ((action->type == MOTION_ACTION_STOP) ||
            ((g_missionRuntime.current_action_index + 1U) >=
                g_missionRuntime.definition->action_count)) {
            mission_finish_success();
            return;
        }

        g_missionRuntime.current_action_index++;
        g_missionRuntime.current_retry_count = 0U;
        g_missionRuntime.action_elapsed_ms = 0U;
        MotionAction_Init();
        return;
    }

    if ((result == MOTION_RESULT_FAILED) ||
        (result == MOTION_RESULT_TIMEOUT)) {
        if ((CarState_Get() != CAR_STATE_ERROR) &&
            (g_missionRuntime.current_retry_count < action->max_retries)) {
            g_missionRuntime.current_retry_count++;
            g_missionRuntime.action_elapsed_ms = 0U;
            (void)MotionAction_Start(action);
            return;
        }

        mission_finish_error(action_runtime->error_code);
        return;
    }

    if (result == MOTION_RESULT_CANCELLED) {
        g_missionRuntime.status = MISSION_STATUS_READY;
        CarState_Set(CAR_STATE_READY);
    }
}

const MissionRuntime *MissionManager_GetRuntime(void)
{
    return &g_missionRuntime;
}

uint8_t MissionManager_GetSelectedMissionId(void)
{
    if (g_missionRuntime.definition == (const MissionDefinition *)0) {
        return MISSION_ID_LEGACY;
    }

    return g_missionRuntime.definition->mission_id;
}

uint16_t MissionManager_GetSelectedMissionIndex(void)
{
    return mission_get_current_registry_index();
}

uint16_t MissionManager_GetMissionCount(void)
{
    return MissionLibrary_GetCount();
}

void MissionManager_SetExternalHold(bool enable)
{
    g_missionRuntime.external_hold = enable;
}

bool MissionManager_IsExternallyHeld(void)
{
    return g_missionRuntime.external_hold;
}

void MissionManager_ReportExternalFailure(uint16_t error_code)
{
    g_missionRuntime.external_hold = true;
    mission_finish_error(error_code);
}
