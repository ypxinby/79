#include "motion_action.h"

#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "imu.h"

#define MOTION_ACTION_PERIOD_MS      (20U)

static MotionActionRuntime g_motionActionRuntime;
static float g_motionActionStartYawDeg;

static void motion_action_set_result(MotionActionResult result,
    MotionErrorCode error_code)
{
    g_motionActionRuntime.result = result;
    g_motionActionRuntime.error_code = (uint16_t)error_code;
}

static void motion_action_stop_car(void)
{
    CarController_Stop();
}

static bool motion_action_car_is_error(void)
{
    return CarState_Get() == CAR_STATE_ERROR;
}

static float motion_action_resolve_yaw_target(const MotionYawTarget *target,
    float mission_start_yaw_deg)
{
    float resolved_yaw;

    switch (target->reference) {
        case YAW_REFERENCE_CURRENT:
            resolved_yaw = g_motionActionStartYawDeg + target->angle_deg;
            break;
        case YAW_REFERENCE_MISSION_START:
            resolved_yaw = mission_start_yaw_deg + target->angle_deg;
            break;
        case YAW_REFERENCE_ABSOLUTE:
        default:
            resolved_yaw = target->angle_deg;
            break;
    }

    return resolved_yaw + (float)g_appConfig.seek_heading_offset_deg;
}

static bool motion_action_check_timeout(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) || (action->timeout_ms == 0U)) {
        return false;
    }

    if (g_motionActionRuntime.elapsed_ms < action->timeout_ms) {
        return false;
    }

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_SEEK_TIMEOUT);
            break;
        case MOTION_ACTION_FOLLOW_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_FOLLOW_TIMEOUT);
            break;
        case MOTION_ACTION_TURN_LEFT_90:
        case MOTION_ACTION_TURN_RIGHT_90:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_TURN_TIMEOUT);
            break;
        case MOTION_ACTION_WAIT:
        case MOTION_ACTION_STOP:
            return false;
        default:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_INVALID_ACTION);
            break;
    }

    motion_action_stop_car();
    return true;
}

void MotionAction_Init(void)
{
    g_motionActionRuntime.action = (const MotionAction *)0;
    g_motionActionRuntime.result = MOTION_RESULT_IDLE;
    g_motionActionRuntime.elapsed_ms = 0U;
    g_motionActionRuntime.error_code = (uint16_t)MOTION_ERROR_NONE;
    g_motionActionRuntime.started = false;
    g_motionActionStartYawDeg = 0.0f;
}

bool MotionAction_Start(const MotionAction *action,
    float mission_start_yaw_deg)
{
    MotionAction_Init();

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return false;
    }

    g_motionActionRuntime.action = action;
    g_motionActionRuntime.started = true;
    g_motionActionStartYawDeg = Imu_GetYaw();

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            CarController_StartSeekLine(
                motion_action_resolve_yaw_target(
                    &action->params.seek_line.yaw,
                    mission_start_yaw_deg));
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_FOLLOW_LINE:
            CarController_StartFollowLine();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_TURN_LEFT_90:
            CarController_StartTurnLeft90();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_TURN_RIGHT_90:
            CarController_StartTurnRight90();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_STOP:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_SUCCESS,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_WAIT:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        default:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_FAILED,
                MOTION_ERROR_INVALID_ACTION);
            return false;
    }
}

MotionActionResult MotionAction_Update_20ms(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if (!g_motionActionRuntime.started) {
        return MOTION_RESULT_IDLE;
    }

    if (g_motionActionRuntime.result != MOTION_RESULT_RUNNING) {
        return g_motionActionRuntime.result;
    }

    if (g_motionActionRuntime.elapsed_ms <
        UINT32_MAX - MOTION_ACTION_PERIOD_MS) {
        g_motionActionRuntime.elapsed_ms += MOTION_ACTION_PERIOD_MS;
    }

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return g_motionActionRuntime.result;
    }

    if (motion_action_check_timeout()) {
        return g_motionActionRuntime.result;
    }

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            if (motion_action_car_is_error()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (CarController_GetRunMode() != TRACK_MODE_SEEK_LINE) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        case MOTION_ACTION_FOLLOW_LINE:
            if ((action->params.follow_line.end_condition ==
                    FOLLOW_END_LINE_LOST) &&
                (CarController_GetRunMode() == TRACK_MODE_LOST_RECOVER)) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
                break;
            }
            if (motion_action_car_is_error()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (action->params.follow_line.end_condition ==
                FOLLOW_END_DURATION) {
                if (g_motionActionRuntime.elapsed_ms >=
                    action->params.follow_line.duration_ms) {
                    motion_action_set_result(MOTION_RESULT_SUCCESS,
                        MOTION_ERROR_NONE);
                }
            }
            break;

        case MOTION_ACTION_TURN_LEFT_90:
            if (motion_action_car_is_error() ||
                (CarController_GetRunMode() == TRACK_MODE_LOST_RECOVER)) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (CarController_GetRunMode() == TRACK_MODE_FOLLOW_LINE) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        case MOTION_ACTION_TURN_RIGHT_90:
            if (motion_action_car_is_error() ||
                (CarController_GetRunMode() == TRACK_MODE_LOST_RECOVER)) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (CarController_GetRunMode() == TRACK_MODE_FOLLOW_LINE) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        case MOTION_ACTION_WAIT:
            motion_action_stop_car();
            if (g_motionActionRuntime.elapsed_ms >=
                action->params.wait.wait_ms) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        default:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_FAILED,
                MOTION_ERROR_INVALID_ACTION);
            break;
    }

    return g_motionActionRuntime.result;
}

void MotionAction_Cancel(void)
{
    motion_action_stop_car();
    motion_action_set_result(MOTION_RESULT_CANCELLED, MOTION_ERROR_NONE);
    g_motionActionRuntime.started = false;
}

const MotionActionRuntime *MotionAction_GetRuntime(void)
{
    return &g_motionActionRuntime;
}
