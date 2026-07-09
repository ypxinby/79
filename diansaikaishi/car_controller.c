#include "car_controller.h"

#include "app_config.h"
#include "car_state.h"
#include "motor.h"
#include "track_sensor.h"

#ifndef TRACK_REVERSE_CORRECTION
#define TRACK_REVERSE_CORRECTION    (0)
#endif

#define CAR_CONTROLLER_PERIOD_MS    (20U)
#define LOST_SWEEP_INTERVAL_MS      (160U)

AppRuntime g_appRuntime;

static int16_t clamp_i16(int32_t value, int16_t minValue, int16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return (int16_t)value;
}

static void update_sensor_runtime(void)
{
    g_appRuntime.sensor_raw = TrackSensor_ReadRaw();
    g_appRuntime.black_count =
        TrackSensor_CountBlack(g_appRuntime.sensor_raw);
    g_appRuntime.line_error =
        TrackSensor_GetErrorFromRaw(g_appRuntime.sensor_raw);
}

static void set_output_speed(int16_t leftSpeed, int16_t rightSpeed)
{
    g_appRuntime.left_speed = clamp_i16(leftSpeed, -MOTOR_MAX_DUTY,
        MOTOR_MAX_DUTY);
    g_appRuntime.right_speed = clamp_i16(rightSpeed, -MOTOR_MAX_DUTY,
        MOTOR_MAX_DUTY);
    Motor_SetSpeed(g_appRuntime.left_speed, g_appRuntime.right_speed);
}

static void handle_seek_line(void)
{
    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.correction = 0;
        set_output_speed(g_appConfig.search_speed, g_appConfig.search_speed);
        return;
    }

    g_appRuntime.has_seen_line = 1;
    g_appRuntime.last_error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = g_appRuntime.line_error;
    g_appRuntime.lost_count = 0;
    g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
}

static void handle_follow_line(void)
{
    TrackTurnType turn;
    int16_t error;
    int16_t derivative;
    int32_t correction;

    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.correction = 0;
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.run_mode = TRACK_MODE_LOST_RECOVER;
        return;
    }

    g_appRuntime.lost_count = 0;

    error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = error;

    turn = TrackSensor_DetectTurn(g_appRuntime.sensor_raw, error);
    if (turn == TRACK_TURN_LEFT_90) {
        g_appRuntime.turn_elapsed_ms = 0;
        g_appRuntime.run_mode = TRACK_MODE_TURN_LEFT_90;
        g_appRuntime.correction = 0;
        return;
    }
    if (turn == TRACK_TURN_RIGHT_90) {
        g_appRuntime.turn_elapsed_ms = 0;
        g_appRuntime.run_mode = TRACK_MODE_TURN_RIGHT_90;
        g_appRuntime.correction = 0;
        return;
    }

    derivative = (int16_t)(error - g_appRuntime.last_error);

    correction =
        ((int32_t)g_appConfig.track_kp * error +
            (int32_t)g_appConfig.track_kd * derivative) /
        g_appConfig.track_scale;

    correction = clamp_i16(correction,
        (int16_t)-g_appConfig.max_correction,
        g_appConfig.max_correction);

    if (TRACK_REVERSE_CORRECTION) {
        correction = -correction;
    }

    g_appRuntime.correction = (int16_t)correction;
    set_output_speed(
        (int16_t)((int32_t)g_appConfig.base_speed +
            g_appRuntime.correction),
        (int16_t)((int32_t)g_appConfig.base_speed -
            g_appRuntime.correction));
    g_appRuntime.last_error = error;
}

static void handle_lost_recover(void)
{
    uint16_t sweepPhase;
    uint8_t searchLeft;

    if (!TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.lost_count = 0;
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.last_error = g_appRuntime.line_error;
        g_appRuntime.last_valid_error = g_appRuntime.line_error;
        g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
        return;
    }

    if (g_appRuntime.lost_count < UINT8_MAX) {
        g_appRuntime.lost_count++;
    }

    if (g_appRuntime.lost_elapsed_ms < UINT16_MAX - CAR_CONTROLLER_PERIOD_MS) {
        g_appRuntime.lost_elapsed_ms += CAR_CONTROLLER_PERIOD_MS;
    }

    g_appRuntime.correction = 0;

    if (g_appRuntime.lost_elapsed_ms >= g_appConfig.lost_recover_max_ms) {
        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        CarState_Set(CAR_STATE_ERROR);
        return;
    }

    sweepPhase = (uint16_t)(g_appRuntime.lost_elapsed_ms /
        LOST_SWEEP_INTERVAL_MS);
    searchLeft = (g_appRuntime.last_valid_error < 0) ? 1U : 0U;
    if ((sweepPhase & 1U) != 0U) {
        searchLeft = (uint8_t)!searchLeft;
    }

    if (searchLeft) {
        set_output_speed(0, g_appConfig.recover_speed);
    } else {
        set_output_speed(g_appConfig.recover_speed, 0);
    }
}

static void handle_turn_90(uint8_t turnLeft)
{
    if (g_appRuntime.turn_elapsed_ms < UINT16_MAX - CAR_CONTROLLER_PERIOD_MS) {
        g_appRuntime.turn_elapsed_ms += CAR_CONTROLLER_PERIOD_MS;
    }

    g_appRuntime.correction = 0;

    if (g_appRuntime.turn_elapsed_ms >= g_appConfig.turn_min_ms) {
        if (TrackSensor_IsCenterDetected(g_appRuntime.sensor_raw)) {
            g_appRuntime.last_error = g_appRuntime.line_error;
            g_appRuntime.last_valid_error = g_appRuntime.line_error;
            g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
            g_appRuntime.turn_elapsed_ms = 0;
            return;
        }
    }

    if (g_appRuntime.turn_elapsed_ms >= g_appConfig.turn_max_ms) {
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.run_mode = TRACK_MODE_LOST_RECOVER;
        return;
    }

    if (turnLeft != 0U) {
        set_output_speed(0, g_appConfig.turn_speed);
    } else {
        set_output_speed(g_appConfig.turn_speed, 0);
    }
}

void CarController_Init(void)
{
    CarController_ResetRuntime();
}

void CarController_ResetRuntime(void)
{
    g_appRuntime.current_lap = 0;
    g_appRuntime.sensor_raw = 0;
    g_appRuntime.black_count = 0;
    g_appRuntime.run_mode = TRACK_MODE_SEEK_LINE;
    g_appRuntime.has_seen_line = 0;
    g_appRuntime.line_error = 0;
    g_appRuntime.last_error = 0;
    g_appRuntime.last_valid_error = 0;
    g_appRuntime.correction = 0;
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.lost_count = 0;
    g_appRuntime.lost_elapsed_ms = 0;
    g_appRuntime.turn_elapsed_ms = 0;
    g_appRuntime.lap_cooldown_ms = 0;
    Motor_Stop();
}

void CarController_Update_20ms(void)
{
    update_sensor_runtime();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        return;
    }

    switch (g_appRuntime.run_mode) {
        case TRACK_MODE_SEEK_LINE:
            handle_seek_line();
            break;
        case TRACK_MODE_FOLLOW_LINE:
            handle_follow_line();
            break;
        case TRACK_MODE_LOST_RECOVER:
            handle_lost_recover();
            break;
        case TRACK_MODE_TURN_LEFT_90:
            handle_turn_90(1U);
            break;
        case TRACK_MODE_TURN_RIGHT_90:
            handle_turn_90(0U);
            break;
        default:
            Motor_Stop();
            g_appRuntime.left_speed = 0;
            g_appRuntime.right_speed = 0;
            g_appRuntime.correction = 0;
            CarState_Set(CAR_STATE_ERROR);
            break;
    }
}

TrackRunMode CarController_GetRunMode(void)
{
    return g_appRuntime.run_mode;
}

const char *CarController_RunModeToString(TrackRunMode mode)
{
    switch (mode) {
        case TRACK_MODE_SEEK_LINE:
            return "SEEK";
        case TRACK_MODE_FOLLOW_LINE:
            return "LINE";
        case TRACK_MODE_TURN_LEFT_90:
            return "L90";
        case TRACK_MODE_TURN_RIGHT_90:
            return "R90";
        case TRACK_MODE_LOST_RECOVER:
            return "LOST";
        default:
            return "ERR";
    }
}
