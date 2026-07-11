#include "car_controller.h"

#include "app_config.h"
#include "app_features.h"
#include "car_state.h"
#include "heading_control.h"
#include "imu.h"
#include "motor.h"
#include "track_sensor.h"

#ifndef TRACK_REVERSE_CORRECTION
#define TRACK_REVERSE_CORRECTION    (0)
#endif

#define CAR_CONTROLLER_PERIOD_MS    (20U)
#define SEEK_HEADING_DEADBAND_DEG   (1.0f)
#define RECOVER_DIRECTION_NONE      (0)
#define RECOVER_DIRECTION_LEFT      (-1)
#define RECOVER_DIRECTION_RIGHT     (1)

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

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
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

static void update_recover_direction_from_error(int16_t error)
{
    if (error < 0) {
        g_appRuntime.recover_direction = RECOVER_DIRECTION_LEFT;
    } else if (error > 0) {
        g_appRuntime.recover_direction = RECOVER_DIRECTION_RIGHT;
    }
}

static void reset_heading_control_runtime(void)
{
#if ENABLE_IMU && ENABLE_HEADING_CONTROL
    HeadingControl_Reset();
    g_appRuntime.heading_correction = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
#endif
}

static bool is_heading_control_allowed(int16_t error, int16_t derivative)
{
#if ENABLE_IMU && ENABLE_HEADING_CONTROL
    if (!Imu_IsReady()) {
        return false;
    }
    if (g_appRuntime.black_count > 3U) {
        return false;
    }
    if (abs_i16(error) > g_appConfig.heading_enable_error) {
        return false;
    }
    if (abs_i16(derivative) > g_appConfig.heading_enable_derivative) {
        return false;
    }
    return true;
#else
    (void)error;
    (void)derivative;
    return false;
#endif
}

static int16_t update_heading_control_correction(int16_t error,
    int16_t derivative)
{
#if ENABLE_IMU && ENABLE_HEADING_CONTROL
    const HeadingControlRuntime *heading;

    if (!is_heading_control_allowed(error, derivative)) {
        reset_heading_control_runtime();
        return 0;
    }

    if (g_appRuntime.heading_straight_elapsed_ms <
        g_appConfig.heading_lock_delay_ms) {
        g_appRuntime.heading_straight_elapsed_ms =
            (uint16_t)(g_appRuntime.heading_straight_elapsed_ms +
                CAR_CONTROLLER_PERIOD_MS);
        g_appRuntime.heading_correction = 0;
        return 0;
    }

    heading = HeadingControl_GetRuntime();
    if (!heading->target_locked) {
        HeadingControl_LockCurrentYaw(Imu_GetYaw());
        HeadingControl_Enable(true);
    }

    g_appRuntime.heading_correction =
        HeadingControl_Update(Imu_GetYaw(), Imu_GetCorrectedGyroZDps(),
            0.02f);

    return g_appRuntime.heading_correction;
#else
    (void)error;
    (void)derivative;
    return 0;
#endif
}

static int16_t update_seek_heading_correction(void)
{
#if ENABLE_IMU && ENABLE_HEADING_CONTROL
    const HeadingControlRuntime *heading;

    if (!Imu_IsReady()) {
        reset_heading_control_runtime();
        return 0;
    }

    heading = HeadingControl_GetRuntime();
    if (!heading->target_locked) {
        if (g_appRuntime.seek_heading_mode == SEEK_HEADING_TARGET) {
            HeadingControl_SetTargetYaw(g_appRuntime.seek_target_yaw_deg);
        } else {
            HeadingControl_SetTargetYaw(Imu_GetYaw() +
                (float)g_appConfig.seek_heading_offset_deg);
        }
        HeadingControl_Enable(true);
    }

    g_appRuntime.heading_correction =
        HeadingControl_Update(Imu_GetYaw(), Imu_GetCorrectedGyroZDps(),
            0.02f);

    if (abs_float(heading->heading_error_deg) <= SEEK_HEADING_DEADBAND_DEG) {
        g_appRuntime.heading_correction = 0;
    }

    return g_appRuntime.heading_correction;
#else
    return 0;
#endif
}

static void handle_seek_line(void)
{
    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        int16_t correction = update_seek_heading_correction();

        g_appRuntime.correction = correction;
        set_output_speed(
            (int16_t)((int32_t)g_appConfig.search_speed + correction),
            (int16_t)((int32_t)g_appConfig.search_speed - correction));
        return;
    }

    g_appRuntime.has_seen_line = 1;
    g_appRuntime.last_error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = g_appRuntime.line_error;
    update_recover_direction_from_error(g_appRuntime.line_error);
    g_appRuntime.lost_count = 0;
    g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
    reset_heading_control_runtime();
}

static void handle_follow_line(void)
{
    TrackTurnType turn;
    int16_t error;
    int16_t derivative;
    int32_t correction;

    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.correction = 0;
        reset_heading_control_runtime();
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.run_mode = TRACK_MODE_LOST_RECOVER;
        return;
    }

    g_appRuntime.lost_count = 0;

    error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = error;
    update_recover_direction_from_error(error);

    turn = TrackSensor_DetectTurn(g_appRuntime.sensor_raw, error);
    if (turn == TRACK_TURN_LEFT_90) {
        g_appRuntime.turn_elapsed_ms = 0;
        g_appRuntime.recover_direction = RECOVER_DIRECTION_LEFT;
        g_appRuntime.run_mode = TRACK_MODE_TURN_LEFT_90;
        g_appRuntime.correction = 0;
        reset_heading_control_runtime();
        return;
    }
    if (turn == TRACK_TURN_RIGHT_90) {
        g_appRuntime.turn_elapsed_ms = 0;
        g_appRuntime.recover_direction = RECOVER_DIRECTION_RIGHT;
        g_appRuntime.run_mode = TRACK_MODE_TURN_RIGHT_90;
        g_appRuntime.correction = 0;
        reset_heading_control_runtime();
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

    correction += update_heading_control_correction(error, derivative);
    correction = clamp_i16(correction,
        (int16_t)-g_appConfig.max_correction,
        g_appConfig.max_correction);

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
    if (!TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.lost_count = 0;
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.last_error = g_appRuntime.line_error;
        g_appRuntime.last_valid_error = g_appRuntime.line_error;
        update_recover_direction_from_error(g_appRuntime.line_error);
        g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
        reset_heading_control_runtime();
        return;
    }

    if (g_appRuntime.lost_count < UINT8_MAX) {
        g_appRuntime.lost_count++;
    }

    if (g_appRuntime.lost_elapsed_ms < UINT16_MAX - CAR_CONTROLLER_PERIOD_MS) {
        g_appRuntime.lost_elapsed_ms += CAR_CONTROLLER_PERIOD_MS;
    }

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();

    if (g_appRuntime.lost_elapsed_ms >= g_appConfig.lost_recover_max_ms) {
        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        CarState_Set(CAR_STATE_ERROR);
        return;
    }

    if (g_appRuntime.recover_direction == RECOVER_DIRECTION_LEFT) {
        set_output_speed(0, g_appConfig.recover_speed);
    } else if (g_appRuntime.recover_direction == RECOVER_DIRECTION_RIGHT) {
        set_output_speed(g_appConfig.recover_speed, 0);
    } else {
        set_output_speed(g_appConfig.recover_speed, g_appConfig.recover_speed);
    }
}

static void handle_turn_90(uint8_t turnLeft)
{
    if (g_appRuntime.turn_elapsed_ms < UINT16_MAX - CAR_CONTROLLER_PERIOD_MS) {
        g_appRuntime.turn_elapsed_ms += CAR_CONTROLLER_PERIOD_MS;
    }

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();

    if (g_appRuntime.turn_elapsed_ms >= g_appConfig.turn_min_ms) {
        if (TrackSensor_IsCenterDetected(g_appRuntime.sensor_raw)) {
            g_appRuntime.last_error = g_appRuntime.line_error;
            g_appRuntime.last_valid_error = g_appRuntime.line_error;
            update_recover_direction_from_error(g_appRuntime.line_error);
            g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
            g_appRuntime.turn_elapsed_ms = 0;
            reset_heading_control_runtime();
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
    g_appRuntime.seek_heading_mode = SEEK_HEADING_CURRENT;
    g_appRuntime.seek_target_yaw_deg = 0.0f;
    g_appRuntime.recover_direction = RECOVER_DIRECTION_NONE;
    g_appRuntime.correction = 0;
    g_appRuntime.heading_correction = 0;
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.lost_count = 0;
    g_appRuntime.lost_elapsed_ms = 0;
    g_appRuntime.turn_elapsed_ms = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
    g_appRuntime.lap_cooldown_ms = 0;
    reset_heading_control_runtime();
    Motor_Stop();
}

void CarController_ResetTransientState(void)
{
    g_appRuntime.last_error = 0;
    g_appRuntime.last_valid_error = 0;
    g_appRuntime.correction = 0;
    g_appRuntime.heading_correction = 0;
    g_appRuntime.lost_count = 0;
    g_appRuntime.lost_elapsed_ms = 0;
    g_appRuntime.turn_elapsed_ms = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
    reset_heading_control_runtime();
}

void CarController_Stop(void)
{
    Motor_Stop();
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.correction = 0;
    reset_heading_control_runtime();
}

void CarController_StartSeekLine(float target_yaw_deg)
{
    CarController_ResetRuntime();
    CarController_SetSeekTargetYaw(target_yaw_deg);
    g_appRuntime.run_mode = TRACK_MODE_SEEK_LINE;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartFollowLine(void)
{
    CarController_ResetTransientState();
    g_appRuntime.has_seen_line = 1;
    g_appRuntime.last_error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = g_appRuntime.line_error;
    g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartTurnLeft90(void)
{
    CarController_ResetTransientState();
    g_appRuntime.recover_direction = RECOVER_DIRECTION_LEFT;
    g_appRuntime.run_mode = TRACK_MODE_TURN_LEFT_90;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartTurnRight90(void)
{
    CarController_ResetTransientState();
    g_appRuntime.recover_direction = RECOVER_DIRECTION_RIGHT;
    g_appRuntime.run_mode = TRACK_MODE_TURN_RIGHT_90;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_UseCurrentHeadingForSeek(void)
{
    g_appRuntime.seek_heading_mode = SEEK_HEADING_CURRENT;
    g_appRuntime.seek_target_yaw_deg = 0.0f;
    reset_heading_control_runtime();
}

void CarController_SetSeekTargetYaw(float target_yaw_deg)
{
    g_appRuntime.seek_heading_mode = SEEK_HEADING_TARGET;
    g_appRuntime.seek_target_yaw_deg = target_yaw_deg;
    reset_heading_control_runtime();
}

void CarController_Update_20ms(void)
{
    update_sensor_runtime();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        reset_heading_control_runtime();
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
            reset_heading_control_runtime();
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
