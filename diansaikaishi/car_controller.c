#include "car_controller.h"

#include "angle_utils.h"
#include "app_config.h"
#include "app_features.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "heading_control.h"
#include "imu.h"
#include "line_controller.h"
#include "motor.h"
#include "motor_control.h"
#include "scheduler_monitor.h"
#include "track_sensor.h"
#include "watchdog_monitor.h"

#ifndef TRACK_REVERSE_CORRECTION
#define TRACK_REVERSE_CORRECTION    (0)
#endif

#define RECOVER_DIRECTION_NONE      (0)
#define RECOVER_DIRECTION_LEFT      (-1)
#define RECOVER_DIRECTION_RIGHT     (1)

#ifndef YAW_TURN_REVERSE_DIRECTION
#define YAW_TURN_REVERSE_DIRECTION  (1)
#endif

AppRuntime g_appRuntime;
static CarControllerFeedback g_carControllerFeedback;
static CarTurnHandlingPolicy g_followTurnPolicy = CAR_TURN_POLICY_AUTO;
static bool g_safetyHold;

static uint32_t add_elapsed_u32(uint32_t value, uint32_t elapsed_ms)
{
    if (value > UINT32_MAX - elapsed_ms) {
        return UINT32_MAX;
    }
    return value + elapsed_ms;
}

static uint16_t add_elapsed_u16(uint16_t value, uint32_t elapsed_ms)
{
    if (elapsed_ms >= UINT16_MAX) {
        return UINT16_MAX;
    }
    if (value > (uint16_t)(UINT16_MAX - (uint16_t)elapsed_ms)) {
        return UINT16_MAX;
    }
    return (uint16_t)(value + (uint16_t)elapsed_ms);
}

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
    LineController_ObserveSensors(g_appRuntime.sensor_raw);
}

static void update_feedback_from_runtime(void)
{
    g_carControllerFeedback.run_mode = g_appRuntime.run_mode;
    g_carControllerFeedback.sensor_raw = g_appRuntime.sensor_raw;
    g_carControllerFeedback.black_count = g_appRuntime.black_count;
    g_carControllerFeedback.line_error = g_appRuntime.line_error;
    g_carControllerFeedback.line_found =
        TrackSensor_IsLineDetected(g_appRuntime.sensor_raw);
    g_carControllerFeedback.line_lost =
        TrackSensor_IsLineLost(g_appRuntime.sensor_raw);
    g_carControllerFeedback.center_detected =
        TrackSensor_IsCenterDetected(g_appRuntime.sensor_raw);
    g_carControllerFeedback.detected_turn = TRACK_TURN_NONE;
    g_carControllerFeedback.turn_completed = false;
    g_carControllerFeedback.operation_failed = false;
    g_carControllerFeedback.error_code = CAR_CONTROLLER_ERROR_NONE;
}

static void stop_output(void)
{
#if FEATURE_WHEEL_SPEED_CONTROL
    MotorControl_Stop();
#else
    Motor_Stop();
#endif
}

static void set_output_speed(int16_t leftSpeed, int16_t rightSpeed)
{
    if (g_safetyHold || EmergencyStop_IsActive() ||
        WatchdogMonitor_HasTripped()) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        return;
    }

    g_appRuntime.left_speed = clamp_i16(leftSpeed, -MOTOR_MAX_DUTY,
        MOTOR_MAX_DUTY);
    g_appRuntime.right_speed = clamp_i16(rightSpeed, -MOTOR_MAX_DUTY,
        MOTOR_MAX_DUTY);
#if FEATURE_WHEEL_SPEED_CONTROL
    MotorControl_SetNormalizedTarget(g_appRuntime.left_speed,
        g_appRuntime.right_speed);
#else
    Motor_SetSpeed(g_appRuntime.left_speed, g_appRuntime.right_speed);
#endif
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

static void handle_seek_line(uint32_t elapsed_ms)
{
    (void)elapsed_ms;

    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.correction = 0;
        g_appRuntime.heading_correction = 0;
        set_output_speed(g_appConfig.search_speed,
            g_appConfig.search_speed);
        return;
    }

    g_appRuntime.has_seen_line = 1;
    g_appRuntime.last_error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = g_appRuntime.line_error;
    update_recover_direction_from_error(g_appRuntime.line_error);
    g_appRuntime.lost_count = 0;
    g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
#if FEATURE_LINE_CONTROL_V2
    LineController_ResetControlState();
#endif
    reset_heading_control_runtime();
}

static void handle_follow_line(uint32_t elapsed_ms)
{
    TrackTurnType turn;
    int16_t error;
#if FEATURE_LINE_CONTROL_V2
    const LineControllerRuntime *line;
    int16_t leftCommand = 0;
    int16_t rightCommand = 0;
#else
    int16_t derivative;
    int32_t correction;
#endif

#if FEATURE_LINE_CONTROL_V2
    LineController_Update(elapsed_ms, g_appRuntime.sensor_raw,
        g_appConfig.line_control_v2_base_command,
        &leftCommand, &rightCommand);
    line = LineController_GetRuntime();
    error = g_appRuntime.line_error;
    g_appRuntime.correction = line->correction;
    g_appRuntime.heading_correction = 0;
    if (line->lost_elapsed_ms > UINT16_MAX) {
        g_appRuntime.lost_elapsed_ms = UINT16_MAX;
    } else {
        g_appRuntime.lost_elapsed_ms =
            (uint16_t)line->lost_elapsed_ms;
    }

    if (line->line_valid) {
        g_appRuntime.lost_count = 0U;
        g_appRuntime.last_error = error;
        g_appRuntime.last_valid_error = error;
        update_recover_direction_from_error(error);
    } else if (g_appRuntime.lost_count < UINT8_MAX) {
        g_appRuntime.lost_count++;
    }

    /* V2 keeps legacy 90-degree detection as report-only task feedback. */
    if ((g_followTurnPolicy == CAR_TURN_POLICY_REPORT_ONLY) &&
        line->line_valid &&
        (line->state == LINE_CONTROL_STATE_FOLLOW)) {
        turn = TrackSensor_DetectTurn(g_appRuntime.sensor_raw, error);
        g_carControllerFeedback.detected_turn = turn;
    } else {
        g_carControllerFeedback.detected_turn = TRACK_TURN_NONE;
    }

    reset_heading_control_runtime();
    set_output_speed(leftCommand, rightCommand);
    return;
#else
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
    g_carControllerFeedback.detected_turn = turn;

    if (g_followTurnPolicy == CAR_TURN_POLICY_REPORT_ONLY) {
        turn = TRACK_TURN_NONE;
    } else if (g_followTurnPolicy == CAR_TURN_POLICY_IGNORE) {
        g_carControllerFeedback.detected_turn = TRACK_TURN_NONE;
        turn = TRACK_TURN_NONE;
    }

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

    g_appRuntime.correction = (int16_t)correction;
    set_output_speed(
        (int16_t)((int32_t)g_appConfig.base_speed +
            g_appRuntime.correction),
        (int16_t)((int32_t)g_appConfig.base_speed -
            g_appRuntime.correction));
    g_appRuntime.last_error = error;
#endif
}

static bool heading_imu_wait_or_fail(uint32_t elapsed_ms)
{
    if (Imu_IsReady()) {
        g_appRuntime.heading_imu_invalid_elapsed_ms = 0U;
        return false;
    }

    stop_output();
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.correction = 0;
    g_appRuntime.heading_correction = 0;
    g_appRuntime.heading_imu_invalid_elapsed_ms = add_elapsed_u16(
        g_appRuntime.heading_imu_invalid_elapsed_ms, elapsed_ms);

    if (g_appRuntime.heading_imu_invalid_elapsed_ms <
        g_appConfig.heading_imu_invalid_grace_ms) {
        return true;
    }

    g_carControllerFeedback.operation_failed = true;
    g_carControllerFeedback.error_code =
        CAR_CONTROLLER_ERROR_IMU_NOT_READY;
    Fault_Raise(FAULT_CODE_TURN_YAW_IMU_NOT_READY,
        g_appRuntime.heading_imu_invalid_elapsed_ms,
        (uint16_t)g_appRuntime.run_mode, SystemTime_GetMs());
    CarState_Set(CAR_STATE_ERROR);
    return true;
}

static void reset_heading_action_runtime(void)
{
    g_appRuntime.turn_elapsed_ms = 0U;
    g_appRuntime.yaw_turn_stable_ms = 0U;
    g_appRuntime.heading_straight_elapsed_ms = 0U;
    g_appRuntime.heading_imu_invalid_elapsed_ms = 0U;
    reset_heading_control_runtime();
}

static void set_yaw_turn_output(float error, int16_t speed)
{
    bool turnLeft = error > 0.0f;

#if YAW_TURN_REVERSE_DIRECTION
    turnLeft = !turnLeft;
#endif

    if (turnLeft) {
        set_output_speed((int16_t)-speed, speed);
    } else {
        set_output_speed(speed, (int16_t)-speed);
    }
}

static void handle_lost_recover(uint32_t elapsed_ms)
{
    if (!TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        g_appRuntime.lost_count = 0;
        g_appRuntime.lost_elapsed_ms = 0;
        g_appRuntime.last_error = g_appRuntime.line_error;
        g_appRuntime.last_valid_error = g_appRuntime.line_error;
        update_recover_direction_from_error(g_appRuntime.line_error);
        g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
#if FEATURE_LINE_CONTROL_V2
        LineController_ResetControlState();
#endif
        reset_heading_control_runtime();
        return;
    }

    if (g_appRuntime.lost_count < UINT8_MAX) {
        g_appRuntime.lost_count++;
    }

    g_appRuntime.lost_elapsed_ms = add_elapsed_u16(
        g_appRuntime.lost_elapsed_ms, elapsed_ms);

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();

    if (g_appRuntime.lost_elapsed_ms >= g_appConfig.lost_recover_max_ms) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_carControllerFeedback.operation_failed = true;
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

static void handle_turn_90(uint8_t turnLeft, uint32_t elapsed_ms)
{
    g_appRuntime.turn_elapsed_ms = add_elapsed_u32(
        g_appRuntime.turn_elapsed_ms, elapsed_ms);

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();

    if (g_appRuntime.turn_elapsed_ms >= g_appConfig.turn_min_ms) {
        if (TrackSensor_IsCenterDetected(g_appRuntime.sensor_raw)) {
            g_appRuntime.last_error = g_appRuntime.line_error;
            g_appRuntime.last_valid_error = g_appRuntime.line_error;
            update_recover_direction_from_error(g_appRuntime.line_error);
            g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
            g_appRuntime.turn_elapsed_ms = 0;
            g_carControllerFeedback.turn_completed = true;
#if FEATURE_LINE_CONTROL_V2
            LineController_ResetControlState();
#endif
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

static void handle_turn_to_yaw(uint32_t elapsed_ms)
{
    float error;
    float absError;
    float absGyro;
    int16_t speed;
    int16_t slowSpeed;

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();

    g_appRuntime.turn_elapsed_ms = add_elapsed_u32(
        g_appRuntime.turn_elapsed_ms, elapsed_ms);

    if (heading_imu_wait_or_fail(elapsed_ms)) {
        return;
    }

    error = Angle_Normalize180(g_appRuntime.yaw_turn_target_deg -
        Imu_GetYaw());
    absError = abs_float(error);
    absGyro = abs_float(Imu_GetCorrectedGyroZDps());
    g_appRuntime.yaw_turn_error_deg = error;

    if ((g_appRuntime.yaw_turn_timeout_ms != 0U) &&
        (g_appRuntime.turn_elapsed_ms >=
            g_appRuntime.yaw_turn_timeout_ms)) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_carControllerFeedback.operation_failed = true;
        g_carControllerFeedback.error_code =
            CAR_CONTROLLER_ERROR_YAW_TURN_TIMEOUT;
        Fault_Raise(FAULT_CODE_TURN_YAW_TIMEOUT,
            (uint16_t)((g_appRuntime.turn_elapsed_ms > UINT16_MAX) ?
                UINT16_MAX : g_appRuntime.turn_elapsed_ms),
            0U, SystemTime_GetMs());
        CarState_Set(CAR_STATE_ERROR);
        return;
    }

    slowSpeed = (int16_t)(g_appConfig.turn_speed / 2);
    if (slowSpeed < g_appConfig.yaw_turn_min_slow_command) {
        slowSpeed = g_appConfig.yaw_turn_min_slow_command;
    }
    if (slowSpeed > g_appConfig.turn_speed) {
        slowSpeed = g_appConfig.turn_speed;
    }

    if (absError <= g_appConfig.yaw_turn_done_tolerance_deg) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;

        if (absGyro <= g_appConfig.yaw_turn_settle_gyro_dps) {
            g_appRuntime.yaw_turn_stable_ms = add_elapsed_u16(
                g_appRuntime.yaw_turn_stable_ms, elapsed_ms);
        } else {
            g_appRuntime.yaw_turn_stable_ms = 0U;
        }

        if (g_appRuntime.yaw_turn_stable_ms >=
            g_appConfig.yaw_turn_settle_ms) {
            g_carControllerFeedback.turn_completed = true;
        }
        return;
    }

    g_appRuntime.yaw_turn_stable_ms = 0U;

    if (absError <= g_appConfig.yaw_turn_slow_threshold_deg) {
        speed = slowSpeed;
    } else {
        speed = g_appConfig.turn_speed;
    }

    set_yaw_turn_output(error, speed);
}

static void handle_drive_heading(uint32_t elapsed_ms)
{
    int16_t correction;

    if (heading_imu_wait_or_fail(elapsed_ms)) {
        return;
    }

    g_appRuntime.heading_straight_elapsed_ms = add_elapsed_u16(
        g_appRuntime.heading_straight_elapsed_ms, elapsed_ms);
    if (g_appRuntime.heading_straight_elapsed_ms >=
        g_appRuntime.drive_heading_duration_ms) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        g_carControllerFeedback.turn_completed = true;
        reset_heading_control_runtime();
        return;
    }

    if (!HeadingControl_GetRuntime()->target_locked) {
        HeadingControl_SetTargetYaw(g_appRuntime.drive_heading_target_yaw_deg);
        HeadingControl_Enable(true);
    }

    correction = HeadingControl_Update(Imu_GetYaw(),
        Imu_GetCorrectedGyroZDps(), (float)elapsed_ms / 1000.0f);
    g_appRuntime.heading_correction = correction;
    g_appRuntime.correction = correction;

    set_output_speed(
        (int16_t)((int32_t)g_appConfig.search_speed + correction),
        (int16_t)((int32_t)g_appConfig.search_speed - correction));
}

void CarController_Init(void)
{
    LineController_Init();
    CarController_ResetRuntime();
}

void CarController_ResetRuntime(void)
{
    g_appRuntime.current_lap = 0;
    g_appRuntime.sensor_raw = 0;
    g_appRuntime.black_count = 0;
    g_appRuntime.run_mode = TRACK_MODE_IDLE;
    g_appRuntime.has_seen_line = 0;
    g_appRuntime.line_error = 0;
    g_appRuntime.last_error = 0;
    g_appRuntime.last_valid_error = 0;
    g_appRuntime.recover_direction = RECOVER_DIRECTION_NONE;
    g_appRuntime.correction = 0;
    g_appRuntime.heading_correction = 0;
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.lost_count = 0;
    g_appRuntime.lost_elapsed_ms = 0;
    g_appRuntime.turn_elapsed_ms = 0;
    g_appRuntime.yaw_turn_stable_ms = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
    g_appRuntime.drive_heading_duration_ms = 0;
    g_appRuntime.heading_imu_invalid_elapsed_ms = 0;
    g_appRuntime.lap_cooldown_ms = 0;
    g_appRuntime.yaw_turn_target_deg = 0.0f;
    g_appRuntime.yaw_turn_error_deg = 0.0f;
    g_appRuntime.yaw_turn_timeout_ms = 0U;
    g_appRuntime.drive_heading_target_yaw_deg = 0.0f;
    g_followTurnPolicy = CAR_TURN_POLICY_AUTO;
    g_safetyHold = false;
    LineController_Reset();
    update_feedback_from_runtime();
    reset_heading_control_runtime();
    stop_output();
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
    g_appRuntime.yaw_turn_stable_ms = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
    g_appRuntime.drive_heading_duration_ms = 0;
    g_appRuntime.heading_imu_invalid_elapsed_ms = 0;
    g_appRuntime.yaw_turn_error_deg = 0.0f;
    g_appRuntime.yaw_turn_timeout_ms = 0U;
    g_appRuntime.drive_heading_target_yaw_deg = 0.0f;
    LineController_Reset();
    update_feedback_from_runtime();
    reset_heading_control_runtime();
}

void CarController_Stop(void)
{
    stop_output();
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.correction = 0;
    LineController_ResetControlState();
    reset_heading_action_runtime();
    g_appRuntime.run_mode = TRACK_MODE_IDLE;
}

void CarController_StartSeekLine(void)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetRuntime();
    g_appRuntime.run_mode = TRACK_MODE_SEEK_LINE;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartFollowLine(CarTurnHandlingPolicy turn_policy)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetTransientState();
    g_followTurnPolicy = turn_policy;
    g_appRuntime.has_seen_line = 1;
    g_appRuntime.last_error = g_appRuntime.line_error;
    g_appRuntime.last_valid_error = g_appRuntime.line_error;
    g_appRuntime.run_mode = TRACK_MODE_FOLLOW_LINE;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartTurnLeft90(void)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_AUTO;
    g_appRuntime.recover_direction = RECOVER_DIRECTION_LEFT;
    g_appRuntime.run_mode = TRACK_MODE_TURN_LEFT_90;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartTurnRight90(void)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_AUTO;
    g_appRuntime.recover_direction = RECOVER_DIRECTION_RIGHT;
    g_appRuntime.run_mode = TRACK_MODE_TURN_RIGHT_90;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartTurnToYawRelative(float angle_deg,
    uint32_t timeout_ms)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    stop_output();
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_IGNORE;
    g_appRuntime.yaw_turn_target_deg = Angle_Normalize180(
        Imu_GetYaw() + angle_deg);
    g_appRuntime.yaw_turn_error_deg = angle_deg;
    g_appRuntime.yaw_turn_timeout_ms =
        (timeout_ms != 0U) ? timeout_ms :
        g_appConfig.yaw_turn_timeout_ms;
    g_appRuntime.yaw_turn_stable_ms = 0U;
    g_appRuntime.run_mode = TRACK_MODE_TURN_TO_YAW;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartDriveHeading(float target_yaw_deg,
    uint32_t duration_ms)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    stop_output();
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_IGNORE;
    g_appRuntime.drive_heading_target_yaw_deg =
        Angle_Normalize180(target_yaw_deg);
    if (duration_ms > UINT16_MAX) {
        g_appRuntime.drive_heading_duration_ms = UINT16_MAX;
    } else {
        g_appRuntime.drive_heading_duration_ms = (uint16_t)duration_ms;
    }
    HeadingControl_SetTargetYaw(g_appRuntime.drive_heading_target_yaw_deg);
    HeadingControl_Enable(true);
    g_appRuntime.run_mode = TRACK_MODE_DRIVE_HEADING;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_SetSafetyHold(bool enable)
{
    if (!enable && (EmergencyStop_IsActive() ||
        WatchdogMonitor_HasTripped())) {
        g_safetyHold = true;
        stop_output();
        return;
    }

    if ((g_safetyHold != enable) && !enable) {
        g_appRuntime.last_error = g_appRuntime.line_error;
        g_appRuntime.last_valid_error = g_appRuntime.line_error;
        g_appRuntime.lost_count = 0U;
        g_appRuntime.lost_elapsed_ms = 0U;
        LineController_ResetControlState();
        reset_heading_control_runtime();
    }

    g_safetyHold = enable;
    if (g_safetyHold) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        LineController_ResetControlState();
        reset_heading_action_runtime();
    }
}

bool CarController_IsSafetyHoldActive(void)
{
    return g_safetyHold;
}

void CarController_Update_20ms(uint32_t elapsed_ms)
{
    update_sensor_runtime();
    update_feedback_from_runtime();

    if ((CarState_Get() != CAR_STATE_RUNNING) ||
        EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        LineController_ResetControlState();
        update_feedback_from_runtime();
        reset_heading_action_runtime();
        return;
    }
    if (g_safetyHold) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        LineController_ResetControlState();
        update_feedback_from_runtime();
        reset_heading_action_runtime();
        return;
    }

    switch (g_appRuntime.run_mode) {
        case TRACK_MODE_IDLE:
            stop_output();
            g_appRuntime.left_speed = 0;
            g_appRuntime.right_speed = 0;
            g_appRuntime.correction = 0;
            break;
        case TRACK_MODE_SEEK_LINE:
            handle_seek_line(elapsed_ms);
            break;
        case TRACK_MODE_FOLLOW_LINE:
            handle_follow_line(elapsed_ms);
            break;
        case TRACK_MODE_LOST_RECOVER:
            handle_lost_recover(elapsed_ms);
            break;
        case TRACK_MODE_TURN_LEFT_90:
            handle_turn_90(1U, elapsed_ms);
            break;
        case TRACK_MODE_TURN_RIGHT_90:
            handle_turn_90(0U, elapsed_ms);
            break;
        case TRACK_MODE_TURN_TO_YAW:
            handle_turn_to_yaw(elapsed_ms);
            break;
        case TRACK_MODE_DRIVE_HEADING:
            handle_drive_heading(elapsed_ms);
            break;
        default:
            stop_output();
            g_appRuntime.left_speed = 0;
            g_appRuntime.right_speed = 0;
            g_appRuntime.correction = 0;
            reset_heading_control_runtime();
            g_carControllerFeedback.operation_failed = true;
            g_carControllerFeedback.error_code =
                CAR_CONTROLLER_ERROR_INVALID_MODE;
            Fault_Raise(FAULT_CODE_CONTROLLER_INVALID_MODE, 0U, 0U,
                SystemTime_GetMs());
            CarState_Set(CAR_STATE_ERROR);
            break;
    }

    g_carControllerFeedback.run_mode = g_appRuntime.run_mode;
}

TrackRunMode CarController_GetRunMode(void)
{
    return g_appRuntime.run_mode;
}

const CarControllerFeedback *CarController_GetFeedback(void)
{
    return &g_carControllerFeedback;
}

const char *CarController_RunModeToString(TrackRunMode mode)
{
    switch (mode) {
        case TRACK_MODE_IDLE:
            return "IDLE";
        case TRACK_MODE_SEEK_LINE:
            return "SEEK";
        case TRACK_MODE_FOLLOW_LINE:
            return "LINE";
        case TRACK_MODE_TURN_LEFT_90:
            return "L90";
        case TRACK_MODE_TURN_RIGHT_90:
            return "R90";
        case TRACK_MODE_TURN_TO_YAW:
            return "YAW";
        case TRACK_MODE_DRIVE_HEADING:
            return "HEAD";
        case TRACK_MODE_LOST_RECOVER:
            return "LOST";
        default:
            return "ERR";
    }
}
