#include "car_controller.h"

#include "app_config.h"
#include "app_features.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "heading_control.h"
#include "imu.h"
#include "motor.h"
#include "motor_control.h"
#include "scheduler_monitor.h"
#include "track_sensor.h"
#include "watchdog_monitor.h"

#ifndef TRACK_REVERSE_CORRECTION
#define TRACK_REVERSE_CORRECTION    (0)
#endif

#define SEEK_HEADING_DEADBAND_DEG   (1.0f)
#define YAW_TURN_SLOW_THRESHOLD_DEG (12.0f)
#define YAW_TURN_DONE_TOLERANCE_DEG (5.0f)
#define YAW_TURN_STABLE_MS          (100U)
#define YAW_TURN_MIN_SLOW_SPEED     (60)
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

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float normalize_yaw_error(float error_deg)
{
    while (error_deg > 180.0f) {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f) {
        error_deg += 360.0f;
    }
    return error_deg;
}

static void update_sensor_runtime(void)
{
    g_appRuntime.sensor_raw = TrackSensor_ReadRaw();
    g_appRuntime.black_count =
        TrackSensor_CountBlack(g_appRuntime.sensor_raw);
    g_appRuntime.line_error =
        TrackSensor_GetErrorFromRaw(g_appRuntime.sensor_raw);
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
    int16_t derivative, uint32_t elapsed_ms)
{
#if ENABLE_IMU && ENABLE_HEADING_CONTROL
    const HeadingControlRuntime *heading;

    if (!is_heading_control_allowed(error, derivative)) {
        reset_heading_control_runtime();
        return 0;
    }

    if (g_appRuntime.heading_straight_elapsed_ms <
        g_appConfig.heading_lock_delay_ms) {
        g_appRuntime.heading_straight_elapsed_ms = add_elapsed_u16(
            g_appRuntime.heading_straight_elapsed_ms, elapsed_ms);
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
            (float)elapsed_ms / 1000.0f);

    return g_appRuntime.heading_correction;
#else
    (void)error;
    (void)derivative;
    return 0;
#endif
}

static int16_t update_seek_heading_correction(uint32_t elapsed_ms)
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
            (float)elapsed_ms / 1000.0f);

    if (abs_float(heading->heading_error_deg) <= SEEK_HEADING_DEADBAND_DEG) {
        g_appRuntime.heading_correction = 0;
    }

    return g_appRuntime.heading_correction;
#else
    return 0;
#endif
}

static void handle_seek_line(uint32_t elapsed_ms)
{
    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        int16_t correction = update_seek_heading_correction(elapsed_ms);

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

static void handle_follow_line(uint32_t elapsed_ms)
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

    correction += update_heading_control_correction(error, derivative,
        elapsed_ms);
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

static void handle_lost_recover(uint32_t elapsed_ms)
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
    int16_t speed;
    int16_t slowSpeed;
    bool turnLeft;

    g_appRuntime.correction = 0;
    reset_heading_control_runtime();
    g_appRuntime.turn_elapsed_ms = add_elapsed_u32(
        g_appRuntime.turn_elapsed_ms, elapsed_ms);

    if (!Imu_IsReady()) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_carControllerFeedback.operation_failed = true;
        g_carControllerFeedback.error_code =
            CAR_CONTROLLER_ERROR_IMU_NOT_READY;
        Fault_Raise(FAULT_CODE_TURN_YAW_IMU_NOT_READY, 0U, 0U,
            SystemTime_GetMs());
        CarState_Set(CAR_STATE_ERROR);
        return;
    }

    error = normalize_yaw_error(g_appRuntime.yaw_turn_target_deg -
        Imu_GetYaw());
    absError = abs_float(error);
    g_appRuntime.yaw_turn_error_deg = error;

    if (absError <= YAW_TURN_DONE_TOLERANCE_DEG) {
        g_appRuntime.yaw_turn_stable_ms = add_elapsed_u16(
            g_appRuntime.yaw_turn_stable_ms, elapsed_ms);
        if (g_appRuntime.yaw_turn_stable_ms >= YAW_TURN_STABLE_MS) {
            stop_output();
            g_appRuntime.left_speed = 0;
            g_appRuntime.right_speed = 0;
            g_appRuntime.turn_elapsed_ms = 0;
            g_appRuntime.yaw_turn_stable_ms = 0;
            g_carControllerFeedback.turn_completed = true;
            return;
        }
    } else {
        g_appRuntime.yaw_turn_stable_ms = 0;
    }

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
    if (slowSpeed < YAW_TURN_MIN_SLOW_SPEED) {
        slowSpeed = YAW_TURN_MIN_SLOW_SPEED;
    }
    if (slowSpeed > g_appConfig.turn_speed) {
        slowSpeed = g_appConfig.turn_speed;
    }

    speed = (absError > YAW_TURN_SLOW_THRESHOLD_DEG) ?
        g_appConfig.turn_speed : slowSpeed;
    turnLeft = error > 0.0f;
#if YAW_TURN_REVERSE_DIRECTION
    turnLeft = !turnLeft;
#endif

    if (turnLeft) {
        set_output_speed(0, speed);
    } else {
        set_output_speed(speed, 0);
    }
}

static void handle_drive_heading(uint32_t elapsed_ms)
{
    int16_t correction;

    if (!Imu_IsReady()) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        g_carControllerFeedback.operation_failed = true;
        g_carControllerFeedback.error_code =
            CAR_CONTROLLER_ERROR_IMU_NOT_READY;
        CarState_Set(CAR_STATE_ERROR);
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
    g_appRuntime.yaw_turn_stable_ms = 0;
    g_appRuntime.heading_straight_elapsed_ms = 0;
    g_appRuntime.drive_heading_duration_ms = 0;
    g_appRuntime.lap_cooldown_ms = 0;
    g_appRuntime.yaw_turn_target_deg = 0.0f;
    g_appRuntime.yaw_turn_error_deg = 0.0f;
    g_appRuntime.yaw_turn_timeout_ms = 0U;
    g_appRuntime.drive_heading_target_yaw_deg = 0.0f;
    g_followTurnPolicy = CAR_TURN_POLICY_AUTO;
    g_safetyHold = false;
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
    g_appRuntime.yaw_turn_error_deg = 0.0f;
    g_appRuntime.yaw_turn_timeout_ms = 0U;
    g_appRuntime.drive_heading_target_yaw_deg = 0.0f;
    update_feedback_from_runtime();
    reset_heading_control_runtime();
}

void CarController_Stop(void)
{
    stop_output();
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.correction = 0;
    reset_heading_control_runtime();
}

void CarController_StartSeekLine(float target_yaw_deg)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetRuntime();
    CarController_SetSeekTargetYaw(target_yaw_deg);
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
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_IGNORE;
    g_appRuntime.yaw_turn_target_deg = Imu_GetYaw() + angle_deg;
    g_appRuntime.yaw_turn_error_deg = angle_deg;
    g_appRuntime.yaw_turn_timeout_ms =
        (timeout_ms != 0U) ? timeout_ms :
        g_appConfig.yaw_turn_timeout_ms;
    g_appRuntime.yaw_turn_stable_ms = 0U;
    g_appRuntime.run_mode = TRACK_MODE_TURN_TO_YAW;
    CarState_Set(CAR_STATE_RUNNING);
}

void CarController_StartDriveHeading(uint32_t duration_ms)
{
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        stop_output();
        return;
    }
    CarController_ResetTransientState();
    g_followTurnPolicy = CAR_TURN_POLICY_IGNORE;
    g_appRuntime.drive_heading_target_yaw_deg = Imu_GetYaw();
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
        reset_heading_control_runtime();
    }

    g_safetyHold = enable;
    if (g_safetyHold) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        reset_heading_control_runtime();
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
        update_feedback_from_runtime();
        reset_heading_control_runtime();
        return;
    }
    if (g_safetyHold) {
        stop_output();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        update_feedback_from_runtime();
        reset_heading_control_runtime();
        return;
    }

    switch (g_appRuntime.run_mode) {
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
