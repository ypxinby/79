#include "motor_control.h"

#include "app_config.h"
#include "app_features.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "mission_manager.h"
#include "motor.h"
#include "scheduler_monitor.h"
#include "watchdog_monitor.h"
#include "wheel_speed_estimator.h"

#define MOTOR_CONTROL_DT_MAX_MS              (100U)
#define MOTOR_CONTROL_CONFIG_SPEED_MAX_CMPS  (500.0f)
#define MOTOR_CONTROL_CONFIG_GAIN_MAX        (100.0f)
#define MOTOR_CONTROL_CONFIG_FF_MAX           (2.0f)
#define MOTOR_CONTROL_CONFIG_INTEGRAL_MAX    (10000.0f)
#define MOTOR_CONTROL_CONFIG_RAMP_MAX_CMPS2  (2000.0f)
#define MOTOR_CONTROL_TARGET_TIMEOUT_MAX_MS  (2000U)
#define MOTOR_CONTROL_FEEDBACK_MAX_CMPS      (1000.0f)

static MotorControlRuntime g_runtime;
static bool g_targetRefreshed;

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t sign_float(float value)
{
    if (value > 0.0f) {
        return 1;
    }
    if (value < 0.0f) {
        return -1;
    }
    return 0;
}

static int16_t clamp_normalized_command(int16_t command)
{
    if (command > MOTOR_MAX_DUTY) {
        return MOTOR_MAX_DUTY;
    }
    if (command < -MOTOR_MAX_DUTY) {
        return -MOTOR_MAX_DUTY;
    }
    return command;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint32_t add_elapsed_u32(uint32_t value, uint32_t elapsed_ms)
{
    if (value > UINT32_MAX - elapsed_ms) {
        return UINT32_MAX;
    }
    return value + elapsed_ms;
}

static bool config_value_in_range(float value, float minimum,
    float maximum)
{
    return (value >= minimum) && (value <= maximum);
}

static bool config_is_valid(void)
{
    return config_value_in_range(g_appConfig.wheel_control_max_speed_cmps,
            0.1f, MOTOR_CONTROL_CONFIG_SPEED_MAX_CMPS) &&
        config_value_in_range(g_appConfig.wheel_control_left_kp, 0.0f,
            MOTOR_CONTROL_CONFIG_GAIN_MAX) &&
        config_value_in_range(g_appConfig.wheel_control_left_ki, 0.0f,
            MOTOR_CONTROL_CONFIG_GAIN_MAX) &&
        config_value_in_range(g_appConfig.wheel_control_right_kp, 0.0f,
            MOTOR_CONTROL_CONFIG_GAIN_MAX) &&
        config_value_in_range(g_appConfig.wheel_control_right_ki, 0.0f,
            MOTOR_CONTROL_CONFIG_GAIN_MAX) &&
        config_value_in_range(
            g_appConfig.wheel_control_left_feedforward_gain, 0.0f,
            MOTOR_CONTROL_CONFIG_FF_MAX) &&
        config_value_in_range(
            g_appConfig.wheel_control_right_feedforward_gain, 0.0f,
            MOTOR_CONTROL_CONFIG_FF_MAX) &&
        config_value_in_range(g_appConfig.wheel_control_integral_limit,
            0.1f, MOTOR_CONTROL_CONFIG_INTEGRAL_MAX) &&
        config_value_in_range(g_appConfig.wheel_control_max_accel_cmps2,
            0.1f, MOTOR_CONTROL_CONFIG_RAMP_MAX_CMPS2) &&
        config_value_in_range(g_appConfig.wheel_control_max_decel_cmps2,
            0.1f, MOTOR_CONTROL_CONFIG_RAMP_MAX_CMPS2) &&
        (g_appConfig.wheel_control_target_timeout_ms >= 20U) &&
        (g_appConfig.wheel_control_target_timeout_ms <=
            MOTOR_CONTROL_TARGET_TIMEOUT_MAX_MS);
}

static bool feedback_is_valid(float value)
{
    return (value >= -MOTOR_CONTROL_FEEDBACK_MAX_CMPS) &&
        (value <= MOTOR_CONTROL_FEEDBACK_MAX_CMPS);
}

static void clear_wheel_controller(MotorControlWheelRuntime *wheel,
    bool clear_target)
{
    if (clear_target) {
        wheel->normalized_target = 0;
        wheel->raw_target_speed_cmps = 0.0f;
    }
    wheel->ramped_target_speed_cmps = 0.0f;
    wheel->error_cmps = 0.0f;
    wheel->integral = 0.0f;
    wheel->proportional_term = 0.0f;
    wheel->integral_term = 0.0f;
    wheel->feedforward_term = 0.0f;
    wheel->output_command = 0;
    wheel->saturated = false;
    wheel->direction_change_pending = false;
}

static void stop_output(bool clear_target)
{
    clear_wheel_controller(&g_runtime.left, clear_target);
    clear_wheel_controller(&g_runtime.right, clear_target);
    Motor_Stop();
}

static void latch_control_error(uint32_t error_flags,
    uint32_t estimator_error_flags, bool report_fault)
{
    g_runtime.error_flags |= error_flags;
    g_runtime.estimator_error_flags |= estimator_error_flags;
    g_runtime.error_latched = true;
    g_runtime.valid = false;
    if ((error_flags & MOTOR_CONTROL_ERROR_TARGET_TIMEOUT) != 0U) {
        g_runtime.target_refresh_timeout = true;
    }
    stop_output(false);

    if (report_fault) {
        Fault_Raise(FAULT_CODE_MOTOR_CONTROL,
            (uint16_t)(g_runtime.error_flags & 0xFFFFU),
            (uint16_t)(g_runtime.estimator_error_flags & 0xFFFFU),
            SystemTime_GetMs());
        MissionManager_ReportExternalFailure(
            (uint16_t)FAULT_CODE_MOTOR_CONTROL);
    }
}

static float move_toward(float current, float target, float maximum_step)
{
    if (current < target) {
        current += maximum_step;
        return (current > target) ? target : current;
    }
    if (current > target) {
        current -= maximum_step;
        return (current < target) ? target : current;
    }
    return target;
}

static bool update_ramped_target(MotorControlWheelRuntime *wheel,
    float elapsed_seconds)
{
    float requested = wheel->raw_target_speed_cmps;
    float maximum_step;

    if (requested == 0.0f) {
        clear_wheel_controller(wheel, false);
        wheel->error_cmps = -wheel->measured_speed_cmps;
        return false;
    }

    if ((wheel->ramped_target_speed_cmps != 0.0f) &&
        (sign_float(requested) !=
            sign_float(wheel->ramped_target_speed_cmps))) {
        wheel->direction_change_pending = true;
    }

    if (wheel->direction_change_pending) {
        if ((wheel->ramped_target_speed_cmps != 0.0f) &&
            (sign_float(requested) !=
                sign_float(wheel->ramped_target_speed_cmps))) {
            maximum_step = g_appConfig.wheel_control_max_decel_cmps2 *
                elapsed_seconds;
            wheel->ramped_target_speed_cmps = move_toward(
                wheel->ramped_target_speed_cmps, 0.0f, maximum_step);
            wheel->integral = 0.0f;
            wheel->proportional_term = 0.0f;
            wheel->integral_term = 0.0f;
            wheel->feedforward_term = 0.0f;
            wheel->output_command = 0;
            wheel->saturated = false;
            wheel->error_cmps = wheel->ramped_target_speed_cmps -
                wheel->measured_speed_cmps;
            if (wheel->ramped_target_speed_cmps == 0.0f) {
                wheel->direction_change_pending = false;
            }
            return false;
        }
        wheel->direction_change_pending = false;
    }

    if (abs_float(requested) >
        abs_float(wheel->ramped_target_speed_cmps)) {
        maximum_step = g_appConfig.wheel_control_max_accel_cmps2 *
            elapsed_seconds;
    } else {
        maximum_step = g_appConfig.wheel_control_max_decel_cmps2 *
            elapsed_seconds;
    }
    wheel->ramped_target_speed_cmps = move_toward(
        wheel->ramped_target_speed_cmps, requested, maximum_step);
    return true;
}

static int16_t update_wheel(MotorControlWheelRuntime *wheel, float kp,
    float ki, float feedforward_gain, float elapsed_seconds)
{
    float candidate_integral;
    float candidate_integral_term;
    float unsaturated_output;
    float minimum_output;
    float maximum_output;
    float limited_output;
    bool reject_integral;

    if (!update_ramped_target(wheel, elapsed_seconds)) {
        return 0;
    }

    wheel->error_cmps = wheel->ramped_target_speed_cmps -
        wheel->measured_speed_cmps;
    wheel->proportional_term = kp * wheel->error_cmps;
    wheel->feedforward_term =
        (wheel->ramped_target_speed_cmps /
            g_appConfig.wheel_control_max_speed_cmps) *
        (float)MOTOR_MAX_DUTY * feedforward_gain;

    candidate_integral = wheel->integral +
        wheel->error_cmps * elapsed_seconds;
    candidate_integral = clamp_float(candidate_integral,
        -g_appConfig.wheel_control_integral_limit,
        g_appConfig.wheel_control_integral_limit);
    candidate_integral_term = ki * candidate_integral;

    if (wheel->ramped_target_speed_cmps > 0.0f) {
        minimum_output = 0.0f;
        maximum_output = (float)MOTOR_MAX_DUTY;
    } else {
        minimum_output = (float)-MOTOR_MAX_DUTY;
        maximum_output = 0.0f;
    }

    unsaturated_output = wheel->feedforward_term +
        wheel->proportional_term + candidate_integral_term;
    reject_integral =
        ((unsaturated_output > maximum_output) &&
            (wheel->error_cmps > 0.0f)) ||
        ((unsaturated_output < minimum_output) &&
            (wheel->error_cmps < 0.0f));
    if (!reject_integral) {
        wheel->integral = candidate_integral;
    }
    wheel->integral_term = ki * wheel->integral;

    unsaturated_output = wheel->feedforward_term +
        wheel->proportional_term + wheel->integral_term;
    limited_output = clamp_float(unsaturated_output, minimum_output,
        maximum_output);
    wheel->saturated = limited_output != unsaturated_output;
    wheel->output_command = (int16_t)limited_output;
    return wheel->output_command;
}

void MotorControl_Init(void)
{
    MotorControl_Reset();
}

void MotorControl_SetNormalizedTarget(int16_t left_command,
    int16_t right_command)
{
    float maximum_speed = g_appConfig.wheel_control_max_speed_cmps;

    left_command = clamp_normalized_command(left_command);
    right_command = clamp_normalized_command(right_command);
    g_runtime.left.normalized_target = left_command;
    g_runtime.right.normalized_target = right_command;

    if (config_value_in_range(maximum_speed, 0.1f,
        MOTOR_CONTROL_CONFIG_SPEED_MAX_CMPS)) {
        g_runtime.left.raw_target_speed_cmps =
            ((float)left_command / (float)MOTOR_MAX_DUTY) * maximum_speed;
        g_runtime.right.raw_target_speed_cmps =
            ((float)right_command / (float)MOTOR_MAX_DUTY) * maximum_speed;
        g_runtime.left.raw_target_speed_cmps = clamp_float(
            g_runtime.left.raw_target_speed_cmps, -maximum_speed,
            maximum_speed);
        g_runtime.right.raw_target_speed_cmps = clamp_float(
            g_runtime.right.raw_target_speed_cmps, -maximum_speed,
            maximum_speed);
    } else {
        g_runtime.left.raw_target_speed_cmps = 0.0f;
        g_runtime.right.raw_target_speed_cmps = 0.0f;
    }

    g_runtime.target_age_ms = 0U;
    g_targetRefreshed = true;
}

void MotorControl_SetSpeedTargetCmps(float left_target_cmps,
    float right_target_cmps)
{
    float maximum_speed = g_appConfig.wheel_control_max_speed_cmps;
    float left_normalized;
    float right_normalized;

    if (!config_value_in_range(maximum_speed, 0.1f,
        MOTOR_CONTROL_CONFIG_SPEED_MAX_CMPS)) {
        g_runtime.left.normalized_target = 0;
        g_runtime.right.normalized_target = 0;
        g_runtime.left.raw_target_speed_cmps = 0.0f;
        g_runtime.right.raw_target_speed_cmps = 0.0f;
        g_runtime.target_age_ms = 0U;
        g_targetRefreshed = true;
        return;
    }

    if (left_target_cmps != left_target_cmps) {
        left_target_cmps = 0.0f;
    }
    if (right_target_cmps != right_target_cmps) {
        right_target_cmps = 0.0f;
    }
    left_target_cmps = clamp_float(left_target_cmps, -maximum_speed,
        maximum_speed);
    right_target_cmps = clamp_float(right_target_cmps, -maximum_speed,
        maximum_speed);

    left_normalized = (left_target_cmps / maximum_speed) *
        (float)MOTOR_MAX_DUTY;
    right_normalized = (right_target_cmps / maximum_speed) *
        (float)MOTOR_MAX_DUTY;
    left_normalized += (left_normalized > 0.0f) ? 0.5f :
        ((left_normalized < 0.0f) ? -0.5f : 0.0f);
    right_normalized += (right_normalized > 0.0f) ? 0.5f :
        ((right_normalized < 0.0f) ? -0.5f : 0.0f);

    g_runtime.left.normalized_target = clamp_normalized_command(
        (int16_t)left_normalized);
    g_runtime.right.normalized_target = clamp_normalized_command(
        (int16_t)right_normalized);
    g_runtime.left.raw_target_speed_cmps = left_target_cmps;
    g_runtime.right.raw_target_speed_cmps = right_target_cmps;
    g_runtime.target_age_ms = 0U;
    g_targetRefreshed = true;
}

void MotorControl_Update(uint32_t elapsed_ms)
{
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();
    uint32_t estimator_errors = 0U;
    int16_t left_output;
    int16_t right_output;
    float elapsed_seconds;

    g_runtime.enabled = (FEATURE_WHEEL_SPEED_CONTROL != 0);
    g_runtime.sample_dt_ms = elapsed_ms;
    g_runtime.update_count++;
    g_runtime.left.measured_speed_cmps = wheel->left_speed_cmps;
    g_runtime.right.measured_speed_cmps = wheel->right_speed_cmps;

    if (!g_runtime.enabled) {
        g_runtime.valid = false;
        return;
    }

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        g_runtime.safety_inhibited = true;
        latch_control_error(MOTOR_CONTROL_ERROR_SAFETY_INHIBIT, 0U,
            false);
        return;
    }

    if (CarController_IsSafetyHoldActive()) {
        MotorControl_Stop();
        g_runtime.safety_inhibited = true;
        g_runtime.error_flags |= MOTOR_CONTROL_ERROR_SAFETY_INHIBIT;
        g_runtime.valid = false;
        return;
    }

    if (!g_runtime.error_latched) {
        g_runtime.error_flags &= ~MOTOR_CONTROL_ERROR_SAFETY_INHIBIT;
    }
    if (CarState_Get() != CAR_STATE_RUNNING) {
        MotorControl_Stop();
        g_runtime.safety_inhibited = false;
        g_runtime.valid = false;
        return;
    }

    g_runtime.safety_inhibited = false;
    if (g_runtime.error_latched) {
        Motor_Stop();
        g_runtime.valid = false;
        return;
    }

    if (!config_is_valid()) {
        latch_control_error(MOTOR_CONTROL_ERROR_INVALID_CONFIG, 0U, true);
        return;
    }
    if ((elapsed_ms == 0U) || (elapsed_ms > MOTOR_CONTROL_DT_MAX_MS)) {
        latch_control_error(MOTOR_CONTROL_ERROR_INVALID_DT, 0U, true);
        return;
    }

    if (g_targetRefreshed) {
        g_runtime.target_age_ms = 0U;
        g_targetRefreshed = false;
    } else if ((g_runtime.left.normalized_target != 0) ||
        (g_runtime.right.normalized_target != 0)) {
        g_runtime.target_age_ms = add_elapsed_u32(
            g_runtime.target_age_ms, elapsed_ms);
    }
    if (((g_runtime.left.normalized_target != 0) ||
        (g_runtime.right.normalized_target != 0)) &&
        (g_runtime.target_age_ms >=
            g_appConfig.wheel_control_target_timeout_ms)) {
        latch_control_error(MOTOR_CONTROL_ERROR_TARGET_TIMEOUT, 0U, true);
        return;
    }

    if (!wheel->valid) {
        estimator_errors |= MOTOR_CONTROL_ERROR_ESTIMATOR_INVALID;
    }
    if (wheel->stale) {
        estimator_errors |= MOTOR_CONTROL_ERROR_ESTIMATOR_STALE;
    }
    if (wheel->error_flags != 0U) {
        estimator_errors |= MOTOR_CONTROL_ERROR_ESTIMATOR_FAULT;
    }
    if (estimator_errors != 0U) {
        latch_control_error(estimator_errors, wheel->error_flags, true);
        return;
    }
    if (!feedback_is_valid(wheel->left_speed_cmps) ||
        !feedback_is_valid(wheel->right_speed_cmps)) {
        latch_control_error(MOTOR_CONTROL_ERROR_FEEDBACK_RANGE,
            wheel->error_flags, true);
        return;
    }

    elapsed_seconds = (float)elapsed_ms / 1000.0f;
    left_output = update_wheel(&g_runtime.left,
        g_appConfig.wheel_control_left_kp,
        g_appConfig.wheel_control_left_ki,
        g_appConfig.wheel_control_left_feedforward_gain, elapsed_seconds);
    right_output = update_wheel(&g_runtime.right,
        g_appConfig.wheel_control_right_kp,
        g_appConfig.wheel_control_right_ki,
        g_appConfig.wheel_control_right_feedforward_gain, elapsed_seconds);

    if ((g_runtime.left.normalized_target == 0) &&
        (g_runtime.right.normalized_target == 0)) {
        Motor_Stop();
    } else {
        Motor_SetSpeed(left_output, right_output);
    }
    g_runtime.valid = true;
}

void MotorControl_Stop(void)
{
    stop_output(true);
    g_runtime.target_age_ms = 0U;
    g_targetRefreshed = false;
    if (!g_runtime.error_latched) {
        g_runtime.target_refresh_timeout = false;
        g_runtime.valid = g_runtime.enabled;
    }
}

void MotorControl_Reset(void)
{
    MotorControlRuntime initial = {0};

    Motor_Stop();
    g_runtime = initial;
    g_runtime.enabled = (FEATURE_WHEEL_SPEED_CONTROL != 0);
    g_targetRefreshed = false;
}

const MotorControlRuntime *MotorControl_GetRuntime(void)
{
    return &g_runtime;
}
