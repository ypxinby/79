#include "line_controller.h"

#include <limits.h>
#include <string.h>

#include "app_config.h"
#include "app_features.h"
#include "track_sensor.h"

#define LINE_NORMALIZED_COMMAND_MAX    (1000)
#define LINE_SENSOR_S1_MASK             ((uint8_t)(1U << 0))
#define LINE_SENSOR_S7_MASK             ((uint8_t)(1U << 6))

static LineControllerRuntime g_lineRuntime;

static int16_t clamp_i16(int32_t value, int16_t min_value,
    int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return (int16_t)value;
}

static int16_t round_clamped_float(float value, int16_t min_value,
    int16_t max_value)
{
    float rounded;

    if (value != value) {
        return 0;
    }
    if (value <= (float)min_value) {
        return min_value;
    }
    if (value >= (float)max_value) {
        return max_value;
    }

    rounded = (value >= 0.0f) ? (value + 0.5f) : (value - 0.5f);
    return (int16_t)rounded;
}

static bool config_is_valid(void)
{
    return
        (g_appConfig.line_control_v2_base_command >= 0) &&
        (g_appConfig.line_control_v2_base_command <=
            LINE_NORMALIZED_COMMAND_MAX) &&
        (g_appConfig.line_control_v2_lost_turn_command >= 0) &&
        (g_appConfig.line_control_v2_lost_turn_command <=
            LINE_NORMALIZED_COMMAND_MAX) &&
        (g_appConfig.line_lost_search_max_ms > 0U) &&
        (g_appConfig.line_control_v2_error_filter_alpha > 0.0f) &&
        (g_appConfig.line_control_v2_error_filter_alpha <= 1.0f) &&
        (g_appConfig.line_control_v2_derivative_filter_alpha > 0.0f) &&
        (g_appConfig.line_control_v2_derivative_filter_alpha <= 1.0f) &&
        (g_appConfig.line_control_v2_kp >= 0.0f) &&
        (g_appConfig.line_control_v2_kd >= 0.0f) &&
        (g_appConfig.line_control_v2_max_correction >= 0) &&
        (g_appConfig.line_control_v2_max_correction <=
            LINE_NORMALIZED_COMMAND_MAX) &&
        (g_appConfig.line_control_v2_min_running_command >= 0) &&
        (g_appConfig.line_control_v2_min_running_command <=
            LINE_NORMALIZED_COMMAND_MAX) &&
        (g_appConfig.line_control_v2_min_dt_ms > 0U) &&
        (g_appConfig.line_control_v2_max_dt_ms >=
            g_appConfig.line_control_v2_min_dt_ms);
}

static uint32_t add_elapsed_u32(uint32_t value, uint32_t elapsed_ms)
{
    if (value > UINT32_MAX - elapsed_ms) {
        return UINT32_MAX;
    }
    return value + elapsed_ms;
}

static void reset_pd_values(void)
{
    g_lineRuntime.dt_valid = false;
    g_lineRuntime.initialized = false;
    g_lineRuntime.filtered_error = 0.0f;
    g_lineRuntime.raw_derivative = 0.0f;
    g_lineRuntime.filtered_derivative = 0.0f;
    g_lineRuntime.correction_raw = 0.0f;
    g_lineRuntime.correction = 0;
    g_lineRuntime.left_low_speed_zeroed = false;
    g_lineRuntime.right_low_speed_zeroed = false;
}

static void clear_turn_mark(void)
{
    g_lineRuntime.turn_mark = LINE_TURN_DIRECTION_UNKNOWN;
    g_lineRuntime.turn_mark_valid = false;
}

static void update_turn_mark(uint8_t sensor_pattern)
{
    bool s1_active = (sensor_pattern & LINE_SENSOR_S1_MASK) != 0U;
    bool s7_active = (sensor_pattern & LINE_SENSOR_S7_MASK) != 0U;

    if (s1_active && !s7_active) {
        g_lineRuntime.turn_mark = LINE_TURN_DIRECTION_LEFT;
    } else if (s7_active && !s1_active) {
        g_lineRuntime.turn_mark = LINE_TURN_DIRECTION_RIGHT;
    } else {
        return;
    }

    g_lineRuntime.turn_mark_valid = true;
    if (g_lineRuntime.turn_mark_update_count < UINT32_MAX) {
        g_lineRuntime.turn_mark_update_count++;
    }
}

static void set_target_commands(int16_t left_command,
    int16_t right_command)
{
    g_lineRuntime.left_target_command = left_command;
    g_lineRuntime.right_target_command = right_command;
}

static void enter_stopped_state(LineControlStopReason reason)
{
    reset_pd_values();
    g_lineRuntime.state = LINE_CONTROL_STATE_STOP;
    g_lineRuntime.stop_reason = reason;
    g_lineRuntime.recover_timeout =
        reason == LINE_CONTROL_STOP_REASON_RECOVER_TIMEOUT;
    set_target_commands(0, 0);
}

static void update_lost_turn_output(uint32_t elapsed_ms)
{
    g_lineRuntime.lost_elapsed_ms = add_elapsed_u32(
        g_lineRuntime.lost_elapsed_ms, elapsed_ms);

    if (g_lineRuntime.lost_elapsed_ms >=
        g_appConfig.line_lost_search_max_ms) {
        enter_stopped_state(LINE_CONTROL_STOP_REASON_RECOVER_TIMEOUT);
        return;
    }

    if (g_lineRuntime.state == LINE_CONTROL_STATE_LOST_TURN_LEFT) {
        set_target_commands(0,
            g_appConfig.line_control_v2_lost_turn_command);
    } else {
        set_target_commands(g_appConfig.line_control_v2_lost_turn_command,
            0);
    }
}

static int16_t apply_running_command_floor(int32_t command,
    bool *zeroed)
{
    int16_t limited = clamp_i16(command, 0, LINE_NORMALIZED_COMMAND_MAX);

    *zeroed = false;
    if ((limited > 0) &&
        (limited < g_appConfig.line_control_v2_min_running_command)) {
        limited = 0;
        *zeroed = true;
    }
    return limited;
}

void LineController_Init(void)
{
    LineController_Reset();
}

void LineController_Reset(void)
{
    memset(&g_lineRuntime, 0, sizeof(g_lineRuntime));
    g_lineRuntime.enabled = FEATURE_LINE_CONTROL_V2 ? true : false;
    g_lineRuntime.config_valid = config_is_valid();
    g_lineRuntime.state = LINE_CONTROL_STATE_FOLLOW;
    g_lineRuntime.turn_mark = LINE_TURN_DIRECTION_UNKNOWN;
}

void LineController_ResetControlState(void)
{
    reset_pd_values();
    g_lineRuntime.base_command = 0;
    set_target_commands(0, 0);
    g_lineRuntime.state = LINE_CONTROL_STATE_FOLLOW;
    clear_turn_mark();
    g_lineRuntime.lost_elapsed_ms = 0U;
    g_lineRuntime.recover_timeout = false;
    g_lineRuntime.stop_reason = LINE_CONTROL_STOP_REASON_NONE;
    g_lineRuntime.enabled = FEATURE_LINE_CONTROL_V2 ? true : false;
    g_lineRuntime.config_valid = config_is_valid();
}

void LineController_ObserveSensors(uint8_t sensor_pattern)
{
    g_lineRuntime.sensor_pattern =
        (uint8_t)(sensor_pattern & TRACK_RAW_VALID_MASK);
    g_lineRuntime.active_count =
        TrackSensor_CountBlack(g_lineRuntime.sensor_pattern);
    g_lineRuntime.line_valid = g_lineRuntime.active_count != 0U;
    g_lineRuntime.raw_error = g_lineRuntime.line_valid ?
        TrackSensor_GetErrorFromRaw(g_lineRuntime.sensor_pattern) : 0;
}

void LineController_Update(uint32_t elapsed_ms, uint8_t sensor_pattern,
    int16_t base_command, int16_t *left_command, int16_t *right_command)
{
    float previous_filtered_error;
    float error_alpha;
    float derivative_alpha;
    int16_t correction_limit;
    int16_t correction;

    if (left_command != (int16_t *)0) {
        *left_command = 0;
    }
    if (right_command != (int16_t *)0) {
        *right_command = 0;
    }

    g_lineRuntime.enabled = FEATURE_LINE_CONTROL_V2 ? true : false;
    g_lineRuntime.config_valid = config_is_valid();
    LineController_ObserveSensors(sensor_pattern);
    g_lineRuntime.base_command = clamp_i16(base_command, 0,
        LINE_NORMALIZED_COMMAND_MAX);
    if (g_lineRuntime.update_count < UINT32_MAX) {
        g_lineRuntime.update_count++;
    }

    if (!g_lineRuntime.enabled || !g_lineRuntime.config_valid) {
        reset_pd_values();
        set_target_commands(0, 0);
        return;
    }

    if (g_lineRuntime.state == LINE_CONTROL_STATE_STOP) {
        set_target_commands(0, 0);
        goto copy_outputs;
    }

    if ((g_lineRuntime.state == LINE_CONTROL_STATE_LOST_TURN_LEFT) ||
        (g_lineRuntime.state == LINE_CONTROL_STATE_LOST_TURN_RIGHT)) {
        if (!g_lineRuntime.line_valid) {
            update_lost_turn_output(elapsed_ms);
            goto copy_outputs;
        }

        g_lineRuntime.state = LINE_CONTROL_STATE_FOLLOW;
        g_lineRuntime.lost_elapsed_ms = 0U;
        g_lineRuntime.recover_timeout = false;
        g_lineRuntime.stop_reason = LINE_CONTROL_STOP_REASON_NONE;
        clear_turn_mark();
        reset_pd_values();
    }

    if (!g_lineRuntime.line_valid) {
        reset_pd_values();
        if (!g_lineRuntime.turn_mark_valid) {
            g_lineRuntime.lost_elapsed_ms = 0U;
            enter_stopped_state(LINE_CONTROL_STOP_REASON_NO_DIRECTION);
            goto copy_outputs;
        }

        g_lineRuntime.state =
            (g_lineRuntime.turn_mark == LINE_TURN_DIRECTION_LEFT) ?
            LINE_CONTROL_STATE_LOST_TURN_LEFT :
            LINE_CONTROL_STATE_LOST_TURN_RIGHT;
        g_lineRuntime.lost_elapsed_ms = 0U;
        g_lineRuntime.recover_timeout = false;
        g_lineRuntime.stop_reason = LINE_CONTROL_STOP_REASON_NONE;
        update_lost_turn_output(elapsed_ms);
        goto copy_outputs;
    }

    g_lineRuntime.last_valid_pattern = g_lineRuntime.sensor_pattern;
    g_lineRuntime.last_valid_error = g_lineRuntime.raw_error;
    update_turn_mark(g_lineRuntime.sensor_pattern);

    previous_filtered_error = g_lineRuntime.filtered_error;
    error_alpha = g_appConfig.line_control_v2_error_filter_alpha;
    derivative_alpha =
        g_appConfig.line_control_v2_derivative_filter_alpha;

    if (!g_lineRuntime.initialized) {
        g_lineRuntime.filtered_error = (float)g_lineRuntime.raw_error;
        g_lineRuntime.raw_derivative = 0.0f;
        g_lineRuntime.filtered_derivative = 0.0f;
        g_lineRuntime.initialized = true;
    } else {
        g_lineRuntime.filtered_error += error_alpha *
            ((float)g_lineRuntime.raw_error -
                g_lineRuntime.filtered_error);

        g_lineRuntime.dt_valid =
            (elapsed_ms >= g_appConfig.line_control_v2_min_dt_ms) &&
            (elapsed_ms <= g_appConfig.line_control_v2_max_dt_ms);
        if (g_lineRuntime.dt_valid) {
            g_lineRuntime.raw_derivative =
                (g_lineRuntime.filtered_error - previous_filtered_error) *
                (1000.0f / (float)elapsed_ms);
            g_lineRuntime.filtered_derivative += derivative_alpha *
                (g_lineRuntime.raw_derivative -
                    g_lineRuntime.filtered_derivative);
        } else {
            g_lineRuntime.raw_derivative = 0.0f;
            g_lineRuntime.filtered_derivative = 0.0f;
        }
    }

    if (!g_lineRuntime.dt_valid) {
        g_lineRuntime.dt_valid =
            (elapsed_ms >= g_appConfig.line_control_v2_min_dt_ms) &&
            (elapsed_ms <= g_appConfig.line_control_v2_max_dt_ms);
    }

    g_lineRuntime.correction_raw =
        g_appConfig.line_control_v2_kp * g_lineRuntime.filtered_error +
        g_appConfig.line_control_v2_kd *
            g_lineRuntime.filtered_derivative;
    correction_limit = g_appConfig.line_control_v2_max_correction;
    correction = round_clamped_float(g_lineRuntime.correction_raw,
        (int16_t)-correction_limit, correction_limit);

    g_lineRuntime.correction = correction;
    g_lineRuntime.left_target_command = apply_running_command_floor(
        (int32_t)g_lineRuntime.base_command + correction,
        &g_lineRuntime.left_low_speed_zeroed);
    g_lineRuntime.right_target_command = apply_running_command_floor(
        (int32_t)g_lineRuntime.base_command - correction,
        &g_lineRuntime.right_low_speed_zeroed);

copy_outputs:
    if (left_command != (int16_t *)0) {
        *left_command = g_lineRuntime.left_target_command;
    }
    if (right_command != (int16_t *)0) {
        *right_command = g_lineRuntime.right_target_command;
    }
}

const LineControllerRuntime *LineController_GetRuntime(void)
{
    return &g_lineRuntime;
}
