#include "balance_position_control.h"

#include <limits.h>

#include "app_features.h"
#include "gimbal_stepper.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_POSITION_CONTROL

static volatile BalancePositionRuntime g_runtime;
static int32_t g_moveStartEncoderCount;
static int32_t g_lastEncoderCount;
static int64_t g_moveStartStepCount;
static int64_t g_lastStepCount;
static uint32_t g_startSoftLimitClampCount;

static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static uint32_t add_u32_saturating(uint32_t value, uint32_t addend)
{
    if (addend > (UINT32_MAX - value)) {
        return UINT32_MAX;
    }
    return value + addend;
}

static uint64_t magnitude_i64(int64_t value)
{
    if (value >= 0) {
        return (uint64_t)value;
    }
    return (uint64_t)(-(value + 1)) + 1U;
}

static int32_t count_error_to_steps(int32_t error_count)
{
    int64_t scaled = (int64_t)error_count *
        BALANCE_STEPPER_COMMAND_STEPS_PER_REV;
    int32_t steps;

    if (scaled > 0) {
        scaled += BALANCE_ENCODER_COUNTS_PER_REV / 2;
    } else if (scaled < 0) {
        scaled -= BALANCE_ENCODER_COUNTS_PER_REV / 2;
    }
    steps = clamp_i64_to_i32(scaled /
        BALANCE_ENCODER_COUNTS_PER_REV);
    if ((steps == 0) && (error_count != 0)) {
        steps = (error_count > 0) ? 1 : -1;
    }
    return steps;
}

static uint32_t calculate_timeout_ms(int32_t error_count)
{
    uint64_t count_magnitude = magnitude_i64(error_count);
    uint64_t step_count =
        (count_magnitude * BALANCE_STEPPER_COMMAND_STEPS_PER_REV +
            BALANCE_ENCODER_COUNTS_PER_REV - 1U) /
        BALANCE_ENCODER_COUNTS_PER_REV;
    uint64_t expected_ms =
        (step_count * 1000U + BALANCE_POSITION_MAX_STEP_RATE_HZ - 1U) /
        BALANCE_POSITION_MAX_STEP_RATE_HZ;
    uint64_t timeout_ms = expected_ms *
        BALANCE_POSITION_TIMEOUT_MULTIPLIER +
        BALANCE_POSITION_TIMEOUT_MARGIN_MS;

    if (timeout_ms < BALANCE_POSITION_TIMEOUT_MIN_MS) {
        timeout_ms = BALANCE_POSITION_TIMEOUT_MIN_MS;
    }
    if (timeout_ms > BALANCE_POSITION_TIMEOUT_MAX_MS) {
        timeout_ms = BALANCE_POSITION_TIMEOUT_MAX_MS;
    }
    return (uint32_t)timeout_ms;
}

static uint16_t calculate_step_rate_hz(int32_t error_count)
{
    uint64_t magnitude = magnitude_i64(error_count);
    uint64_t rate;

    if (magnitude <= BALANCE_POSITION_DEADBAND_COUNTS) {
        return 0U;
    }
    rate = BALANCE_POSITION_MIN_STEP_RATE_HZ +
        (magnitude - BALANCE_POSITION_DEADBAND_COUNTS) *
            BALANCE_POSITION_STEP_RATE_KP;
    if (rate > BALANCE_POSITION_MAX_STEP_RATE_HZ) {
        rate = BALANCE_POSITION_MAX_STEP_RATE_HZ;
    }
    return (uint16_t)rate;
}

static uint16_t step_rate_to_half_period_ticks(uint16_t step_rate_hz)
{
    uint32_t ticks;

    if (step_rate_hz == 0U) {
        return 0U;
    }
    ticks = (5000U + (step_rate_hz / 2U)) / step_rate_hz;
    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks > UINT16_MAX) {
        ticks = UINT16_MAX;
    }
    return (uint16_t)ticks;
}

static void reset_motion_observers(int32_t current_count,
    uint32_t soft_limit_clamp_count)
{
    GimbalStepperFeedback stepper;

    GimbalStepper_GetFeedbackSnapshot(&stepper);
    g_moveStartEncoderCount = current_count;
    g_lastEncoderCount = current_count;
    g_moveStartStepCount = stepper.estimated_steps;
    g_lastStepCount = stepper.estimated_steps;
    g_startSoftLimitClampCount = soft_limit_clamp_count;
    g_runtime.elapsed_ms = 0U;
    g_runtime.timeout_ms = calculate_timeout_ms(
        clamp_i64_to_i32(
            (int64_t)g_runtime.target_count - current_count));
    g_runtime.settle_ms = 0U;
    g_runtime.follow_error_ms = 0U;
    g_runtime.direction_error_ms = 0U;
    g_runtime.steps_without_feedback = 0U;
    g_runtime.following_error_count = 0;
}

static void latch_fault(BalancePositionFault fault)
{
    GimbalStepper_StopHold();
    g_runtime.commanded_step_rate_hz = 0U;
    g_runtime.step_half_period_ticks = 0U;
    g_runtime.busy = 0U;
    g_runtime.hold_enabled = 0U;
    g_runtime.target_reached = 0U;
    g_runtime.tracking_enabled = 0U;
    g_runtime.state = BALANCE_POSITION_STATE_FAULT;
    g_runtime.fault = fault;
}

void BalancePositionControl_Init(void)
{
    g_runtime = (BalancePositionRuntime){0};
    g_runtime.state = BALANCE_POSITION_STATE_IDLE;
    g_runtime.fault = BALANCE_POSITION_FAULT_NONE;
    g_moveStartEncoderCount = 0;
    g_lastEncoderCount = 0;
    g_moveStartStepCount = 0;
    g_lastStepCount = 0;
    g_startSoftLimitClampCount = 0U;
}

uint8_t BalancePositionControl_Start(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count)
{
    if ((g_runtime.fault != BALANCE_POSITION_FAULT_NONE) ||
        (minimum_count >= maximum_count) ||
        ((int64_t)target_count <
            ((int64_t)minimum_count +
                BALANCE_POSITION_LIMIT_MARGIN_COUNTS)) ||
        ((int64_t)target_count >
            ((int64_t)maximum_count -
                BALANCE_POSITION_LIMIT_MARGIN_COUNTS))) {
        return 0U;
    }

    GimbalStepper_StopHold();
    g_runtime.target_count = target_count;
    g_runtime.current_count = current_count;
    g_runtime.position_error_count = clamp_i64_to_i32(
        (int64_t)target_count - current_count);
    g_runtime.minimum_count = minimum_count;
    g_runtime.maximum_count = maximum_count;
    g_runtime.commanded_step_rate_hz = 0U;
    g_runtime.step_half_period_ticks = 0U;
    g_runtime.busy = 1U;
    g_runtime.hold_enabled = 1U;
    g_runtime.target_reached = 0U;
    g_runtime.tracking_enabled = 0U;
    g_runtime.state = BALANCE_POSITION_STATE_MOVING;
    reset_motion_observers(current_count, soft_limit_clamp_count);
    return 1U;
}

uint8_t BalancePositionControl_StartTracking(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count)
{
    if (BalancePositionControl_Start(target_count, current_count,
            minimum_count, maximum_count,
            soft_limit_clamp_count) == 0U) {
        return 0U;
    }
    g_runtime.tracking_enabled = 1U;
    g_runtime.elapsed_ms = 0U;
    g_runtime.timeout_ms = 0U;
    return 1U;
}

uint8_t BalancePositionControl_SetTrackingTarget(int32_t target_count)
{
    if ((g_runtime.tracking_enabled == 0U) ||
        (g_runtime.fault != BALANCE_POSITION_FAULT_NONE) ||
        ((int64_t)target_count <
            ((int64_t)g_runtime.minimum_count +
                BALANCE_POSITION_LIMIT_MARGIN_COUNTS)) ||
        ((int64_t)target_count >
            ((int64_t)g_runtime.maximum_count -
                BALANCE_POSITION_LIMIT_MARGIN_COUNTS))) {
        return 0U;
    }
    g_runtime.target_count = target_count;
    g_runtime.target_reached = 0U;
    return 1U;
}

void BalancePositionControl_Update20ms(int32_t current_count,
    uint32_t elapsed_ms, uint32_t soft_limit_clamp_count)
{
    GimbalStepperFeedback stepper;
    int64_t step_delta;
    int32_t encoder_delta;
    int64_t expected_encoder_delta;
    int64_t actual_encoder_delta;
    int64_t following_error;
    int32_t command_steps;
    uint16_t step_rate_hz;
    uint16_t half_period_ticks;
    uint64_t error_magnitude;

    if (g_runtime.fault != BALANCE_POSITION_FAULT_NONE) {
        return;
    }

    g_runtime.current_count = current_count;
    g_runtime.position_error_count =
        clamp_i64_to_i32((int64_t)g_runtime.target_count - current_count);

    if (g_runtime.hold_enabled == 0U) {
        return;
    }

    error_magnitude = magnitude_i64(g_runtime.position_error_count);
    if (g_runtime.busy == 0U) {
        if (error_magnitude <= BALANCE_POSITION_REENGAGE_COUNTS) {
            return;
        }
        g_runtime.busy = 1U;
        g_runtime.target_reached = 0U;
        g_runtime.state = BALANCE_POSITION_STATE_MOVING;
        reset_motion_observers(current_count, soft_limit_clamp_count);
    }

    if (soft_limit_clamp_count != g_startSoftLimitClampCount) {
        latch_fault(BALANCE_POSITION_FAULT_SOFT_LIMIT);
        return;
    }

    g_runtime.elapsed_ms = add_u32_saturating(
        g_runtime.elapsed_ms, elapsed_ms);
    if ((g_runtime.tracking_enabled == 0U) &&
        (g_runtime.elapsed_ms > g_runtime.timeout_ms)) {
        latch_fault(BALANCE_POSITION_FAULT_TIMEOUT);
        return;
    }

    GimbalStepper_GetFeedbackSnapshot(&stepper);
    step_delta = stepper.estimated_steps - g_lastStepCount;
    encoder_delta = current_count - g_lastEncoderCount;

    if (encoder_delta != 0) {
        g_runtime.steps_without_feedback = 0U;
    } else {
        uint64_t step_magnitude = magnitude_i64(step_delta);

        if (step_magnitude >
            (UINT32_MAX - g_runtime.steps_without_feedback)) {
            g_runtime.steps_without_feedback = UINT32_MAX;
        } else {
            g_runtime.steps_without_feedback += (uint32_t)step_magnitude;
        }
    }
    if (g_runtime.steps_without_feedback >=
        BALANCE_POSITION_NO_FEEDBACK_STEP_THRESHOLD) {
        latch_fault(BALANCE_POSITION_FAULT_NO_FEEDBACK);
        return;
    }

    expected_encoder_delta =
        ((stepper.estimated_steps - g_moveStartStepCount) *
            BALANCE_ENCODER_COUNTS_PER_REV) /
        BALANCE_STEPPER_COMMAND_STEPS_PER_REV;
    actual_encoder_delta =
        (int64_t)current_count - g_moveStartEncoderCount;
    following_error = expected_encoder_delta - actual_encoder_delta;
    g_runtime.following_error_count = clamp_i64_to_i32(following_error);

    if (magnitude_i64(following_error) >
        BALANCE_POSITION_FOLLOW_ERROR_COUNTS) {
        g_runtime.follow_error_ms = add_u32_saturating(
            g_runtime.follow_error_ms, elapsed_ms);
    } else {
        g_runtime.follow_error_ms = 0U;
    }
    if (g_runtime.follow_error_ms >=
        BALANCE_POSITION_FOLLOW_ERROR_DURATION_MS) {
        latch_fault(BALANCE_POSITION_FAULT_FOLLOW_ERROR);
        return;
    }

    if (((step_delta > 0) && (encoder_delta < 0)) ||
        ((step_delta < 0) && (encoder_delta > 0))) {
        g_runtime.direction_error_ms = add_u32_saturating(
            g_runtime.direction_error_ms, elapsed_ms);
    } else {
        g_runtime.direction_error_ms = 0U;
    }
    if (g_runtime.direction_error_ms >=
        BALANCE_POSITION_DIRECTION_ERROR_DURATION_MS) {
        latch_fault(BALANCE_POSITION_FAULT_DIRECTION);
        return;
    }

    g_lastStepCount = stepper.estimated_steps;
    g_lastEncoderCount = current_count;

    if (error_magnitude <= BALANCE_POSITION_DEADBAND_COUNTS) {
        GimbalStepper_StopHold();
        g_runtime.commanded_step_rate_hz = 0U;
        g_runtime.step_half_period_ticks = 0U;
        g_runtime.state = BALANCE_POSITION_STATE_SETTLING;
        g_runtime.settle_ms = add_u32_saturating(
            g_runtime.settle_ms, elapsed_ms);
        if (g_runtime.settle_ms >= BALANCE_POSITION_SETTLE_MS) {
            g_runtime.busy = 0U;
            g_runtime.target_reached = 1U;
            g_runtime.state = BALANCE_POSITION_STATE_HOLD;
        }
        return;
    }

    g_runtime.settle_ms = 0U;
    g_runtime.state = BALANCE_POSITION_STATE_MOVING;
    step_rate_hz = calculate_step_rate_hz(
        g_runtime.position_error_count);
    half_period_ticks = step_rate_to_half_period_ticks(step_rate_hz);
    command_steps = count_error_to_steps(
        g_runtime.position_error_count);
    g_runtime.commanded_step_rate_hz = step_rate_hz;
    g_runtime.step_half_period_ticks = half_period_ticks;
    GimbalStepper_SetStepHalfPeriodTicks(half_period_ticks);
    GimbalStepper_MoveRelativeSteps(command_steps);
}

void BalancePositionControl_Cancel(void)
{
    GimbalStepper_StopHold();
    g_runtime.commanded_step_rate_hz = 0U;
    g_runtime.step_half_period_ticks = 0U;
    g_runtime.busy = 0U;
    g_runtime.hold_enabled = 0U;
    g_runtime.target_reached = 0U;
    g_runtime.tracking_enabled = 0U;
    if (g_runtime.fault == BALANCE_POSITION_FAULT_NONE) {
        g_runtime.state = BALANCE_POSITION_STATE_IDLE;
    }
}

uint8_t BalancePositionControl_ResetFault(void)
{
    if (GimbalStepper_GetFeedback()->running != 0U) {
        return 0U;
    }
    GimbalStepper_StopHold();
    BalancePositionControl_Init();
    return 1U;
}

uint8_t BalancePositionControl_IsBusy(void)
{
    return g_runtime.busy;
}

uint8_t BalancePositionControl_HasFault(void)
{
    return (g_runtime.fault != BALANCE_POSITION_FAULT_NONE) ? 1U : 0U;
}

void BalancePositionControl_GetSnapshot(BalancePositionRuntime *snapshot)
{
    uint32_t primask;

    if (snapshot == (BalancePositionRuntime *)0) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_runtime;
    if (primask == 0U) {
        __enable_irq();
    }
}

#else

static BalancePositionRuntime g_runtime;

void BalancePositionControl_Init(void)
{
    g_runtime = (BalancePositionRuntime){0};
}

uint8_t BalancePositionControl_Start(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count)
{
    (void)target_count;
    (void)current_count;
    (void)minimum_count;
    (void)maximum_count;
    (void)soft_limit_clamp_count;
    return 0U;
}

uint8_t BalancePositionControl_StartTracking(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count)
{
    (void)target_count;
    (void)current_count;
    (void)minimum_count;
    (void)maximum_count;
    (void)soft_limit_clamp_count;
    return 0U;
}

uint8_t BalancePositionControl_SetTrackingTarget(int32_t target_count)
{
    (void)target_count;
    return 0U;
}

void BalancePositionControl_Update20ms(int32_t current_count,
    uint32_t elapsed_ms, uint32_t soft_limit_clamp_count)
{
    (void)current_count;
    (void)elapsed_ms;
    (void)soft_limit_clamp_count;
}

void BalancePositionControl_Cancel(void)
{
}

uint8_t BalancePositionControl_ResetFault(void)
{
    return 0U;
}

uint8_t BalancePositionControl_IsBusy(void)
{
    return 0U;
}

uint8_t BalancePositionControl_HasFault(void)
{
    return 0U;
}

void BalancePositionControl_GetSnapshot(BalancePositionRuntime *snapshot)
{
    if (snapshot != (BalancePositionRuntime *)0) {
        *snapshot = g_runtime;
    }
}

#endif
