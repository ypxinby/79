#include "balance_soft_limits.h"

#include <limits.h>

#include "app_features.h"
#include "balance_encoder.h"
#include "gimbal_stepper.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_SOFT_LIMITS

static volatile BalanceSoftLimitsRuntime g_runtime;

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

static int32_t logical_position_from_raw(int32_t raw_count)
{
    return clamp_i64_to_i32((int64_t)raw_count -
        (int64_t)g_runtime.zero_raw_count);
}

void BalanceSoftLimits_Init(void)
{
    g_runtime.zero_raw_count = 0;
    g_runtime.current_logical_count = 0;
    g_runtime.low_logical_count = 0;
    g_runtime.high_logical_count = 0;
    g_runtime.minimum_logical_count = 0;
    g_runtime.maximum_logical_count = 0;
    g_runtime.clamp_count = 0U;
    g_runtime.last_clamp_direction = 0;
    g_runtime.zero_valid = 0U;
    g_runtime.low_valid = 0U;
    g_runtime.high_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.calibration_active = 0U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
}

uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void)
{
    uint32_t primask;
    int32_t raw_count;

    if (GimbalStepper_GetFeedback()->running != 0U) {
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RUNNING;
        return 0U;
    }
    if (GimbalStepper_ConfirmZero() == 0U) {
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RUNNING;
        return 0U;
    }

    raw_count = BalanceEncoder_GetCountAtomic();
    primask = __get_PRIMASK();
    __disable_irq();
    g_runtime.zero_raw_count = raw_count;
    g_runtime.current_logical_count = 0;
    g_runtime.low_logical_count = 0;
    g_runtime.high_logical_count = 0;
    g_runtime.minimum_logical_count = 0;
    g_runtime.maximum_logical_count = 0;
    g_runtime.clamp_count = 0U;
    g_runtime.last_clamp_direction = 0;
    g_runtime.zero_valid = 1U;
    g_runtime.low_valid = 0U;
    g_runtime.high_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.calibration_active = 1U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_LOW;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

uint8_t BalanceSoftLimits_CaptureCurrentStage(void)
{
    uint32_t primask;
    int32_t logical_count;
    int32_t low_count;
    int32_t high_count;
    int32_t minimum_count;
    int32_t maximum_count;
    int64_t span;

    if (g_runtime.calibration_active == 0U) {
        return 0U;
    }
    if (GimbalStepper_GetFeedback()->running != 0U) {
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RUNNING;
        return 0U;
    }

    if (g_runtime.calibration_stage == BALANCE_SOFT_LIMIT_CAL_COMPLETE) {
        g_runtime.calibration_active = 0U;
        g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
        return 1U;
    }

    logical_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    if (g_runtime.calibration_stage == BALANCE_SOFT_LIMIT_CAL_LOW) {
        primask = __get_PRIMASK();
        __disable_irq();
        g_runtime.low_logical_count = logical_count;
        g_runtime.low_valid = 1U;
        g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_HIGH;
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
        if (primask == 0U) {
            __enable_irq();
        }
        return 1U;
    }

    if (g_runtime.calibration_stage != BALANCE_SOFT_LIMIT_CAL_HIGH) {
        return 0U;
    }

    low_count = g_runtime.low_logical_count;
    high_count = logical_count;
    minimum_count = (low_count < high_count) ? low_count : high_count;
    maximum_count = (low_count > high_count) ? low_count : high_count;
    span = (int64_t)maximum_count - (int64_t)minimum_count;
    if ((minimum_count >= 0) || (maximum_count <= 0) ||
        (span < BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS)) {
        g_runtime.high_logical_count = high_count;
        g_runtime.high_valid = 0U;
        g_runtime.limits_valid = 0U;
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RANGE;
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_runtime.high_logical_count = high_count;
    g_runtime.minimum_logical_count = minimum_count;
    g_runtime.maximum_logical_count = maximum_count;
    g_runtime.high_valid = 1U;
    g_runtime.limits_valid = 1U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_COMPLETE;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void BalanceSoftLimits_AbortCalibration(void)
{
    g_runtime.calibration_active = 0U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
}

uint8_t BalanceSoftLimits_IsCalibrationActive(void)
{
    return g_runtime.calibration_active;
}

void BalanceSoftLimits_Enforce100usFromIsr(void)
{
    const GimbalStepperFeedback *stepper;
    int32_t logical_count;
    int8_t rejected_direction = 0;

    stepper = GimbalStepper_GetFeedback();
    if (stepper->running == 0U) {
        return;
    }

    /* Outside calibration, an uncalibrated axis must fail closed. This keeps
     * future application commands from moving the mechanism after power-up. */
    if ((g_runtime.limits_valid == 0U) ||
        (g_runtime.zero_valid == 0U)) {
        if (g_runtime.calibration_active != 0U) {
            return;
        }
        rejected_direction = stepper->direction;
    } else {
        logical_count = logical_position_from_raw(
            BalanceEncoder_GetCountAtomic());
        if ((stepper->direction > 0) &&
            (logical_count >= g_runtime.maximum_logical_count)) {
            rejected_direction = 1;
        } else if ((stepper->direction < 0) &&
            (logical_count <= g_runtime.minimum_logical_count)) {
            rejected_direction = -1;
        }
    }

    if (rejected_direction != 0) {
        GimbalStepper_StopHold();
        if (g_runtime.clamp_count != UINT32_MAX) {
            g_runtime.clamp_count++;
        }
        g_runtime.last_clamp_direction = rejected_direction;
    }
}

void BalanceSoftLimits_GetSnapshot(BalanceSoftLimitsRuntime *snapshot)
{
    uint32_t primask;
    int32_t raw_count;

    if (snapshot == (BalanceSoftLimitsRuntime *)0) {
        return;
    }

    raw_count = BalanceEncoder_GetCountAtomic();
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->zero_raw_count = g_runtime.zero_raw_count;
    snapshot->current_logical_count = (g_runtime.zero_valid != 0U) ?
        logical_position_from_raw(raw_count) : 0;
    snapshot->low_logical_count = g_runtime.low_logical_count;
    snapshot->high_logical_count = g_runtime.high_logical_count;
    snapshot->minimum_logical_count = g_runtime.minimum_logical_count;
    snapshot->maximum_logical_count = g_runtime.maximum_logical_count;
    snapshot->clamp_count = g_runtime.clamp_count;
    snapshot->last_clamp_direction = g_runtime.last_clamp_direction;
    snapshot->zero_valid = g_runtime.zero_valid;
    snapshot->low_valid = g_runtime.low_valid;
    snapshot->high_valid = g_runtime.high_valid;
    snapshot->limits_valid = g_runtime.limits_valid;
    snapshot->calibration_active = g_runtime.calibration_active;
    snapshot->calibration_stage = g_runtime.calibration_stage;
    snapshot->error = g_runtime.error;
    if (primask == 0U) {
        __enable_irq();
    }
}

#else

static BalanceSoftLimitsRuntime g_runtime;

void BalanceSoftLimits_Init(void)
{
    g_runtime = (BalanceSoftLimitsRuntime){0};
}

uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void)
{
    return 0U;
}

uint8_t BalanceSoftLimits_CaptureCurrentStage(void)
{
    return 0U;
}

void BalanceSoftLimits_AbortCalibration(void)
{
}

uint8_t BalanceSoftLimits_IsCalibrationActive(void)
{
    return 0U;
}

void BalanceSoftLimits_Enforce100usFromIsr(void)
{
}

void BalanceSoftLimits_GetSnapshot(BalanceSoftLimitsRuntime *snapshot)
{
    if (snapshot != (BalanceSoftLimitsRuntime *)0) {
        *snapshot = g_runtime;
    }
}

#endif
