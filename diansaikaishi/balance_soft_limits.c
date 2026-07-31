#include "balance_soft_limits.h"

#include <limits.h>

#include "app_features.h"
#include "balance_encoder.h"
#include "balance_position_control.h"
#include "gimbal_stepper.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_SOFT_LIMITS

static volatile BalanceSoftLimitsRuntime g_runtime;
static BalanceCalibrationStoredLimits g_storedLimits;
static uint8_t g_storedLimitsValid;

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

static void restore_stored_limit_display(void)
{
    if (g_storedLimitsValid != 0U) {
        g_runtime.low_logical_count = g_storedLimits.low_offset_count;
        g_runtime.high_logical_count = g_storedLimits.high_offset_count;
        g_runtime.minimum_logical_count = g_storedLimits.low_offset_count;
        g_runtime.maximum_logical_count = g_storedLimits.high_offset_count;
        g_runtime.low_valid = 1U;
        g_runtime.high_valid = 1U;
        g_runtime.flash_sequence = g_storedLimits.sequence;
    } else {
        g_runtime.low_logical_count = 0;
        g_runtime.high_logical_count = 0;
        g_runtime.minimum_logical_count = 0;
        g_runtime.maximum_logical_count = 0;
        g_runtime.low_valid = 0U;
        g_runtime.high_valid = 0U;
        g_runtime.flash_sequence = 0U;
    }
    g_runtime.flash_record_valid = g_storedLimitsValid;
}

static void clear_test_runtime(void)
{
    g_runtime.test_active = 0U;
    g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_IDLE;
    g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_NONE;
    g_runtime.test_elapsed_ms = 0U;
    g_runtime.test_timeout_ms = 0U;
    g_runtime.test_target_logical_count = 0;
    g_runtime.test_position_error_count = 0;
}

static void clear_oscillation_runtime(void)
{
    g_runtime.oscillation_active = 0U;
    g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_IDLE;
    g_runtime.oscillation_error = BALANCE_SOFT_LIMIT_TEST_ERROR_NONE;
    g_runtime.oscillation_phase_ms = 0U;
    g_runtime.oscillation_cycle_count = 0U;
    g_runtime.oscillation_low_logical_count = 0;
    g_runtime.oscillation_high_logical_count = 0;
    g_runtime.oscillation_target_logical_count = 0;
    g_runtime.oscillation_max_error_count = 0;
}

void BalanceSoftLimits_Init(void)
{
    BalanceCalibrationFlashStatus flash_status;

    g_runtime = (BalanceSoftLimitsRuntime){0};
    g_storedLimits = (BalanceCalibrationStoredLimits){0};
    flash_status = BalanceCalibrationStore_Load(&g_storedLimits);
    g_storedLimitsValid =
        (flash_status == BALANCE_CALIBRATION_FLASH_VALID) ? 1U : 0U;
    g_runtime.flash_status = flash_status;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    restore_stored_limit_display();
    clear_test_runtime();
    clear_oscillation_runtime();
    BalancePositionControl_Init();
}

uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void)
{
    uint32_t primask;
    int32_t raw_count;
    uint8_t start_full_calibration;

    if ((GimbalStepper_GetFeedback()->running != 0U) ||
        (BalancePositionControl_HasFault() != 0U)) {
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RUNNING;
        return 0U;
    }
    if (GimbalStepper_ConfirmZero() == 0U) {
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_RUNNING;
        return 0U;
    }

    raw_count = BalanceEncoder_GetCountAtomic();
    start_full_calibration = (uint8_t)(
        (g_runtime.recalibration_armed != 0U) ||
        (g_storedLimitsValid == 0U));

    primask = __get_PRIMASK();
    __disable_irq();
    g_runtime.zero_raw_count = raw_count;
    g_runtime.current_logical_count = 0;
    g_runtime.clamp_count = 0U;
    g_runtime.last_clamp_direction = 0;
    g_runtime.zero_valid = 1U;
    g_runtime.zero_confirmation_required = 0U;
    g_runtime.recalibration_armed = 0U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    clear_test_runtime();
    clear_oscillation_runtime();

    if (start_full_calibration != 0U) {
        g_runtime.low_logical_count = 0;
        g_runtime.high_logical_count = 0;
        g_runtime.minimum_logical_count = 0;
        g_runtime.maximum_logical_count = 0;
        g_runtime.low_valid = 0U;
        g_runtime.high_valid = 0U;
        g_runtime.limits_valid = 0U;
        g_runtime.calibration_active = 1U;
        g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_LOW;
    } else {
        restore_stored_limit_display();
        g_runtime.limits_valid = 1U;
        g_runtime.calibration_active = 0U;
        g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    GimbalStepper_StopHold();
    BalancePositionControl_Cancel();
    return 1U;
}

uint8_t BalanceSoftLimits_CaptureCurrentStage(void)
{
    BalanceCalibrationFlashStatus flash_status;
    BalanceCalibrationStoredLimits saved_limits;
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
        GimbalStepper_StopHold();
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
    g_runtime.low_logical_count = minimum_count;
    g_runtime.high_logical_count = maximum_count;
    g_runtime.minimum_logical_count = minimum_count;
    g_runtime.maximum_logical_count = maximum_count;
    g_runtime.high_valid = 1U;
    g_runtime.limits_valid = 1U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_COMPLETE;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    if (primask == 0U) {
        __enable_irq();
    }

    flash_status = BalanceCalibrationStore_Save(minimum_count,
        maximum_count, &saved_limits);
    g_runtime.flash_status = flash_status;
    if (flash_status == BALANCE_CALIBRATION_FLASH_VALID) {
        g_storedLimits = saved_limits;
        g_storedLimitsValid = 1U;
        g_runtime.flash_record_valid = 1U;
        g_runtime.flash_sequence = saved_limits.sequence;
    } else {
        g_storedLimitsValid = 0U;
        g_runtime.flash_record_valid = 0U;
        g_runtime.flash_sequence = 0U;
        g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_FLASH;
    }
    return 1U;
}

void BalanceSoftLimits_AbortCalibration(void)
{
    BalancePositionControl_Cancel();
    GimbalStepper_Release();
    g_runtime.zero_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.calibration_active = 0U;
    g_runtime.calibration_stage = BALANCE_SOFT_LIMIT_CAL_IDLE;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.recalibration_armed = 0U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    restore_stored_limit_display();
    clear_test_runtime();
    clear_oscillation_runtime();
}

uint8_t BalanceSoftLimits_IsCalibrationActive(void)
{
    return g_runtime.calibration_active;
}

uint8_t BalanceSoftLimits_ArmRecalibration(void)
{
    if ((GimbalStepper_GetFeedback()->running != 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.test_active != 0U) ||
        (g_runtime.oscillation_active != 0U) ||
        (BalancePositionControl_HasFault() != 0U) ||
        (BalancePositionControl_IsBusy() != 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.zero_valid == 0U)) {
        return 0U;
    }

    BalancePositionControl_Cancel();
    GimbalStepper_Release();
    g_runtime.zero_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.recalibration_armed = 1U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    clear_test_runtime();
    clear_oscillation_runtime();
    return 1U;
}

void BalanceSoftLimits_CancelRecalibration(void)
{
    BalancePositionControl_Cancel();
    GimbalStepper_Release();
    g_runtime.zero_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.recalibration_armed = 0U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    restore_stored_limit_display();
    clear_test_runtime();
    clear_oscillation_runtime();
}

static uint8_t test_start_target(BalanceSoftLimitTestStage stage,
    int32_t target_logical_count)
{
    int32_t current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    BalancePositionRuntime position;

    g_runtime.test_stage = stage;
    g_runtime.test_target_logical_count = target_logical_count;
    g_runtime.test_position_error_count =
        clamp_i64_to_i32((int64_t)target_logical_count - current_count);
    g_runtime.test_elapsed_ms = 0U;
    if (BalancePositionControl_Start(target_logical_count, current_count,
            g_runtime.minimum_logical_count,
            g_runtime.maximum_logical_count,
            g_runtime.clamp_count) == 0U) {
        return 0U;
    }
    BalancePositionControl_GetSnapshot(&position);
    g_runtime.test_timeout_ms = position.timeout_ms;
    return 1U;
}

static void test_fail(BalanceSoftLimitTestError error)
{
    BalancePositionControl_Cancel();
    g_runtime.test_active = 0U;
    g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_ERROR;
    g_runtime.test_error = error;
}

uint8_t BalanceSoftLimits_StartTravelTest(void)
{
    int64_t span;
    int64_t inset;
    int32_t test_low;
    int32_t test_high;

    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.recalibration_armed != 0U) ||
        (g_runtime.oscillation_active != 0U) ||
        (BalancePositionControl_HasFault() != 0U) ||
        (BalancePositionControl_IsBusy() != 0U) ||
        (GimbalStepper_GetFeedback()->running != 0U)) {
        g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_ERROR;
        g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_NOT_READY;
        return 0U;
    }

    span = (int64_t)g_runtime.maximum_logical_count -
        (int64_t)g_runtime.minimum_logical_count;
    inset = span * BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT / 100;
    if (inset < BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS) {
        inset = BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS;
    }
    test_low = clamp_i64_to_i32(
        (int64_t)g_runtime.minimum_logical_count + inset);
    test_high = clamp_i64_to_i32(
        (int64_t)g_runtime.maximum_logical_count - inset);
    if ((test_low >= 0) || (test_high <= 0) ||
        (test_low >= test_high)) {
        g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_ERROR;
        g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE;
        return 0U;
    }

    g_runtime.test_active = 1U;
    g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_NONE;
    if (test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_LOW,
            test_low) == 0U) {
        test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE);
        return 0U;
    }
    return 1U;
}

void BalanceSoftLimits_CancelTravelTest(void)
{
    if (g_runtime.test_active != 0U) {
        test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_CANCELLED);
    }
}

uint8_t BalanceSoftLimits_IsTravelTestActive(void)
{
    return g_runtime.test_active;
}

static uint32_t magnitude_i32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }
    return (uint32_t)(-(int64_t)value);
}

static int32_t oscillation_target_for_phase(uint32_t phase_ms)
{
    uint32_t half_period_ms = BALANCE_OSCILLATION_PERIOD_MS / 2U;
    uint32_t progress_x1000;
    uint64_t progress_squared;
    uint64_t smooth_x1000;
    int64_t span =
        (int64_t)g_runtime.oscillation_high_logical_count -
        (int64_t)g_runtime.oscillation_low_logical_count;

    if (phase_ms <= half_period_ms) {
        progress_x1000 = (phase_ms * 1000U) / half_period_ms;
    } else {
        progress_x1000 =
            ((BALANCE_OSCILLATION_PERIOD_MS - phase_ms) * 1000U) /
            half_period_ms;
    }
    progress_squared =
        (uint64_t)progress_x1000 * progress_x1000;
    smooth_x1000 = progress_squared *
        (3000U - 2U * progress_x1000) / 1000000U;
    return clamp_i64_to_i32(
        (int64_t)g_runtime.oscillation_low_logical_count +
        span * (int64_t)smooth_x1000 / 1000);
}

static void oscillation_fail(BalanceSoftLimitTestError error)
{
    BalancePositionControl_Cancel();
    g_runtime.oscillation_active = 0U;
    g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_ERROR;
    g_runtime.oscillation_error = error;
}

uint8_t BalanceSoftLimits_StartOscillationTest(void)
{
#if FEATURE_BALANCE_OSCILLATION_TEST
    int32_t current_count;
    int32_t safe_minimum;
    int32_t safe_maximum;
    int32_t test_low;
    int32_t test_high;

    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.recalibration_armed != 0U) ||
        (g_runtime.test_active != 0U) ||
        (g_runtime.oscillation_active != 0U) ||
        (BalancePositionControl_HasFault() != 0U) ||
        (BalancePositionControl_IsBusy() != 0U) ||
        (GimbalStepper_GetFeedback()->running != 0U)) {
        g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_ERROR;
        g_runtime.oscillation_error =
            BALANCE_SOFT_LIMIT_TEST_ERROR_NOT_READY;
        return 0U;
    }

    safe_minimum = clamp_i64_to_i32(
        (int64_t)g_runtime.minimum_logical_count +
            BALANCE_POSITION_LIMIT_MARGIN_COUNTS);
    safe_maximum = clamp_i64_to_i32(
        (int64_t)g_runtime.maximum_logical_count -
            BALANCE_POSITION_LIMIT_MARGIN_COUNTS);
    test_low = clamp_i64_to_i32(
        (int64_t)g_runtime.minimum_logical_count *
            BALANCE_OSCILLATION_RANGE_PERCENT / 100);
    test_high = clamp_i64_to_i32(
        (int64_t)g_runtime.maximum_logical_count *
            BALANCE_OSCILLATION_RANGE_PERCENT / 100);
    if (test_low < safe_minimum) {
        test_low = safe_minimum;
    }
    if (test_high > safe_maximum) {
        test_high = safe_maximum;
    }
    if ((test_low >= 0) || (test_high <= 0) ||
        (test_low >= test_high)) {
        g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_ERROR;
        g_runtime.oscillation_error = BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE;
        return 0U;
    }

    clear_oscillation_runtime();
    g_runtime.oscillation_active = 1U;
    g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_TO_START;
    g_runtime.oscillation_low_logical_count = test_low;
    g_runtime.oscillation_high_logical_count = test_high;
    g_runtime.oscillation_target_logical_count = test_low;
    current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    if (BalancePositionControl_Start(test_low, current_count,
            g_runtime.minimum_logical_count,
            g_runtime.maximum_logical_count,
            g_runtime.clamp_count) == 0U) {
        oscillation_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_CONTROL);
        return 0U;
    }
    return 1U;
#else
    return 0U;
#endif
}

void BalanceSoftLimits_CancelOscillationTest(void)
{
    if (g_runtime.oscillation_active != 0U) {
        BalancePositionControl_Cancel();
        g_runtime.oscillation_active = 0U;
        g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_IDLE;
        g_runtime.oscillation_error =
            BALANCE_SOFT_LIMIT_TEST_ERROR_CANCELLED;
    }
}

uint8_t BalanceSoftLimits_IsOscillationTestActive(void)
{
    return g_runtime.oscillation_active;
}

uint8_t BalanceSoftLimits_StartRelativePositionMoveSteps(
    int32_t delta_steps)
{
    int32_t current_count;
    int64_t scaled_delta;
    int32_t delta_count;
    int32_t target_count;
    BalancePositionRuntime position;

    BalancePositionControl_GetSnapshot(&position);
    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.test_active != 0U) ||
        (g_runtime.oscillation_active != 0U) ||
        (g_runtime.recalibration_armed != 0U) ||
        (position.fault != BALANCE_POSITION_FAULT_NONE) ||
        (position.busy != 0U) ||
        (position.tracking_enabled != 0U)) {
        return 0U;
    }

    scaled_delta = (int64_t)delta_steps *
        BALANCE_ENCODER_COUNTS_PER_REV;
    if (scaled_delta > 0) {
        scaled_delta += BALANCE_STEPPER_COMMAND_STEPS_PER_REV / 2;
    } else if (scaled_delta < 0) {
        scaled_delta -= BALANCE_STEPPER_COMMAND_STEPS_PER_REV / 2;
    }
    delta_count = clamp_i64_to_i32(scaled_delta /
        BALANCE_STEPPER_COMMAND_STEPS_PER_REV);
    current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    target_count = clamp_i64_to_i32(
        (int64_t)current_count + delta_count);
    return BalanceSoftLimits_StartPositionMoveToLogicalCount(
        target_count);
}

uint8_t BalanceSoftLimits_StartPositionMoveToLogicalCount(
    int32_t target_count)
{
    int32_t current_count;
    int32_t safe_minimum;
    int32_t safe_maximum;
    BalancePositionRuntime position;

    BalancePositionControl_GetSnapshot(&position);
    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.test_active != 0U) ||
        (g_runtime.oscillation_active != 0U) ||
        (g_runtime.recalibration_armed != 0U) ||
        (position.fault != BALANCE_POSITION_FAULT_NONE) ||
        (position.busy != 0U) ||
        (position.tracking_enabled != 0U)) {
        return 0U;
    }

    current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    safe_minimum = clamp_i64_to_i32(
        (int64_t)g_runtime.minimum_logical_count +
            BALANCE_POSITION_LIMIT_MARGIN_COUNTS);
    safe_maximum = clamp_i64_to_i32(
        (int64_t)g_runtime.maximum_logical_count -
            BALANCE_POSITION_LIMIT_MARGIN_COUNTS);
    if (target_count < safe_minimum) {
        target_count = safe_minimum;
    }
    if (target_count > safe_maximum) {
        target_count = safe_maximum;
    }
    return BalancePositionControl_Start(target_count, current_count,
        g_runtime.minimum_logical_count,
        g_runtime.maximum_logical_count, g_runtime.clamp_count);
}

void BalanceSoftLimits_CancelPositionMove(void)
{
    BalancePositionControl_Cancel();
}

uint8_t BalanceSoftLimits_IsPositionMoveActive(void)
{
    return BalancePositionControl_IsBusy();
}

uint8_t BalanceSoftLimits_ResetPositionFault(void)
{
    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U)) {
        return 0U;
    }
    if (BalancePositionControl_ResetFault() == 0U) {
        return 0U;
    }
    clear_test_runtime();
    clear_oscillation_runtime();
    return 1U;
}

void BalanceSoftLimits_StopMotion(void)
{
    if (g_runtime.test_active != 0U) {
        g_runtime.test_active = 0U;
        g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_ERROR;
        g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_CANCELLED;
    }
    if (g_runtime.oscillation_active != 0U) {
        g_runtime.oscillation_active = 0U;
        g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_ERROR;
        g_runtime.oscillation_error =
            BALANCE_SOFT_LIMIT_TEST_ERROR_CANCELLED;
    }
    BalancePositionControl_Cancel();
}

void BalanceSoftLimits_Update20ms(uint32_t elapsed_ms)
{
    int32_t current_count;
    int64_t span;
    int64_t inset;
    BalancePositionRuntime position;

    if ((g_runtime.zero_valid == 0U) ||
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.calibration_active != 0U) ||
        (g_runtime.recalibration_armed != 0U)) {
        if (BalancePositionControl_IsBusy() != 0U) {
            BalancePositionControl_Cancel();
        }
        return;
    }

    current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    BalancePositionControl_Update20ms(current_count, elapsed_ms,
        g_runtime.clamp_count);
    BalancePositionControl_GetSnapshot(&position);

    if (g_runtime.oscillation_active != 0U) {
        uint32_t error_magnitude = magnitude_i32(
            position.position_error_count);

        if (error_magnitude >
            (uint32_t)g_runtime.oscillation_max_error_count) {
            g_runtime.oscillation_max_error_count =
                (error_magnitude > INT32_MAX) ? INT32_MAX :
                    (int32_t)error_magnitude;
        }
        if (position.fault != BALANCE_POSITION_FAULT_NONE) {
            g_runtime.oscillation_active = 0U;
            g_runtime.oscillation_stage = BALANCE_OSCILLATION_STAGE_ERROR;
            g_runtime.oscillation_error =
                BALANCE_SOFT_LIMIT_TEST_ERROR_CONTROL;
            return;
        }

        if (g_runtime.oscillation_stage ==
            BALANCE_OSCILLATION_STAGE_TO_START) {
            if ((position.busy == 0U) &&
                (position.target_reached != 0U)) {
                if (BalancePositionControl_StartTracking(
                        g_runtime.oscillation_low_logical_count,
                        current_count,
                        g_runtime.minimum_logical_count,
                        g_runtime.maximum_logical_count,
                        g_runtime.clamp_count) == 0U) {
                    oscillation_fail(
                        BALANCE_SOFT_LIMIT_TEST_ERROR_CONTROL);
                    return;
                }
                g_runtime.oscillation_stage =
                    BALANCE_OSCILLATION_STAGE_RUNNING;
                g_runtime.oscillation_phase_ms = 0U;
                g_runtime.oscillation_cycle_count = 0U;
                g_runtime.oscillation_max_error_count = 0;
            }
            return;
        }

        if (g_runtime.oscillation_stage ==
            BALANCE_OSCILLATION_STAGE_RUNNING) {
            g_runtime.oscillation_phase_ms += elapsed_ms;
            while (g_runtime.oscillation_phase_ms >=
                BALANCE_OSCILLATION_PERIOD_MS) {
                g_runtime.oscillation_phase_ms -=
                    BALANCE_OSCILLATION_PERIOD_MS;
                if (g_runtime.oscillation_cycle_count != UINT32_MAX) {
                    g_runtime.oscillation_cycle_count++;
                }
            }
            g_runtime.oscillation_target_logical_count =
                oscillation_target_for_phase(
                    g_runtime.oscillation_phase_ms);
            if (BalancePositionControl_SetTrackingTarget(
                    g_runtime.oscillation_target_logical_count) == 0U) {
                oscillation_fail(
                    BALANCE_SOFT_LIMIT_TEST_ERROR_CONTROL);
            }
            return;
        }

        oscillation_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_POSITION);
        return;
    }

    if (g_runtime.test_active == 0U) {
        return;
    }

    g_runtime.test_elapsed_ms = position.elapsed_ms;
    g_runtime.test_timeout_ms = position.timeout_ms;
    g_runtime.test_position_error_count =
        position.position_error_count;
    if (position.fault != BALANCE_POSITION_FAULT_NONE) {
        g_runtime.test_active = 0U;
        g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_ERROR;
        if (position.fault == BALANCE_POSITION_FAULT_TIMEOUT) {
            g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_TIMEOUT;
        } else if (position.fault ==
            BALANCE_POSITION_FAULT_SOFT_LIMIT) {
            g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_CLAMPED;
        } else {
            g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_CONTROL;
        }
        return;
    }
    if ((position.busy != 0U) || (position.target_reached == 0U)) {
        return;
    }

    span = (int64_t)g_runtime.maximum_logical_count -
        (int64_t)g_runtime.minimum_logical_count;
    inset = (span * BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT) / 100;
    if (inset < BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS) {
        inset = BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS;
    }

    switch (g_runtime.test_stage) {
        case BALANCE_SOFT_LIMIT_TEST_TO_LOW:
            if (test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_HIGH,
                clamp_i64_to_i32(
                    (int64_t)g_runtime.maximum_logical_count - inset)) ==
                    0U) {
                test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE);
            }
            break;
        case BALANCE_SOFT_LIMIT_TEST_TO_HIGH:
            if (test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_ZERO,
                    0) == 0U) {
                test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE);
            }
            break;
        case BALANCE_SOFT_LIMIT_TEST_TO_ZERO:
            g_runtime.test_active = 0U;
            g_runtime.test_stage = BALANCE_SOFT_LIMIT_TEST_DONE;
            g_runtime.test_error = BALANCE_SOFT_LIMIT_TEST_ERROR_NONE;
            break;
        default:
            test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_POSITION);
            break;
    }
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

    /* Outside calibration, an uncalibrated axis must fail closed. */
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
    *snapshot = g_runtime;
    snapshot->current_logical_count = (g_runtime.zero_valid != 0U) ?
        logical_position_from_raw(raw_count) : 0;
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

uint8_t BalanceSoftLimits_ArmRecalibration(void)
{
    return 0U;
}

void BalanceSoftLimits_CancelRecalibration(void)
{
}

uint8_t BalanceSoftLimits_StartTravelTest(void)
{
    return 0U;
}

void BalanceSoftLimits_CancelTravelTest(void)
{
}

uint8_t BalanceSoftLimits_IsTravelTestActive(void)
{
    return 0U;
}

uint8_t BalanceSoftLimits_StartOscillationTest(void)
{
    return 0U;
}

void BalanceSoftLimits_CancelOscillationTest(void)
{
}

uint8_t BalanceSoftLimits_IsOscillationTestActive(void)
{
    return 0U;
}

uint8_t BalanceSoftLimits_StartRelativePositionMoveSteps(
    int32_t delta_steps)
{
    (void)delta_steps;
    return 0U;
}

void BalanceSoftLimits_CancelPositionMove(void)
{
}

uint8_t BalanceSoftLimits_IsPositionMoveActive(void)
{
    return 0U;
}

uint8_t BalanceSoftLimits_ResetPositionFault(void)
{
    return 0U;
}

void BalanceSoftLimits_StopMotion(void)
{
}

void BalanceSoftLimits_Update20ms(uint32_t elapsed_ms)
{
    (void)elapsed_ms;
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
