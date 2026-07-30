#include "balance_soft_limits.h"

#include <limits.h>

#include "app_features.h"
#include "balance_encoder.h"
#include "gimbal_stepper.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_SOFT_LIMITS

static volatile BalanceSoftLimitsRuntime g_runtime;
static BalanceCalibrationStoredLimits g_storedLimits;
static uint8_t g_storedLimitsValid;
static uint32_t g_testStartClampCount;

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
    g_testStartClampCount = g_runtime.clamp_count;
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
}

uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void)
{
    uint32_t primask;
    int32_t raw_count;
    uint8_t start_full_calibration;

    if (GimbalStepper_GetFeedback()->running != 0U) {
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
        (g_runtime.limits_valid == 0U) ||
        (g_runtime.zero_valid == 0U)) {
        return 0U;
    }

    GimbalStepper_Release();
    g_runtime.zero_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.recalibration_armed = 1U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    clear_test_runtime();
    return 1U;
}

void BalanceSoftLimits_CancelRecalibration(void)
{
    GimbalStepper_Release();
    g_runtime.zero_valid = 0U;
    g_runtime.limits_valid = 0U;
    g_runtime.zero_confirmation_required = 1U;
    g_runtime.recalibration_armed = 0U;
    g_runtime.error = BALANCE_SOFT_LIMIT_ERROR_NONE;
    restore_stored_limit_display();
    clear_test_runtime();
}

static int32_t test_count_delta_to_steps(int32_t delta_count)
{
    int64_t scaled = (int64_t)delta_count *
        BALANCE_STEPPER_COMMAND_STEPS_PER_REV;
    int32_t steps;

    if (scaled > 0) {
        scaled += BALANCE_ENCODER_COUNTS_PER_REV / 2;
    } else if (scaled < 0) {
        scaled -= BALANCE_ENCODER_COUNTS_PER_REV / 2;
    }
    steps = clamp_i64_to_i32(scaled /
        BALANCE_ENCODER_COUNTS_PER_REV);
    if ((steps == 0) && (delta_count != 0)) {
        steps = (delta_count > 0) ? 1 : -1;
    }
    return steps;
}

static uint32_t test_calculate_timeout_ms(int32_t delta_steps)
{
    int64_t magnitude = delta_steps;
    uint64_t expected_ms;

    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    expected_ms = ((uint64_t)magnitude * 2U *
        BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS + 9U) / 10U;
    expected_ms += BALANCE_SOFT_LIMIT_TEST_TIMEOUT_MARGIN_MS;
    if (expected_ms < BALANCE_SOFT_LIMIT_TEST_TIMEOUT_MIN_MS) {
        expected_ms = BALANCE_SOFT_LIMIT_TEST_TIMEOUT_MIN_MS;
    }
    if (expected_ms > BALANCE_SOFT_LIMIT_TEST_TIMEOUT_MAX_MS) {
        expected_ms = BALANCE_SOFT_LIMIT_TEST_TIMEOUT_MAX_MS;
    }
    return (uint32_t)expected_ms;
}

static void test_start_target(BalanceSoftLimitTestStage stage,
    int32_t target_logical_count)
{
    int32_t current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    int32_t delta_steps = test_count_delta_to_steps(
        clamp_i64_to_i32((int64_t)target_logical_count - current_count));

    g_runtime.test_stage = stage;
    g_runtime.test_target_logical_count = target_logical_count;
    g_runtime.test_position_error_count =
        clamp_i64_to_i32((int64_t)target_logical_count - current_count);
    g_runtime.test_elapsed_ms = 0U;
    g_runtime.test_timeout_ms = test_calculate_timeout_ms(delta_steps);
    GimbalStepper_SetStepHalfPeriodTicks(
        BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
    GimbalStepper_MoveRelativeSteps(delta_steps);
}

static void test_fail(BalanceSoftLimitTestError error)
{
    GimbalStepper_StopHold();
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
    g_testStartClampCount = g_runtime.clamp_count;
    test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_LOW, test_low);
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

void BalanceSoftLimits_Update20ms(uint32_t elapsed_ms)
{
    int32_t current_count;
    int64_t position_error;
    int64_t span;
    int64_t inset;

    if (g_runtime.test_active == 0U) {
        return;
    }

    g_runtime.test_elapsed_ms = add_u32_saturating(
        g_runtime.test_elapsed_ms, elapsed_ms);
    if (g_runtime.clamp_count != g_testStartClampCount) {
        test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_CLAMPED);
        return;
    }
    if (g_runtime.test_elapsed_ms > g_runtime.test_timeout_ms) {
        test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_TIMEOUT);
        return;
    }

    current_count = logical_position_from_raw(
        BalanceEncoder_GetCountAtomic());
    position_error = (int64_t)g_runtime.test_target_logical_count -
        (int64_t)current_count;
    g_runtime.test_position_error_count =
        clamp_i64_to_i32(position_error);
    if (GimbalStepper_GetFeedback()->running != 0U) {
        return;
    }

    if (position_error < 0) {
        position_error = -position_error;
    }
    if (position_error > BALANCE_SOFT_LIMIT_TEST_TOLERANCE_COUNTS) {
        test_fail(BALANCE_SOFT_LIMIT_TEST_ERROR_POSITION);
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
            test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_HIGH,
                clamp_i64_to_i32(
                    (int64_t)g_runtime.maximum_logical_count - inset));
            break;
        case BALANCE_SOFT_LIMIT_TEST_TO_HIGH:
            test_start_target(BALANCE_SOFT_LIMIT_TEST_TO_ZERO, 0);
            break;
        case BALANCE_SOFT_LIMIT_TEST_TO_ZERO:
            GimbalStepper_StopHold();
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
