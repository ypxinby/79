#include "balance_encoder.h"

#include <limits.h>

#include "app_features.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_ENCODER_CAPTURE

/* Index is previous AB in bits 3:2 and current AB in bits 1:0. */
static const int8_t g_quadratureDelta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static volatile BalanceEncoderRuntime g_runtime;
static int32_t g_speedLastCount;

static void increment_u32_saturated(volatile uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

static uint8_t balance_encoder_read_ab(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_BALANCE_ENCODER_PORT,
        GPIO_BALANCE_ENCODER_ENCODER_A_PIN |
        GPIO_BALANCE_ENCODER_ENCODER_B_PIN);
    uint8_t phase_a =
        ((pins & GPIO_BALANCE_ENCODER_ENCODER_A_PIN) != 0U) ? 1U : 0U;
    uint8_t phase_b =
        ((pins & GPIO_BALANCE_ENCODER_ENCODER_B_PIN) != 0U) ? 1U : 0U;

    return (uint8_t)((phase_a << 1U) | phase_b);
}

static void balance_encoder_apply_delta(int8_t delta)
{
    int32_t count = g_runtime.count;

    delta = (int8_t)(delta * BALANCE_ENCODER_DIRECTION_SIGN);
    if (((delta > 0) && (count == INT32_MAX)) ||
        ((delta < 0) && (count == INT32_MIN))) {
        increment_u32_saturated(&g_runtime.overflow_count);
        return;
    }

    g_runtime.count = count + delta;
    g_runtime.direction = (delta > 0) ? 1 : -1;
    increment_u32_saturated(&g_runtime.valid_transition_count);
}

void BalanceEncoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t pins = GPIO_BALANCE_ENCODER_ENCODER_A_PIN |
        GPIO_BALANCE_ENCODER_ENCODER_B_PIN;

    __disable_irq();
    DL_GPIO_clearInterruptStatus(GPIO_BALANCE_ENCODER_PORT, pins);
    g_runtime.count = 0;
    g_runtime.valid_transition_count = 0U;
    g_runtime.invalid_transition_count = 0U;
    g_runtime.overflow_count = 0U;
    g_runtime.speed_sample_delta_count = 0;
    g_runtime.speed_sample_count = 0U;
    g_runtime.ab_state = balance_encoder_read_ab();
    g_runtime.direction = 0;
    g_runtime.initialized = 1U;
    g_speedLastCount = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void BalanceEncoder_Init(void)
{
    BalanceEncoder_Reset();
}

void BalanceEncoder_HandleGpioInterrupt(void)
{
    uint32_t pins = GPIO_BALANCE_ENCODER_ENCODER_A_PIN |
        GPIO_BALANCE_ENCODER_ENCODER_B_PIN;
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        GPIO_BALANCE_ENCODER_PORT, pins);

    if (status != 0U) {
        uint8_t previous = g_runtime.ab_state;
        uint8_t current = balance_encoder_read_ab();
        uint8_t changed = (uint8_t)(previous ^ current);

        if (changed == 3U) {
            increment_u32_saturated(&g_runtime.invalid_transition_count);
        } else if (changed != 0U) {
            int8_t delta = g_quadratureDelta[
                (uint8_t)((previous << 2U) | current)];

            if (delta != 0) {
                balance_encoder_apply_delta(delta);
            }
        }
        g_runtime.ab_state = current;
    }

    DL_GPIO_clearInterruptStatus(GPIO_BALANCE_ENCODER_PORT, status);
}

void BalanceEncoder_SampleSpeed10msFromIsr(void)
{
    int32_t current = g_runtime.count;

    g_runtime.speed_sample_delta_count = current - g_speedLastCount;
    g_speedLastCount = current;
    increment_u32_saturated(&g_runtime.speed_sample_count);
}

void BalanceEncoder_GetSnapshot(BalanceEncoderRuntime *snapshot)
{
    uint32_t primask;

    if (snapshot == (BalanceEncoderRuntime *)0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->count = g_runtime.count;
    snapshot->valid_transition_count = g_runtime.valid_transition_count;
    snapshot->invalid_transition_count = g_runtime.invalid_transition_count;
    snapshot->overflow_count = g_runtime.overflow_count;
    snapshot->speed_sample_delta_count =
        g_runtime.speed_sample_delta_count;
    snapshot->speed_sample_count = g_runtime.speed_sample_count;
    snapshot->ab_state = g_runtime.ab_state;
    snapshot->direction = g_runtime.direction;
    snapshot->initialized = g_runtime.initialized;
    if (primask == 0U) {
        __enable_irq();
    }
}

int32_t BalanceEncoder_CalculateSpeedRpmX10(int32_t sample_delta_count)
{
#if BALANCE_ENCODER_COUNTS_PER_REV > 0
    int64_t numerator = (int64_t)sample_delta_count * 600000LL;
    int64_t denominator =
        (int64_t)BALANCE_ENCODER_COUNTS_PER_REV *
        (int64_t)BALANCE_ENCODER_SPEED_SAMPLE_MS;
    int64_t rpm_x10;

    if (numerator >= 0) {
        rpm_x10 = (numerator + (denominator / 2LL)) / denominator;
    } else {
        rpm_x10 = -(((-numerator) + (denominator / 2LL)) / denominator);
    }

    if (rpm_x10 > INT32_MAX) {
        return INT32_MAX;
    }
    if (rpm_x10 < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)rpm_x10;
#else
    (void)sample_delta_count;
    return 0;
#endif
}

uint8_t BalanceEncoder_CalculateFollowError(int64_t step_count,
    int32_t encoder_count, int32_t *error_count)
{
    if (error_count == (int32_t *)0) {
        return 0U;
    }

#if BALANCE_ENCODER_COUNTS_PER_REV > 0
    {
        int64_t expected_count;
        int64_t error;

        expected_count =
            (step_count * (int64_t)BALANCE_ENCODER_COUNTS_PER_REV) /
            (int64_t)BALANCE_STEPPER_COMMAND_STEPS_PER_REV;
        error = expected_count - (int64_t)encoder_count;
        if (error > INT32_MAX) {
            error = INT32_MAX;
        } else if (error < INT32_MIN) {
            error = INT32_MIN;
        }
        *error_count = (int32_t)error;
        return 1U;
    }
#else
    (void)step_count;
    (void)encoder_count;
    *error_count = 0;
    return 0U;
#endif
}

#else

static BalanceEncoderRuntime g_runtime;

void BalanceEncoder_Init(void)
{
    g_runtime = (BalanceEncoderRuntime){0};
}

void BalanceEncoder_Reset(void)
{
    BalanceEncoder_Init();
}

void BalanceEncoder_HandleGpioInterrupt(void)
{
}

void BalanceEncoder_SampleSpeed10msFromIsr(void)
{
}

void BalanceEncoder_GetSnapshot(BalanceEncoderRuntime *snapshot)
{
    if (snapshot != (BalanceEncoderRuntime *)0) {
        *snapshot = g_runtime;
    }
}

int32_t BalanceEncoder_CalculateSpeedRpmX10(int32_t sample_delta_count)
{
    (void)sample_delta_count;
    return 0;
}

uint8_t BalanceEncoder_CalculateFollowError(int64_t step_count,
    int32_t encoder_count, int32_t *error_count)
{
    (void)step_count;
    (void)encoder_count;
    if (error_count != (int32_t *)0) {
        *error_count = 0;
    }
    return 0U;
}

#endif
