#include "encoder.h"

#include <limits.h>

#include "ti_msp_dl_config.h"

static volatile int32_t g_motorAPulses;
static volatile int32_t g_motorBPulses;
static volatile uint8_t g_overflowFlags;

static uint8_t pin_is_high(uint32_t pin)
{
    return (DL_GPIO_readPins(GPIO_ENCODERS_PORT, pin) != 0U) ? 1U : 0U;
}

static void add_signed_pulse(
    volatile int32_t *pulses, int32_t delta, uint8_t overflowMask)
{
    int32_t current = *pulses;

    if (((delta > 0) && (current == INT32_MAX)) ||
        ((delta < 0) && (current == INT32_MIN))) {
        g_overflowFlags |= overflowMask;
        return;
    }
    *pulses = current + delta;
}

void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_motorAPulses = 0;
    g_motorBPulses = 0;
    g_overflowFlags = ENCODER_READ_ERROR_NONE;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Encoder_HandleGpioInterrupt(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODERS_PORT,
        GPIO_ENCODERS_MOTOR_A_ENCA_PIN | GPIO_ENCODERS_MOTOR_B_ENCA_PIN);

    if ((status & GPIO_ENCODERS_MOTOR_A_ENCA_PIN) != 0U) {
        uint8_t phaseA = pin_is_high(GPIO_ENCODERS_MOTOR_A_ENCA_PIN);
        uint8_t phaseB = pin_is_high(GPIO_ENCODERS_MOTOR_A_ENCB_PIN);
        add_signed_pulse(&g_motorAPulses,
            (phaseA == phaseB) ? 1 : -1,
            ENCODER_READ_ERROR_MOTOR_A_OVERFLOW);
    }

    if ((status & GPIO_ENCODERS_MOTOR_B_ENCA_PIN) != 0U) {
        uint8_t phaseA = pin_is_high(GPIO_ENCODERS_MOTOR_B_ENCA_PIN);
        uint8_t phaseB = pin_is_high(GPIO_ENCODERS_MOTOR_B_ENCB_PIN);
        add_signed_pulse(&g_motorBPulses,
            (phaseA == phaseB) ? 1 : -1,
            ENCODER_READ_ERROR_MOTOR_B_OVERFLOW);
    }

    DL_GPIO_clearInterruptStatus(GPIO_ENCODERS_PORT, status);
}

uint8_t Encoder_GetAndClearPulseDeltas(int32_t *motorA, int32_t *motorB)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t errors;

    if ((motorA == (int32_t *)0) || (motorB == (int32_t *)0)) {
        return ENCODER_READ_ERROR_INVALID_ARGUMENT;
    }

    __disable_irq();
    *motorA = g_motorAPulses;
    *motorB = g_motorBPulses;
    errors = g_overflowFlags;
    g_motorAPulses = 0;
    g_motorBPulses = 0;
    g_overflowFlags = ENCODER_READ_ERROR_NONE;
    if (primask == 0U) {
        __enable_irq();
    }

    return errors;
}
