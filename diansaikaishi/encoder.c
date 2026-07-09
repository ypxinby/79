#include "encoder.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_motorAPulses;
static volatile int32_t g_motorBPulses;

static uint8_t pin_is_high(uint32_t pin)
{
    return (DL_GPIO_readPins(GPIO_ENCODERS_PORT, pin) != 0U) ? 1U : 0U;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_motorAPulses = 0;
    g_motorBPulses = 0;
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
        g_motorAPulses += (phaseA == phaseB) ? 1 : -1;
    }

    if ((status & GPIO_ENCODERS_MOTOR_B_ENCA_PIN) != 0U) {
        uint8_t phaseA = pin_is_high(GPIO_ENCODERS_MOTOR_B_ENCA_PIN);
        uint8_t phaseB = pin_is_high(GPIO_ENCODERS_MOTOR_B_ENCB_PIN);
        g_motorBPulses += (phaseA == phaseB) ? 1 : -1;
    }

    DL_GPIO_clearInterruptStatus(GPIO_ENCODERS_PORT, status);
}

void Encoder_GetAndClearPulseDeltas(int32_t *motorA, int32_t *motorB)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *motorA = abs_i32(g_motorAPulses);
    *motorB = abs_i32(g_motorBPulses);
    g_motorAPulses = 0;
    g_motorBPulses = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}
