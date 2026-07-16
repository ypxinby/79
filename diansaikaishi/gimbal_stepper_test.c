#include "gimbal_stepper_test.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_features.h"
#include "ti_msp_dl_config.h"

#ifndef GPIO_GIMBAL_A_PORT
#define GPIO_GIMBAL_A_PORT                     (GPIOA)
#endif

#ifndef GPIO_GIMBAL_B_PORT
#define GPIO_GIMBAL_B_PORT                     (GPIOB)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_STEP_PIN
#define GPIO_GIMBAL_A_PITCH_STEP_PIN           (DL_GPIO_PIN_3)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_STEP_IOMUX
#define GPIO_GIMBAL_A_PITCH_STEP_IOMUX         (IOMUX_PINCM8)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_DIR_PIN
#define GPIO_GIMBAL_A_PITCH_DIR_PIN            (DL_GPIO_PIN_1)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_DIR_IOMUX
#define GPIO_GIMBAL_A_PITCH_DIR_IOMUX          (IOMUX_PINCM2)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_EN_PIN
#define GPIO_GIMBAL_A_PITCH_EN_PIN             (DL_GPIO_PIN_2)
#endif

#ifndef GPIO_GIMBAL_A_PITCH_EN_IOMUX
#define GPIO_GIMBAL_A_PITCH_EN_IOMUX           (IOMUX_PINCM7)
#endif

#ifndef GPIO_GIMBAL_A_YAW_DIR_PIN
#define GPIO_GIMBAL_A_YAW_DIR_PIN              (DL_GPIO_PIN_4)
#endif

#ifndef GPIO_GIMBAL_A_YAW_DIR_IOMUX
#define GPIO_GIMBAL_A_YAW_DIR_IOMUX            (IOMUX_PINCM9)
#endif

#ifndef GPIO_GIMBAL_A_YAW_EN_PIN
#define GPIO_GIMBAL_A_YAW_EN_PIN               (DL_GPIO_PIN_5)
#endif

#ifndef GPIO_GIMBAL_A_YAW_EN_IOMUX
#define GPIO_GIMBAL_A_YAW_EN_IOMUX             (IOMUX_PINCM10)
#endif

#ifndef GPIO_GIMBAL_B_YAW_STEP_PIN
#define GPIO_GIMBAL_B_YAW_STEP_PIN             (DL_GPIO_PIN_4)
#endif

#ifndef GPIO_GIMBAL_B_YAW_STEP_IOMUX
#define GPIO_GIMBAL_B_YAW_STEP_IOMUX           (IOMUX_PINCM17)
#endif

#define GIMBAL_P1_STEP_HALF_PERIOD_TICKS_100US (25U)
#define GIMBAL_P1_PITCH_DIR_HIGH               (0)
#define GIMBAL_EN_ACTIVE_LOW                   (1)

static uint16_t g_pitchHalfPeriodTicks;
static bool g_pitchStepHigh;
static bool g_pitchRunning;

static void gimbal_set_enable(bool enable)
{
#if GIMBAL_EN_ACTIVE_LOW
    if (enable) {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_EN_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_EN_PIN);
    }
#else
    if (enable) {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_EN_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_EN_PIN);
    }
#endif
}

static void gimbal_set_pitch_dir(void)
{
#if GIMBAL_P1_PITCH_DIR_HIGH
    DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
#else
    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
#endif
}

void GimbalStepperTest_Init(void)
{
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHigh = false;
    g_pitchRunning = false;

    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_PITCH_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_PITCH_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_PITCH_EN_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_YAW_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_YAW_EN_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_B_YAW_STEP_IOMUX);

    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT,
        GPIO_GIMBAL_A_PITCH_STEP_PIN | GPIO_GIMBAL_A_PITCH_DIR_PIN |
            GPIO_GIMBAL_A_YAW_DIR_PIN);
    DL_GPIO_setPins(GPIO_GIMBAL_A_PORT,
        GPIO_GIMBAL_A_PITCH_EN_PIN | GPIO_GIMBAL_A_YAW_EN_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_YAW_STEP_PIN);

    DL_GPIO_enableOutput(GPIO_GIMBAL_A_PORT,
        GPIO_GIMBAL_A_PITCH_STEP_PIN | GPIO_GIMBAL_A_PITCH_DIR_PIN |
            GPIO_GIMBAL_A_PITCH_EN_PIN | GPIO_GIMBAL_A_YAW_DIR_PIN |
            GPIO_GIMBAL_A_YAW_EN_PIN);
    DL_GPIO_enableOutput(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_YAW_STEP_PIN);

#if FEATURE_GIMBAL_P1_SMOKE_TEST
    gimbal_set_pitch_dir();
    gimbal_set_enable(true);
    g_pitchRunning = true;
#else
    gimbal_set_enable(false);
#endif
}

void GimbalStepperTest_Tick100us(void)
{
    if (!g_pitchRunning) {
        return;
    }

    g_pitchHalfPeriodTicks++;
    if (g_pitchHalfPeriodTicks < GIMBAL_P1_STEP_HALF_PERIOD_TICKS_100US) {
        return;
    }

    g_pitchHalfPeriodTicks = 0U;
    if (g_pitchStepHigh) {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_STEP_PIN);
        g_pitchStepHigh = false;
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_STEP_PIN);
        g_pitchStepHigh = true;
    }
}
