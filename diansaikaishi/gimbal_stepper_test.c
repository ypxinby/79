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

#ifndef GPIO_GIMBAL_B_PITCH_STEP_PIN
#define GPIO_GIMBAL_B_PITCH_STEP_PIN           (DL_GPIO_PIN_4)
#endif

#ifndef GPIO_GIMBAL_B_PITCH_STEP_IOMUX
#define GPIO_GIMBAL_B_PITCH_STEP_IOMUX         (IOMUX_PINCM17)
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

#define GIMBAL_P2_STEP_HALF_PERIOD_TICKS_100US (25U)
#define GIMBAL_P2_TARGET_STEPS                 (800)
#define GIMBAL_P2_PITCH_DIR_HIGH               (0)
#define GIMBAL_EN_ACTIVE_LOW                   (0)

static uint16_t g_pitchHalfPeriodTicks;
static int32_t g_pitchTargetSteps;
static int32_t g_pitchCompletedSteps;
static bool g_pitchStepHigh;
static bool g_pitchRunning;
static GimbalStepperTestFeedback g_feedback;

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
#if GIMBAL_P2_PITCH_DIR_HIGH
    DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
#else
    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
#endif
}

static void gimbal_stop_pitch_hold(void)
{
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHigh = false;
    g_pitchRunning = false;
    g_feedback.running = 0U;
    g_feedback.target_reached =
        (g_pitchCompletedSteps >= g_pitchTargetSteps) ? 1U : 0U;
}

static void gimbal_start_pitch_fixed_steps(int32_t steps)
{
    gimbal_stop_pitch_hold();
    g_pitchTargetSteps = steps;
    g_pitchCompletedSteps = 0;
    g_feedback.target_steps = steps;
    g_feedback.completed_steps = 0;
    g_feedback.target_reached = 0U;

    if (steps <= 0) {
        g_feedback.target_reached = 1U;
        return;
    }

    gimbal_set_pitch_dir();
    gimbal_set_enable(true);
    g_pitchRunning = true;
    g_feedback.running = 1U;
}

void GimbalStepperTest_Init(void)
{
    g_pitchHalfPeriodTicks = 0U;
    g_pitchTargetSteps = 0;
    g_pitchCompletedSteps = 0;
    g_pitchStepHigh = false;
    g_pitchRunning = false;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.running = 0U;
    g_feedback.target_reached = 0U;

    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_B_PITCH_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_PITCH_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_A_PITCH_EN_IOMUX);

    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_EN_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);

    DL_GPIO_enableOutput(GPIO_GIMBAL_A_PORT,
        GPIO_GIMBAL_A_PITCH_DIR_PIN | GPIO_GIMBAL_A_PITCH_EN_PIN);
    DL_GPIO_enableOutput(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);

#if FEATURE_GIMBAL_P1_SMOKE_TEST
    gimbal_start_pitch_fixed_steps(GIMBAL_P2_TARGET_STEPS);
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
    if (g_pitchHalfPeriodTicks < GIMBAL_P2_STEP_HALF_PERIOD_TICKS_100US) {
        return;
    }

    g_pitchHalfPeriodTicks = 0U;
    if (g_pitchStepHigh) {
        DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
        g_pitchStepHigh = false;
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
        g_pitchStepHigh = true;
        g_pitchCompletedSteps++;
        g_feedback.completed_steps = g_pitchCompletedSteps;
        if (g_pitchCompletedSteps >= g_pitchTargetSteps) {
            gimbal_stop_pitch_hold();
        }
    }
}

const GimbalStepperTestFeedback *GimbalStepperTest_GetFeedback(void)
{
    return &g_feedback;
}
