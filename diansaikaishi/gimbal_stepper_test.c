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

#define GIMBAL_P4_STEP_HALF_PERIOD_TICKS_100US (25U)
#define GIMBAL_P4_SMOKE_TEST_DELTA_DEG         (-90.0f)
#define GIMBAL_P4_STEPS_PER_REV                (3200.0f)
#define GIMBAL_P4_DEGREES_PER_REV              (360.0f)
#define GIMBAL_PITCH_POSITIVE_DIR_HIGH         (0)
#define GIMBAL_EN_ACTIVE_LOW                   (0)

static uint16_t g_pitchHalfPeriodTicks;
static int32_t g_pitchTargetSteps;
static int32_t g_pitchCompletedSteps;
static int8_t g_pitchDirection;
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
    g_feedback.enabled = enable ? 1U : 0U;
}

static void gimbal_set_pitch_dir(int8_t direction)
{
#if GIMBAL_PITCH_POSITIVE_DIR_HIGH
    if (direction >= 0) {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    }
#else
    if (direction >= 0) {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    }
#endif
}

static int32_t gimbal_deg_to_steps(float delta_deg)
{
    float raw_steps =
        (delta_deg * GIMBAL_P4_STEPS_PER_REV) / GIMBAL_P4_DEGREES_PER_REV;

    if (raw_steps >= 0.0f) {
        return (int32_t)(raw_steps + 0.5f);
    }
    return (int32_t)(raw_steps - 0.5f);
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

void GimbalStepperTest_StopHold(void)
{
    gimbal_stop_pitch_hold();
    gimbal_set_enable(true);
}

void GimbalStepperTest_Release(void)
{
    gimbal_stop_pitch_hold();
    gimbal_set_enable(false);
}

static void gimbal_start_pitch_fixed_steps(int32_t steps)
{
    gimbal_stop_pitch_hold();
    g_pitchDirection = (steps < 0) ? -1 : 1;
    g_pitchTargetSteps = (steps < 0) ? -steps : steps;
    g_pitchCompletedSteps = 0;
    g_feedback.target_steps = steps;
    g_feedback.completed_steps = 0;
    g_feedback.direction = g_pitchDirection;
    g_feedback.target_reached = 0U;

    if (steps == 0) {
        g_feedback.target_reached = 1U;
        return;
    }

    gimbal_set_pitch_dir(g_pitchDirection);
    gimbal_set_enable(true);
    g_pitchRunning = true;
    g_feedback.running = 1U;
}

void GimbalStepperTest_MoveRelativeDeg(float delta_deg)
{
    int32_t steps = gimbal_deg_to_steps(delta_deg);

    gimbal_start_pitch_fixed_steps(steps);
}

void GimbalStepperTest_Init(void)
{
    g_pitchHalfPeriodTicks = 0U;
    g_pitchTargetSteps = 0;
    g_pitchCompletedSteps = 0;
    g_pitchDirection = 1;
    g_pitchStepHigh = false;
    g_pitchRunning = false;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.direction = 1;
    g_feedback.enabled = 0U;
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

#if FEATURE_GIMBAL_TEST_AUTO_RUN
    GimbalStepperTest_MoveRelativeDeg(GIMBAL_P4_SMOKE_TEST_DELTA_DEG);
#else
    GimbalStepperTest_Release();
#endif
}

void GimbalStepperTest_Tick100us(void)
{
    if (!g_pitchRunning) {
        return;
    }

    g_pitchHalfPeriodTicks++;
    if (g_pitchHalfPeriodTicks < GIMBAL_P4_STEP_HALF_PERIOD_TICKS_100US) {
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
