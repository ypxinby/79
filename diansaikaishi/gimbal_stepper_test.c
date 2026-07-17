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

#define GIMBAL_P4_SMOKE_TEST_DELTA_DEG         (-90.0f)
#define GIMBAL_P4_STEPS_PER_REV                (3200.0f)
#define GIMBAL_P4_DEGREES_PER_REV              (360.0f)
#define GIMBAL_PITCH_POSITIVE_DIR_HIGH         (0)
#define GIMBAL_EN_ACTIVE_LOW                   (0)

static uint16_t g_pitchHalfPeriodTicks;
static uint16_t g_pitchStepHalfPeriodTicks;
static int64_t g_pitchEstimatedSteps;
static int64_t g_pitchTargetEstimatedSteps;
static int32_t g_pitchCompletedSteps;
static int8_t g_pitchDirection;
static bool g_pitchStepHigh;
static bool g_pitchRunning;
static bool g_pitchStopAfterStepLow;
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

static int32_t gimbal_clamp_i64_to_i32(int64_t value)
{
    if (value > 2147483647LL) {
        return 2147483647;
    }
    if (value < -2147483647LL - 1LL) {
        return (int32_t)(-2147483647LL - 1LL);
    }
    return (int32_t)value;
}

static void gimbal_update_remaining_feedback(void)
{
    g_feedback.target_steps = gimbal_clamp_i64_to_i32(
        g_pitchTargetEstimatedSteps - g_pitchEstimatedSteps);
}

static void gimbal_stop_pitch_hold(void)
{
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHigh = false;
    g_pitchRunning = false;
    g_pitchStopAfterStepLow = false;
    g_feedback.running = 0U;
    gimbal_update_remaining_feedback();
    g_feedback.target_reached =
        (g_pitchEstimatedSteps == g_pitchTargetEstimatedSteps) ? 1U : 0U;
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
    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_A_PITCH_DIR_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
    g_pitchTargetEstimatedSteps = g_pitchEstimatedSteps;
    g_pitchCompletedSteps = 0;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.target_reached = 1U;
}

static void gimbal_start_pitch_to_estimated_steps(
    int64_t target_estimated_steps)
{
    int64_t delta_steps = target_estimated_steps - g_pitchEstimatedSteps;
    int8_t new_direction = (delta_steps < 0) ? -1 : 1;

    g_pitchTargetEstimatedSteps = target_estimated_steps;
    if (delta_steps == 0) {
        gimbal_stop_pitch_hold();
        g_pitchCompletedSteps = 0;
        g_feedback.completed_steps = 0;
        g_feedback.target_reached = 1U;
        return;
    }

    if (g_pitchRunning && (new_direction != g_pitchDirection)) {
        gimbal_stop_pitch_hold();
        delta_steps = g_pitchTargetEstimatedSteps - g_pitchEstimatedSteps;
        new_direction = (delta_steps < 0) ? -1 : 1;
    }

    if (!g_pitchRunning) {
        gimbal_stop_pitch_hold();
        g_pitchCompletedSteps = 0;
        g_feedback.completed_steps = 0;
        g_pitchHalfPeriodTicks = 0U;
    }

    g_pitchDirection = new_direction;
    g_pitchStopAfterStepLow = false;
    gimbal_update_remaining_feedback();
    g_feedback.direction = g_pitchDirection;
    g_feedback.target_reached = 0U;

    gimbal_set_pitch_dir(g_pitchDirection);
    gimbal_set_enable(true);
    g_pitchRunning = true;
    g_feedback.running = 1U;
}

void GimbalStepperTest_MoveToEstimatedSteps(int64_t target_estimated_steps)
{
    gimbal_start_pitch_to_estimated_steps(target_estimated_steps);
}

void GimbalStepperTest_MoveRelativeDeg(float delta_deg)
{
    int32_t steps = gimbal_deg_to_steps(delta_deg);

    gimbal_start_pitch_to_estimated_steps(g_pitchEstimatedSteps + steps);
}

void GimbalStepperTest_SetStepHalfPeriodTicks(uint16_t half_period_ticks)
{
    g_pitchStepHalfPeriodTicks = half_period_ticks;
    g_feedback.step_half_period_ticks = half_period_ticks;
}

void GimbalStepperTest_Init(void)
{
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHalfPeriodTicks = 0U;
    g_pitchEstimatedSteps = 0;
    g_pitchTargetEstimatedSteps = 0;
    g_pitchCompletedSteps = 0;
    g_pitchDirection = 1;
    g_pitchStepHigh = false;
    g_pitchRunning = false;
    g_pitchStopAfterStepLow = false;
    g_feedback.estimated_steps = 0;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.step_half_period_ticks = 0U;
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
    if (!g_pitchRunning || (g_pitchStepHalfPeriodTicks == 0U)) {
        return;
    }

    g_pitchHalfPeriodTicks++;
    if (g_pitchHalfPeriodTicks < g_pitchStepHalfPeriodTicks) {
        return;
    }

    g_pitchHalfPeriodTicks = 0U;
    if (g_pitchStepHigh) {
        DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
        g_pitchStepHigh = false;
        if (g_pitchStopAfterStepLow) {
            gimbal_stop_pitch_hold();
        }
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_B_PITCH_STEP_PIN);
        g_pitchStepHigh = true;
        g_pitchCompletedSteps++;
        g_pitchEstimatedSteps += g_pitchDirection;
        g_feedback.estimated_steps = g_pitchEstimatedSteps;
        g_feedback.completed_steps = g_pitchCompletedSteps;
        gimbal_update_remaining_feedback();
        if (((g_pitchDirection > 0) &&
                (g_pitchEstimatedSteps >= g_pitchTargetEstimatedSteps)) ||
            ((g_pitchDirection < 0) &&
                (g_pitchEstimatedSteps <= g_pitchTargetEstimatedSteps))) {
            g_feedback.target_reached = 1U;
            g_feedback.target_steps = 0;
            g_pitchStopAfterStepLow = true;
        }
    }
}

const GimbalStepperTestFeedback *GimbalStepperTest_GetFeedback(void)
{
    return &g_feedback;
}
