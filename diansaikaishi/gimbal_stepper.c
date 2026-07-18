#include "gimbal_stepper.h"

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

#ifndef GPIO_GIMBAL_YAW_STEP_PIN
#ifdef GPIO_GIMBAL_B_PITCH_STEP_PIN
#define GPIO_GIMBAL_YAW_STEP_PIN               (GPIO_GIMBAL_B_PITCH_STEP_PIN)
#else
#define GPIO_GIMBAL_YAW_STEP_PIN               (DL_GPIO_PIN_4)
#endif
#endif

#ifndef GPIO_GIMBAL_YAW_STEP_IOMUX
#ifdef GPIO_GIMBAL_B_PITCH_STEP_IOMUX
#define GPIO_GIMBAL_YAW_STEP_IOMUX             (GPIO_GIMBAL_B_PITCH_STEP_IOMUX)
#else
#define GPIO_GIMBAL_YAW_STEP_IOMUX             (IOMUX_PINCM17)
#endif
#endif

#ifndef GPIO_GIMBAL_YAW_DIR_PIN
#ifdef GPIO_GIMBAL_A_PITCH_DIR_PIN
#define GPIO_GIMBAL_YAW_DIR_PIN                (GPIO_GIMBAL_A_PITCH_DIR_PIN)
#else
#define GPIO_GIMBAL_YAW_DIR_PIN                (DL_GPIO_PIN_1)
#endif
#endif

#ifndef GPIO_GIMBAL_YAW_DIR_IOMUX
#ifdef GPIO_GIMBAL_A_PITCH_DIR_IOMUX
#define GPIO_GIMBAL_YAW_DIR_IOMUX              (GPIO_GIMBAL_A_PITCH_DIR_IOMUX)
#else
#define GPIO_GIMBAL_YAW_DIR_IOMUX              (IOMUX_PINCM2)
#endif
#endif

#ifndef GPIO_GIMBAL_YAW_EN_PIN
#ifdef GPIO_GIMBAL_A_PITCH_EN_PIN
#define GPIO_GIMBAL_YAW_EN_PIN                 (GPIO_GIMBAL_A_PITCH_EN_PIN)
#else
#define GPIO_GIMBAL_YAW_EN_PIN                 (DL_GPIO_PIN_2)
#endif
#endif

#ifndef GPIO_GIMBAL_YAW_EN_IOMUX
#ifdef GPIO_GIMBAL_A_PITCH_EN_IOMUX
#define GPIO_GIMBAL_YAW_EN_IOMUX               (GPIO_GIMBAL_A_PITCH_EN_IOMUX)
#else
#define GPIO_GIMBAL_YAW_EN_IOMUX               (IOMUX_PINCM7)
#endif
#endif

#define GIMBAL_STEPPER_SMOKE_TEST_DELTA_DEG    (-90.0f)
#define GIMBAL_STEPPER_STEPS_PER_REV           (3200.0f)
#define GIMBAL_STEPPER_DEGREES_PER_REV         (360.0f)
#define GIMBAL_STEPPER_POSITIVE_DIR_HIGH       (0)
#define GIMBAL_EN_ACTIVE_LOW                   (0)

static uint16_t g_stepperHalfPeriodTicks;
static uint16_t g_stepperStepHalfPeriodTicks;
static int64_t g_stepperEstimatedSteps;
static int64_t g_stepperTargetEstimatedSteps;
static int32_t g_stepperCompletedSteps;
static int8_t g_stepperDirection;
static bool g_stepperStepHigh;
static bool g_stepperRunning;
static bool g_stepperStopAfterStepLow;
static GimbalStepperFeedback g_feedback;

static void gimbal_set_enable(bool enable)
{
#if GIMBAL_EN_ACTIVE_LOW
    if (enable) {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_EN_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_EN_PIN);
    }
#else
    if (enable) {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_EN_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_EN_PIN);
    }
#endif
    g_feedback.enabled = enable ? 1U : 0U;
}

static void gimbal_stepper_set_dir(int8_t direction)
{
#if GIMBAL_STEPPER_POSITIVE_DIR_HIGH
    if (direction >= 0) {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    }
#else
    if (direction >= 0) {
        DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    }
#endif
}

static int32_t gimbal_deg_to_steps(float delta_deg)
{
    float raw_steps =
        (delta_deg * GIMBAL_STEPPER_STEPS_PER_REV) / GIMBAL_STEPPER_DEGREES_PER_REV;

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
        g_stepperTargetEstimatedSteps - g_stepperEstimatedSteps);
}

static void gimbal_stepper_stop_hold(void)
{
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);
    g_stepperHalfPeriodTicks = 0U;
    g_stepperStepHigh = false;
    g_stepperRunning = false;
    g_stepperStopAfterStepLow = false;
    g_feedback.running = 0U;
    gimbal_update_remaining_feedback();
    g_feedback.target_reached =
        (g_stepperEstimatedSteps == g_stepperTargetEstimatedSteps) ? 1U : 0U;
}

void GimbalStepper_StopHold(void)
{
    gimbal_stepper_stop_hold();
    gimbal_set_enable(true);
}

void GimbalStepper_Release(void)
{
    gimbal_stepper_stop_hold();
    gimbal_set_enable(false);
    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);
    g_stepperTargetEstimatedSteps = g_stepperEstimatedSteps;
    g_stepperCompletedSteps = 0;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.target_reached = 1U;
}

static void gimbal_stepper_start_to_estimated_steps(
    int64_t target_estimated_steps)
{
    int64_t delta_steps = target_estimated_steps - g_stepperEstimatedSteps;
    int8_t new_direction = (delta_steps < 0) ? -1 : 1;

    g_stepperTargetEstimatedSteps = target_estimated_steps;
    if (delta_steps == 0) {
        gimbal_stepper_stop_hold();
        g_stepperCompletedSteps = 0;
        g_feedback.completed_steps = 0;
        g_feedback.target_reached = 1U;
        return;
    }

    if (g_stepperRunning && (new_direction != g_stepperDirection)) {
        gimbal_stepper_stop_hold();
        delta_steps = g_stepperTargetEstimatedSteps - g_stepperEstimatedSteps;
        new_direction = (delta_steps < 0) ? -1 : 1;
    }

    if (!g_stepperRunning) {
        gimbal_stepper_stop_hold();
        g_stepperCompletedSteps = 0;
        g_feedback.completed_steps = 0;
        g_stepperHalfPeriodTicks = 0U;
    }

    g_stepperDirection = new_direction;
    g_stepperStopAfterStepLow = false;
    gimbal_update_remaining_feedback();
    g_feedback.direction = g_stepperDirection;
    g_feedback.target_reached = 0U;

    gimbal_stepper_set_dir(g_stepperDirection);
    gimbal_set_enable(true);
    g_stepperRunning = true;
    g_feedback.running = 1U;
}

void GimbalStepper_MoveToEstimatedSteps(int64_t target_estimated_steps)
{
    gimbal_stepper_start_to_estimated_steps(target_estimated_steps);
}

void GimbalStepper_MoveRelativeDeg(float delta_deg)
{
    int32_t steps = gimbal_deg_to_steps(delta_deg);

    gimbal_stepper_start_to_estimated_steps(g_stepperEstimatedSteps + steps);
}

uint8_t GimbalStepper_ConfirmZero(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    if (g_stepperRunning) {
        if ((interrupt_state & 1U) == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);
    g_stepperHalfPeriodTicks = 0U;
    g_stepperStepHalfPeriodTicks = 0U;
    g_stepperEstimatedSteps = 0;
    g_stepperTargetEstimatedSteps = 0;
    g_stepperCompletedSteps = 0;
    g_stepperStepHigh = false;
    g_stepperStopAfterStepLow = false;
    g_feedback.estimated_steps = 0;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.step_half_period_ticks = 0U;
    g_feedback.running = 0U;
    g_feedback.target_reached = 1U;

    if ((interrupt_state & 1U) == 0U) {
        __enable_irq();
    }
    return 1U;
}

void GimbalStepper_SetStepHalfPeriodTicks(uint16_t half_period_ticks)
{
    g_stepperStepHalfPeriodTicks = half_period_ticks;
    g_feedback.step_half_period_ticks = half_period_ticks;
}

void GimbalStepper_Init(void)
{
    g_stepperHalfPeriodTicks = 0U;
    g_stepperStepHalfPeriodTicks = 0U;
    g_stepperEstimatedSteps = 0;
    g_stepperTargetEstimatedSteps = 0;
    g_stepperCompletedSteps = 0;
    g_stepperDirection = 1;
    g_stepperStepHigh = false;
    g_stepperRunning = false;
    g_stepperStopAfterStepLow = false;
    g_feedback.estimated_steps = 0;
    g_feedback.target_steps = 0;
    g_feedback.completed_steps = 0;
    g_feedback.step_half_period_ticks = 0U;
    g_feedback.direction = 1;
    g_feedback.enabled = 0U;
    g_feedback.running = 0U;
    g_feedback.target_reached = 0U;

    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_YAW_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_YAW_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_YAW_EN_IOMUX);

    DL_GPIO_clearPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_DIR_PIN);
    DL_GPIO_setPins(GPIO_GIMBAL_A_PORT, GPIO_GIMBAL_YAW_EN_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);

    DL_GPIO_enableOutput(GPIO_GIMBAL_A_PORT,
        GPIO_GIMBAL_YAW_DIR_PIN | GPIO_GIMBAL_YAW_EN_PIN);
    DL_GPIO_enableOutput(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);

#if FEATURE_GIMBAL_TEST_AUTO_RUN
    GimbalStepper_MoveRelativeDeg(GIMBAL_STEPPER_SMOKE_TEST_DELTA_DEG);
#else
    GimbalStepper_Release();
#endif
}

void GimbalStepper_Tick100us(void)
{
    if (!g_stepperRunning || (g_stepperStepHalfPeriodTicks == 0U)) {
        return;
    }

    g_stepperHalfPeriodTicks++;
    if (g_stepperHalfPeriodTicks < g_stepperStepHalfPeriodTicks) {
        return;
    }

    g_stepperHalfPeriodTicks = 0U;
    if (g_stepperStepHigh) {
        DL_GPIO_clearPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);
        g_stepperStepHigh = false;
        if (g_stepperStopAfterStepLow) {
            gimbal_stepper_stop_hold();
        }
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_B_PORT, GPIO_GIMBAL_YAW_STEP_PIN);
        g_stepperStepHigh = true;
        g_stepperCompletedSteps++;
        g_stepperEstimatedSteps += g_stepperDirection;
        g_feedback.estimated_steps = g_stepperEstimatedSteps;
        g_feedback.completed_steps = g_stepperCompletedSteps;
        gimbal_update_remaining_feedback();
        if (((g_stepperDirection > 0) &&
                (g_stepperEstimatedSteps >= g_stepperTargetEstimatedSteps)) ||
            ((g_stepperDirection < 0) &&
                (g_stepperEstimatedSteps <= g_stepperTargetEstimatedSteps))) {
            g_feedback.target_reached = 1U;
            g_feedback.target_steps = 0;
            g_stepperStopAfterStepLow = true;
        }
    }
}

const GimbalStepperFeedback *GimbalStepper_GetFeedback(void)
{
    return &g_feedback;
}
