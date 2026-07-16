#include "gimbal.h"

#include "gimbal_stepper_test.h"

#define GIMBAL_STEPS_PER_REV   (3200L)
#define GIMBAL_DEG_X10_PER_REV (3600L)

static GimbalFeedback g_feedback;

static int16_t clamp_i16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t steps_to_deg_x10(int32_t steps)
{
    int32_t value = (steps * GIMBAL_DEG_X10_PER_REV) / GIMBAL_STEPS_PER_REV;

    return clamp_i16(value);
}

static void gimbal_update_feedback(void)
{
    const GimbalStepperTestFeedback *src = GimbalStepperTest_GetFeedback();
    int32_t signed_completed = src->completed_steps;

    if (src->direction < 0) {
        signed_completed = -signed_completed;
    }

    g_feedback.target_steps = src->target_steps;
    g_feedback.completed_steps = signed_completed;
    g_feedback.target_deg_x10 = steps_to_deg_x10(src->target_steps);
    g_feedback.completed_deg_x10 = steps_to_deg_x10(signed_completed);
    g_feedback.enabled = src->enabled;
    g_feedback.direction = src->direction;
    g_feedback.running = src->running;
    g_feedback.target_reached = src->target_reached;
    if (src->running) {
        g_feedback.mode = GIMBAL_MODE_MOVING;
    } else if (src->enabled) {
        g_feedback.mode = GIMBAL_MODE_HOLDING;
    } else {
        g_feedback.mode = GIMBAL_MODE_RELEASED;
    }
}

void Gimbal_Init(void)
{
    g_feedback = (GimbalFeedback){0};
    GimbalStepperTest_Init();
    gimbal_update_feedback();
}

void Gimbal_Tick100us(void)
{
    GimbalStepperTest_Tick100us();
}

void Gimbal_Update5ms(void)
{
    g_feedback.control_tick_5ms++;
}

void Gimbal_MoveRelativeDeg(float delta_deg)
{
    GimbalStepperTest_MoveRelativeDeg(delta_deg);
    gimbal_update_feedback();
}

void Gimbal_StopHold(void)
{
    GimbalStepperTest_StopHold();
    gimbal_update_feedback();
}

void Gimbal_Release(void)
{
    GimbalStepperTest_Release();
    gimbal_update_feedback();
}

const GimbalFeedback *Gimbal_GetFeedback(void)
{
    gimbal_update_feedback();
    return &g_feedback;
}
