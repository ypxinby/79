#include "gimbal.h"

#include "gimbal_stepper_test.h"

#define GIMBAL_STEPS_PER_REV   (3200LL)
#define GIMBAL_DEG_X10_PER_REV (3600LL)

static GimbalFeedback g_feedback;
static int64_t g_yawContinuousDegX10;
static int64_t g_yawWrappedDegX10;
static int64_t g_yawTargetContinuousDegX10;
static int32_t g_yawTurnCount;
static uint8_t g_yawPositionValid;

static int16_t clamp_i16(int64_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static int64_t deg_to_x10(float deg)
{
    float raw = deg * 10.0f;

    if (raw >= 0.0f) {
        return (int64_t)(raw + 0.5f);
    }
    return (int64_t)(raw - 0.5f);
}

static int64_t wrap_x10_360(int64_t angle_deg_x10)
{
    int64_t wrapped = angle_deg_x10 % GIMBAL_DEG_X10_PER_REV;

    if (wrapped < 0) {
        wrapped += GIMBAL_DEG_X10_PER_REV;
    }
    return wrapped;
}

static int64_t shortest_error_x10(int64_t target_wrapped_deg_x10,
    int64_t current_wrapped_deg_x10)
{
    int64_t error =
        target_wrapped_deg_x10 - current_wrapped_deg_x10;

    while (error > (GIMBAL_DEG_X10_PER_REV / 2)) {
        error -= GIMBAL_DEG_X10_PER_REV;
    }

    while (error < -(GIMBAL_DEG_X10_PER_REV / 2)) {
        error += GIMBAL_DEG_X10_PER_REV;
    }

    return error;
}

static int64_t floor_div_i64(int64_t numerator, int64_t denominator)
{
    int64_t quotient = numerator / denominator;
    int64_t remainder = numerator % denominator;

    if ((remainder != 0) &&
        ((remainder < 0) != (denominator < 0))) {
        quotient--;
    }
    return quotient;
}

static int64_t steps_to_deg_x10(int64_t steps)
{
    return (steps * GIMBAL_DEG_X10_PER_REV) / GIMBAL_STEPS_PER_REV;
}

static float x10_to_deg(int64_t deg_x10)
{
    return ((float)deg_x10) / 10.0f;
}

static void gimbal_update_angle_from_steps(int64_t estimated_steps)
{
    int64_t turn_count;

    g_yawContinuousDegX10 = steps_to_deg_x10(estimated_steps);
    turn_count = floor_div_i64(g_yawContinuousDegX10,
        GIMBAL_DEG_X10_PER_REV);
    g_yawTurnCount = (int32_t)turn_count;
    g_yawWrappedDegX10 = g_yawContinuousDegX10 -
        (turn_count * GIMBAL_DEG_X10_PER_REV);
}

static void gimbal_update_feedback(void)
{
    const GimbalStepperTestFeedback *src = GimbalStepperTest_GetFeedback();
    int32_t signed_completed = src->completed_steps;

    if (src->direction < 0) {
        signed_completed = -signed_completed;
    }

    gimbal_update_angle_from_steps(src->estimated_steps);

    g_feedback.estimated_steps = src->estimated_steps;
    g_feedback.target_steps = src->target_steps;
    g_feedback.completed_steps = signed_completed;
    g_feedback.target_deg_x10 =
        clamp_i16(g_yawTargetContinuousDegX10);
    g_feedback.completed_deg_x10 =
        clamp_i16(g_yawContinuousDegX10);
    g_feedback.continuous_deg_x10 =
        clamp_i16(g_yawContinuousDegX10);
    g_feedback.wrapped_deg_x10 =
        clamp_i16(g_yawWrappedDegX10);
    g_feedback.turn_count = g_yawTurnCount;
    g_feedback.min_limit_deg_x10 = 0;
    g_feedback.max_limit_deg_x10 = 0;
    g_feedback.enabled = src->enabled;
    g_feedback.direction = src->direction;
    g_feedback.running = src->running;
    g_feedback.target_reached = src->target_reached;
    g_feedback.limit_clamped = 0U;
    g_feedback.position_valid = g_yawPositionValid;
    if (src->running) {
        g_feedback.mode = GIMBAL_MODE_MOVING;
    } else if (src->enabled) {
        g_feedback.mode = GIMBAL_MODE_HOLDING;
    } else {
        g_feedback.mode = GIMBAL_MODE_RELEASED;
    }
}

void Gimbal_YawInit(void)
{
    g_feedback = (GimbalFeedback){0};
    g_yawContinuousDegX10 = 0;
    g_yawWrappedDegX10 = 0;
    g_yawTargetContinuousDegX10 = 0;
    g_yawTurnCount = 0;
    g_yawPositionValid = 1U;
    GimbalStepperTest_Init();
    gimbal_update_feedback();
}

void Gimbal_YawTick100us(void)
{
    GimbalStepperTest_Tick100us();
}

void Gimbal_YawUpdate5ms(void)
{
    g_feedback.control_tick_5ms++;
    gimbal_update_feedback();
}

static void gimbal_yaw_move_to_x10(int64_t target_continuous_deg_x10)
{
    int64_t delta_deg_x10;

    gimbal_update_feedback();
    g_yawTargetContinuousDegX10 = target_continuous_deg_x10;
    delta_deg_x10 =
        g_yawTargetContinuousDegX10 - g_yawContinuousDegX10;

    if (delta_deg_x10 == 0) {
        if (GimbalStepperTest_GetFeedback()->running) {
            GimbalStepperTest_StopHold();
        }
        gimbal_update_feedback();
        return;
    }

    GimbalStepperTest_MoveRelativeDeg(x10_to_deg(delta_deg_x10));
    gimbal_update_feedback();
}

void Gimbal_YawMoveToDeg(float target_deg)
{
    gimbal_yaw_move_to_x10(deg_to_x10(target_deg));
}

void Gimbal_YawMoveWrappedDeg(float target_deg)
{
    int64_t target_wrapped_deg_x10;
    int64_t error_deg_x10;

    gimbal_update_feedback();
    target_wrapped_deg_x10 = wrap_x10_360(deg_to_x10(target_deg));
    error_deg_x10 = shortest_error_x10(target_wrapped_deg_x10,
        g_yawWrappedDegX10);
    gimbal_yaw_move_to_x10(g_yawContinuousDegX10 + error_deg_x10);
}

void Gimbal_YawMoveRelativeDeg(float delta_deg)
{
    gimbal_update_feedback();
    gimbal_yaw_move_to_x10(g_yawTargetContinuousDegX10 +
        deg_to_x10(delta_deg));
}

void Gimbal_YawStopHold(void)
{
    GimbalStepperTest_StopHold();
    gimbal_update_feedback();
    g_yawTargetContinuousDegX10 = g_yawContinuousDegX10;
    gimbal_update_feedback();
}

void Gimbal_YawRelease(void)
{
    gimbal_update_feedback();
    g_yawTargetContinuousDegX10 = g_yawContinuousDegX10;
    GimbalStepperTest_Release();
    gimbal_update_feedback();
}

const GimbalFeedback *Gimbal_YawGetFeedback(void)
{
    gimbal_update_feedback();
    return &g_feedback;
}

void Gimbal_Init(void)
{
    Gimbal_YawInit();
}

void Gimbal_Tick100us(void)
{
    Gimbal_YawTick100us();
}

void Gimbal_Update5ms(void)
{
    Gimbal_YawUpdate5ms();
}

void Gimbal_MoveToDeg(float target_deg)
{
    Gimbal_YawMoveToDeg(target_deg);
}

void Gimbal_MoveWrappedDeg(float target_deg)
{
    Gimbal_YawMoveWrappedDeg(target_deg);
}

void Gimbal_MoveRelativeDeg(float delta_deg)
{
    Gimbal_YawMoveRelativeDeg(delta_deg);
}

void Gimbal_StopHold(void)
{
    Gimbal_YawStopHold();
}

void Gimbal_Release(void)
{
    Gimbal_YawRelease();
}

const GimbalFeedback *Gimbal_GetFeedback(void)
{
    return Gimbal_YawGetFeedback();
}
