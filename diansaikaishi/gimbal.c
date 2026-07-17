#include "gimbal.h"

#include "gimbal_stepper_test.h"
#include "imu.h"

#define GIMBAL_STEPS_PER_REV   (3200LL)
#define GIMBAL_DEG_X10_PER_REV (3600LL)
#define GIMBAL_CONTROL_DT_S    (0.005f)
#define GIMBAL_MAX_POSITION_RPM (4.0f)
#define GIMBAL_MAX_LOCK_RPM    (12.0f)
#define GIMBAL_MIN_EFFECTIVE_RPM (0.5f)
#define GIMBAL_ACCEL_LIMIT_RPM_PER_S (10.0f)
#define GIMBAL_LOCK_ACCEL_LIMIT_RPM_PER_S (30.0f)
#define GIMBAL_TICK_HZ         (10000.0f)

static GimbalFeedback g_feedback;
static int64_t g_yawContinuousDegX10;
static int64_t g_yawWrappedDegX10;
static int64_t g_yawTargetContinuousDegX10;
static int32_t g_yawTurnCount;
static uint8_t g_yawPositionValid;
static float g_yawTargetRpm;
static float g_yawCommandedRpm;
static uint8_t g_yawWorldLockEnabled;
static int64_t g_yawCarYawDegX10;
static int64_t g_yawLockedWorldYawDegX10;
static int64_t g_yawWorldTargetDegX10;

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

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t rpm_to_x10(float rpm)
{
    float raw = rpm * 10.0f;

    if (raw >= 0.0f) {
        return clamp_i16((int64_t)(raw + 0.5f));
    }
    return clamp_i16((int64_t)(raw - 0.5f));
}

static uint8_t rpm_matches_direction(float rpm, int8_t direction)
{
    if ((rpm > 0.0f) && (direction >= 0)) {
        return 1U;
    }
    if ((rpm < 0.0f) && (direction < 0)) {
        return 1U;
    }
    return 0U;
}

static uint16_t rpm_to_half_period_ticks(float rpm)
{
    float abs_rpm = abs_f32(rpm);
    float step_frequency_hz;
    float half_period_ticks;

    if (abs_rpm < GIMBAL_MIN_EFFECTIVE_RPM) {
        return 0U;
    }

    step_frequency_hz =
        (abs_rpm * (float)GIMBAL_STEPS_PER_REV) / 60.0f;
    half_period_ticks =
        GIMBAL_TICK_HZ / (2.0f * step_frequency_hz);

    if (half_period_ticks < 1.0f) {
        return 1U;
    }
    if (half_period_ticks > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)(half_period_ticks + 0.5f);
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

static int64_t deg_x10_to_steps(int64_t deg_x10)
{
    int64_t numerator = deg_x10 * GIMBAL_STEPS_PER_REV;

    if (numerator >= 0) {
        return (numerator + (GIMBAL_DEG_X10_PER_REV / 2)) /
            GIMBAL_DEG_X10_PER_REV;
    }
    return (numerator - (GIMBAL_DEG_X10_PER_REV / 2)) /
        GIMBAL_DEG_X10_PER_REV;
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
    g_feedback.target_rpm_x10 = rpm_to_x10(g_yawTargetRpm);
    g_feedback.commanded_rpm_x10 = rpm_to_x10(g_yawCommandedRpm);
    g_feedback.turn_count = g_yawTurnCount;
    g_feedback.step_half_period_ticks = src->step_half_period_ticks;
    g_yawCarYawDegX10 = deg_to_x10(Imu_GetYaw());
    g_feedback.car_yaw_deg_x10 = clamp_i16(g_yawCarYawDegX10);
    g_feedback.locked_world_yaw_deg_x10 =
        clamp_i16(g_yawLockedWorldYawDegX10);
    g_feedback.world_target_deg_x10 = clamp_i16(g_yawWorldTargetDegX10);
    g_feedback.min_limit_deg_x10 = 0;
    g_feedback.max_limit_deg_x10 = 0;
    g_feedback.enabled = src->enabled;
    g_feedback.direction = src->direction;
    g_feedback.running = src->running;
    g_feedback.target_reached = src->target_reached;
    g_feedback.limit_clamped = 0U;
    g_feedback.position_valid = g_yawPositionValid;
    g_feedback.world_lock_enabled = g_yawWorldLockEnabled;
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
    g_yawTargetRpm = 0.0f;
    g_yawCommandedRpm = 0.0f;
    g_yawWorldLockEnabled = 0U;
    g_yawCarYawDegX10 = 0;
    g_yawLockedWorldYawDegX10 = 0;
    g_yawWorldTargetDegX10 = 0;
    GimbalStepperTest_Init();
    gimbal_update_feedback();
}

void Gimbal_YawTick100us(void)
{
    GimbalStepperTest_Tick100us();
}

void Gimbal_YawUpdate5ms(void)
{
    const GimbalStepperTestFeedback *src;
    float rpm_diff;
    float max_delta_rpm;
    float max_target_rpm;
    float accel_limit_rpm_per_s;
    uint16_t step_half_period_ticks = 0U;

    g_feedback.control_tick_5ms++;
    gimbal_update_feedback();

    if (g_yawWorldLockEnabled != 0U) {
        g_yawCarYawDegX10 = deg_to_x10(Imu_GetYaw());
        g_yawWorldTargetDegX10 =
            g_yawLockedWorldYawDegX10 - g_yawCarYawDegX10;
        g_yawTargetContinuousDegX10 = g_yawWorldTargetDegX10;
        GimbalStepperTest_MoveToEstimatedSteps(
            deg_x10_to_steps(g_yawTargetContinuousDegX10));
        gimbal_update_feedback();
    }

    src = GimbalStepperTest_GetFeedback();
    if (g_yawWorldLockEnabled != 0U) {
        max_target_rpm = GIMBAL_MAX_LOCK_RPM;
        accel_limit_rpm_per_s = GIMBAL_LOCK_ACCEL_LIMIT_RPM_PER_S;
    } else {
        max_target_rpm = GIMBAL_MAX_POSITION_RPM;
        accel_limit_rpm_per_s = GIMBAL_ACCEL_LIMIT_RPM_PER_S;
    }

    if (src->running) {
        g_yawTargetRpm =
            (src->direction >= 0) ?
                max_target_rpm : -max_target_rpm;
    } else {
        g_yawTargetRpm = 0.0f;
    }

    max_delta_rpm = accel_limit_rpm_per_s * GIMBAL_CONTROL_DT_S;
    rpm_diff = g_yawTargetRpm - g_yawCommandedRpm;
    if (rpm_diff > max_delta_rpm) {
        rpm_diff = max_delta_rpm;
    } else if (rpm_diff < -max_delta_rpm) {
        rpm_diff = -max_delta_rpm;
    }
    g_yawCommandedRpm += rpm_diff;

    if (!src->running && (abs_f32(g_yawCommandedRpm) <
        GIMBAL_MIN_EFFECTIVE_RPM)) {
        g_yawCommandedRpm = 0.0f;
    }

    if (src->running &&
        (rpm_matches_direction(g_yawCommandedRpm, src->direction) != 0U)) {
        step_half_period_ticks =
            rpm_to_half_period_ticks(g_yawCommandedRpm);
    }

    GimbalStepperTest_SetStepHalfPeriodTicks(step_half_period_ticks);
    gimbal_update_feedback();
}

static void gimbal_yaw_set_target_x10(int64_t target_continuous_deg_x10,
    uint8_t reset_speed)
{
    gimbal_update_feedback();
    g_yawTargetContinuousDegX10 = target_continuous_deg_x10;
    g_yawWorldTargetDegX10 = target_continuous_deg_x10;
    if (reset_speed != 0U) {
        g_yawTargetRpm = 0.0f;
        g_yawCommandedRpm = 0.0f;
        GimbalStepperTest_SetStepHalfPeriodTicks(0U);
    }
    GimbalStepperTest_MoveToEstimatedSteps(
        deg_x10_to_steps(g_yawTargetContinuousDegX10));
    gimbal_update_feedback();
}

static void gimbal_yaw_move_to_x10(int64_t target_continuous_deg_x10)
{
    g_yawWorldLockEnabled = 0U;
    gimbal_yaw_set_target_x10(target_continuous_deg_x10, 1U);
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

void Gimbal_YawSetWrappedTargetDeg(float target_deg)
{
    int64_t target_wrapped_deg_x10;
    int64_t error_deg_x10;

    gimbal_update_feedback();
    target_wrapped_deg_x10 = wrap_x10_360(deg_to_x10(target_deg));
    error_deg_x10 = shortest_error_x10(target_wrapped_deg_x10,
        g_yawWrappedDegX10);
    gimbal_yaw_set_target_x10(g_yawContinuousDegX10 + error_deg_x10, 0U);
}

void Gimbal_YawMoveRelativeDeg(float delta_deg)
{
    gimbal_update_feedback();
    gimbal_yaw_move_to_x10(g_yawTargetContinuousDegX10 +
        deg_to_x10(delta_deg));
}

void Gimbal_YawEnableWorldLock(void)
{
    gimbal_update_feedback();
    g_yawCarYawDegX10 = deg_to_x10(Imu_GetYaw());
    g_yawLockedWorldYawDegX10 = g_yawCarYawDegX10 + g_yawContinuousDegX10;
    g_yawWorldTargetDegX10 = g_yawContinuousDegX10;
    g_yawTargetContinuousDegX10 = g_yawContinuousDegX10;
    g_yawWorldLockEnabled = 1U;
    GimbalStepperTest_MoveToEstimatedSteps(
        deg_x10_to_steps(g_yawTargetContinuousDegX10));
    GimbalStepperTest_StopHold();
    gimbal_update_feedback();
}

void Gimbal_YawDisableWorldLock(void)
{
    g_yawWorldLockEnabled = 0U;
    Gimbal_YawStopHold();
}

void Gimbal_YawToggleWorldLock(void)
{
    if (g_yawWorldLockEnabled != 0U) {
        Gimbal_YawDisableWorldLock();
    } else {
        Gimbal_YawEnableWorldLock();
    }
}

void Gimbal_YawStopHold(void)
{
    g_yawWorldLockEnabled = 0U;
    gimbal_update_feedback();
    g_yawTargetContinuousDegX10 = g_yawContinuousDegX10;
    g_yawWorldTargetDegX10 = g_yawTargetContinuousDegX10;
    GimbalStepperTest_MoveToEstimatedSteps(
        deg_x10_to_steps(g_yawTargetContinuousDegX10));
    GimbalStepperTest_StopHold();
    g_yawTargetRpm = 0.0f;
    g_yawCommandedRpm = 0.0f;
    GimbalStepperTest_SetStepHalfPeriodTicks(0U);
    gimbal_update_feedback();
}

void Gimbal_YawRelease(void)
{
    gimbal_update_feedback();
    g_yawWorldLockEnabled = 0U;
    g_yawTargetContinuousDegX10 = g_yawContinuousDegX10;
    g_yawWorldTargetDegX10 = g_yawTargetContinuousDegX10;
    g_yawTargetRpm = 0.0f;
    g_yawCommandedRpm = 0.0f;
    GimbalStepperTest_SetStepHalfPeriodTicks(0U);
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

void Gimbal_SetWrappedTargetDeg(float target_deg)
{
    Gimbal_YawSetWrappedTargetDeg(target_deg);
}

void Gimbal_MoveRelativeDeg(float delta_deg)
{
    Gimbal_YawMoveRelativeDeg(delta_deg);
}

void Gimbal_EnableWorldLock(void)
{
    Gimbal_YawEnableWorldLock();
}

void Gimbal_DisableWorldLock(void)
{
    Gimbal_YawDisableWorldLock();
}

void Gimbal_ToggleWorldLock(void)
{
    Gimbal_YawToggleWorldLock();
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
