#include "gimbal.h"

#include "gimbal_stepper.h"
#include "imu.h"
#include "ti_msp_dl_config.h"

#define GIMBAL_STEPS_PER_REV   (3200LL)
#define GIMBAL_DEG_X10_PER_REV (3600LL)
#define GIMBAL_CONTROL_DT_S    (0.005f)
#define GIMBAL_MAX_POSITION_RPM (4.0f)
#define GIMBAL_MAX_LOCK_RPM    (12.0f)
#define GIMBAL_MIN_EFFECTIVE_RPM (0.5f)
#define GIMBAL_ACCEL_LIMIT_RPM_PER_S (10.0f)
#define GIMBAL_LOCK_ACCEL_LIMIT_RPM_PER_S (30.0f)
#define GIMBAL_TICK_HZ         (10000.0f)
#define GIMBAL_PITCH_STEPS_PER_REV (3200LL)
#define GIMBAL_PITCH_STEP_HALF_PERIOD_TICKS (25U)

#ifndef GPIO_GIMBAL_PITCH_PORT
#define GPIO_GIMBAL_PITCH_PORT                 (GPIOB)
#endif

#ifndef GPIO_GIMBAL_PITCH_STEP_PIN
#define GPIO_GIMBAL_PITCH_STEP_PIN             (DL_GPIO_PIN_5)
#endif

#ifndef GPIO_GIMBAL_PITCH_STEP_IOMUX
#define GPIO_GIMBAL_PITCH_STEP_IOMUX           (IOMUX_PINCM18)
#endif

#ifndef GPIO_GIMBAL_PITCH_DIR_PIN
#define GPIO_GIMBAL_PITCH_DIR_PIN              (DL_GPIO_PIN_6)
#endif

#ifndef GPIO_GIMBAL_PITCH_DIR_IOMUX
#define GPIO_GIMBAL_PITCH_DIR_IOMUX            (IOMUX_PINCM23)
#endif

#ifndef GPIO_GIMBAL_PITCH_EN_PIN
#define GPIO_GIMBAL_PITCH_EN_PIN               (DL_GPIO_PIN_7)
#endif

#ifndef GPIO_GIMBAL_PITCH_EN_IOMUX
#define GPIO_GIMBAL_PITCH_EN_IOMUX             (IOMUX_PINCM24)
#endif

#define GIMBAL_PITCH_POSITIVE_DIR_HIGH         (1)
#define GIMBAL_PITCH_EN_ACTIVE_LOW             (0)

static GimbalFeedback g_feedback;
static GimbalFeedback g_pitchFeedback;
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
static uint8_t g_pitchInitialized;
static int64_t g_pitchContinuousDegX10;
static int64_t g_pitchTargetDegX10;
static int64_t g_pitchEstimatedSteps;
static int64_t g_pitchTargetEstimatedSteps;
static int32_t g_pitchCompletedSteps;
static uint16_t g_pitchHalfPeriodTicks;
static int8_t g_pitchDirection;
static uint8_t g_pitchStepHigh;
static uint8_t g_pitchRunning;
static uint8_t g_pitchStopAfterStepLow;

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

static int64_t pitch_deg_x10_to_steps(int64_t deg_x10)
{
    int64_t numerator = deg_x10 * GIMBAL_PITCH_STEPS_PER_REV;

    if (numerator >= 0) {
        return (numerator + (GIMBAL_DEG_X10_PER_REV / 2)) /
            GIMBAL_DEG_X10_PER_REV;
    }
    return (numerator - (GIMBAL_DEG_X10_PER_REV / 2)) /
        GIMBAL_DEG_X10_PER_REV;
}

static void gimbal_pitch_ensure_initialized(void);

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
    const GimbalStepperFeedback *src = GimbalStepper_GetFeedback();
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

static void gimbal_pitch_reset_feedback(void)
{
    g_pitchFeedback = (GimbalFeedback){0};
    g_pitchFeedback.mode = GIMBAL_MODE_RELEASED;
    g_pitchFeedback.target_reached = 1U;
    g_pitchFeedback.position_valid = 0U;
    g_pitchInitialized = 1U;
}

static void gimbal_pitch_set_enable(uint8_t enable)
{
#if GIMBAL_PITCH_EN_ACTIVE_LOW
    if (enable != 0U) {
        DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_EN_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_EN_PIN);
    }
#else
    if (enable != 0U) {
        DL_GPIO_setPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_EN_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_EN_PIN);
    }
#endif
    g_pitchFeedback.enabled = enable;
}

static void gimbal_pitch_set_dir(int8_t direction)
{
#if GIMBAL_PITCH_POSITIVE_DIR_HIGH
    if (direction >= 0) {
        DL_GPIO_setPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_DIR_PIN);
    }
#else
    if (direction >= 0) {
        DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_DIR_PIN);
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_DIR_PIN);
    }
#endif
}

static void gimbal_pitch_update_feedback(void)
{
    g_pitchContinuousDegX10 =
        steps_to_deg_x10(g_pitchEstimatedSteps);
    g_pitchFeedback.estimated_steps = g_pitchEstimatedSteps;
    g_pitchFeedback.target_steps = (int32_t)(
        g_pitchTargetEstimatedSteps - g_pitchEstimatedSteps);
    g_pitchFeedback.completed_steps =
        (g_pitchDirection >= 0) ?
            g_pitchCompletedSteps : -g_pitchCompletedSteps;
    g_pitchFeedback.target_deg_x10 = clamp_i16(g_pitchTargetDegX10);
    g_pitchFeedback.completed_deg_x10 =
        clamp_i16(g_pitchContinuousDegX10);
    g_pitchFeedback.continuous_deg_x10 =
        clamp_i16(g_pitchContinuousDegX10);
    g_pitchFeedback.wrapped_deg_x10 =
        clamp_i16(wrap_x10_360(g_pitchContinuousDegX10));
    g_pitchFeedback.step_half_period_ticks =
        g_pitchRunning ? GIMBAL_PITCH_STEP_HALF_PERIOD_TICKS : 0U;
    g_pitchFeedback.direction = g_pitchDirection;
    g_pitchFeedback.running = g_pitchRunning;
    g_pitchFeedback.position_valid = 1U;
    if (g_pitchRunning != 0U) {
        g_pitchFeedback.mode = GIMBAL_MODE_MOVING;
    } else if (g_pitchFeedback.enabled != 0U) {
        g_pitchFeedback.mode = GIMBAL_MODE_HOLDING;
    } else {
        g_pitchFeedback.mode = GIMBAL_MODE_RELEASED;
    }
}

static void gimbal_pitch_stop_hold_internal(void)
{
    DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_STEP_PIN);
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHigh = 0U;
    g_pitchRunning = 0U;
    g_pitchStopAfterStepLow = 0U;
    g_pitchFeedback.running = 0U;
    g_pitchFeedback.target_reached =
        (g_pitchEstimatedSteps == g_pitchTargetEstimatedSteps) ? 1U : 0U;
    gimbal_pitch_update_feedback();
}

static void gimbal_pitch_move_to_x10(int64_t target_deg_x10)
{
    int64_t delta_steps;

    gimbal_pitch_ensure_initialized();
    g_pitchTargetDegX10 = target_deg_x10;
    g_pitchTargetEstimatedSteps =
        pitch_deg_x10_to_steps(g_pitchTargetDegX10);
    delta_steps = g_pitchTargetEstimatedSteps - g_pitchEstimatedSteps;

    if (delta_steps == 0) {
        gimbal_pitch_stop_hold_internal();
        g_pitchCompletedSteps = 0;
        g_pitchFeedback.completed_steps = 0;
        g_pitchFeedback.target_reached = 1U;
        return;
    }

    g_pitchDirection = (delta_steps < 0) ? -1 : 1;
    g_pitchCompletedSteps = 0;
    g_pitchHalfPeriodTicks = 0U;
    g_pitchStepHigh = 0U;
    g_pitchStopAfterStepLow = 0U;
    g_pitchFeedback.target_reached = 0U;
    gimbal_pitch_set_dir(g_pitchDirection);
    gimbal_pitch_set_enable(1U);
    g_pitchRunning = 1U;
    gimbal_pitch_update_feedback();
}

static void gimbal_pitch_ensure_initialized(void)
{
    if (g_pitchInitialized == 0U) {
        gimbal_pitch_reset_feedback();
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
    GimbalStepper_Init();
    gimbal_update_feedback();
}

void Gimbal_YawTick100us(void)
{
    GimbalStepper_Tick100us();
}

void Gimbal_YawUpdate5ms(void)
{
    const GimbalStepperFeedback *src;
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
        GimbalStepper_MoveToEstimatedSteps(
            deg_x10_to_steps(g_yawTargetContinuousDegX10));
        gimbal_update_feedback();
    }

    src = GimbalStepper_GetFeedback();
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

    GimbalStepper_SetStepHalfPeriodTicks(step_half_period_ticks);
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
        GimbalStepper_SetStepHalfPeriodTicks(0U);
    }
    GimbalStepper_MoveToEstimatedSteps(
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
    GimbalStepper_MoveToEstimatedSteps(
        deg_x10_to_steps(g_yawTargetContinuousDegX10));
    GimbalStepper_StopHold();
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
    GimbalStepper_MoveToEstimatedSteps(
        deg_x10_to_steps(g_yawTargetContinuousDegX10));
    GimbalStepper_StopHold();
    g_yawTargetRpm = 0.0f;
    g_yawCommandedRpm = 0.0f;
    GimbalStepper_SetStepHalfPeriodTicks(0U);
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
    GimbalStepper_SetStepHalfPeriodTicks(0U);
    GimbalStepper_Release();
    gimbal_update_feedback();
}

const GimbalFeedback *Gimbal_YawGetFeedback(void)
{
    gimbal_update_feedback();
    return &g_feedback;
}

void Gimbal_PitchInit(void)
{
    g_pitchContinuousDegX10 = 0;
    g_pitchTargetDegX10 = 0;
    g_pitchEstimatedSteps = 0;
    g_pitchTargetEstimatedSteps = 0;
    g_pitchCompletedSteps = 0;
    g_pitchHalfPeriodTicks = 0U;
    g_pitchDirection = 1;
    g_pitchStepHigh = 0U;
    g_pitchRunning = 0U;
    g_pitchStopAfterStepLow = 0U;
    gimbal_pitch_reset_feedback();

    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_PITCH_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_PITCH_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GIMBAL_PITCH_EN_IOMUX);

    DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_STEP_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_DIR_PIN);
    DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT, GPIO_GIMBAL_PITCH_EN_PIN);
    DL_GPIO_enableOutput(GPIO_GIMBAL_PITCH_PORT,
        GPIO_GIMBAL_PITCH_STEP_PIN | GPIO_GIMBAL_PITCH_DIR_PIN |
        GPIO_GIMBAL_PITCH_EN_PIN);
    gimbal_pitch_set_enable(0U);
    gimbal_pitch_update_feedback();
}

void Gimbal_PitchTick100us(void)
{
    gimbal_pitch_ensure_initialized();

    if (g_pitchRunning == 0U) {
        return;
    }

    g_pitchHalfPeriodTicks++;
    if (g_pitchHalfPeriodTicks < GIMBAL_PITCH_STEP_HALF_PERIOD_TICKS) {
        return;
    }

    g_pitchHalfPeriodTicks = 0U;
    if (g_pitchStepHigh != 0U) {
        DL_GPIO_clearPins(GPIO_GIMBAL_PITCH_PORT,
            GPIO_GIMBAL_PITCH_STEP_PIN);
        g_pitchStepHigh = 0U;
        if (g_pitchStopAfterStepLow != 0U) {
            gimbal_pitch_stop_hold_internal();
        }
    } else {
        DL_GPIO_setPins(GPIO_GIMBAL_PITCH_PORT,
            GPIO_GIMBAL_PITCH_STEP_PIN);
        g_pitchStepHigh = 1U;
        g_pitchCompletedSteps++;
        g_pitchEstimatedSteps += g_pitchDirection;
        if (((g_pitchDirection > 0) &&
                (g_pitchEstimatedSteps >= g_pitchTargetEstimatedSteps)) ||
            ((g_pitchDirection < 0) &&
                (g_pitchEstimatedSteps <= g_pitchTargetEstimatedSteps))) {
            g_pitchFeedback.target_reached = 1U;
            g_pitchStopAfterStepLow = 1U;
        }
        gimbal_pitch_update_feedback();
    }
}

void Gimbal_PitchUpdate5ms(void)
{
    gimbal_pitch_ensure_initialized();
    g_pitchFeedback.control_tick_5ms++;
    gimbal_pitch_update_feedback();
}

void Gimbal_PitchMoveToDeg(float target_deg)
{
    gimbal_pitch_move_to_x10(deg_to_x10(target_deg));
}

void Gimbal_PitchMoveRelativeDeg(float delta_deg)
{
    gimbal_pitch_ensure_initialized();
    gimbal_pitch_update_feedback();
    gimbal_pitch_move_to_x10(g_pitchTargetDegX10 + deg_to_x10(delta_deg));
}

void Gimbal_PitchStopHold(void)
{
    gimbal_pitch_ensure_initialized();
    g_pitchTargetDegX10 = g_pitchContinuousDegX10;
    g_pitchTargetEstimatedSteps = g_pitchEstimatedSteps;
    gimbal_pitch_stop_hold_internal();
    gimbal_pitch_set_enable(1U);
    gimbal_pitch_update_feedback();
}

void Gimbal_PitchRelease(void)
{
    gimbal_pitch_ensure_initialized();
    g_pitchTargetDegX10 = g_pitchContinuousDegX10;
    g_pitchTargetEstimatedSteps = g_pitchEstimatedSteps;
    g_pitchCompletedSteps = 0;
    gimbal_pitch_stop_hold_internal();
    gimbal_pitch_set_enable(0U);
    gimbal_pitch_update_feedback();
}

const GimbalFeedback *Gimbal_PitchGetFeedback(void)
{
    gimbal_pitch_ensure_initialized();
    return &g_pitchFeedback;
}

void Gimbal_Init(void)
{
    Gimbal_YawInit();
    Gimbal_PitchInit();
}

void Gimbal_Tick100us(void)
{
    Gimbal_YawTick100us();
    Gimbal_PitchTick100us();
}

void Gimbal_Update5ms(void)
{
    Gimbal_YawUpdate5ms();
    Gimbal_PitchUpdate5ms();
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
