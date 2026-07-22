#include "motor.h"

#include "app_features.h"
#include "ti_msp_dl_config.h"

#define MOTOR_SW_PWM_LEVELS             (100U)

/* TIMG8 uses 32 MHz BUSCLK and counts 0..1599 for a 20 kHz PWM period. */
#define MOTOR_HW_PWM_TIMER_CLOCK_HZ     (32000000U)
#define MOTOR_HW_PWM_FREQUENCY_HZ       (20000U)
#define MOTOR_HW_PWM_PERIOD_COUNTS      \
    (MOTOR_HW_PWM_TIMER_CLOCK_HZ / MOTOR_HW_PWM_FREQUENCY_HZ)
#define MOTOR_HW_PWM_LOAD_VALUE         (MOTOR_HW_PWM_PERIOD_COUNTS - 1U)

#if FEATURE_HW_MOTOR_PWM && \
    (PWM_MOTOR_INST_CLK_FREQ != MOTOR_HW_PWM_TIMER_CLOCK_HZ)
#error PWM_MOTOR timer clock does not match the motor PWM calculation
#endif

#ifndef MOTOR_A_INVERT_DIRECTION
#define MOTOR_A_INVERT_DIRECTION (0)
#endif

#ifndef MOTOR_B_INVERT_DIRECTION
#define MOTOR_B_INVERT_DIRECTION (0)
#endif

#if !FEATURE_HW_MOTOR_PWM
static volatile uint8_t g_pwmPhase;
static volatile uint8_t g_motorADutyLevel;
static volatile uint8_t g_motorBDutyLevel;
#endif
static volatile MotorRuntime g_motorRuntime;

static int16_t clamp_speed(int16_t speed)
{
    if (speed > MOTOR_MAX_DUTY) {
        return MOTOR_MAX_DUTY;
    }
    if (speed < -MOTOR_MAX_DUTY) {
        return -MOTOR_MAX_DUTY;
    }
    return speed;
}

static uint16_t speed_magnitude(int16_t speed)
{
    return (speed < 0) ? (uint16_t)(-speed) : (uint16_t)speed;
}

static int8_t speed_to_direction(int16_t speed, bool invert)
{
    int8_t direction = 0;

    if (speed > 0) {
        direction = 1;
    } else if (speed < 0) {
        direction = -1;
    }

    return invert ? (int8_t)-direction : direction;
}

#if !FEATURE_HW_MOTOR_PWM
static uint8_t duty_to_level(int16_t speed)
{
    uint16_t duty = speed_magnitude(speed);

    return (uint8_t)((duty + 5U) / 10U);
}
#else
static uint16_t duty_to_compare(int16_t speed)
{
    /* EDGE_ALIGN_UP duty is (compare + 1) / period; zero stays forced low. */
    uint32_t activeCounts =
        ((uint32_t)speed_magnitude(speed) * MOTOR_HW_PWM_PERIOD_COUNTS +
            (MOTOR_MAX_DUTY / 2U)) /
        MOTOR_MAX_DUTY;

    if (activeCounts == 0U) {
        return 0U;
    }
    if (activeCounts >= MOTOR_HW_PWM_PERIOD_COUNTS) {
        return (uint16_t)MOTOR_HW_PWM_LOAD_VALUE;
    }

    return (uint16_t)(activeCounts - 1U);
}
#endif

static void motor_a_set_direction(int8_t direction)
{
    if (direction > 0) {
        DL_GPIO_setPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN2_PIN);
    } else if (direction < 0) {
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT,
            GPIO_TB6612_A_AIN1_PIN | GPIO_TB6612_A_AIN2_PIN);
    }
}

static void motor_b_set_direction(int8_t direction)
{
    if (direction > 0) {
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN2_PIN);
    } else if (direction < 0) {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT,
            GPIO_TB6612_B_BIN1_PIN | GPIO_TB6612_B_BIN2_PIN);
    }
}

#if FEATURE_HW_MOTOR_PWM
static void motor_hw_force_low(DL_TIMER_CC_INDEX channel)
{
    DL_Timer_overrideCCPOut(PWM_MOTOR_INST, DL_TIMER_FORCE_OUT_LOW,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, channel);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0U, channel);
}

static void motor_hw_apply(
    DL_TIMER_CC_INDEX channel, int16_t speed, uint16_t compare)
{
    if (speed == 0) {
        motor_hw_force_low(channel);
        return;
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, compare, channel);
    DL_Timer_overrideCCPOut(PWM_MOTOR_INST, DL_TIMER_FORCE_OUT_DISABLED,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, channel);
}
#else
static void motor_sw_force_a_low(void)
{
    g_motorADutyLevel = 0U;
    DL_GPIO_clearPins(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
}

static void motor_sw_force_b_low(void)
{
    g_motorBDutyLevel = 0U;
    DL_GPIO_clearPins(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
}
#endif

static void motor_a_force_low(void)
{
#if FEATURE_HW_MOTOR_PWM
    motor_hw_force_low(GPIO_PWM_MOTOR_C0_IDX);
#else
    motor_sw_force_a_low();
#endif
    g_motorRuntime.pwm_compare_a = 0U;
}

static void motor_b_force_low(void)
{
#if FEATURE_HW_MOTOR_PWM
    motor_hw_force_low(GPIO_PWM_MOTOR_C1_IDX);
#else
    motor_sw_force_b_low();
#endif
    g_motorRuntime.pwm_compare_b = 0U;
}

static void motor_a_apply_output(int16_t speed)
{
#if FEATURE_HW_MOTOR_PWM
    uint16_t compare = duty_to_compare(speed);

    motor_hw_apply(GPIO_PWM_MOTOR_C0_IDX, speed, compare);
    g_motorRuntime.pwm_compare_a = compare;
#else
    uint8_t dutyLevel = duty_to_level(speed);

    g_motorADutyLevel = dutyLevel;
    g_motorRuntime.pwm_compare_a = dutyLevel;
#endif
}

static void motor_b_apply_output(int16_t speed)
{
#if FEATURE_HW_MOTOR_PWM
    uint16_t compare = duty_to_compare(speed);

    motor_hw_apply(GPIO_PWM_MOTOR_C1_IDX, speed, compare);
    g_motorRuntime.pwm_compare_b = compare;
#else
    uint8_t dutyLevel = duty_to_level(speed);

    g_motorBDutyLevel = dutyLevel;
    g_motorRuntime.pwm_compare_b = dutyLevel;
#endif
}

void Motor_Init(void)
{
#if FEATURE_HW_MOTOR_PWM
    DL_TimerG_stopCounter(PWM_MOTOR_INST);
    motor_a_force_low();
    motor_b_force_low();
#else
    g_pwmPhase = 0U;
    g_motorADutyLevel = 0U;
    g_motorBDutyLevel = 0U;

    DL_GPIO_clearPins(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    DL_GPIO_clearPins(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
    DL_GPIO_initDigitalOutput(GPIO_PWM_MOTOR_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_PWM_MOTOR_C1_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
#endif

    motor_a_set_direction(0);
    motor_b_set_direction(0);

    g_motorRuntime.requested_motor_a = 0;
    g_motorRuntime.requested_motor_b = 0;
    g_motorRuntime.applied_motor_a = 0;
    g_motorRuntime.applied_motor_b = 0;
    g_motorRuntime.pwm_compare_a = 0U;
    g_motorRuntime.pwm_compare_b = 0U;
    g_motorRuntime.hardware_pwm_enabled = (FEATURE_HW_MOTOR_PWM != 0);
    g_motorRuntime.direction_a = 0;
    g_motorRuntime.direction_b = 0;

#if FEATURE_HW_MOTOR_PWM
    DL_TimerG_startCounter(PWM_MOTOR_INST);
#endif
}

void Motor_SetSpeed(int16_t motorA, int16_t motorB)
{
    uint32_t primask = __get_PRIMASK();
    int8_t directionA;
    int8_t directionB;

    __disable_irq();
    g_motorRuntime.requested_motor_a = motorA;
    g_motorRuntime.requested_motor_b = motorB;

    motorA = clamp_speed(motorA);
    motorB = clamp_speed(motorB);
    directionA = speed_to_direction(
        motorA, (MOTOR_A_INVERT_DIRECTION != 0));
    directionB = speed_to_direction(
        motorB, (MOTOR_B_INVERT_DIRECTION != 0));

    if (directionA != g_motorRuntime.direction_a) {
        motor_a_force_low();
        motor_a_set_direction(directionA);
    }
    if (directionB != g_motorRuntime.direction_b) {
        motor_b_force_low();
        motor_b_set_direction(directionB);
    }

    motor_a_apply_output(motorA);
    motor_b_apply_output(motorB);

    g_motorRuntime.applied_motor_a = motorA;
    g_motorRuntime.applied_motor_b = motorB;
    g_motorRuntime.direction_a = directionA;
    g_motorRuntime.direction_b = directionB;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Motor_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    motor_a_force_low();
    motor_b_force_low();
#if !FEATURE_HW_MOTOR_PWM
    g_pwmPhase = 0U;
    g_motorADutyLevel = 0;
    g_motorBDutyLevel = 0;
#endif

    motor_a_set_direction(0);
    motor_b_set_direction(0);
    g_motorRuntime.requested_motor_a = 0;
    g_motorRuntime.requested_motor_b = 0;
    g_motorRuntime.applied_motor_a = 0;
    g_motorRuntime.applied_motor_b = 0;
    g_motorRuntime.direction_a = 0;
    g_motorRuntime.direction_b = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Motor_PwmTick100us(void)
{
#if !FEATURE_HW_MOTOR_PWM
    g_pwmPhase++;
    if (g_pwmPhase >= MOTOR_SW_PWM_LEVELS) {
        g_pwmPhase = 0;
    }

    if (g_pwmPhase < g_motorADutyLevel) {
        DL_GPIO_setPins(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    }

    if (g_pwmPhase < g_motorBDutyLevel) {
        DL_GPIO_setPins(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
    }
#endif
}

const volatile MotorRuntime *Motor_GetRuntime(void)
{
    return &g_motorRuntime;
}
