#include "motor.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_LEVELS        (100U)

#ifndef MOTOR_A_INVERT_DIRECTION
#define MOTOR_A_INVERT_DIRECTION (0)
#endif

#ifndef MOTOR_B_INVERT_DIRECTION
#define MOTOR_B_INVERT_DIRECTION (0)
#endif

static volatile uint8_t g_pwmPhase;
static volatile uint8_t g_motorADutyLevel;
static volatile uint8_t g_motorBDutyLevel;

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

static uint8_t duty_to_level(int16_t speed)
{
    uint16_t duty = (speed < 0) ? (uint16_t)(-speed) : (uint16_t)speed;

    return (uint8_t)((duty + 5U) / 10U);
}

static void motor_a_set_direction(int16_t speed)
{
    if (MOTOR_A_INVERT_DIRECTION) {
        speed = -speed;
    }

    if (speed > 0) {
        DL_GPIO_setPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_A_PORT, GPIO_TB6612_A_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_A_PORT,
            GPIO_TB6612_A_AIN1_PIN | GPIO_TB6612_A_AIN2_PIN);
    }
}

static void motor_b_set_direction(int16_t speed)
{
    if (MOTOR_B_INVERT_DIRECTION) {
        speed = -speed;
    }

    if (speed > 0) {
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT,
            GPIO_TB6612_B_BIN1_PIN | GPIO_TB6612_B_BIN2_PIN);
    }
}

void Motor_Init(void)
{
    g_pwmPhase = 0;
    g_motorADutyLevel = 0;
    g_motorBDutyLevel = 0;
    Motor_Stop();
}

void Motor_SetSpeed(int16_t motorA, int16_t motorB)
{
    motorA = clamp_speed(motorA);
    motorB = clamp_speed(motorB);

    motor_a_set_direction(motorA);
    motor_b_set_direction(motorB);

    g_motorADutyLevel = duty_to_level(motorA);
    g_motorBDutyLevel = duty_to_level(motorB);
}

void Motor_Stop(void)
{
    g_motorADutyLevel = 0;
    g_motorBDutyLevel = 0;

    DL_GPIO_clearPins(GPIO_TB6612_B_PORT,
        GPIO_TB6612_B_PWMA_PIN | GPIO_TB6612_B_PWMB_PIN);
    DL_GPIO_clearPins(GPIO_TB6612_A_PORT,
        GPIO_TB6612_A_AIN1_PIN | GPIO_TB6612_A_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_TB6612_B_PORT,
        GPIO_TB6612_B_BIN1_PIN | GPIO_TB6612_B_BIN2_PIN);
}

void Motor_PwmTick100us(void)
{
    g_pwmPhase++;
    if (g_pwmPhase >= MOTOR_PWM_LEVELS) {
        g_pwmPhase = 0;
    }

    if (g_pwmPhase < g_motorADutyLevel) {
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_PWMA_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_PWMA_PIN);
    }

    if (g_pwmPhase < g_motorBDutyLevel) {
        DL_GPIO_setPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_PWMB_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_B_PORT, GPIO_TB6612_B_PWMB_PIN);
    }
}
