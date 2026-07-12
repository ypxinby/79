#include "servo.h"

#include "ti_msp_dl_config.h"

#ifndef GPIO_SERVO_PORT
#define GPIO_SERVO_PORT                 (GPIOB)
#endif

#ifndef GPIO_SERVO_SERVO_PIN
#define GPIO_SERVO_SERVO_PIN            (DL_GPIO_PIN_9)
#endif

#define SERVO_MIN_ANGLE_DEG             (35)
#define SERVO_MAX_ANGLE_DEG             (145)
#define SERVO_MIN_PULSE_TICKS           (9U)
#define SERVO_MAX_PULSE_TICKS           (21U)
#define SERVO_PWM_PERIOD_TICKS          (200U)

static ServoFeedback g_servoFeedback;
static uint16_t g_servoPwmPhase;

static int16_t servo_clamp_angle(int16_t angle_deg)
{
    if (angle_deg < SERVO_MIN_ANGLE_DEG) {
        return SERVO_MIN_ANGLE_DEG;
    }
    if (angle_deg > SERVO_MAX_ANGLE_DEG) {
        return SERVO_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

static uint8_t servo_angle_to_pulse_ticks(int16_t angle_deg)
{
    int16_t clampedAngle = servo_clamp_angle(angle_deg);
    uint32_t numerator =
        (uint32_t)(clampedAngle - SERVO_MIN_ANGLE_DEG) *
        (uint32_t)(SERVO_MAX_PULSE_TICKS - SERVO_MIN_PULSE_TICKS);
    uint32_t denominator =
        (uint32_t)(SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);

    return (uint8_t)(SERVO_MIN_PULSE_TICKS +
        (uint8_t)((numerator + (denominator / 2U)) / denominator));
}

void Servo_Init(void)
{
    g_servoPwmPhase = 0U;
    g_servoFeedback.target_angle_deg = 90;
    g_servoFeedback.pulse_ticks_100us =
        servo_angle_to_pulse_ticks(g_servoFeedback.target_angle_deg);
    g_servoFeedback.output_high = false;

    DL_GPIO_clearPins(GPIO_SERVO_PORT, GPIO_SERVO_SERVO_PIN);
    DL_GPIO_enableOutput(GPIO_SERVO_PORT, GPIO_SERVO_SERVO_PIN);
}

void Servo_SetAngleDeg(int16_t angle_deg)
{
    g_servoFeedback.target_angle_deg = servo_clamp_angle(angle_deg);
    g_servoFeedback.pulse_ticks_100us =
        servo_angle_to_pulse_ticks(g_servoFeedback.target_angle_deg);
}

void Servo_Tick100us(void)
{
    g_servoPwmPhase++;
    if (g_servoPwmPhase >= SERVO_PWM_PERIOD_TICKS) {
        g_servoPwmPhase = 0U;
    }

    if (g_servoPwmPhase < g_servoFeedback.pulse_ticks_100us) {
        DL_GPIO_setPins(GPIO_SERVO_PORT, GPIO_SERVO_SERVO_PIN);
        g_servoFeedback.output_high = true;
    } else {
        DL_GPIO_clearPins(GPIO_SERVO_PORT, GPIO_SERVO_SERVO_PIN);
        g_servoFeedback.output_high = false;
    }
}

const ServoFeedback *Servo_GetFeedback(void)
{
    return &g_servoFeedback;
}
