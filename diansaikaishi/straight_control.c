#include "straight_control.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

#define STRAIGHT_BASE_DUTY          (350)
#define STRAIGHT_MIN_DUTY           (180)
#define STRAIGHT_MAX_DUTY           (650)

/*
 * If correction makes the car bend harder instead of straighter, change this
 * value from 0 to 1.
 */
#define STRAIGHT_REVERSE_CORRECTION (0)

static PID_Controller g_straightPid;
static uint8_t g_running;

static int16_t clamp_duty(int32_t duty)
{
    if (duty > STRAIGHT_MAX_DUTY) {
        return STRAIGHT_MAX_DUTY;
    }
    if (duty < STRAIGHT_MIN_DUTY) {
        return STRAIGHT_MIN_DUTY;
    }
    return (int16_t)duty;
}

void StraightControl_Init(void)
{
    PID_Init(&g_straightPid, 1800, 35, 600, 100, 250, 2000);
    g_running = 0;
}

void StraightControl_Start(void)
{
    PID_Reset(&g_straightPid);
    Encoder_Reset();
    g_running = 1;
    Motor_SetSpeed(STRAIGHT_BASE_DUTY, STRAIGHT_BASE_DUTY);
}

void StraightControl_Stop(void)
{
    g_running = 0;
    Motor_Stop();
}

void StraightControl_Update(void)
{
    int32_t motorA;
    int32_t motorB;
    int32_t error;
    int32_t correction;

    if (g_running == 0U) {
        return;
    }

    Encoder_GetAndClearPulseDeltas(&motorA, &motorB);

    error = motorA - motorB;
    correction = PID_Update(&g_straightPid, error);

    if (STRAIGHT_REVERSE_CORRECTION) {
        correction = -correction;
    }

    Motor_SetSpeed(clamp_duty(STRAIGHT_BASE_DUTY - correction),
        clamp_duty(STRAIGHT_BASE_DUTY + correction));
}
