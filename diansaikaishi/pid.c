#include "pid.h"

static int32_t clamp_i32(int32_t value, int32_t minValue, int32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

void PID_Init(PID_Controller *pid, int32_t kp, int32_t ki, int32_t kd,
    int32_t scale, int32_t outputLimit, int32_t integralLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->scale = scale;
    pid->outputLimit = outputLimit;
    pid->integralLimit = integralLimit;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0;
    pid->lastError = 0;
}

int32_t PID_Update(PID_Controller *pid, int32_t error)
{
    int32_t derivative;
    int32_t output;

    pid->integral += error;
    pid->integral =
        clamp_i32(pid->integral, -pid->integralLimit, pid->integralLimit);

    derivative = error - pid->lastError;
    pid->lastError = error;

    output = (pid->kp * error + pid->ki * pid->integral +
                 pid->kd * derivative) /
             pid->scale;

    return clamp_i32(output, -pid->outputLimit, pid->outputLimit);
}
