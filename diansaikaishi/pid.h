#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t scale;
    int32_t integral;
    int32_t lastError;
    int32_t integralLimit;
    int32_t outputLimit;
} PID_Controller;

void PID_Init(PID_Controller *pid, int32_t kp, int32_t ki, int32_t kd,
    int32_t scale, int32_t outputLimit, int32_t integralLimit);
void PID_Reset(PID_Controller *pid);
int32_t PID_Update(PID_Controller *pid, int32_t error);

#endif
