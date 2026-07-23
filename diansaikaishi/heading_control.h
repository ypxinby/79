#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float target_yaw_deg;
    float heading_error_deg;
    float last_heading_error_deg;
    float heading_derivative;
    float update_dt_s;

    int16_t correction;

    bool enabled;
    bool target_locked;
    bool dt_valid;
} HeadingControlRuntime;

void HeadingControl_Init(void);
void HeadingControl_Reset(void);
void HeadingControl_Enable(bool enable);
void HeadingControl_LockCurrentYaw(float current_yaw_deg);
void HeadingControl_SetTargetYaw(float target_yaw_deg);
int16_t HeadingControl_Update(float current_yaw_deg, float gyro_z_dps,
    float dt_s);
const HeadingControlRuntime *HeadingControl_GetRuntime(void);

#endif
