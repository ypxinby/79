#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_CONTROL_ERROR_NONE = 0U,
    MOTOR_CONTROL_ERROR_INVALID_CONFIG = (1UL << 0),
    MOTOR_CONTROL_ERROR_INVALID_DT = (1UL << 1),
    MOTOR_CONTROL_ERROR_ESTIMATOR_INVALID = (1UL << 2),
    MOTOR_CONTROL_ERROR_ESTIMATOR_STALE = (1UL << 3),
    MOTOR_CONTROL_ERROR_ESTIMATOR_FAULT = (1UL << 4),
    MOTOR_CONTROL_ERROR_TARGET_TIMEOUT = (1UL << 5),
    MOTOR_CONTROL_ERROR_SAFETY_INHIBIT = (1UL << 6),
    MOTOR_CONTROL_ERROR_FEEDBACK_RANGE = (1UL << 7)
} MotorControlError;

typedef struct {
    int16_t normalized_target;
    float raw_target_speed_cmps;
    float ramped_target_speed_cmps;
    float measured_speed_cmps;
    float error_cmps;
    float integral;
    float proportional_term;
    float integral_term;
    float feedforward_term;
    int16_t output_command;
    bool saturated;
    bool direction_change_pending;
} MotorControlWheelRuntime;

typedef struct {
    bool enabled;
    bool valid;
    bool error_latched;
    bool safety_inhibited;
    bool target_refresh_timeout;
    uint32_t error_flags;
    uint32_t estimator_error_flags;
    uint32_t target_age_ms;
    uint32_t sample_dt_ms;
    uint32_t update_count;
    MotorControlWheelRuntime left;
    MotorControlWheelRuntime right;
} MotorControlRuntime;

void MotorControl_Init(void);
void MotorControl_SetNormalizedTarget(int16_t left_command,
    int16_t right_command);
void MotorControl_SetSpeedTargetCmps(float left_target_cmps,
    float right_target_cmps);
void MotorControl_Update(uint32_t elapsed_ms);
void MotorControl_Stop(void);
void MotorControl_Reset(void);
const MotorControlRuntime *MotorControl_GetRuntime(void);

#endif
