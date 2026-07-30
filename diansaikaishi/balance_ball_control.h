#ifndef BALANCE_BALL_CONTROL_H
#define BALANCE_BALL_CONTROL_H

#include <stdint.h>

typedef enum {
    BALANCE_BALL_STATE_DISABLED = 0,
    BALANCE_BALL_STATE_WAIT_AXIS,
    BALANCE_BALL_STATE_WAIT_VISION,
    BALANCE_BALL_STATE_ACTIVE,
    BALANCE_BALL_STATE_RETURN_ZERO,
    BALANCE_BALL_STATE_VISION_LOST,
    BALANCE_BALL_STATE_FAULT
} BalanceBallControlState;

typedef struct {
    int16_t target_mm;
    int16_t position_mm;
    int16_t error_mm;
    int16_t velocity_mm_s;
    int32_t pd_output_count;
    int32_t commanded_offset_count;
    int32_t actuator_target_count;
    uint32_t last_measurement_time_ms;
    uint32_t accepted_measurement_count;
    uint32_t rejected_jump_count;
    uint32_t vision_lost_count;
    uint16_t axis_span_mm;
    uint16_t confidence;
    uint8_t enable_requested;
    uint8_t tracking_owned;
    uint8_t measurement_valid;
    uint8_t valid_streak;
    BalanceBallControlState state;
} BalanceBallControlRuntime;

void BalanceBallControl_Init(void);
uint8_t BalanceBallControl_Enable(int16_t target_mm);
void BalanceBallControl_Disable(void);
void BalanceBallControl_ForceStop(void);
uint8_t BalanceBallControl_SetTargetMm(int16_t target_mm);
void BalanceBallControl_Update20ms(uint32_t now_ms,
    uint32_t elapsed_ms);
void BalanceBallControl_GetSnapshot(BalanceBallControlRuntime *snapshot);

#endif
