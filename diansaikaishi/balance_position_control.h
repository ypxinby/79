#ifndef BALANCE_POSITION_CONTROL_H
#define BALANCE_POSITION_CONTROL_H

#include <stdint.h>

typedef enum {
    BALANCE_POSITION_STATE_IDLE = 0,
    BALANCE_POSITION_STATE_MOVING,
    BALANCE_POSITION_STATE_SETTLING,
    BALANCE_POSITION_STATE_HOLD,
    BALANCE_POSITION_STATE_FAULT
} BalancePositionState;

typedef enum {
    BALANCE_POSITION_FAULT_NONE = 0,
    BALANCE_POSITION_FAULT_SOFT_LIMIT,
    BALANCE_POSITION_FAULT_TIMEOUT,
    BALANCE_POSITION_FAULT_NO_FEEDBACK,
    BALANCE_POSITION_FAULT_FOLLOW_ERROR,
    BALANCE_POSITION_FAULT_DIRECTION
} BalancePositionFault;

typedef struct {
    int32_t target_count;
    int32_t current_count;
    int32_t position_error_count;
    int32_t following_error_count;
    int32_t minimum_count;
    int32_t maximum_count;
    uint32_t elapsed_ms;
    uint32_t timeout_ms;
    uint32_t settle_ms;
    uint32_t follow_error_ms;
    uint32_t direction_error_ms;
    uint32_t steps_without_feedback;
    uint16_t commanded_step_rate_hz;
    uint16_t step_half_period_ticks;
    uint8_t busy;
    uint8_t hold_enabled;
    uint8_t target_reached;
    uint8_t tracking_enabled;
    BalancePositionState state;
    BalancePositionFault fault;
} BalancePositionRuntime;

void BalancePositionControl_Init(void);
uint8_t BalancePositionControl_Start(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count);
uint8_t BalancePositionControl_StartTracking(int32_t target_count,
    int32_t current_count, int32_t minimum_count, int32_t maximum_count,
    uint32_t soft_limit_clamp_count);
uint8_t BalancePositionControl_SetTrackingTarget(int32_t target_count);
void BalancePositionControl_Update20ms(int32_t current_count,
    uint32_t elapsed_ms, uint32_t soft_limit_clamp_count);
void BalancePositionControl_Cancel(void);
uint8_t BalancePositionControl_ResetFault(void);
uint8_t BalancePositionControl_IsBusy(void);
uint8_t BalancePositionControl_HasFault(void);
void BalancePositionControl_GetSnapshot(BalancePositionRuntime *snapshot);

#endif
