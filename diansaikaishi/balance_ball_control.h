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
    BALANCE_BALL_STATE_FAULT,
    BALANCE_BALL_STATE_HOLD_LAST
} BalanceBallControlState;

/* Latest reason why a newly received K230 observation was not admitted to
 * the PD measurement chain.  This is diagnostic state only; it does not
 * change the existing protection decisions. */
typedef enum {
    BALANCE_BALL_OBSERVATION_ACCEPTED = 0,
    BALANCE_BALL_OBSERVATION_START_DISCARDED,
    BALANCE_BALL_OBSERVATION_TARGET_INVALID,
    BALANCE_BALL_OBSERVATION_NOT_MEASURED,
    BALANCE_BALL_OBSERVATION_STALE,
    BALANCE_BALL_OBSERVATION_POSITION_JUMP,
    BALANCE_BALL_OBSERVATION_JUMP_CANDIDATE,
    BALANCE_BALL_OBSERVATION_POSITION_REBASED
} BalanceBallObservationResult;

typedef enum {
    BALANCE_BALL_START_OK = 0,
    BALANCE_BALL_START_NO_ZERO,
    BALANCE_BALL_START_LIMIT_INVALID,
    BALANCE_BALL_START_POSITION_FAULT,
    BALANCE_BALL_START_ESTOP,
    BALANCE_BALL_START_BUSY,
    BALANCE_BALL_START_TARGET_INVALID,
    BALANCE_BALL_START_DATA_INVALID
} BalanceBallStartResult;

typedef struct {
    /* Command names remain KP/KD for compatibility. kp_x100 is KPOS in
     * (mm/s)/mm; kd_x100 is KVEL in count/(mm/s). */
    int32_t kp_x100;
    int32_t kd_x100;
    int32_t maximum_target_velocity_mm_s;
    int32_t neutral_bias_count;
    uint16_t positive_brake_accel_mm_s2;
    uint16_t negative_brake_accel_mm_s2;
    uint16_t brake_delay_ms;
    uint16_t brake_margin_mm;
    uint16_t approach_velocity_mm_s;
    int32_t maximum_offset_count;
    int32_t target_slew_count_per_20ms;
    uint16_t tracking_deadband_count;
    uint16_t tracking_reengage_count;
    int8_t tilt_sign;
} BalanceBallControlConfig;

typedef struct {
    int16_t target_mm;
    int16_t position_mm;
    int16_t error_mm;
    int16_t velocity_mm_s;
    int16_t raw_velocity_mm_s;
    uint16_t velocity_sample_dt_ms;
    uint16_t velocity_filter_alpha_x1000;
    uint8_t velocity_reversal_fast;
    uint32_t velocity_gap_reset_count;
    int16_t target_velocity_mm_s;
    int16_t velocity_error_mm_s;
    int16_t angle_feedforward_count;
    uint16_t braking_distance_mm;
    uint8_t braking_active;
    uint8_t weak_control_active;
    uint8_t startup_discard_remaining;
    int32_t pd_output_count;
    int32_t p_output_count;
    int32_t d_output_count;
    int32_t commanded_offset_count;
    int32_t actuator_target_count;
    uint32_t last_measurement_time_ms;
    uint32_t accepted_measurement_count;
    uint32_t rejected_jump_count;
    uint32_t jump_candidate_count;
    uint32_t rebase_count;
    uint32_t target_invalid_count;
    uint32_t not_measured_count;
    uint32_t stale_count;
    uint32_t speed_clamp_count;
    uint32_t held_invalid_count;
    uint32_t vision_lost_count;
    uint32_t last_observation_time_ms;
    uint16_t axis_span_mm;
    uint16_t confidence;
    uint8_t enable_requested;
    uint8_t tracking_owned;
    /* Latest processed observation was valid=1 and measured=1. */
    uint8_t raw_vision_valid;
    /* Three-frame acquisition is complete and the last real sample is
     * still inside the short hold window. */
    uint8_t measurement_valid;
    uint8_t valid_streak;
    BalanceBallStartResult start_result;
    BalanceBallObservationResult last_observation_result;
    BalanceBallControlState state;
} BalanceBallControlRuntime;

void BalanceBallControl_Init(void);
uint8_t BalanceBallControl_Enable(int16_t target_mm);
void BalanceBallControl_Disable(void);
void BalanceBallControl_ForceStop(void);
uint8_t BalanceBallControl_SetTargetMm(int16_t target_mm);
/* Task-transfer helper: outside error_threshold_mm the normal cascade may use
 * transfer_maximum_count, and at low ball speed it receives at least
 * transfer_minimum_count authority. Passing enable=0 clears the helper. */
uint8_t BalanceBallControl_SetTransferAssist(uint8_t enable,
    uint16_t transfer_minimum_count, uint16_t transfer_maximum_count,
    uint16_t error_threshold_mm, uint16_t maximum_ball_speed_mm_s);
/* Independent physical tilt feedforward in logical encoder counts.  It is
 * added after the ball cascade and therefore does not alter K230 velocity or
 * depend on KP/KD. */
void BalanceBallControl_SetAngleFeedforwardCount(int16_t offset_count);
void BalanceBallControl_Update20ms(uint32_t now_ms,
    uint32_t elapsed_ms);
void BalanceBallControl_GetSnapshot(BalanceBallControlRuntime *snapshot);
void BalanceBallControl_GetConfig(BalanceBallControlConfig *config);
uint8_t BalanceBallControl_SetConfig(
    const BalanceBallControlConfig *config);
void BalanceBallControl_ResetConfig(void);

#endif
