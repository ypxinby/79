#ifndef RUNTIME_SNAPSHOT_H
#define RUNTIME_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "car_controller.h"
#include "car_state.h"
#include "fault.h"
#include "mission_manager.h"
#include "motion_types.h"
#include "obstacle_avoidance.h"
#include "obstacle_monitor.h"

typedef struct {
    uint32_t timestamp_ms;
    CarState car_state;
    uint8_t mission_id;
    uint16_t action_index;
    MotionActionType action_type;
    MotionActionResult action_result;
    TrackRunMode run_mode;
    uint8_t track_raw;
    int16_t track_error;
    float yaw_deg;
    float gyro_z_dps;
    ObstacleState obstacle_state;
    bool safety_hold;
    bool external_hold;
    ObstacleAvoidState avoidance_state;
    FaultCode fault_code;
    uint16_t fault_detail;
    uint16_t fault_context;
    uint32_t turn_to_yaw_elapsed_ms;
    float turn_to_yaw_error_deg;
    uint32_t app_missed_count;
    uint32_t app_drop_count;
    uint32_t app_overrun_count;
    uint32_t uart_overflow_count;
    bool heartbeat_seen;
    bool watchdog_tripped;
    bool hardware_watchdog_enabled;
    uint32_t heartbeat_age_ms;
    int32_t left_delta_pulse;
    int32_t right_delta_pulse;
    int64_t left_total_pulse;
    int64_t right_total_pulse;
    float left_raw_speed_cmps;
    float right_raw_speed_cmps;
    float left_speed_cmps;
    float right_speed_cmps;
    float left_total_distance_cm;
    float right_total_distance_cm;
    float center_distance_cm;
    bool wheel_estimator_valid;
    bool wheel_estimator_stale;
    bool wheel_estimator_overflow;
    uint32_t wheel_estimator_error_flags;
    uint32_t encoder_ppr_x2;
    float wheel_diameter_cm;
    float wheel_track_cm;
    int8_t left_encoder_direction;
    int8_t right_encoder_direction;
} RuntimeSnapshot;

void RuntimeSnapshot_Init(void);
void RuntimeSnapshot_Update(uint32_t timestamp_ms);
const RuntimeSnapshot *RuntimeSnapshot_Get(void);

#endif
