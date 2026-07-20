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
} RuntimeSnapshot;

void RuntimeSnapshot_Init(void);
void RuntimeSnapshot_Update(uint32_t timestamp_ms);
const RuntimeSnapshot *RuntimeSnapshot_Get(void);

#endif
