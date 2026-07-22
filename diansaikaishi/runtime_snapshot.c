#include "runtime_snapshot.h"

#include <string.h>

#include "imu.h"
#include "motion_action.h"
#include "obstacle_safety.h"
#include "scheduler_monitor.h"
#include "vision_receiver.h"
#include "vision_tuning_console.h"
#include "watchdog_monitor.h"
#include "wheel_speed_estimator.h"

static RuntimeSnapshot g_snapshot;

void RuntimeSnapshot_Init(void)
{
    memset(&g_snapshot, 0, sizeof(g_snapshot));
    g_snapshot.action_type = MOTION_ACTION_STOP;
}

void RuntimeSnapshot_Update(uint32_t timestamp_ms)
{
    const MissionRuntime *mission = MissionManager_GetRuntime();
    const MotionActionRuntime *action = MotionAction_GetRuntime();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
    const ObstacleAvoidanceFeedback *avoid =
        ObstacleAvoidance_GetFeedback();
    const FaultRecord *fault = Fault_GetRecord();
    const VisionReceiverStatus *vision = VisionReceiver_GetStatus();
    const VisionTuningConsoleStatus *console =
        VisionTuningConsole_GetStatus();
    const WatchdogMonitorStatus *watchdog = WatchdogMonitor_GetStatus();
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();
    SchedulerStats scheduler;

    SchedulerMonitor_GetStats(&scheduler);
    g_snapshot.timestamp_ms = timestamp_ms;
    g_snapshot.car_state = CarState_Get();
    g_snapshot.mission_id = MissionManager_GetSelectedMissionId();
    g_snapshot.action_index = mission->current_action_index;
    g_snapshot.action_type = (action->action != (const MotionAction *)0) ?
        action->action->type : MOTION_ACTION_STOP;
    g_snapshot.action_result = action->result;
    g_snapshot.run_mode = CarController_GetRunMode();
    g_snapshot.track_raw = g_appRuntime.sensor_raw;
    g_snapshot.track_error = g_appRuntime.line_error;
    g_snapshot.yaw_deg = Imu_GetYaw();
    g_snapshot.gyro_z_dps = Imu_GetCorrectedGyroZDps();
    g_snapshot.obstacle_state = obstacle->state;
    g_snapshot.safety_hold = CarController_IsSafetyHoldActive();
    g_snapshot.external_hold = MissionManager_IsExternallyHeld();
    g_snapshot.avoidance_state = avoid->state;
    g_snapshot.fault_code = fault->code;
    g_snapshot.fault_detail = fault->detail;
    g_snapshot.fault_context = fault->context;
    g_snapshot.turn_to_yaw_elapsed_ms = g_appRuntime.turn_elapsed_ms;
    g_snapshot.turn_to_yaw_error_deg = g_appRuntime.yaw_turn_error_deg;
    g_snapshot.app_missed_count = scheduler.app_20ms_missed_count;
    g_snapshot.app_drop_count = scheduler.app_20ms_drop_count;
    g_snapshot.app_overrun_count = scheduler.app_20ms_overrun_count;
    g_snapshot.uart_overflow_count = vision->ring_overflow_count +
        console->rx_overflow_count + console->tx_overflow_count;
    g_snapshot.heartbeat_seen = watchdog->heartbeat_seen;
    g_snapshot.watchdog_tripped = watchdog->tripped;
    g_snapshot.hardware_watchdog_enabled =
        watchdog->hardware_watchdog_enabled;
    g_snapshot.heartbeat_age_ms = watchdog->heartbeat_age_ms;
    g_snapshot.left_delta_pulse = wheel->left_delta_pulse;
    g_snapshot.right_delta_pulse = wheel->right_delta_pulse;
    g_snapshot.left_total_pulse = wheel->left_total_pulse;
    g_snapshot.right_total_pulse = wheel->right_total_pulse;
    g_snapshot.left_raw_speed_cmps = wheel->left_raw_speed_cmps;
    g_snapshot.right_raw_speed_cmps = wheel->right_raw_speed_cmps;
    g_snapshot.left_speed_cmps = wheel->left_speed_cmps;
    g_snapshot.right_speed_cmps = wheel->right_speed_cmps;
    g_snapshot.left_total_distance_cm = wheel->left_total_distance_cm;
    g_snapshot.right_total_distance_cm = wheel->right_total_distance_cm;
    g_snapshot.center_distance_cm = wheel->center_distance_cm;
    g_snapshot.wheel_estimator_valid = wheel->valid;
    g_snapshot.wheel_estimator_stale = wheel->stale;
    g_snapshot.wheel_estimator_overflow = wheel->overflow;
    g_snapshot.wheel_estimator_error_flags = wheel->error_flags;
    g_snapshot.encoder_ppr_x2 = wheel->encoder_ppr_x2;
    g_snapshot.wheel_diameter_cm = wheel->wheel_diameter_cm;
    g_snapshot.wheel_track_cm = wheel->wheel_track_cm;
    g_snapshot.left_encoder_direction = wheel->left_encoder_direction;
    g_snapshot.right_encoder_direction = wheel->right_encoder_direction;
}

const RuntimeSnapshot *RuntimeSnapshot_Get(void)
{
    return &g_snapshot;
}
