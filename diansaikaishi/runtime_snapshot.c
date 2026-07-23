#include "runtime_snapshot.h"

#include <string.h>

#include "imu.h"
#include "line_controller.h"
#include "motor_control.h"
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
    const ImuRuntime *imu = Imu_GetRuntime();
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();
    const MotorControlRuntime *motorControl = MotorControl_GetRuntime();
    const LineControllerRuntime *lineControl = LineController_GetRuntime();
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
    g_snapshot.line_control_v2_enabled = lineControl->enabled;
    g_snapshot.line_control_v2_config_valid = lineControl->config_valid;
    g_snapshot.line_control_v2_dt_valid = lineControl->dt_valid;
    g_snapshot.line_control_sensor_pattern = lineControl->sensor_pattern;
    g_snapshot.line_control_active_count = lineControl->active_count;
    g_snapshot.line_control_raw_error = lineControl->raw_error;
    g_snapshot.line_control_filtered_error = lineControl->filtered_error;
    g_snapshot.line_control_derivative = lineControl->filtered_derivative;
    g_snapshot.line_control_correction_raw = lineControl->correction_raw;
    g_snapshot.line_control_correction = lineControl->correction;
    g_snapshot.line_control_left_command =
        lineControl->left_target_command;
    g_snapshot.line_control_right_command =
        lineControl->right_target_command;
    g_snapshot.line_control_left_low_speed_zeroed =
        lineControl->left_low_speed_zeroed;
    g_snapshot.line_control_right_low_speed_zeroed =
        lineControl->right_low_speed_zeroed;
    g_snapshot.line_control_state = lineControl->state;
    g_snapshot.line_control_turn_mark = lineControl->turn_mark;
    g_snapshot.line_control_turn_mark_valid =
        lineControl->turn_mark_valid;
    g_snapshot.line_control_lost_elapsed_ms =
        lineControl->lost_elapsed_ms;
    g_snapshot.line_control_recover_timeout =
        lineControl->recover_timeout;
    g_snapshot.line_control_stop_reason = lineControl->stop_reason;
    g_snapshot.line_control_last_valid_pattern =
        lineControl->last_valid_pattern;
    g_snapshot.line_control_last_valid_error =
        lineControl->last_valid_error;
    g_snapshot.line_control_turn_mark_update_count =
        lineControl->turn_mark_update_count;
    g_snapshot.yaw_deg = imu->yaw_deg;
    g_snapshot.gyro_z_dps = imu->corrected_gyro_z_dps;
    g_snapshot.imu_gyro_bias_dps = imu->gyro_bias_dps;
    g_snapshot.imu_raw_gyro_z_counts = imu->raw_gyro_z;
    g_snapshot.imu_gyro_z_before_bias_dps =
        imu->gyro_z_before_bias_dps;
    g_snapshot.imu_gyro_z_after_bias_dps =
        imu->gyro_z_after_bias_dps;
    g_snapshot.imu_angle_increment_deg = imu->angle_increment_deg;
    g_snapshot.imu_gyro_sensitivity_lsb_per_dps =
        imu->gyro_sensitivity_lsb_per_dps;
    g_snapshot.imu_gyro_config_readback = imu->gyro_config_readback;
    g_snapshot.imu_gyro_fs_sel = imu->gyro_fs_sel;
    g_snapshot.imu_gyro_full_scale_dps = imu->gyro_full_scale_dps;
    g_snapshot.imu_yaw_axis_sign = imu->yaw_axis_sign;
    g_snapshot.imu_sample_dt_ms = imu->sample_dt_ms;
    g_snapshot.imu_sample_dt_s = imu->sample_dt_s;
    g_snapshot.imu_initialized = imu->initialized;
    g_snapshot.imu_calibrated = imu->calibrated;
    g_snapshot.imu_valid = imu->valid;
    g_snapshot.imu_stale = imu->stale;
    g_snapshot.imu_dt_valid = imu->dt_valid;
    g_snapshot.imu_short_gap_compensating =
        imu->short_gap_compensating;
    g_snapshot.imu_integration_applied = imu->integration_applied;
    g_snapshot.imu_integration_history_valid =
        imu->integration_history_valid;
    g_snapshot.imu_update_count = imu->update_count;
    g_snapshot.imu_successful_read_count = imu->successful_read_count;
    g_snapshot.imu_integration_count = imu->integration_count;
    g_snapshot.imu_integration_skip_count = imu->integration_skip_count;
    g_snapshot.imu_history_rebuild_count = imu->history_rebuild_count;
    g_snapshot.imu_dt_invalid_skip_count = imu->dt_invalid_skip_count;
    g_snapshot.imu_read_fail_skip_count = imu->read_fail_skip_count;
    g_snapshot.imu_gyro_invalid_skip_count = imu->gyro_invalid_skip_count;
    g_snapshot.imu_yaw_reset_count = imu->yaw_reset_count;
    g_snapshot.imu_cumulative_elapsed_ms = imu->cumulative_elapsed_ms;
    g_snapshot.imu_cumulative_integrated_dt_ms =
        imu->cumulative_integrated_dt_ms;
    g_snapshot.imu_cumulative_angle_increment_deg =
        imu->cumulative_angle_increment_deg;
    g_snapshot.imu_last_success_age_ms = imu->last_success_age_ms;
    g_snapshot.imu_read_fail_count = imu->read_fail_count;
    g_snapshot.imu_consecutive_read_fail_count =
        imu->consecutive_read_fail_count;
    g_snapshot.imu_gyro_range_error_count =
        imu->gyro_range_error_count;
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
    g_snapshot.motor_control_enabled = motorControl->enabled;
    g_snapshot.motor_control_valid = motorControl->valid;
    g_snapshot.motor_control_error_latched = motorControl->error_latched;
    g_snapshot.motor_control_safety_inhibited =
        motorControl->safety_inhibited;
    g_snapshot.motor_control_target_refresh_timeout =
        motorControl->target_refresh_timeout;
    g_snapshot.motor_control_error_flags = motorControl->error_flags;
    g_snapshot.motor_control_estimator_error_flags =
        motorControl->estimator_error_flags;
    g_snapshot.motor_control_target_age_ms = motorControl->target_age_ms;
    g_snapshot.motor_control_left_normalized_target =
        motorControl->left.normalized_target;
    g_snapshot.motor_control_right_normalized_target =
        motorControl->right.normalized_target;
    g_snapshot.motor_control_left_raw_target_cmps =
        motorControl->left.raw_target_speed_cmps;
    g_snapshot.motor_control_right_raw_target_cmps =
        motorControl->right.raw_target_speed_cmps;
    g_snapshot.motor_control_left_ramped_target_cmps =
        motorControl->left.ramped_target_speed_cmps;
    g_snapshot.motor_control_right_ramped_target_cmps =
        motorControl->right.ramped_target_speed_cmps;
    g_snapshot.motor_control_left_measured_cmps =
        motorControl->left.measured_speed_cmps;
    g_snapshot.motor_control_right_measured_cmps =
        motorControl->right.measured_speed_cmps;
    g_snapshot.motor_control_left_error_cmps =
        motorControl->left.error_cmps;
    g_snapshot.motor_control_right_error_cmps =
        motorControl->right.error_cmps;
    g_snapshot.motor_control_left_integral = motorControl->left.integral;
    g_snapshot.motor_control_right_integral = motorControl->right.integral;
    g_snapshot.motor_control_left_proportional_term =
        motorControl->left.proportional_term;
    g_snapshot.motor_control_right_proportional_term =
        motorControl->right.proportional_term;
    g_snapshot.motor_control_left_integral_term =
        motorControl->left.integral_term;
    g_snapshot.motor_control_right_integral_term =
        motorControl->right.integral_term;
    g_snapshot.motor_control_left_feedforward_term =
        motorControl->left.feedforward_term;
    g_snapshot.motor_control_right_feedforward_term =
        motorControl->right.feedforward_term;
    g_snapshot.motor_control_left_kp_used = motorControl->left.kp_used;
    g_snapshot.motor_control_right_kp_used = motorControl->right.kp_used;
    g_snapshot.motor_control_left_output =
        motorControl->left.output_command;
    g_snapshot.motor_control_right_output =
        motorControl->right.output_command;
    g_snapshot.motor_control_left_saturated = motorControl->left.saturated;
    g_snapshot.motor_control_right_saturated = motorControl->right.saturated;
    g_snapshot.motor_control_left_overspeed_gain_active =
        motorControl->left.overspeed_gain_active;
    g_snapshot.motor_control_right_overspeed_gain_active =
        motorControl->right.overspeed_gain_active;
    g_snapshot.motor_control_left_error_sign_changed =
        motorControl->left.error_sign_changed;
    g_snapshot.motor_control_right_error_sign_changed =
        motorControl->right.error_sign_changed;
    g_snapshot.motor_control_left_integral_releasing =
        motorControl->left.integral_releasing;
    g_snapshot.motor_control_right_integral_releasing =
        motorControl->right.integral_releasing;
}

const RuntimeSnapshot *RuntimeSnapshot_Get(void)
{
    return &g_snapshot;
}
