#include "oled_ui.h"
#include "app.h"
#include "app_config.h"
#include "app_features.h"
#include "balance_encoder.h"
#include "balance_soft_limits.h"
#include "bluetooth_uart.h"
#include "car_controller.h"
#include "car_state.h"
#include "fault.h"
#include "gimbal.h"
#include "gimbal_stepper.h"
#include "gimbal_tracker.h"
#include "heading_control.h"
#include "imu.h"
#include "line_controller.h"
#include "menu.h"
#include "gimbal_vision_adapter.h"
#include "gimbal_vision_pitch_tracker.h"
#include "gimbal_vision_yaw_tracker.h"
#include "mission_manager.h"
#include "motion_action.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "obstacle_monitor.h"
#include "obstacle_scanner.h"
#include "obstacle_safety.h"
#include "oled.h"
#include "runtime_snapshot.h"
#include "track_sensor.h"
#include "ultrasonic.h"
#include "vision_receiver.h"
#include "vision_pitch_tuning.h"
#include "wheel_speed_estimator.h"

static int16_t clamp_display_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t clamp_display_float_i16(float value)
{
    if (value != value) {
        return 0;
    }
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t clamp_control_term_i16(float value)
{
    if (value != value) {
        return 0;
    }
    if (value > 999.0f) {
        return 999;
    }
    if (value < -999.0f) {
        return -999;
    }
    return (int16_t)value;
}

#if !FEATURE_BALANCE_SOFT_LIMITS || FEATURE_GIMBAL_OLED_TEST
static void print_signed_x10(int16_t value)
{
    int32_t signedValue = value;
    uint16_t magnitude;

    if (signedValue < 0) {
        OLED_PrintChar('-');
        signedValue = -signedValue;
    }
    magnitude = (uint16_t)signedValue;
    OLED_PrintUInt16((uint16_t)(magnitude / 10U));
    OLED_PrintChar('.');
    OLED_PrintChar((char)('0' + (magnitude % 10U)));
}
#endif

static void print_track_pattern_s1_to_s7(uint8_t pattern)
{
    uint8_t sensor;

    for (sensor = 0U; sensor < TRACK_SENSOR_COUNT; sensor++) {
        OLED_PrintChar((pattern & (uint8_t)(1U << sensor)) ? '1' : '0');
    }
}

static void print_uint64_decimal(uint64_t value)
{
    char digits[20];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while (count > 0U) {
        count--;
        OLED_PrintChar(digits[count]);
    }
}

static void print_signed_total_tail(int64_t value)
{
    uint64_t magnitude;

    if (value < 0) {
        OLED_PrintChar('-');
        magnitude = (uint64_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint64_t)value;
    }

    print_uint64_decimal(magnitude % 1000000U);
}

static const char *motion_action_type_to_string(MotionActionType type)
{
    switch (type) {
        case MOTION_ACTION_SEEK_LINE:
            return "SEEK";
        case MOTION_ACTION_FOLLOW_LINE:
            return "LINE";
        case MOTION_ACTION_TURN_LEFT_90:
            return "L90";
        case MOTION_ACTION_TURN_RIGHT_90:
            return "R90";
        case MOTION_ACTION_TURN_TO_YAW:
            return "YAW";
        case MOTION_ACTION_DRIVE_HEADING_TIME:
            return "HEAD";
        case MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE:
            return "FIND";
        case MOTION_ACTION_DRIVE_DISTANCE:
            return "DIST";
        case MOTION_ACTION_DRIVE_DISTANCE_HEADING:
            return "D-YAW";
        case MOTION_ACTION_WAIT:
            return "WAIT";
        case MOTION_ACTION_STOP:
            return "STOP";
        default:
            return "NONE";
    }
}

static const char *mission_status_to_string(MissionStatus status)
{
    switch (status) {
        case MISSION_STATUS_IDLE:
            return "IDLE";
        case MISSION_STATUS_READY:
            return "READY";
        case MISSION_STATUS_RUNNING:
            return "RUN";
        case MISSION_STATUS_PAUSED:
            return "PAUSE";
        case MISSION_STATUS_DONE:
            return "DONE";
        case MISSION_STATUS_ERROR:
            return "ERR";
        default:
            return "ERR";
    }
}

static void print_elapsed_mm_ss_t(uint32_t elapsed_ms)
{
    uint32_t total_tenths = elapsed_ms / 100U;
    uint32_t minutes = total_tenths / 600U;
    uint32_t seconds = (total_tenths / 10U) % 60U;
    uint32_t tenths = total_tenths % 10U;

    if (minutes < 10U) {
        OLED_PrintChar('0');
    }
    print_uint64_decimal((uint64_t)minutes);
    OLED_PrintChar(':');
    if (seconds < 10U) {
        OLED_PrintChar('0');
    }
    print_uint64_decimal((uint64_t)seconds);
    OLED_PrintChar('.');
    OLED_PrintChar((char)('0' + tenths));
}

static const char *motion_result_to_string(MotionActionResult result);

#if FEATURE_OBSTACLE_SCANNER
static const char *obstacle_scan_state_to_string(ObstacleScanState state)
{
    switch (state) {
        case OBSTACLE_SCAN_IDLE:
            return "IDLE";
        case OBSTACLE_SCAN_MOVE_CENTER:
            return "M-C";
        case OBSTACLE_SCAN_WAIT_CENTER:
            return "W-C";
        case OBSTACLE_SCAN_SAMPLE_CENTER:
            return "S-C";
        case OBSTACLE_SCAN_MOVE_LEFT:
            return "M-L";
        case OBSTACLE_SCAN_WAIT_LEFT:
            return "W-L";
        case OBSTACLE_SCAN_SAMPLE_LEFT:
            return "S-L";
        case OBSTACLE_SCAN_MOVE_RIGHT:
            return "M-R";
        case OBSTACLE_SCAN_WAIT_RIGHT:
            return "W-R";
        case OBSTACLE_SCAN_SAMPLE_RIGHT:
            return "S-R";
        case OBSTACLE_SCAN_RETURN_CENTER:
            return "RET";
        case OBSTACLE_SCAN_COMPLETE:
            return "DONE";
        default:
            return "ERR";
    }
}
#endif

static void print_status_page(uint8_t raw, int16_t error, uint8_t keyEvent)
{
    const MissionRuntime *mission = MissionManager_GetRuntime();
    const MotionActionRuntime *action = MotionAction_GetRuntime();
    const RuntimeSnapshot *snapshot = RuntimeSnapshot_Get();
    const char *actionName = "NONE";
    const char *missionName = "NONE";
    const FaultRecord *fault = Fault_GetRecord();
    uint16_t missionIndex = MissionManager_GetSelectedMissionIndex();
    uint16_t missionCount = MissionManager_GetMissionCount();

    (void)raw;
    (void)error;
    (void)keyEvent;

    if (action->action != (const MotionAction *)0) {
        actionName = motion_action_type_to_string(action->action->type);
    }
    if (mission->definition != (const MissionDefinition *)0) {
        missionName = mission->definition->name;
    }

    OLED_SetCursor(0, 0);
#if APP_PROFILE == APP_PROFILE_DEVELOPMENT
    OLED_PrintString("TEST T:");
#else
    OLED_PrintString("COMP T:");
#endif
    OLED_PrintInt16((int16_t)(missionIndex + 1U));
    OLED_PrintChar('/');
    OLED_PrintInt16((int16_t)missionCount);
    OLED_PrintChar(' ');
    OLED_PrintString(mission_status_to_string(mission->status));

    OLED_SetCursor(2, 0);
    OLED_PrintString(missionName);
    OLED_PrintString(" A:");
    OLED_PrintString(actionName);

    OLED_SetCursor(4, 0);
    OLED_PrintString("E:");
    OLED_PrintInt16(snapshot->wheel_estimator_valid ? 1 : 0);
    OLED_PrintString(" I:");
    OLED_PrintInt16(Imu_IsReady() ? 1 : 0);
    OLED_PrintString(" M:");
    OLED_PrintInt16((snapshot->motor_control_enabled &&
        !snapshot->motor_control_error_latched) ? 1 : 0);
    OLED_PrintString(" H:");
    OLED_PrintInt16((int16_t)ObstacleSafety_IsHolding());

    OLED_SetCursor(6, 0);
    if (fault->code != FAULT_CODE_NONE) {
        OLED_PrintString("F:");
        OLED_PrintString(FaultCode_ToShortString(fault->code));
        OLED_PrintString(" D:");
        OLED_PrintInt16((int16_t)fault->detail);
        OLED_PrintString(" S:");
        OLED_PrintInt16((int16_t)fault->context);
    } else if (mission->status == MISSION_STATUS_ERROR) {
        OLED_PrintString("ME:");
        OLED_PrintInt16((int16_t)mission->last_error_code);
        OLED_PrintString(" AE:");
        OLED_PrintInt16((int16_t)action->error_code);
    } else {
        OLED_PrintString("F:NONE R:");
        OLED_PrintString(motion_result_to_string(action->result));
    }
}

static void print_obstacle_page(void)
{
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
#if FEATURE_OBSTACLE_SCANNER
    const ObstacleScanFeedback *scan = ObstacleScanner_GetFeedback();
#endif
    const ObstacleAvoidanceFeedback *avoid = ObstacleAvoidance_GetFeedback();
    const FaultRecord *fault = Fault_GetRecord();

    OLED_SetCursor(0, 0);
    OLED_PrintString("O:");
    OLED_PrintInt16((int16_t)obstacle->blocked);
    OLED_PrintString(" H:");
    OLED_PrintInt16((int16_t)ObstacleSafety_IsHolding());
    OLED_PrintString(" A:");
    OLED_PrintInt16((int16_t)ObstacleAvoidance_IsActive());
    OLED_PrintString(" V:");
    OLED_PrintInt16((int16_t)avoid->state);
    OLED_PrintString(" D:");
    OLED_PrintInt16((int16_t)avoid->direction);

    OLED_SetCursor(2, 0);
    OLED_PrintString("U:");
    if (ultrasonic->measurement_valid) {
        OLED_PrintInt16((int16_t)ultrasonic->distance_cm);
    } else {
        OLED_PrintInt16(0);
    }
#if FEATURE_OBSTACLE_SCANNER
    OLED_PrintString(" C:");
    if (scan->center_valid) {
        OLED_PrintInt16((int16_t)scan->center_distance_cm);
    } else {
        OLED_PrintInt16(0);
    }

    OLED_SetCursor(4, 0);
    OLED_PrintString("L:");
    if (scan->left_valid) {
        OLED_PrintInt16((int16_t)scan->left_distance_cm);
    } else {
        OLED_PrintInt16(0);
    }
    OLED_PrintString(" R:");
    if (scan->right_valid) {
        OLED_PrintInt16((int16_t)scan->right_distance_cm);
    } else {
        OLED_PrintInt16(0);
    }

    OLED_SetCursor(6, 0);
    if (fault->code != FAULT_CODE_NONE) {
        OLED_PrintString("F:");
        OLED_PrintString(FaultCode_ToShortString(fault->code));
        OLED_PrintString(" S:");
        OLED_PrintInt16((int16_t)fault->context);
    } else {
        OLED_PrintString("SC:");
        OLED_PrintString(obstacle_scan_state_to_string(scan->state));
        OLED_PrintString(" D:");
        OLED_PrintString(ObstacleScanner_DirectionToString(
            scan->recommended_direction));
    }
#else
    OLED_PrintString(" V:");
    OLED_PrintInt16((int16_t)obstacle->distance_valid);

    OLED_SetCursor(4, 0);
    OLED_PrintString("BC:");
    OLED_PrintInt16((int16_t)obstacle->block_confirm_count);
    OLED_PrintString(" CC:");
    OLED_PrintInt16((int16_t)obstacle->clear_confirm_count);

    OLED_SetCursor(6, 0);
    if (fault->code != FAULT_CODE_NONE) {
        OLED_PrintString("F:");
        OLED_PrintString(FaultCode_ToShortString(fault->code));
        OLED_PrintString(" S:");
        OLED_PrintInt16((int16_t)fault->context);
    } else {
        OLED_PrintString("DIST:");
        OLED_PrintInt16((int16_t)obstacle->distance_cm);
    }
#endif
}

static void print_encoder_page(void)
{
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();
    int16_t leftRawSpeed =
        clamp_display_i16((int32_t)wheel->left_raw_speed_cmps);
    int16_t rightRawSpeed =
        clamp_display_i16((int32_t)wheel->right_raw_speed_cmps);

    OLED_SetCursor(0, 0);
    OLED_PrintString("LD:");
    OLED_PrintInt16(clamp_display_i16(wheel->left_delta_pulse));
    OLED_PrintString(" RD:");
    OLED_PrintInt16(clamp_display_i16(wheel->right_delta_pulse));

    OLED_SetCursor(2, 0);
    OLED_PrintString("LT:");
    print_signed_total_tail(wheel->left_total_pulse);
    OLED_PrintString(" RT:");
    print_signed_total_tail(wheel->right_total_pulse);

    OLED_SetCursor(4, 0);
    OLED_PrintString("LS:");
    OLED_PrintInt16(leftRawSpeed);
    OLED_PrintString(" RS:");
    OLED_PrintInt16(rightRawSpeed);

    OLED_SetCursor(6, 0);
    OLED_PrintString("V:");
    OLED_PrintInt16(wheel->valid ? 1 : 0);
    OLED_PrintString(" S:");
    OLED_PrintInt16(wheel->stale ? 1 : 0);
    OLED_PrintString(" E:");
    print_uint64_decimal((uint64_t)wheel->error_flags);
}

static void print_bluetooth_page(void)
{
    const volatile BluetoothUartRuntime *bluetooth =
        BluetoothUart_GetRuntime();
    uint16_t rxCount = (bluetooth->rx_byte_count > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)bluetooth->rx_byte_count;
    uint16_t txCount = (bluetooth->tx_byte_count > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)bluetooth->tx_byte_count;
    uint16_t overflowCount =
        (bluetooth->rx_overflow_count > UINT16_MAX) ?
            UINT16_MAX : (uint16_t)bluetooth->rx_overflow_count;
    uint16_t irqCount = (bluetooth->irq_count > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)bluetooth->irq_count;
    uint16_t polledCount =
        (bluetooth->polled_rx_byte_count > UINT16_MAX) ?
            UINT16_MAX : (uint16_t)bluetooth->polled_rx_byte_count;
    uint16_t dropCount = (bluetooth->tx_drop_count > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)bluetooth->tx_drop_count;

    OLED_SetCursor(0, 0);
    OLED_PrintString("HC06 I:");
    OLED_PrintInt16(bluetooth->initialized ? 1 : 0);
    OLED_PrintString(" M:");
    OLED_PrintInt16(bluetooth->pinmux_valid ? 1 : 0);
    OLED_PrintString(" H:");
    OLED_PrintInt16(bluetooth->rx_pin_high ? 1 : 0);

    OLED_SetCursor(2, 0);
    OLED_PrintString("RX:");
    OLED_PrintUInt16(rxCount);
    OLED_PrintString(" L:");
    OLED_PrintUInt16((uint16_t)bluetooth->last_rx_byte);

    OLED_SetCursor(4, 0);
    OLED_PrintString("IRQ:");
    OLED_PrintUInt16(irqCount);
    OLED_PrintString(" P:");
    OLED_PrintUInt16(polledCount);

    OLED_SetCursor(6, 0);
    OLED_PrintString("T:");
    OLED_PrintUInt16(txCount);
    OLED_PrintString(" O:");
    OLED_PrintUInt16(overflowCount);
    OLED_PrintString(" D:");
    OLED_PrintUInt16(dropCount);
}

static const char *drive_distance_state_to_string(DriveDistanceState state)
{
    switch (state) {
        case DRIVE_DISTANCE_STATE_DRIVE:
            return "DRV";
        case DRIVE_DISTANCE_STATE_DECEL:
            return "DEC";
        case DRIVE_DISTANCE_STATE_SLOW:
            return "SLW";
        case DRIVE_DISTANCE_STATE_SETTLE:
            return "SET";
        case DRIVE_DISTANCE_STATE_DONE:
            return "DONE";
        case DRIVE_DISTANCE_STATE_ABORTED:
            return "ABT";
        case DRIVE_DISTANCE_STATE_ERROR:
            return "ERR";
        case DRIVE_DISTANCE_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static const char *motion_result_to_string(MotionActionResult result)
{
    switch (result) {
        case MOTION_RESULT_RUNNING:
            return "RUN";
        case MOTION_RESULT_SUCCESS:
            return "DONE";
        case MOTION_RESULT_FAILED:
            return "FAIL";
        case MOTION_RESULT_TIMEOUT:
            return "TIME";
        case MOTION_RESULT_CANCELLED:
            return "CANC";
        case MOTION_RESULT_IDLE:
        default:
            return "IDLE";
    }
}

static void print_drive_distance_page(void)
{
    const RuntimeSnapshot *snapshot = RuntimeSnapshot_Get();

    OLED_SetCursor(0, 0);
    OLED_PrintString("D10:");
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->drive_distance_travelled_cm * 10.0f));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->drive_distance_target_cm * 10.0f));
    OLED_PrintString(" R:");
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->drive_distance_remaining_cm * 10.0f));

    OLED_SetCursor(2, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(clamp_display_float_i16(snapshot->yaw_deg));
    OLED_PrintString(" T:");
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->drive_distance_target_yaw_deg));
    OLED_PrintString(" E:");
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->drive_distance_heading_error_deg * 10.0f));

    OLED_SetCursor(4, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(snapshot->motor_control_left_normalized_target);
    OLED_PrintChar('/');
    OLED_PrintInt16(snapshot->motor_control_right_normalized_target);
    OLED_PrintString(" H:");
    OLED_PrintInt16(snapshot->drive_distance_heading_correction);

    OLED_SetCursor(6, 0);
    OLED_PrintString("M:");
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->left_speed_cmps * 10.0f));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_display_float_i16(
        snapshot->right_speed_cmps * 10.0f));
    OLED_PrintString(" V:");
    OLED_PrintInt16(snapshot->wheel_estimator_valid ? 1 : 0);
    OLED_PrintInt16(snapshot->imu_initialized && snapshot->imu_calibrated &&
        snapshot->imu_valid && !snapshot->imu_stale ? 1 : 0);
    OLED_PrintChar(' ');
    OLED_PrintString(drive_distance_state_to_string(
        snapshot->drive_distance_state));
}

static void print_motor_control_page(void)
{
    const MotorControlRuntime *control = MotorControl_GetRuntime();
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();

    OLED_SetCursor(0, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(g_appRuntime.left_speed);
    OLED_PrintChar('/');
    OLED_PrintInt16(g_appRuntime.right_speed);

    OLED_SetCursor(2, 0);
    OLED_PrintString("T10:");
    OLED_PrintInt16(clamp_display_float_i16(
        control->left.ramped_target_speed_cmps * 10.0f));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_display_float_i16(
        control->right.ramped_target_speed_cmps * 10.0f));

    OLED_SetCursor(4, 0);
    OLED_PrintString("M10:");
    OLED_PrintInt16(clamp_display_float_i16(
        wheel->left_speed_cmps * 10.0f));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_display_float_i16(
        wheel->right_speed_cmps * 10.0f));

    OLED_SetCursor(6, 0);
    OLED_PrintString("O:");
    OLED_PrintInt16(control->left.output_command);
    OLED_PrintChar('/');
    OLED_PrintInt16(control->right.output_command);
    OLED_PrintString(" E:");
    print_uint64_decimal((uint64_t)control->error_flags);
}

static void print_motor_control_detail_page(void)
{
    const MotorControlRuntime *control = MotorControl_GetRuntime();

    OLED_SetCursor(0, 0);
    OLED_PrintString("E:");
    OLED_PrintInt16(clamp_control_term_i16(control->left.error_cmps));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(control->right.error_cmps));

    OLED_SetCursor(2, 0);
    OLED_PrintString("F:");
    OLED_PrintInt16(clamp_control_term_i16(
        control->left.feedforward_term));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(
        control->right.feedforward_term));
    OLED_PrintString(" P:");
    OLED_PrintInt16(clamp_control_term_i16(
        control->left.proportional_term));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(
        control->right.proportional_term));

    OLED_SetCursor(4, 0);
    OLED_PrintString("IV:");
    OLED_PrintInt16(clamp_control_term_i16(control->left.integral));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(control->right.integral));
    OLED_PrintString(" I:");
    OLED_PrintInt16(clamp_control_term_i16(control->left.integral_term));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(control->right.integral_term));

    OLED_SetCursor(6, 0);
    OLED_PrintString("K:");
    OLED_PrintInt16(clamp_control_term_i16(control->left.kp_used));
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_control_term_i16(control->right.kp_used));
    OLED_PrintString(" M");
    OLED_PrintChar(control->left.overspeed_gain_active ? 'O' : 'A');
    OLED_PrintChar(control->right.overspeed_gain_active ? 'O' : 'A');
    OLED_PrintString(" R");
    OLED_PrintChar(control->left.integral_releasing ? '1' : '0');
    OLED_PrintChar(control->right.integral_releasing ? '1' : '0');
    OLED_PrintString(" X");
    OLED_PrintChar(control->left.error_sign_changed ? '1' : '0');
    OLED_PrintChar(control->right.error_sign_changed ? '1' : '0');
}

static void print_param_page(uint8_t keyEvent)
{
    ParamItem item = Menu_GetParamItem();
    uint16_t missionIndex;
    uint16_t missionCount;
    uint16_t nextMissionIndex;

    OLED_SetCursor(0, 0);
    OLED_PrintString("PARAM");

    OLED_SetCursor(2, 0);
    OLED_PrintChar('>');
    OLED_PrintString(Menu_ParamItemToString(item));

    OLED_SetCursor(4, 0);
    OLED_PrintString("VAL:");
    OLED_PrintInt16(Menu_GetParamValue(item));

    OLED_SetCursor(6, 0);
    if (item == PARAM_TASK) {
        missionIndex = MissionManager_GetSelectedMissionIndex();
        missionCount = MissionManager_GetMissionCount();
        nextMissionIndex = 0U;

        if (missionCount > 0U) {
            nextMissionIndex = (uint16_t)((missionIndex + 1U) % missionCount);
        }

        OLED_PrintString("ID:");
        OLED_PrintInt16((int16_t)MissionManager_GetSelectedMissionId());
        OLED_PrintString(" N:");
        OLED_PrintInt16((int16_t)missionCount);
        OLED_PrintString(" NX:");
        OLED_PrintInt16((int16_t)(nextMissionIndex + 1U));
    } else if (item == PARAM_GIMBAL_WORLD_LOCK) {
        const GimbalFeedback *yaw = Gimbal_YawGetFeedback();

        OLED_PrintString("Z:");
        OLED_PrintInt16((int16_t)yaw->position_valid);
        OLED_PrintString(" K2:ON K3:OFF");
    } else if (item == PARAM_BASE_SPEED) {
        float speed_cmps =
            ((float)g_appConfig.line_control_v2_base_command / 1000.0f) *
            g_appConfig.wheel_control_max_speed_cmps;

        OLED_PrintString("CM10:");
        OLED_PrintInt16(clamp_display_float_i16(speed_cmps * 10.0f));
        OLED_PrintString(" M:");
        OLED_PrintInt16(g_appConfig.line_control_v2_max_correction);
    } else if (item == PARAM_WHEEL_FEEDFORWARD) {
        OLED_PrintString("X0.01 BOTH");
    } else if (item == PARAM_KP) {
        OLED_PrintString("X0.01 V2");
    } else if (item == PARAM_KD) {
        OLED_PrintString("X0.001 V2");
    } else if (item == PARAM_MAX_CORRECTION) {
        OLED_PrintString("SPD:");
        OLED_PrintInt16(g_appConfig.line_control_v2_base_command);
        OLED_PrintString(" C:");
        OLED_PrintInt16(g_appRuntime.correction);
    } else {
        OLED_PrintString("C:");
        OLED_PrintInt16(g_appRuntime.correction);
        OLED_PrintString(" K:");
        OLED_PrintInt16((int16_t)keyEvent);
    }
}

static void print_sensor_page(uint8_t raw, uint8_t blackCount, int16_t error)
{
#if FEATURE_LINE_CONTROL_V2
    const LineControllerRuntime *line = LineController_GetRuntime();
    const RuntimeSnapshot *snapshot = RuntimeSnapshot_Get();
    const char *state = "FOL";
    const char *stopReason = "-";
    char turnMark = 'U';

    (void)raw;
    (void)blackCount;
    (void)error;

    if (line->state == LINE_CONTROL_STATE_LOST_TURN_LEFT) {
        state = "LOSTL";
    } else if (line->state == LINE_CONTROL_STATE_LOST_TURN_RIGHT) {
        state = "LOSTR";
    } else if (line->state == LINE_CONTROL_STATE_STOP) {
        state = "STOP";
    }

    if (line->turn_mark == LINE_TURN_DIRECTION_LEFT) {
        turnMark = 'L';
    } else if (line->turn_mark == LINE_TURN_DIRECTION_RIGHT) {
        turnMark = 'R';
    }

    if (line->stop_reason == LINE_CONTROL_STOP_REASON_NO_DIRECTION) {
        stopReason = "NOD";
    } else if (line->stop_reason ==
        LINE_CONTROL_STOP_REASON_RECOVER_TIMEOUT) {
        stopReason = "TIME";
    }

    OLED_SetCursor(0, 0);
    OLED_PrintString("LINE P:");
    print_track_pattern_s1_to_s7(line->sensor_pattern);

    OLED_SetCursor(2, 0);
    OLED_PrintString("S:");
    OLED_PrintString(state);
    OLED_PrintString(" E:");
    OLED_PrintInt16(line->raw_error);
    OLED_PrintChar('/');
    OLED_PrintInt16(clamp_display_float_i16(line->filtered_error));

    OLED_SetCursor(4, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(line->correction);
    OLED_PrintString(" L:");
    OLED_PrintInt16(line->left_target_command);
    OLED_PrintString(" R:");
    OLED_PrintInt16(line->right_target_command);

    OLED_SetCursor(6, 0);
    OLED_PrintString("T:");
    print_elapsed_mm_ss_t(snapshot->line_follow_elapsed_ms);
    OLED_PrintString(" M:");
    OLED_PrintChar(turnMark);
    OLED_PrintString(" R:");
    OLED_PrintString(stopReason);
#else
    (void)error;

    OLED_SetCursor(0, 0);
    OLED_PrintString("GRAY B=1 S1->S7");

    OLED_SetCursor(2, 0);
    OLED_PrintString("S1:");
    OLED_PrintChar((raw & (1U << 0)) ? '1' : '0');
    OLED_PrintString(" S2:");
    OLED_PrintChar((raw & (1U << 1)) ? '1' : '0');
    OLED_PrintString(" S3:");
    OLED_PrintChar((raw & (1U << 2)) ? '1' : '0');
    OLED_PrintString(" S4:");
    OLED_PrintChar((raw & (1U << 3)) ? '1' : '0');

    OLED_SetCursor(4, 0);
    OLED_PrintString("S5:");
    OLED_PrintChar((raw & (1U << 4)) ? '1' : '0');
    OLED_PrintString(" S6:");
    OLED_PrintChar((raw & (1U << 5)) ? '1' : '0');
    OLED_PrintString(" S7:");
    OLED_PrintChar((raw & (1U << 6)) ? '1' : '0');

    OLED_SetCursor(6, 0);
    OLED_PrintString("RAW7..1:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));
    OLED_PrintString(" C:");
    OLED_PrintInt16((int16_t)blackCount);
#endif
}

#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
#if FEATURE_BALANCE_SOFT_LIMITS
static const char *balance_cal_stage_to_string(
    BalanceSoftLimitCalibrationStage stage)
{
    switch (stage) {
        case BALANCE_SOFT_LIMIT_CAL_LOW:
            return "LOW";
        case BALANCE_SOFT_LIMIT_CAL_HIGH:
            return "HIGH";
        case BALANCE_SOFT_LIMIT_CAL_COMPLETE:
            return "DONE";
        case BALANCE_SOFT_LIMIT_CAL_IDLE:
        default:
            return "IDLE";
    }
}

static void print_balance_limit_value(uint8_t valid, int32_t value)
{
    if (valid != 0U) {
        print_signed_total_tail((int64_t)value);
    } else {
        OLED_PrintString("NA");
    }
}
#endif

static void print_balance_stepper_test_page(void)
{
    GimbalStepperFeedback stepper;
    BalanceEncoderRuntime encoder;
    int32_t follow_error;
#if !FEATURE_BALANCE_SOFT_LIMITS
    int32_t speed_rpm_x10;
#endif
    uint8_t error_valid;
    const char *state;
#if FEATURE_BALANCE_SOFT_LIMITS
    BalanceSoftLimitsRuntime limits;
#endif

    GimbalStepper_GetFeedbackSnapshot(&stepper);
    BalanceEncoder_GetSnapshot(&encoder);
#if FEATURE_BALANCE_SOFT_LIMITS
    BalanceSoftLimits_GetSnapshot(&limits);
    error_valid = BalanceEncoder_CalculateFollowError(
        stepper.estimated_steps,
        (limits.zero_valid != 0U) ? limits.current_logical_count :
            encoder.count,
        &follow_error);
#else
    error_valid = BalanceEncoder_CalculateFollowError(
        stepper.estimated_steps, encoder.count, &follow_error);
    speed_rpm_x10 = BalanceEncoder_CalculateSpeedRpmX10(
        encoder.speed_sample_delta_count);
#endif

    if (stepper.running != 0U) {
        state = "RUN";
    } else if (stepper.enabled != 0U) {
        state = "HOLD";
    } else {
        state = "REL";
    }

#if FEATURE_BALANCE_SOFT_LIMITS
    if (limits.calibration_active != 0U) {
        OLED_SetCursor(0, 0);
        OLED_PrintString("CAL:");
        OLED_PrintString(balance_cal_stage_to_string(
            limits.calibration_stage));
        OLED_PrintChar(' ');
        OLED_PrintString(state);

        OLED_SetCursor(2, 0);
        OLED_PrintString("POS:");
        print_balance_limit_value(limits.zero_valid,
            limits.current_logical_count);

        OLED_SetCursor(4, 0);
        OLED_PrintString("L:");
        print_balance_limit_value(limits.low_valid,
            limits.low_logical_count);
        OLED_PrintString(" H:");
        print_balance_limit_value(limits.high_valid,
            limits.high_logical_count);

        OLED_SetCursor(6, 0);
        if (limits.error == BALANCE_SOFT_LIMIT_ERROR_RANGE) {
            OLED_PrintString("ERR:RANGE K1=SET");
        } else if (limits.calibration_stage ==
            BALANCE_SOFT_LIMIT_CAL_COMPLETE) {
            OLED_PrintString("K1=EXIT LIM=OK");
        } else {
            OLED_PrintString("K1=SET K3L=X");
        }
        return;
    }

    OLED_SetCursor(0, 0);
    OLED_PrintString("POS:");
    print_balance_limit_value(limits.zero_valid,
        limits.current_logical_count);
    OLED_PrintChar(' ');
    OLED_PrintString(state);

    OLED_SetCursor(2, 0);
    OLED_PrintString("ENC:");
    print_signed_total_tail((int64_t)encoder.count);
    OLED_PrintString(" S:");
    print_signed_total_tail(stepper.estimated_steps);

    OLED_SetCursor(4, 0);
    OLED_PrintString("L:");
    print_balance_limit_value(limits.low_valid,
        limits.low_logical_count);
    OLED_PrintString(" H:");
    print_balance_limit_value(limits.high_valid,
        limits.high_logical_count);

    OLED_SetCursor(6, 0);
    OLED_PrintString("E:");
    if (error_valid != 0U) {
        print_signed_total_tail((int64_t)follow_error);
    } else {
        OLED_PrintString("NA");
    }
    OLED_PrintString(" IV:");
    print_uint64_decimal((uint64_t)encoder.invalid_transition_count);
    OLED_PrintString(" C:");
    print_uint64_decimal((uint64_t)limits.clamp_count);
#else
    OLED_SetCursor(0, 0);
    OLED_PrintString("ENC:");
    print_signed_total_tail((int64_t)encoder.count);
    OLED_PrintChar(' ');
    OLED_PrintString(state);

    OLED_SetCursor(2, 0);
    OLED_PrintString("STEP:");
    print_signed_total_tail(stepper.estimated_steps);
    OLED_PrintString(" R:");
    print_signed_x10(clamp_display_i16(speed_rpm_x10));

    OLED_SetCursor(4, 0);
    OLED_PrintString("ERR:");
    if (error_valid != 0U) {
        print_signed_total_tail((int64_t)follow_error);
    } else {
        OLED_PrintString("NA");
    }

    OLED_SetCursor(6, 0);
    OLED_PrintString("DIR:");
    OLED_PrintChar((encoder.direction > 0) ? '+' :
        ((encoder.direction < 0) ? '-' : '0'));
    OLED_PrintString(" N:");
    OLED_PrintInt16(clamp_display_i16(
        encoder.speed_sample_delta_count));
    OLED_PrintString(" IV:");
    print_uint64_decimal((uint64_t)encoder.invalid_transition_count);
#endif
}
#endif

static void print_imu_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    int16_t gyroDpsX10 =
        clamp_display_i16((int32_t)(imu->corrected_gyro_z_dps * 10.0f));
    int16_t biasDpsX10 =
        clamp_display_i16((int32_t)(imu->gyro_bias_dps * 10.0f));
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);

    OLED_SetCursor(0, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" G10:");
    OLED_PrintInt16(gyroDpsX10);

    OLED_SetCursor(2, 0);
    OLED_PrintString("DT:");
    OLED_PrintInt16((int16_t)imu->sample_dt_ms);
    OLED_PrintString(" V:");
    OLED_PrintInt16(imu->valid ? 1 : 0);
    OLED_PrintString(" S:");
    OLED_PrintInt16(imu->stale ? 1 : 0);
    OLED_PrintString(" A:");
    OLED_PrintInt16(clamp_display_i16((int32_t)imu->last_success_age_ms));

    OLED_SetCursor(4, 0);
    OLED_PrintString("RF:");
    OLED_PrintInt16(clamp_display_i16((int32_t)imu->read_fail_count));
    OLED_PrintString(" CF:");
    OLED_PrintInt16(clamp_display_i16(
        (int32_t)imu->consecutive_read_fail_count));
    OLED_PrintString(" CP:");
    OLED_PrintInt16(imu->short_gap_compensating ? 1 : 0);

    OLED_SetCursor(6, 0);
    OLED_PrintString("B10:");
    OLED_PrintInt16(biasDpsX10);
    OLED_PrintString(" I:");
    OLED_PrintInt16(imu->initialized ? 1 : 0);
    OLED_PrintString(" C:");
    OLED_PrintInt16(imu->calibrated ? 1 : 0);
}

static void print_imu_detail_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    int16_t sensitivityX10 = clamp_display_float_i16(
        imu->gyro_sensitivity_lsb_per_dps * 10.0f);
    int16_t beforeBiasX10 = clamp_display_float_i16(
        imu->gyro_z_before_bias_dps * 10.0f);
    int16_t afterBiasX10 = clamp_display_float_i16(
        imu->gyro_z_after_bias_dps * 10.0f);
    int16_t incrementX100 = clamp_display_float_i16(
        imu->angle_increment_deg * 100.0f);

    OLED_SetCursor(0, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16((int16_t)imu->gyro_config_readback);
    OLED_PrintString(" F:");
    OLED_PrintInt16((int16_t)imu->gyro_fs_sel);
    OLED_PrintString(" S10:");
    OLED_PrintInt16(sensitivityX10);
    OLED_PrintString(" X:");
    OLED_PrintInt16((int16_t)imu->yaw_axis_sign);

    OLED_SetCursor(2, 0);
    OLED_PrintString("R:");
    OLED_PrintInt16(imu->raw_gyro_z);
    OLED_PrintString(" B10:");
    OLED_PrintInt16(beforeBiasX10);

    OLED_SetCursor(4, 0);
    OLED_PrintString("A10:");
    OLED_PrintInt16(afterBiasX10);
    OLED_PrintString(" D100:");
    OLED_PrintInt16(incrementX100);

    OLED_SetCursor(6, 0);
    OLED_PrintString("IA:");
    OLED_PrintInt16(imu->integration_applied ? 1 : 0);
    OLED_PrintString(" T:");
    print_uint64_decimal((uint64_t)imu->cumulative_integrated_dt_ms);
}

static void print_imu_counters_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();

    OLED_SetCursor(0, 0);
    OLED_PrintString("U:");
    print_uint64_decimal((uint64_t)imu->update_count);
    OLED_PrintString(" OK:");
    print_uint64_decimal((uint64_t)imu->successful_read_count);

    OLED_SetCursor(2, 0);
    OLED_PrintString("IC:");
    print_uint64_decimal((uint64_t)imu->integration_count);
    OLED_PrintString(" IS:");
    print_uint64_decimal((uint64_t)imu->integration_skip_count);

    OLED_SetCursor(4, 0);
    OLED_PrintString("HR:");
    print_uint64_decimal((uint64_t)imu->history_rebuild_count);
    OLED_PrintString(" YR:");
    print_uint64_decimal((uint64_t)imu->yaw_reset_count);

    OLED_SetCursor(6, 0);
    OLED_PrintString("DI:");
    print_uint64_decimal((uint64_t)imu->dt_invalid_skip_count);
    OLED_PrintString(" RF:");
    print_uint64_decimal((uint64_t)imu->read_fail_skip_count);
    OLED_PrintString(" GI:");
    print_uint64_decimal((uint64_t)imu->gyro_invalid_skip_count);
}

static float heading_display_target_yaw(void)
{
    if (g_appRuntime.run_mode == TRACK_MODE_TURN_TO_YAW) {
        return g_appRuntime.yaw_turn_target_deg;
    }
    if (MissionManager_GetSelectedMissionId() == MISSION_ID_TEST_YAW) {
        return g_appRuntime.yaw_turn_target_deg;
    }
    if (g_appRuntime.run_mode == TRACK_MODE_DRIVE_HEADING) {
        return g_appRuntime.drive_heading_target_yaw_deg;
    }
    return HeadingControl_GetRuntime()->target_yaw_deg;
}

static uint32_t heading_display_action_elapsed_ms(void)
{
    const MotionActionRuntime *action = MotionAction_GetRuntime();

    if ((action->action != (const MotionAction *)0) &&
        ((action->action->type == MOTION_ACTION_TURN_TO_YAW) ||
            (action->action->type == MOTION_ACTION_DRIVE_HEADING_TIME) ||
            (action->action->type ==
                MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE))) {
        return action->elapsed_ms;
    }
    if (g_appRuntime.run_mode == TRACK_MODE_TURN_TO_YAW) {
        return g_appRuntime.turn_elapsed_ms;
    }
    if (g_appRuntime.run_mode == TRACK_MODE_DRIVE_HEADING) {
        return g_appRuntime.heading_straight_elapsed_ms;
    }
    return 0U;
}

static const char *turn_display_state_to_string(void)
{
    const MissionRuntime *mission = MissionManager_GetRuntime();
    float absError = g_appRuntime.yaw_turn_error_deg;

    if (absError < 0.0f) {
        absError = -absError;
    }

    if (mission->status == MISSION_STATUS_ERROR) {
        return "ERR";
    }
    if ((MissionManager_GetSelectedMissionId() == MISSION_ID_TEST_YAW) &&
        (mission->status == MISSION_STATUS_DONE)) {
        return "DONE";
    }
    if (g_appRuntime.run_mode != TRACK_MODE_TURN_TO_YAW) {
        return "IDLE";
    }
    if (absError <= g_appConfig.yaw_turn_done_tolerance_deg) {
        return "SET";
    }
    if (absError <= g_appConfig.yaw_turn_slow_threshold_deg) {
        return "SLOW";
    }
    return "FAST";
}

static void print_heading_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    const MotionActionRuntime *action = MotionAction_GetRuntime();
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);
    int16_t targetDeg =
        clamp_display_i16((int32_t)heading_display_target_yaw());
    int16_t errorX10 =
        clamp_display_i16((int32_t)(
            g_appRuntime.yaw_turn_error_deg * 10.0f));
    int16_t gyroX10 = clamp_display_i16(
        (int32_t)(imu->corrected_gyro_z_dps * 10.0f));

    OLED_SetCursor(0, 0);
    OLED_PrintString("TURN Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" T:");
    OLED_PrintInt16(targetDeg);

    OLED_SetCursor(2, 0);
    OLED_PrintString("E10:");
    OLED_PrintInt16(errorX10);
    OLED_PrintString(" G10:");
    OLED_PrintInt16(gyroX10);

    OLED_SetCursor(4, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(g_appRuntime.left_speed);
    OLED_PrintChar('/');
    OLED_PrintInt16(g_appRuntime.right_speed);
    OLED_PrintString(" S:");
    OLED_PrintString(turn_display_state_to_string());

    OLED_SetCursor(6, 0);
    OLED_PrintString("R:");
    OLED_PrintString(motion_result_to_string(action->result));
    OLED_PrintString(" A:");
    print_uint64_decimal((uint64_t)heading_display_action_elapsed_ms());
    OLED_PrintString(" V:");
    OLED_PrintInt16(Imu_IsReady() ? 1 : 0);
}

#if FEATURE_GIMBAL_OLED_TEST
static const char *gimbal_mode_to_string(GimbalMode mode)
{
    switch (mode) {
        case GIMBAL_MODE_RELEASED:
            return "REL";
        case GIMBAL_MODE_HOLDING:
            return "HLD";
        case GIMBAL_MODE_MOVING:
            return "MOV";
        default:
            return "ERR";
    }
}

static void print_uint_scaled(uint16_t value, uint8_t fractionalDigits);
static void print_alpha_x1000(uint16_t value);

static void print_gimbal_page(void)
{
    const GimbalFeedback *gimbal = Gimbal_YawGetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("YAW Z");
    OLED_PrintInt16((int16_t)gimbal->position_valid);
    OLED_PrintString(" WL");
    OLED_PrintInt16((int16_t)gimbal->world_lock_enabled);

    OLED_SetCursor(2, 0);
    OLED_PrintString("ANG:");
    print_signed_x10(gimbal->continuous_deg_x10);

    OLED_SetCursor(4, 0);
    OLED_PrintString("TGT:");
    print_signed_x10(gimbal->target_deg_x10);
    OLED_PrintString(" L:");
    OLED_PrintInt16((int16_t)gimbal->limit_direction);

    OLED_SetCursor(6, 0);
    OLED_PrintString("M:");
    OLED_PrintString(gimbal_mode_to_string(gimbal->mode));
    OLED_PrintString(" RPM:");
    print_signed_x10(gimbal->commanded_rpm_x10);
}

static void print_gimbal_pitch_page(void)
{
    const GimbalFeedback *gimbal = Gimbal_PitchGetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("PITCH Z");
    OLED_PrintInt16((int16_t)gimbal->position_valid);
    OLED_PrintString(" L");
    OLED_PrintInt16((int16_t)gimbal->limit_direction);

    OLED_SetCursor(2, 0);
    OLED_PrintString("ANG:");
    print_signed_x10(gimbal->continuous_deg_x10);

    OLED_SetCursor(4, 0);
    OLED_PrintString("TGT:");
    print_signed_x10(gimbal->target_deg_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("M:");
    OLED_PrintString(gimbal_mode_to_string(gimbal->mode));
    OLED_PrintString(" RPM:");
    print_signed_x10(gimbal->commanded_rpm_x10);
}

static void print_gimbal_tracker_page(void)
{
    const GimbalTrackerFeedback *tracker =
        GimbalTracker_GetFeedback();
    const GimbalFeedback *yaw = Gimbal_YawGetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("P23X E:");
    OLED_PrintInt16((int16_t)tracker->enabled);
    OLED_PrintString(" V:");
    OLED_PrintInt16((int16_t)tracker->target_valid);
    OLED_PrintString(" Z:");
    OLED_PrintInt16((int16_t)yaw->position_valid);

    OLED_SetCursor(2, 0);
    OLED_PrintString("EX:");
    OLED_PrintInt16(tracker->raw_error_x_px);
    OLED_PrintString(" FX:");
    OLED_PrintInt16(tracker->filtered_error_x_px);

    OLED_SetCursor(4, 0);
    OLED_PrintString("YS:");
    OLED_PrintInt16(tracker->yaw_speed_deg_s_x10);
    OLED_PrintString(" YD:");
    OLED_PrintInt16(tracker->yaw_delta_deg_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("YT:");
    OLED_PrintInt16(tracker->yaw_target_deg_x10);
    OLED_PrintString(" DB:");
    OLED_PrintInt16((int16_t)tracker->yaw_deadbanded);
    OLED_PrintString(" L:");
    OLED_PrintInt16((int16_t)yaw->limit_direction);
}

static void print_uint_min_width(uint16_t value, uint8_t minWidth)
{
    uint32_t divisor = 10U;
    uint8_t digits = 1U;

    while ((uint32_t)value >= divisor) {
        digits++;
        divisor *= 10U;
    }
    while (digits < minWidth) {
        OLED_PrintChar('0');
        digits++;
    }
    OLED_PrintUInt16(value);
}

static void print_uint_scaled(uint16_t value, uint8_t fractionalDigits)
{
    uint16_t divisor = 1U;
    uint8_t index;

    for (index = 0U; index < fractionalDigits; index++) {
        divisor = (uint16_t)(divisor * 10U);
    }
    OLED_PrintUInt16((uint16_t)(value / divisor));
    if (fractionalDigits != 0U) {
        OLED_PrintChar('.');
        print_uint_min_width((uint16_t)(value % divisor),
            fractionalDigits);
    }
}

static void print_alpha_x1000(uint16_t value)
{
    print_uint_scaled((uint16_t)((value + 5U) / 10U), 2U);
}

static void print_gimbal_tracker_pitch_page(void)
{
    const GimbalTrackerFeedback *tracker =
        GimbalTracker_GetFeedback();
    const GimbalFeedback *pitch = Gimbal_PitchGetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("P23V E:");
    OLED_PrintInt16((int16_t)tracker->enabled);
    OLED_PrintString(" V:");
    OLED_PrintInt16((int16_t)tracker->target_valid);
    OLED_PrintString(" Z:");
    OLED_PrintInt16((int16_t)pitch->position_valid);

    OLED_SetCursor(2, 0);
    OLED_PrintString("EY:");
    OLED_PrintInt16(tracker->raw_error_y_px);
    OLED_PrintString(" FY:");
    OLED_PrintInt16(tracker->filtered_error_y_px);

    OLED_SetCursor(4, 0);
    OLED_PrintString("PS:");
    OLED_PrintInt16(tracker->pitch_speed_deg_s_x10);
    OLED_PrintString(" R:");
    OLED_PrintInt16(pitch->commanded_rpm_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("PA:");
    OLED_PrintInt16(pitch->continuous_deg_x10);
    OLED_PrintString(" D:");
    OLED_PrintInt16((int16_t)pitch->direction);
}

static const char *vision_event_to_string(VisionReceiverEvent event)
{
    switch (event) {
        case VISION_RECEIVER_EVENT_TARGET:
            return "TGT";
        case VISION_RECEIVER_EVENT_NO_TARGET:
            return "NONE";
        case VISION_RECEIVER_EVENT_NEW_SESSION:
            return "NEW";
        case VISION_RECEIVER_EVENT_DUPLICATE:
            return "DUP";
        case VISION_RECEIVER_EVENT_OLD_SEQUENCE:
            return "OLD";
        case VISION_RECEIVER_EVENT_LENGTH_ERROR:
            return "LEN";
        case VISION_RECEIVER_EVENT_CRC_ERROR:
            return "CRC";
        case VISION_RECEIVER_EVENT_VERSION_ERROR:
            return "VER";
        case VISION_RECEIVER_EVENT_TYPE_ERROR:
            return "TYP";
        case VISION_RECEIVER_EVENT_RESERVED_ERROR:
            return "RES";
        case VISION_RECEIVER_EVENT_FLAGS_ERROR:
            return "FLG";
        case VISION_RECEIVER_EVENT_FIELD_ERROR:
            return "FLD";
        case VISION_RECEIVER_EVENT_WAITING:
        default:
            return "WAIT";
    }
}

static uint16_t display_count_3digit(uint32_t value)
{
    return (value > 999U) ? 999U : (uint16_t)value;
}

static void print_vision_receiver_page(void)
{
    const VisionReceiverStatus *status = VisionReceiver_GetStatus();
    const VisionReceiverObservation *observation =
        VisionReceiver_GetObservation();

    OLED_SetCursor(0, 0);
    OLED_PrintString("VRX:");
    OLED_PrintString(vision_event_to_string(status->last_event));
    OLED_PrintString(" R:");
    OLED_PrintUInt16(display_count_3digit(status->ring_overflow_count));

    OLED_SetCursor(2, 0);
    OLED_PrintString("S:");
    OLED_PrintUInt16((uint16_t)status->session_id);
    OLED_PrintString(" Q:");
    OLED_PrintUInt16(status->last_sequence);

    OLED_SetCursor(4, 0);
    OLED_PrintString("V:");
    OLED_PrintUInt16(observation->target_valid);
    OLED_PrintString(" X:");
    OLED_PrintUInt16(observation->packet.target_center_x);
    OLED_PrintString(" Y:");
    OLED_PrintUInt16(observation->packet.target_center_y);

    OLED_SetCursor(6, 0);
    OLED_PrintString("A");
    OLED_PrintUInt16(display_count_3digit(status->accepted_frame_count));
    OLED_PrintString(" E");
    OLED_PrintUInt16(display_count_3digit(
        VisionReceiver_GetProtocolErrorCount()));
    OLED_PrintString(" D");
    OLED_PrintUInt16(display_count_3digit(status->duplicate_count));
    OLED_PrintString(" O");
    OLED_PrintUInt16(display_count_3digit(status->old_sequence_count));
}

static const char *vision_adapter_state_to_string(
    GimbalVisionAdapterState state)
{
    switch (state) {
        case GIMBAL_VISION_ADAPTER_TARGET:
            return "OK";
        case GIMBAL_VISION_ADAPTER_NO_TARGET:
            return "NONE";
        case GIMBAL_VISION_ADAPTER_SOURCE_ERROR:
            return "ERR";
        case GIMBAL_VISION_ADAPTER_WAITING:
        default:
            return "WAIT";
    }
}

static void print_gimbal_vision_adapter_page(void)
{
    const GimbalVisionAdapterFeedback *adapter =
        GimbalVisionAdapter_GetFeedback();
    const GimbalTargetObservation *observation =
        GimbalVisionAdapter_GetObservation();

    OLED_SetCursor(0, 0);
    OLED_PrintString("VAD:");
    OLED_PrintString(vision_adapter_state_to_string(adapter->state));
    OLED_PrintString(" V:");
    OLED_PrintUInt16(observation->valid);

    OLED_SetCursor(2, 0);
    OLED_PrintString("S:");
    OLED_PrintUInt16((uint16_t)adapter->session_id);
    OLED_PrintString(" Q:");
    OLED_PrintUInt16(observation->sequence);

    OLED_SetCursor(4, 0);
    OLED_PrintString("X");
    OLED_PrintUInt16(adapter->source_packet.target_center_x);
    OLED_PrintString(" Y");
    OLED_PrintUInt16(adapter->source_packet.target_center_y);
    OLED_PrintString(" C");
    OLED_PrintUInt16(adapter->source_packet.confidence);

    OLED_SetCursor(6, 0);
    OLED_PrintString("EX:");
    OLED_PrintInt16(observation->error_x_px);
    OLED_PrintString(" EY:");
    OLED_PrintInt16(observation->error_y_px);
}

static const char *vision_yaw_state_to_string(GimbalVisionYawState state)
{
    switch (state) {
        case GIMBAL_VISION_YAW_WAIT_ZERO:
            return "ZER";
        case GIMBAL_VISION_YAW_WAIT_OBSERVATION:
            return "WAI";
        case GIMBAL_VISION_YAW_TRACKING:
            return "TRK";
        case GIMBAL_VISION_YAW_CENTERED:
            return "CTR";
        case GIMBAL_VISION_YAW_TARGET_LOST:
            return "LOS";
        case GIMBAL_VISION_YAW_COMM_TIMEOUT:
            return "TMO";
        case GIMBAL_VISION_YAW_LIMITED:
            return "LIM";
        case GIMBAL_VISION_YAW_WORLD_LOCKED:
            return "WLK";
        case GIMBAL_VISION_YAW_PREEMPTED:
            return "PRE";
        case GIMBAL_VISION_YAW_DISABLED:
        default:
            return "DIS";
    }
}

static void print_gimbal_vision_yaw_page(void)
{
    const GimbalVisionYawFeedback *yaw =
        GimbalVisionYawTracker_GetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("YVT:");
    OLED_PrintString(vision_yaw_state_to_string(yaw->state));
    OLED_PrintString(" E");
    OLED_PrintUInt16(yaw->enabled);
    OLED_PrintString(" Z");
    OLED_PrintUInt16(yaw->position_valid);

    OLED_SetCursor(2, 0);
    OLED_PrintString("EX:");
    OLED_PrintInt16(yaw->error_x_px);
    OLED_PrintString(" S:");
    print_signed_x10(yaw->command_speed_deg_s_x10);

    OLED_SetCursor(4, 0);
    OLED_PrintString("T:");
    print_signed_x10(yaw->target_wrapped_deg_x10);
    OLED_PrintString(" C:");
    print_signed_x10(yaw->current_wrapped_deg_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("L:");
    OLED_PrintInt16((int16_t)yaw->limit_direction);
    OLED_PrintString(" P");
    OLED_PrintUInt16(yaw->positive_limit);
    OLED_PrintString(" N");
    OLED_PrintUInt16(yaw->negative_limit);
}

static const char *vision_pitch_state_to_string(
    GimbalVisionPitchState state)
{
    switch (state) {
        case GIMBAL_VISION_PITCH_WAIT_ZERO:
            return "ZER";
        case GIMBAL_VISION_PITCH_WAIT_OBSERVATION:
            return "WAI";
        case GIMBAL_VISION_PITCH_TRACKING:
            return "TRK";
        case GIMBAL_VISION_PITCH_CENTERED:
            return "CTR";
        case GIMBAL_VISION_PITCH_TARGET_LOST:
            return "LOS";
        case GIMBAL_VISION_PITCH_STALE:
            return "OLD";
        case GIMBAL_VISION_PITCH_LIMITED:
            return "LIM";
        case GIMBAL_VISION_PITCH_PREEMPTED:
            return "PRE";
        case GIMBAL_VISION_PITCH_DISABLED:
        default:
            return "DIS";
    }
}

static void print_gimbal_vision_pitch_page(void)
{
    const GimbalVisionPitchFeedback *pitch =
        GimbalVisionPitchTracker_GetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("PVT ");
    OLED_PrintString(vision_pitch_state_to_string(pitch->state));
    OLED_PrintString(" E");
    OLED_PrintUInt16(pitch->enabled);
    OLED_PrintString(" Z");
    OLED_PrintUInt16(pitch->position_valid);

    OLED_SetCursor(2, 0);
    OLED_PrintString("EY:");
    OLED_PrintInt16(pitch->error_y_px);
    OLED_PrintString(" A:");
    print_uint_min_width(
        display_count_3digit(pitch->observation_age_ms), 3U);

    OLED_SetCursor(4, 0);
    OLED_PrintString("S:");
    print_signed_x10(pitch->command_speed_deg_s_x10);
    OLED_PrintString(" P:");
    print_signed_x10(pitch->pitch_angle_deg_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("Q:");
    OLED_PrintUInt16(pitch->observation_sequence);
    OLED_PrintString(" L:");
    OLED_PrintInt16((int16_t)pitch->limit_direction);
}

static void print_signed_deg_int_from_x10(int16_t value)
{
    OLED_PrintInt16((int16_t)(value / 10));
}

static void print_gimbal_vision_dual_page(void)
{
    const GimbalVisionYawFeedback *yaw =
        GimbalVisionYawTracker_GetFeedback();
    const GimbalVisionPitchFeedback *pitch =
        GimbalVisionPitchTracker_GetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("DVT Y");
    OLED_PrintString(vision_yaw_state_to_string(yaw->state));
    OLED_PrintString(" P");
    OLED_PrintString(vision_pitch_state_to_string(pitch->state));

    OLED_SetCursor(2, 0);
    OLED_PrintString("EX");
    OLED_PrintInt16(yaw->error_x_px);
    OLED_PrintString(" EY");
    OLED_PrintInt16(pitch->error_y_px);

    OLED_SetCursor(4, 0);
    OLED_PrintString("YS");
    print_signed_x10(yaw->command_speed_deg_s_x10);
    OLED_PrintString(" PS");
    print_signed_x10(pitch->command_speed_deg_s_x10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("Z");
    OLED_PrintUInt16(yaw->position_valid);
    OLED_PrintUInt16(pitch->position_valid);
    OLED_PrintString("W");
    OLED_PrintUInt16(yaw->world_lock_enabled);
    OLED_PrintString(" Y");
    print_signed_deg_int_from_x10(yaw->current_wrapped_deg_x10);
    OLED_PrintString("P");
    print_signed_deg_int_from_x10(pitch->pitch_angle_deg_x10);
}

static void print_tuning_change_value(VisionPitchTuningParam parameter,
    uint16_t value)
{
    switch (parameter) {
        case VISION_PITCH_TUNING_PARAM_KP:
            print_uint_scaled(value, 3U);
            break;
        case VISION_PITCH_TUNING_PARAM_MAX_SPEED:
            print_uint_scaled(value, 1U);
            break;
        case VISION_PITCH_TUNING_PARAM_FILTER_ALPHA:
            print_alpha_x1000(value);
            break;
        case VISION_PITCH_TUNING_PARAM_DEADBAND:
        case VISION_PITCH_TUNING_PARAM_TIMEOUT:
        default:
            OLED_PrintUInt16(value);
            break;
    }
}

static void print_vision_pitch_tuning_page(void)
{
    const VisionPitchTuningStatus *status =
        VisionPitchTuning_GetStatus();
    const VisionPitchTuningLastChange *change =
        &status->last_change;

    OLED_SetCursor(0, 0);
    OLED_PrintString("VPT DB");
    print_uint_min_width(status->params.deadband_px, 2U);
    OLED_PrintString(" TO");
    OLED_PrintUInt16(status->params.observation_timeout_ms);

    OLED_SetCursor(2, 0);
    OLED_PrintString("KP");
    print_uint_scaled(status->params.kp_x1000, 3U);
    OLED_PrintString(" MAX");
    print_uint_scaled(status->params.max_speed_deg_s_x10, 1U);

    OLED_SetCursor(4, 0);
    OLED_PrintString("ALPHA");
    print_alpha_x1000(status->params.filter_alpha_x1000);

    OLED_SetCursor(6, 0);
    OLED_PrintString("L:");
    if (change->valid == 0U) {
        OLED_PrintString("NONE");
    } else if (change->parameter == VISION_PITCH_TUNING_PARAM_DEFAULT) {
        OLED_PrintString("DEFAULT");
    } else {
        if (change->parameter == VISION_PITCH_TUNING_PARAM_MAX_SPEED) {
            OLED_PrintString("MAX");
        } else {
            OLED_PrintString(VisionPitchTuning_ParamName(
                change->parameter));
        }
        print_tuning_change_value(change->parameter,
            change->old_value);
        OLED_PrintChar('>');
        print_tuning_change_value(change->parameter,
            change->new_value);
    }
}
#endif

void OledUi_Init(void)
{
    OLED_Init();
}

void OledUi_Update_20ms(uint8_t raw, uint8_t blackCount, int16_t error,
    uint8_t keyEvent)
{
    static uint8_t refreshDivider;
    static uint8_t flushRow;

    refreshDivider++;
    if (refreshDivider >= 5U) {
        refreshDivider = 0U;
        OLED_ClearBuffer();

        switch (Menu_GetPage()) {
        case OLED_PAGE_PARAM:
            print_param_page(keyEvent);
            break;
        case OLED_PAGE_SENSOR:
            print_sensor_page(raw, blackCount, error);
            break;
        case OLED_PAGE_IMU:
            print_imu_page();
            break;
        case OLED_PAGE_IMU_DETAIL:
            print_imu_detail_page();
            break;
        case OLED_PAGE_IMU_COUNTERS:
            print_imu_counters_page();
            break;
        case OLED_PAGE_HEADING:
            print_heading_page();
            break;
        case OLED_PAGE_DISTANCE:
            print_drive_distance_page();
            break;
        case OLED_PAGE_BALANCE_STEPPER_TEST:
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
            print_balance_stepper_test_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_BLUETOOTH:
            print_bluetooth_page();
            break;
        case OLED_PAGE_OBSTACLE:
            print_obstacle_page();
            break;
        case OLED_PAGE_ENCODER:
            print_encoder_page();
            break;
        case OLED_PAGE_MOTOR_CONTROL:
            print_motor_control_page();
            break;
        case OLED_PAGE_MOTOR_CONTROL_DETAIL:
            print_motor_control_detail_page();
            break;
        case OLED_PAGE_GIMBAL:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_PITCH:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_pitch_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_TRACKER:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_tracker_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_TRACKER_PITCH:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_tracker_pitch_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_VISION_RECEIVER:
#if FEATURE_GIMBAL_OLED_TEST
            print_vision_receiver_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_VISION_ADAPTER:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_vision_adapter_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_VISION_YAW:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_vision_yaw_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_VISION_PITCH:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_vision_pitch_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_GIMBAL_VISION_DUAL:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_vision_dual_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_VISION_PITCH_TUNING:
#if FEATURE_GIMBAL_OLED_TEST
            print_vision_pitch_tuning_page();
#else
            print_status_page(raw, error, keyEvent);
#endif
            break;
        case OLED_PAGE_STATUS:
        default:
            print_status_page(raw, error, keyEvent);
            break;
        }
    }

    /* The UI uses pages 0/2/4/6 as four text rows. Flush only one row per
     * 20 ms control cycle so software-I2C traffic cannot stall the complete
     * estimator and MotorControl update interval. */
    OLED_FlushPage((uint8_t)(flushRow * 2U));
    flushRow++;
    if (flushRow >= 4U) {
        flushRow = 0U;
    }
}
