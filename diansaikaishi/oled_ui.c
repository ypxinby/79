#include "oled_ui.h"
#include "app.h"
#include "app_config.h"
#include "app_features.h"
#include "car_controller.h"
#include "car_state.h"
#include "gimbal.h"
#include "heading_control.h"
#include "imu.h"
#include "menu.h"
#include "mission_manager.h"
#include "motion_action.h"
#include "obstacle_avoidance.h"
#include "obstacle_monitor.h"
#include "obstacle_scanner.h"
#include "obstacle_safety.h"
#include "oled.h"
#include "track_sensor.h"
#include "ultrasonic.h"

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
        case MOTION_ACTION_REACQUIRE_LINE:
            return "REQ";
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
    const char *actionName = "NONE";
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
#if FEATURE_OBSTACLE_SCANNER
    const ObstacleScanFeedback *scan = ObstacleScanner_GetFeedback();
#endif
    uint16_t missionIndex = MissionManager_GetSelectedMissionIndex();
    uint16_t missionCount = MissionManager_GetMissionCount();
    uint32_t actionTimeS = action->elapsed_ms / 1000U;

    (void)error;
    (void)keyEvent;

    if (action->action != (const MotionAction *)0) {
        actionName = motion_action_type_to_string(action->action->type);
    }

    OLED_SetCursor(0, 0);
    OLED_PrintString("TASK:");
    OLED_PrintInt16((int16_t)(missionIndex + 1U));
    OLED_PrintChar('/');
    OLED_PrintInt16((int16_t)missionCount);
    OLED_PrintChar(' ');
    OLED_PrintString(mission_status_to_string(mission->status));

    OLED_SetCursor(2, 0);
    OLED_PrintString("ACT:");
    OLED_PrintString(actionName);
    OLED_PrintString(" M:");
    OLED_PrintString(CarController_RunModeToString(CarController_GetRunMode()));

    OLED_SetCursor(4, 0);
    OLED_PrintString("RAW:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));
    OLED_PrintString(" O:");
    OLED_PrintInt16((int16_t)obstacle->blocked);
    OLED_PrintString(" H:");
    OLED_PrintInt16((int16_t)ObstacleSafety_IsHolding());
    OLED_PrintString(" A:");
    OLED_PrintInt16((int16_t)ObstacleAvoidance_IsActive());

    OLED_SetCursor(6, 0);
#if FEATURE_OBSTACLE_SCANNER
    if (scan->active || scan->complete) {
        OLED_PrintString("L:");
        OLED_PrintInt16((int16_t)scan->left_distance_cm);
        OLED_PrintString(" R:");
        OLED_PrintInt16((int16_t)scan->right_distance_cm);
        OLED_PrintString(" D:");
        OLED_PrintString(ObstacleScanner_DirectionToString(
            scan->recommended_direction));
    } else {
#endif
        OLED_PrintString("TIME:");
        OLED_PrintInt16(clamp_display_i16((int32_t)actionTimeS));
        OLED_PrintString(" U:");
        if (ultrasonic->measurement_valid) {
            OLED_PrintInt16((int16_t)ultrasonic->distance_cm);
        } else {
            OLED_PrintInt16(0);
        }
#if FEATURE_OBSTACLE_SCANNER
    }
#endif
}

static void print_obstacle_page(void)
{
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
#if FEATURE_OBSTACLE_SCANNER
    const ObstacleScanFeedback *scan = ObstacleScanner_GetFeedback();
#endif
    const ObstacleAvoidanceFeedback *avoid = ObstacleAvoidance_GetFeedback();

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
    OLED_PrintString("SC:");
    OLED_PrintString(obstacle_scan_state_to_string(scan->state));
    OLED_PrintString(" D:");
    OLED_PrintString(ObstacleScanner_DirectionToString(
        scan->recommended_direction));
#else
    OLED_PrintString(" V:");
    OLED_PrintInt16((int16_t)obstacle->distance_valid);

    OLED_SetCursor(4, 0);
    OLED_PrintString("BC:");
    OLED_PrintInt16((int16_t)obstacle->block_confirm_count);
    OLED_PrintString(" CC:");
    OLED_PrintInt16((int16_t)obstacle->clear_confirm_count);

    OLED_SetCursor(6, 0);
    OLED_PrintString("DIST:");
    OLED_PrintInt16((int16_t)obstacle->distance_cm);
#endif
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
    } else {
        OLED_PrintString("C:");
        OLED_PrintInt16(g_appRuntime.correction);
        OLED_PrintString(" K:");
        OLED_PrintInt16((int16_t)keyEvent);
    }
}

static void print_sensor_page(uint8_t raw, uint8_t blackCount, int16_t error)
{
    OLED_SetCursor(0, 0);
    OLED_PrintString("SENSOR");
    OLED_PrintString(" M:");
    OLED_PrintString(CarController_RunModeToString(CarController_GetRunMode()));

    OLED_SetCursor(2, 0);
    OLED_PrintString("RAW:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));

    OLED_SetCursor(4, 0);
    OLED_PrintString("CNT:");
    OLED_PrintInt16((int16_t)blackCount);

    OLED_SetCursor(6, 0);
    OLED_PrintString("LOST:");
    OLED_PrintInt16((int16_t)g_appRuntime.lost_count);
    OLED_PrintString(" T:");
    OLED_PrintInt16((int16_t)g_trackTurnDebug);
    OLED_PrintString(" E:");
    OLED_PrintInt16(error);
}

static void print_imu_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    int16_t gyroDpsX10 =
        clamp_display_i16((int32_t)(imu->corrected_gyro_z_dps * 10.0f));
    int16_t biasDpsX10 =
        clamp_display_i16((int32_t)(imu->gyro_bias_dps * 10.0f));
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);

    OLED_SetCursor(0, 0);
    OLED_PrintString("IMU:");
    if (imu->initialized) {
        OLED_PrintString("OK");
    } else {
        OLED_PrintString("ERR");
    }
    OLED_PrintString(" C:");
    OLED_PrintInt16((int16_t)imu->last_error_code);
    OLED_PrintString(" b:");
    OLED_PrintInt16((int16_t)imu->bus_state);

    OLED_SetCursor(2, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" CAL:");
    OLED_PrintInt16((int16_t)imu->calibrated);

    OLED_SetCursor(4, 0);
    OLED_PrintString("R:");
    OLED_PrintInt16(imu->raw_gyro_z);
    OLED_PrintString(" G:");
    OLED_PrintInt16(gyroDpsX10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("BI:");
    OLED_PrintInt16(biasDpsX10);
    OLED_PrintString(" ID:");
    OLED_PrintInt16((int16_t)imu->last_who_am_i);
}

static void print_heading_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    const HeadingControlRuntime *heading = HeadingControl_GetRuntime();
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);
    int16_t targetDeg =
        clamp_display_i16((int32_t)heading->target_yaw_deg);
    int16_t errorX10 =
        clamp_display_i16((int32_t)(heading->heading_error_deg * 10.0f));
    int16_t derivativeX10 =
        clamp_display_i16((int32_t)(heading->heading_derivative * 10.0f));

    OLED_SetCursor(0, 0);
    OLED_PrintString("HEAD OBS");

    OLED_SetCursor(2, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" T:");
    OLED_PrintInt16(targetDeg);

    OLED_SetCursor(4, 0);
    OLED_PrintString("E10:");
    OLED_PrintInt16(errorX10);
    OLED_PrintString(" D10:");
    OLED_PrintInt16(derivativeX10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(heading->correction);
    OLED_PrintString(" LK:");
    OLED_PrintInt16((int16_t)heading->target_locked);
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

static void print_gimbal_page(uint8_t keyEvent)
{
    const GimbalFeedback *gimbal = Gimbal_GetFeedback();

    OLED_SetCursor(0, 0);
    OLED_PrintString("GIMBAL P7R2");

    OLED_SetCursor(2, 0);
    OLED_PrintString("K2S:+5 K2L:H");

    OLED_SetCursor(4, 0);
    OLED_PrintString("K3:REL T5:");
    OLED_PrintInt16(clamp_display_i16((int32_t)gimbal->control_tick_5ms));

    OLED_SetCursor(6, 0);
    OLED_PrintString("M:");
    OLED_PrintString(gimbal_mode_to_string(gimbal->mode));
    OLED_PrintString(" E:");
    OLED_PrintInt16((int16_t)gimbal->enabled);
    OLED_PrintString(" K:");
    OLED_PrintInt16((int16_t)keyEvent);
}
#endif

void OledUi_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

void OledUi_Update_20ms(uint8_t raw, uint8_t blackCount, int16_t error,
    uint8_t keyEvent)
{
    static uint8_t refreshDivider;

    refreshDivider++;
    if (refreshDivider < 5U) {
        return;
    }
    refreshDivider = 0;

    OLED_Clear();

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
        case OLED_PAGE_HEADING:
            print_heading_page();
            break;
        case OLED_PAGE_OBSTACLE:
            print_obstacle_page();
            break;
        case OLED_PAGE_GIMBAL:
#if FEATURE_GIMBAL_OLED_TEST
            print_gimbal_page(keyEvent);
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
