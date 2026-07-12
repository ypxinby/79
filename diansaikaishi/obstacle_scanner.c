#include "obstacle_scanner.h"

#include "app_config.h"
#include "obstacle_safety.h"
#include "servo.h"
#include "ultrasonic.h"

#define OBSTACLE_SCAN_LEFT_ANGLE_DEG         (145)
#define OBSTACLE_SCAN_RIGHT_ANGLE_DEG        (35)
#define OBSTACLE_SCAN_SETTLE_TICKS           (15U)
#define OBSTACLE_SCAN_DIRECTION_MARGIN_CM    (10U)
#define OBSTACLE_SCAN_MIN_CLEAR_CM           (25U)

static ObstacleScanFeedback g_obstacleScanFeedback;
static uint16_t g_scanWaitTicks;

static void scanner_set_state(ObstacleScanState state)
{
    g_obstacleScanFeedback.state = state;
    g_scanWaitTicks = 0U;
}

static void scanner_set_servo(int16_t angle_deg)
{
    g_obstacleScanFeedback.servo_target_angle_deg = angle_deg;
    Servo_SetAngleDeg(angle_deg);
}

static uint16_t scanner_get_distance(bool *valid)
{
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();

    *valid = ultrasonic->measurement_valid;
    if (!ultrasonic->measurement_valid) {
        return 0U;
    }

    return ultrasonic->distance_cm;
}

static void scanner_update_recommendation(void)
{
    uint16_t left = g_obstacleScanFeedback.left_distance_cm;
    uint16_t right = g_obstacleScanFeedback.right_distance_cm;

    if (!g_obstacleScanFeedback.left_valid ||
        !g_obstacleScanFeedback.right_valid) {
        g_obstacleScanFeedback.recommended_direction =
            OBSTACLE_DIRECTION_UNKNOWN;
        return;
    }

    if ((left < OBSTACLE_SCAN_MIN_CLEAR_CM) &&
        (right < OBSTACLE_SCAN_MIN_CLEAR_CM)) {
        g_obstacleScanFeedback.recommended_direction =
            OBSTACLE_DIRECTION_BLOCKED;
        return;
    }

    if (left >= (uint16_t)(right + OBSTACLE_SCAN_DIRECTION_MARGIN_CM)) {
        g_obstacleScanFeedback.recommended_direction =
            OBSTACLE_DIRECTION_LEFT;
        return;
    }

    if (right >= (uint16_t)(left + OBSTACLE_SCAN_DIRECTION_MARGIN_CM)) {
        g_obstacleScanFeedback.recommended_direction =
            OBSTACLE_DIRECTION_RIGHT;
        return;
    }

    g_obstacleScanFeedback.recommended_direction =
        OBSTACLE_DIRECTION_UNKNOWN;
}

static void scanner_reset_result(void)
{
    g_obstacleScanFeedback.recommended_direction = OBSTACLE_DIRECTION_NONE;
    g_obstacleScanFeedback.active = false;
    g_obstacleScanFeedback.complete = false;
    g_obstacleScanFeedback.center_valid = false;
    g_obstacleScanFeedback.left_valid = false;
    g_obstacleScanFeedback.right_valid = false;
    g_obstacleScanFeedback.center_distance_cm = 0U;
    g_obstacleScanFeedback.left_distance_cm = 0U;
    g_obstacleScanFeedback.right_distance_cm = 0U;
}

void ObstacleScanner_Init(void)
{
    scanner_reset_result();
    g_obstacleScanFeedback.servo_target_angle_deg =
        g_appConfig.servo_angle_deg;
    scanner_set_state(OBSTACLE_SCAN_IDLE);
}

void ObstacleScanner_Update_20ms(void)
{
    bool valid;

    if (!ObstacleSafety_IsHolding()) {
        scanner_reset_result();
        scanner_set_servo(g_appConfig.servo_angle_deg);
        scanner_set_state(OBSTACLE_SCAN_IDLE);
        return;
    }

    switch (g_obstacleScanFeedback.state) {
        case OBSTACLE_SCAN_IDLE:
            scanner_reset_result();
            g_obstacleScanFeedback.active = true;
            scanner_set_state(OBSTACLE_SCAN_MOVE_CENTER);
            break;

        case OBSTACLE_SCAN_MOVE_CENTER:
            scanner_set_servo(g_appConfig.servo_angle_deg);
            scanner_set_state(OBSTACLE_SCAN_WAIT_CENTER);
            break;

        case OBSTACLE_SCAN_WAIT_CENTER:
            g_scanWaitTicks++;
            if (g_scanWaitTicks >= OBSTACLE_SCAN_SETTLE_TICKS) {
                scanner_set_state(OBSTACLE_SCAN_SAMPLE_CENTER);
            }
            break;

        case OBSTACLE_SCAN_SAMPLE_CENTER:
            g_obstacleScanFeedback.center_distance_cm =
                scanner_get_distance(&valid);
            g_obstacleScanFeedback.center_valid = valid;
            scanner_set_state(OBSTACLE_SCAN_MOVE_LEFT);
            break;

        case OBSTACLE_SCAN_MOVE_LEFT:
            scanner_set_servo(OBSTACLE_SCAN_LEFT_ANGLE_DEG);
            scanner_set_state(OBSTACLE_SCAN_WAIT_LEFT);
            break;

        case OBSTACLE_SCAN_WAIT_LEFT:
            g_scanWaitTicks++;
            if (g_scanWaitTicks >= OBSTACLE_SCAN_SETTLE_TICKS) {
                scanner_set_state(OBSTACLE_SCAN_SAMPLE_LEFT);
            }
            break;

        case OBSTACLE_SCAN_SAMPLE_LEFT:
            g_obstacleScanFeedback.left_distance_cm =
                scanner_get_distance(&valid);
            g_obstacleScanFeedback.left_valid = valid;
            scanner_set_state(OBSTACLE_SCAN_MOVE_RIGHT);
            break;

        case OBSTACLE_SCAN_MOVE_RIGHT:
            scanner_set_servo(OBSTACLE_SCAN_RIGHT_ANGLE_DEG);
            scanner_set_state(OBSTACLE_SCAN_WAIT_RIGHT);
            break;

        case OBSTACLE_SCAN_WAIT_RIGHT:
            g_scanWaitTicks++;
            if (g_scanWaitTicks >= OBSTACLE_SCAN_SETTLE_TICKS) {
                scanner_set_state(OBSTACLE_SCAN_SAMPLE_RIGHT);
            }
            break;

        case OBSTACLE_SCAN_SAMPLE_RIGHT:
            g_obstacleScanFeedback.right_distance_cm =
                scanner_get_distance(&valid);
            g_obstacleScanFeedback.right_valid = valid;
            scanner_update_recommendation();
            scanner_set_state(OBSTACLE_SCAN_RETURN_CENTER);
            break;

        case OBSTACLE_SCAN_RETURN_CENTER:
            scanner_set_servo(g_appConfig.servo_angle_deg);
            g_obstacleScanFeedback.complete = true;
            g_obstacleScanFeedback.active = false;
            scanner_set_state(OBSTACLE_SCAN_COMPLETE);
            break;

        case OBSTACLE_SCAN_COMPLETE:
        default:
            scanner_set_servo(g_appConfig.servo_angle_deg);
            break;
    }
}

const ObstacleScanFeedback *ObstacleScanner_GetFeedback(void)
{
    return &g_obstacleScanFeedback;
}

const char *ObstacleScanner_DirectionToString(ObstacleDirection direction)
{
    switch (direction) {
        case OBSTACLE_DIRECTION_LEFT:
            return "L";
        case OBSTACLE_DIRECTION_RIGHT:
            return "R";
        case OBSTACLE_DIRECTION_BLOCKED:
            return "B";
        case OBSTACLE_DIRECTION_UNKNOWN:
            return "U";
        case OBSTACLE_DIRECTION_NONE:
        default:
            return "N";
    }
}
