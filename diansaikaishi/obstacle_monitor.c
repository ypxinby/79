#include "obstacle_monitor.h"

#include "app_config.h"
#include "servo.h"
#include "ultrasonic.h"

#define OBSTACLE_BLOCK_DISTANCE_CM       (20U)
#define OBSTACLE_CLEAR_DISTANCE_CM       (25U)
#define OBSTACLE_STOP_CONFIRM_COUNT      (3U)
#define OBSTACLE_CLEAR_CONFIRM_COUNT     (3U)

static ObstacleFeedback g_obstacleFeedback;

static bool obstacle_monitor_servo_is_centered(void)
{
    const ServoFeedback *servo = Servo_GetFeedback();

    return servo->target_angle_deg == g_appConfig.servo_angle_deg;
}

static void obstacle_set_clear(void)
{
    g_obstacleFeedback.state = OBSTACLE_STATE_CLEAR;
    g_obstacleFeedback.blocked = false;
    g_obstacleFeedback.block_confirm_count = 0U;
    g_obstacleFeedback.clear_confirm_count = 0U;
}

static void obstacle_set_blocked(void)
{
    g_obstacleFeedback.state = OBSTACLE_STATE_BLOCKED;
    g_obstacleFeedback.blocked = true;
    g_obstacleFeedback.block_confirm_count = 0U;
    g_obstacleFeedback.clear_confirm_count = 0U;
}

void ObstacleMonitor_Init(void)
{
    g_obstacleFeedback.distance_valid = false;
    g_obstacleFeedback.distance_cm = 0U;
    obstacle_set_clear();
}

void ObstacleMonitor_Update_20ms(void)
{
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();

    g_obstacleFeedback.distance_valid = ultrasonic->measurement_valid;
    if (!ultrasonic->measurement_valid) {
        g_obstacleFeedback.distance_cm = 0U;
        return;
    }

    g_obstacleFeedback.distance_cm = ultrasonic->distance_cm;

    if (!obstacle_monitor_servo_is_centered()) {
        return;
    }

    if (g_obstacleFeedback.state == OBSTACLE_STATE_BLOCKED) {
        if (ultrasonic->distance_cm >= OBSTACLE_CLEAR_DISTANCE_CM) {
            if (g_obstacleFeedback.clear_confirm_count < UINT8_MAX) {
                g_obstacleFeedback.clear_confirm_count++;
            }
            if (g_obstacleFeedback.clear_confirm_count >=
                OBSTACLE_CLEAR_CONFIRM_COUNT) {
                obstacle_set_clear();
            }
        } else {
            g_obstacleFeedback.clear_confirm_count = 0U;
        }
        return;
    }

    if (ultrasonic->distance_cm < OBSTACLE_BLOCK_DISTANCE_CM) {
        if (g_obstacleFeedback.block_confirm_count < UINT8_MAX) {
            g_obstacleFeedback.block_confirm_count++;
        }
        if (g_obstacleFeedback.block_confirm_count >=
            OBSTACLE_STOP_CONFIRM_COUNT) {
            obstacle_set_blocked();
        }
    } else {
        g_obstacleFeedback.block_confirm_count = 0U;
    }
}

const ObstacleFeedback *ObstacleMonitor_GetFeedback(void)
{
    return &g_obstacleFeedback;
}
