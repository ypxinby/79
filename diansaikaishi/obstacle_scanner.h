#ifndef OBSTACLE_SCANNER_H
#define OBSTACLE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBSTACLE_SCAN_IDLE = 0,
    OBSTACLE_SCAN_MOVE_CENTER,
    OBSTACLE_SCAN_WAIT_CENTER,
    OBSTACLE_SCAN_SAMPLE_CENTER,
    OBSTACLE_SCAN_MOVE_LEFT,
    OBSTACLE_SCAN_WAIT_LEFT,
    OBSTACLE_SCAN_SAMPLE_LEFT,
    OBSTACLE_SCAN_MOVE_RIGHT,
    OBSTACLE_SCAN_WAIT_RIGHT,
    OBSTACLE_SCAN_SAMPLE_RIGHT,
    OBSTACLE_SCAN_RETURN_CENTER,
    OBSTACLE_SCAN_COMPLETE
} ObstacleScanState;

typedef enum {
    OBSTACLE_DIRECTION_NONE = 0,
    OBSTACLE_DIRECTION_LEFT,
    OBSTACLE_DIRECTION_RIGHT,
    OBSTACLE_DIRECTION_BLOCKED,
    OBSTACLE_DIRECTION_UNKNOWN
} ObstacleDirection;

typedef struct {
    ObstacleScanState state;
    ObstacleDirection recommended_direction;
    bool active;
    bool complete;
    bool center_valid;
    bool left_valid;
    bool right_valid;
    uint16_t center_distance_cm;
    uint16_t left_distance_cm;
    uint16_t right_distance_cm;
    int16_t servo_target_angle_deg;
} ObstacleScanFeedback;

void ObstacleScanner_Init(void);
void ObstacleScanner_Update_20ms(void);
const ObstacleScanFeedback *ObstacleScanner_GetFeedback(void);
const char *ObstacleScanner_DirectionToString(ObstacleDirection direction);

#endif
