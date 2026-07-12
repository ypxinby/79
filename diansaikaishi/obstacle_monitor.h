#ifndef OBSTACLE_MONITOR_H
#define OBSTACLE_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBSTACLE_STATE_CLEAR = 0,
    OBSTACLE_STATE_BLOCKED
} ObstacleState;

typedef struct {
    ObstacleState state;
    bool blocked;
    bool distance_valid;
    uint16_t distance_cm;
    uint8_t block_confirm_count;
    uint8_t clear_confirm_count;
} ObstacleFeedback;

void ObstacleMonitor_Init(void);
void ObstacleMonitor_Update_20ms(void);
const ObstacleFeedback *ObstacleMonitor_GetFeedback(void);

#endif
