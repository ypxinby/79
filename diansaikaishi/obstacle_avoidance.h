#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AVOID_STATE_IDLE = 0,
    AVOID_STATE_TURN_OUT,
    AVOID_STATE_DRIVE_OUT,
    AVOID_STATE_TURN_TO_LINE,
    AVOID_STATE_REACQUIRE_SEARCH,
    AVOID_STATE_REACQUIRE_SETTLE,
    AVOID_STATE_COMPLETE,
    AVOID_STATE_FAILED
} ObstacleAvoidState;

typedef struct {
    ObstacleAvoidState state;
    bool active;
    bool failed;
    uint16_t settle_ms;
    uint8_t center_count;
} ObstacleAvoidanceFeedback;

void ObstacleAvoidance_Init(void);
void ObstacleAvoidance_Update_20ms(void);
bool ObstacleAvoidance_IsActive(void);
bool ObstacleAvoidance_IsFailed(void);
bool ObstacleAvoidance_IsResumeGraceActive(void);
const ObstacleAvoidanceFeedback *ObstacleAvoidance_GetFeedback(void);

#endif
