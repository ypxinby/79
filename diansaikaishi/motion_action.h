#ifndef MOTION_ACTION_H
#define MOTION_ACTION_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"

typedef struct {
    const MotionAction *action;
    MotionActionResult result;
    uint32_t elapsed_ms;
    uint16_t imu_wait_elapsed_ms;
    uint16_t encoder_invalid_elapsed_ms;
    uint16_t error_code;
    float follow_finish_start_distance_cm;
    float follow_finish_travelled_cm;
    float follow_finish_decel_distance_cm;
    int16_t follow_finish_high_command;
    int16_t follow_finish_low_command;
    uint8_t follow_finish_confirm_count;
    bool follow_finish_low_speed;
    bool started;
    bool controller_started;
    bool waiting_for_imu;
} MotionActionRuntime;

void MotionAction_Init(void);
bool MotionAction_Start(const MotionAction *action);
bool MotionAction_Resume(const MotionAction *action);
MotionActionResult MotionAction_Update_20ms(uint32_t elapsed_ms);
void MotionAction_Cancel(void);
bool MotionAction_ReapplyControllerTarget(void);
bool MotionAction_RefreshFollowLineTuning(void);
ObstaclePolicy MotionAction_GetCurrentObstaclePolicy(void);
BypassDirection MotionAction_GetCurrentBypassDirection(void);
const MotionActionRuntime *MotionAction_GetRuntime(void);

#endif
