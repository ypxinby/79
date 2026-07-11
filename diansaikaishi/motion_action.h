#ifndef MOTION_ACTION_H
#define MOTION_ACTION_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"

typedef struct {
    const MotionAction *action;
    MotionActionResult result;
    uint32_t elapsed_ms;
    uint16_t error_code;
    bool started;
} MotionActionRuntime;

void MotionAction_Init(void);
bool MotionAction_Start(const MotionAction *action,
    float mission_start_yaw_deg);
MotionActionResult MotionAction_Update_20ms(void);
void MotionAction_Cancel(void);
const MotionActionRuntime *MotionAction_GetRuntime(void);

#endif
