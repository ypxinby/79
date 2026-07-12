#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "mission_library.h"

typedef enum {
    MISSION_STATUS_IDLE = 0,
    MISSION_STATUS_READY,
    MISSION_STATUS_RUNNING,
    MISSION_STATUS_PAUSED,
    MISSION_STATUS_DONE,
    MISSION_STATUS_ERROR
} MissionStatus;

typedef struct {
    const MissionDefinition *definition;
    MissionStatus status;

    uint16_t current_action_index;
    uint8_t current_retry_count;

    uint32_t mission_elapsed_ms;
    uint32_t action_elapsed_ms;

    float mission_start_yaw;

    MotionActionResult last_action_result;
    uint16_t last_error_code;
} MissionRuntime;

void MissionManager_Init(void);
bool MissionManager_Select(uint8_t mission_id);
bool MissionManager_SelectNext(void);
bool MissionManager_SelectPrevious(void);
bool MissionManager_Start(void);
void MissionManager_Pause(void);
void MissionManager_Resume(void);
void MissionManager_Cancel(void);
void MissionManager_Reset(void);
void MissionManager_Update_20ms(void);
const MissionRuntime *MissionManager_GetRuntime(void);
uint8_t MissionManager_GetSelectedMissionId(void);
uint16_t MissionManager_GetSelectedMissionIndex(void);
uint16_t MissionManager_GetMissionCount(void);

#endif
