#ifndef H_TASK_CONTROLLER_H
#define H_TASK_CONTROLLER_H

#include <stdint.h>

typedef enum {
    H_TASK_ID_NONE = 0,
    H_TASK_ID_H3 = 3,
    H_TASK_ID_H4 = 4,
    H_TASK_ID_H5 = 5,
    H_TASK_ID_H6 = 6
} HTaskId;

typedef enum {
    H_TASK_STATE_IDLE = 0,
    H_TASK_STATE_START_VERIFY,
    H_TASK_STATE_WAIT_BALL_ACTIVE,
    H_TASK_STATE_BALL_POSITIVE,
    H_TASK_STATE_BALL_NEGATIVE,
    H_TASK_STATE_CAR_RUNNING,
    H_TASK_STATE_VISION_HOLD,
    H_TASK_STATE_DONE,
    H_TASK_STATE_FAULT,
    H_TASK_STATE_ABORTED
} HTaskState;

typedef enum {
    H_TASK_VEHICLE_STOP = 0,
    H_TASK_VEHICLE_LOW,
    H_TASK_VEHICLE_NORMAL
} HTaskVehicleLevel;

typedef enum {
    H_TASK_FAULT_NONE = 0,
    H_TASK_FAULT_INVALID_TASK,
    H_TASK_FAULT_ESTOP,
    H_TASK_FAULT_AXIS_NOT_READY,
    H_TASK_FAULT_VISION_NOT_READY,
    H_TASK_FAULT_START_TIMEOUT,
    H_TASK_FAULT_TARGET_INVALID,
    H_TASK_FAULT_POSITION_CONTROL,
    H_TASK_FAULT_BALL_CONTROL,
    H_TASK_FAULT_CAR_CONTROL,
    H_TASK_FAULT_TASK_TIMEOUT
} HTaskFault;

typedef struct {
    HTaskId task_id;
    HTaskState state;
    HTaskVehicleLevel vehicle_level;
    HTaskFault fault;
    uint32_t task_elapsed_ms;
    uint32_t stage_elapsed_ms;
    uint32_t completion_time_ms;
    uint32_t start_observation_update_count;
    uint16_t start_sequence;
    int16_t target_mm;
    int16_t target_lock_mm;
    int16_t position_mm;
    int16_t error_mm;
    uint8_t target_lock_valid;
    uint8_t valid_confirm_count;
    uint8_t finish_latched;
    uint8_t overtime;
    uint8_t b_passed_estimate;
    uint8_t recovery_count;
} HTaskRuntime;

void HTaskController_Init(void);
uint8_t HTaskController_Start(HTaskId task_id);
void HTaskController_Update20ms(uint32_t elapsed_ms);
void HTaskController_RequestNormalStop(void);
void HTaskController_Reset(void);
uint8_t HTaskController_IsActive(void);
uint8_t HTaskController_HasFault(void);
void HTaskController_GetSnapshot(HTaskRuntime *snapshot);
const char *HTaskController_StateToString(HTaskState state);
const char *HTaskController_VehicleToString(HTaskVehicleLevel level);

#endif
