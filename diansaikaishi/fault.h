#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_TURN_YAW_TIMEOUT = 101,
    FAULT_CODE_TURN_YAW_IMU_NOT_READY = 102,
    FAULT_CODE_AVOID_TURN_OUT = 201,
    FAULT_CODE_AVOID_DRIVE_OUT = 202,
    FAULT_CODE_AVOID_TURN_TO_LINE = 203,
    FAULT_CODE_AVOID_REACQUIRE_SEARCH = 204,
    FAULT_CODE_AVOID_REACQUIRE_SETTLE = 205,
    FAULT_CODE_AVOID_INVALID_STATE = 206,
    FAULT_CODE_APP_HEARTBEAT_TIMEOUT = 301,
    FAULT_CODE_SOFTWARE_EMERGENCY_STOP = 401,
    FAULT_CODE_CONTROLLER_INVALID_MODE = 501
} FaultCode;

typedef struct {
    FaultCode code;
    uint16_t detail;
    uint16_t context;
    uint32_t timestamp_ms;
    uint32_t occurrence_count;
} FaultRecord;

void Fault_Init(void);
void Fault_Raise(FaultCode code, uint16_t detail, uint16_t context,
    uint32_t timestamp_ms);
void Fault_Clear(void);
const FaultRecord *Fault_GetRecord(void);
const char *FaultCode_ToShortString(FaultCode code);

#endif
