#include "fault.h"

static FaultRecord g_faultRecord;

void Fault_Init(void)
{
    g_faultRecord.code = FAULT_CODE_NONE;
    g_faultRecord.detail = 0U;
    g_faultRecord.context = 0U;
    g_faultRecord.timestamp_ms = 0U;
    g_faultRecord.occurrence_count = 0U;
}

void Fault_Raise(FaultCode code, uint16_t detail, uint16_t context,
    uint32_t timestamp_ms)
{
    if (code == FAULT_CODE_NONE) {
        return;
    }

    if ((g_faultRecord.code == FAULT_CODE_SOFTWARE_EMERGENCY_STOP) &&
        (code != FAULT_CODE_SOFTWARE_EMERGENCY_STOP)) {
        return;
    }

    g_faultRecord.code = code;
    g_faultRecord.detail = detail;
    g_faultRecord.context = context;
    g_faultRecord.timestamp_ms = timestamp_ms;
    if (g_faultRecord.occurrence_count < UINT32_MAX) {
        g_faultRecord.occurrence_count++;
    }
}

void Fault_Clear(void)
{
    Fault_Init();
}

const FaultRecord *Fault_GetRecord(void)
{
    return &g_faultRecord;
}

const char *FaultCode_ToShortString(FaultCode code)
{
    switch (code) {
        case FAULT_CODE_NONE:
            return "OK";
        case FAULT_CODE_TURN_YAW_TIMEOUT:
            return "YTO";
        case FAULT_CODE_TURN_YAW_IMU_NOT_READY:
            return "YIMU";
        case FAULT_CODE_AVOID_TURN_OUT:
            return "AOUT";
        case FAULT_CODE_AVOID_DRIVE_OUT:
            return "ADRV";
        case FAULT_CODE_AVOID_TURN_TO_LINE:
            return "ALIN";
        case FAULT_CODE_AVOID_REACQUIRE_SEARCH:
            return "ASRC";
        case FAULT_CODE_AVOID_REACQUIRE_SETTLE:
            return "ASET";
        case FAULT_CODE_AVOID_INVALID_STATE:
            return "AINV";
        case FAULT_CODE_APP_HEARTBEAT_TIMEOUT:
            return "HBT";
        case FAULT_CODE_SOFTWARE_EMERGENCY_STOP:
            return "ESTP";
        case FAULT_CODE_CONTROLLER_INVALID_MODE:
            return "CTRL";
        case FAULT_CODE_MOTOR_CONTROL:
            return "MCTL";
        default:
            return "UNK";
    }
}
