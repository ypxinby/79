#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include <stdbool.h>

void EmergencyStop_Init(void);
void EmergencyStop_Trigger(void);
void EmergencyStop_Enforce(void);
void EmergencyStop_Reset(void);
bool EmergencyStop_IsActive(void);

#endif
