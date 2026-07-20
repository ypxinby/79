#ifndef DEBUG_TELEMETRY_H
#define DEBUG_TELEMETRY_H

#include <stdint.h>

typedef struct {
    uint32_t frame_count;
    uint32_t suppressed_count;
    uint32_t tx_overflow_count;
} DebugTelemetryStatus;

void DebugTelemetry_Init(void);
void DebugTelemetry_Update(uint32_t timestamp_ms);
uint8_t DebugTelemetry_TryPopTxByte(uint8_t *byte);
const DebugTelemetryStatus *DebugTelemetry_GetStatus(void);

#endif
