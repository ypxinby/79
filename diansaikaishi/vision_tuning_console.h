#ifndef VISION_TUNING_CONSOLE_H
#define VISION_TUNING_CONSOLE_H

#include <stdint.h>

typedef struct {
    volatile uint32_t rx_byte_count;
    volatile uint32_t rx_overflow_count;
    uint32_t command_count;
    uint32_t success_count;
    uint32_t error_count;
    uint32_t tx_overflow_count;
} VisionTuningConsoleStatus;

void VisionTuningConsole_Init(void);
void VisionTuningConsole_PushByteFromIsr(uint8_t byte);
uint16_t VisionTuningConsole_Process(uint16_t maxBytesToProcess);
uint8_t VisionTuningConsole_TryPopTxByte(uint8_t *byte);
const VisionTuningConsoleStatus *VisionTuningConsole_GetStatus(void);

#endif
