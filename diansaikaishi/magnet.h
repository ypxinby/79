#ifndef MAGNET_H
#define MAGNET_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    volatile bool enabled;
    volatile uint32_t enable_count;
    volatile uint32_t disable_count;
} MagnetRuntime;

void Magnet_Init(void);
bool Magnet_Set(bool enable);
void Magnet_ForceOff(void);
bool Magnet_IsOn(void);
const volatile MagnetRuntime *Magnet_GetRuntime(void);

#endif
