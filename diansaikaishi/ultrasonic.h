#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool echo_high;
    bool measurement_valid;
    uint16_t distance_cm;
    uint16_t echo_ticks_100us;
    uint16_t trigger_count;
    uint16_t update_count;
    uint8_t state;
} UltrasonicFeedback;

void Ultrasonic_Init(void);
void Ultrasonic_Tick100us(void);
void Ultrasonic_Update_20ms(void);
const UltrasonicFeedback *Ultrasonic_GetFeedback(void);

#endif
