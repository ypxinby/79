#ifndef STRAIGHT_CONTROL_H
#define STRAIGHT_CONTROL_H

#include <stdint.h>

#define STRAIGHT_CONTROL_PERIOD_MS  (20U)

void StraightControl_Init(void);
void StraightControl_Start(void);
void StraightControl_Stop(void);
void StraightControl_Update(void);

#endif
