#ifndef OBSTACLE_SAFETY_H
#define OBSTACLE_SAFETY_H

#include <stdbool.h>

void ObstacleSafety_Init(void);
void ObstacleSafety_Update_20ms(void);
bool ObstacleSafety_IsHolding(void);

#endif
