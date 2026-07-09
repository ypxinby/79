#ifndef CAR_STATE_H
#define CAR_STATE_H

typedef enum {
    CAR_STATE_MENU = 0,
    CAR_STATE_READY,
    CAR_STATE_RUNNING,
    CAR_STATE_PAUSED,
    CAR_STATE_FINISHED,
    CAR_STATE_ERROR
} CarState;

void CarState_Init(void);
void CarState_Set(CarState state);
CarState CarState_Get(void);
const char *CarState_ToString(CarState state);

#endif
