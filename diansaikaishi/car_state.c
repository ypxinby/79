#include "car_state.h"

static CarState g_carState;

void CarState_Init(void)
{
    g_carState = CAR_STATE_READY;
}

void CarState_Set(CarState state)
{
    g_carState = state;
}

CarState CarState_Get(void)
{
    return g_carState;
}

const char *CarState_ToString(CarState state)
{
    switch (state) {
        case CAR_STATE_MENU:
            return "MENU";
        case CAR_STATE_READY:
            return "READY";
        case CAR_STATE_RUNNING:
            return "RUN";
        case CAR_STATE_PAUSED:
            return "PAUSE";
        case CAR_STATE_FINISHED:
            return "DONE";
        case CAR_STATE_ERROR:
            return "ERR";
        default:
            return "ERR";
    }
}
