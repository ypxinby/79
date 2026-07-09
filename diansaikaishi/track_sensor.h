#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifndef TRACK_BLACK_LEVEL
#define TRACK_BLACK_LEVEL       (1)
#endif

#define TRACK_SENSOR_COUNT      (7U)
#define TRACK_RAW_VALID_MASK    (0x7FU)
#define TRACK_LEFT_EDGE_MASK    ((uint8_t)((1U << 0) | (1U << 1)))
#define TRACK_RIGHT_EDGE_MASK   ((uint8_t)((1U << 5) | (1U << 6)))
#define TRACK_CENTER_MASK       ((uint8_t)((1U << 2) | (1U << 3) | (1U << 4)))

typedef enum {
    TRACK_TURN_NONE = 0,
    TRACK_TURN_LEFT_90,
    TRACK_TURN_RIGHT_90
} TrackTurnType;

void TrackSensor_Init(void);
uint8_t TrackSensor_ReadRaw(void);
uint8_t TrackSensor_CountBlack(uint8_t raw);
int16_t TrackSensor_GetErrorFromRaw(uint8_t raw);
bool TrackSensor_IsLineLost(uint8_t raw);
bool TrackSensor_IsStartLine(uint8_t raw, uint8_t threshold);
bool TrackSensor_IsLineDetected(uint8_t raw);
bool TrackSensor_IsCenterDetected(uint8_t raw);
TrackTurnType TrackSensor_DetectTurn(uint8_t raw, int16_t error);

#endif
