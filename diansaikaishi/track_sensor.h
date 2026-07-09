#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifndef TRACK_BLACK_LEVEL
#define TRACK_BLACK_LEVEL       (1)
#endif

#define TRACK_SENSOR_COUNT      (7U)
#define TRACK_RAW_VALID_MASK    (0x7FU)

void TrackSensor_Init(void);
uint8_t TrackSensor_ReadRaw(void);
uint8_t TrackSensor_CountBlack(uint8_t raw);
int16_t TrackSensor_GetErrorFromRaw(uint8_t raw);
bool TrackSensor_IsLineLost(uint8_t raw);
bool TrackSensor_IsStartLine(uint8_t raw, uint8_t threshold);

#endif
