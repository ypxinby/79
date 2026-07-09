#include "track_sensor.h"
#include "ti_msp_dl_config.h"

static const int16_t g_trackWeights[TRACK_SENSOR_COUNT] = {
    -300, -200, -100, 0, 100, 200, 300
};

static bool is_black_level(uint32_t level)
{
#if TRACK_BLACK_LEVEL
    return level != 0U;
#else
    return level == 0U;
#endif
}

void TrackSensor_Init(void)
{
}

uint8_t TrackSensor_ReadRaw(void)
{
    uint8_t raw = 0;

    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S1_PIN))) {
        raw |= (1U << 0);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S2_PIN))) {
        raw |= (1U << 1);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S3_PIN))) {
        raw |= (1U << 2);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_A_PORT, GPIO_TRACK_A_S4_PIN))) {
        raw |= (1U << 3);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S5_PIN))) {
        raw |= (1U << 4);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S6_PIN))) {
        raw |= (1U << 5);
    }
    if (is_black_level(DL_GPIO_readPins(GPIO_TRACK_B_PORT, GPIO_TRACK_B_S7_PIN))) {
        raw |= (1U << 6);
    }

    return (uint8_t)(raw & TRACK_RAW_VALID_MASK);
}

uint8_t TrackSensor_CountBlack(uint8_t raw)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < TRACK_SENSOR_COUNT; i++) {
        if ((raw & (uint8_t)(1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

int16_t TrackSensor_GetErrorFromRaw(uint8_t raw)
{
    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < TRACK_SENSOR_COUNT; i++) {
        if ((raw & (uint8_t)(1U << i)) != 0U) {
            sum += g_trackWeights[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0;
    }

    return (int16_t)(sum / count);
}

bool TrackSensor_IsLineLost(uint8_t raw)
{
    return (raw & TRACK_RAW_VALID_MASK) == 0U;
}

bool TrackSensor_IsStartLine(uint8_t raw, uint8_t threshold)
{
    return TrackSensor_CountBlack(raw) >= threshold;
}
