#ifndef VISION_YAW_TUNING_H
#define VISION_YAW_TUNING_H

#include <stdint.h>

#define VISION_YAW_TUNING_DEFAULT_DEADBAND_PX   (8U)
#define VISION_YAW_TUNING_DEFAULT_KP_X1000      (80U)
#define VISION_YAW_TUNING_DEFAULT_MAX_SPEED_X10 (240U)
#define VISION_YAW_TUNING_DEFAULT_TIMEOUT_MS    (300U)

typedef enum {
    VISION_YAW_TUNING_PARAM_NONE = 0,
    VISION_YAW_TUNING_PARAM_DEADBAND,
    VISION_YAW_TUNING_PARAM_KP,
    VISION_YAW_TUNING_PARAM_MAX_SPEED,
    VISION_YAW_TUNING_PARAM_TIMEOUT,
    VISION_YAW_TUNING_PARAM_DEFAULT
} VisionYawTuningParam;

typedef struct {
    uint16_t deadband_px;
    uint16_t kp_x1000;
    uint16_t max_speed_deg_s_x10;
    uint16_t observation_timeout_ms;
} VisionYawTuningParams;

typedef struct {
    VisionYawTuningParam parameter;
    uint16_t old_value;
    uint16_t new_value;
    uint8_t valid;
} VisionYawTuningLastChange;

typedef struct {
    VisionYawTuningParams params;
    VisionYawTuningLastChange last_change;
    uint32_t revision;
} VisionYawTuningStatus;

void VisionYawTuning_Init(void);
void VisionYawTuning_GetSnapshot(VisionYawTuningParams *params);
const VisionYawTuningStatus *VisionYawTuning_GetStatus(void);
uint8_t VisionYawTuning_Set(VisionYawTuningParam parameter,
    uint16_t value);
void VisionYawTuning_RestoreDefault(void);
const char *VisionYawTuning_ParamName(VisionYawTuningParam parameter);

#endif
