#ifndef VISION_PITCH_TUNING_H
#define VISION_PITCH_TUNING_H

#include <stdint.h>

#define VISION_PITCH_TUNING_DEFAULT_DEADBAND_PX        (8U)
#define VISION_PITCH_TUNING_DEFAULT_KP_X1000           (60U)
#define VISION_PITCH_TUNING_DEFAULT_MAX_SPEED_X10      (60U)
#define VISION_PITCH_TUNING_DEFAULT_FILTER_ALPHA_X1000 (1000U)
#define VISION_PITCH_TUNING_DEFAULT_TIMEOUT_MS         (300U)

typedef enum {
    VISION_PITCH_TUNING_PARAM_NONE = 0,
    VISION_PITCH_TUNING_PARAM_DEADBAND,
    VISION_PITCH_TUNING_PARAM_KP,
    VISION_PITCH_TUNING_PARAM_MAX_SPEED,
    VISION_PITCH_TUNING_PARAM_FILTER_ALPHA,
    VISION_PITCH_TUNING_PARAM_TIMEOUT,
    VISION_PITCH_TUNING_PARAM_DEFAULT
} VisionPitchTuningParam;

typedef struct {
    uint16_t deadband_px;
    uint16_t kp_x1000;
    uint16_t max_speed_deg_s_x10;
    uint16_t filter_alpha_x1000;
    uint16_t observation_timeout_ms;
} VisionPitchTuningParams;

typedef struct {
    VisionPitchTuningParam parameter;
    uint16_t old_value;
    uint16_t new_value;
    uint8_t valid;
} VisionPitchTuningLastChange;

typedef struct {
    VisionPitchTuningParams params;
    VisionPitchTuningLastChange last_change;
    uint32_t revision;
} VisionPitchTuningStatus;

void VisionPitchTuning_Init(void);
void VisionPitchTuning_GetSnapshot(VisionPitchTuningParams *params);
const VisionPitchTuningStatus *VisionPitchTuning_GetStatus(void);
uint8_t VisionPitchTuning_Set(VisionPitchTuningParam parameter,
    uint16_t value);
void VisionPitchTuning_RestoreDefault(void);
const char *VisionPitchTuning_ParamName(VisionPitchTuningParam parameter);

#endif
