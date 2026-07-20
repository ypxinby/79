#include "vision_pitch_tuning.h"

static VisionPitchTuningStatus g_status;

static VisionPitchTuningParams default_params(void)
{
    VisionPitchTuningParams params = {
        .deadband_px = VISION_PITCH_TUNING_DEFAULT_DEADBAND_PX,
        .kp_x1000 = VISION_PITCH_TUNING_DEFAULT_KP_X1000,
        .max_speed_deg_s_x10 =
            VISION_PITCH_TUNING_DEFAULT_MAX_SPEED_X10,
        .filter_alpha_x1000 =
            VISION_PITCH_TUNING_DEFAULT_FILTER_ALPHA_X1000,
        .observation_timeout_ms =
            VISION_PITCH_TUNING_DEFAULT_TIMEOUT_MS
    };

    return params;
}

static uint8_t value_is_valid(VisionPitchTuningParam parameter,
    uint16_t value)
{
    switch (parameter) {
        case VISION_PITCH_TUNING_PARAM_DEADBAND:
            return (value <= 100U) ? 1U : 0U;
        case VISION_PITCH_TUNING_PARAM_KP:
            return (value <= 1000U) ? 1U : 0U;
        case VISION_PITCH_TUNING_PARAM_MAX_SPEED:
            return ((value >= 30U) && (value <= 240U)) ? 1U : 0U;
        case VISION_PITCH_TUNING_PARAM_FILTER_ALPHA:
            return (value <= 1000U) ? 1U : 0U;
        case VISION_PITCH_TUNING_PARAM_TIMEOUT:
            return ((value >= 50U) && (value <= 5000U)) ? 1U : 0U;
        default:
            return 0U;
    }
}

static uint16_t *parameter_address(VisionPitchTuningParam parameter)
{
    switch (parameter) {
        case VISION_PITCH_TUNING_PARAM_DEADBAND:
            return &g_status.params.deadband_px;
        case VISION_PITCH_TUNING_PARAM_KP:
            return &g_status.params.kp_x1000;
        case VISION_PITCH_TUNING_PARAM_MAX_SPEED:
            return &g_status.params.max_speed_deg_s_x10;
        case VISION_PITCH_TUNING_PARAM_FILTER_ALPHA:
            return &g_status.params.filter_alpha_x1000;
        case VISION_PITCH_TUNING_PARAM_TIMEOUT:
            return &g_status.params.observation_timeout_ms;
        default:
            return 0;
    }
}

void VisionPitchTuning_Init(void)
{
    g_status.params = default_params();
    g_status.last_change = (VisionPitchTuningLastChange){0};
    g_status.revision = 0U;
}

void VisionPitchTuning_GetSnapshot(VisionPitchTuningParams *params)
{
    if (params != 0) {
        *params = g_status.params;
    }
}

const VisionPitchTuningStatus *VisionPitchTuning_GetStatus(void)
{
    return &g_status;
}

uint8_t VisionPitchTuning_Set(VisionPitchTuningParam parameter,
    uint16_t value)
{
    uint16_t *destination;
    uint16_t oldValue;

    if (value_is_valid(parameter, value) == 0U) {
        return 0U;
    }

    destination = parameter_address(parameter);
    if (destination == 0) {
        return 0U;
    }

    oldValue = *destination;
    *destination = value;
    g_status.last_change.parameter = parameter;
    g_status.last_change.old_value = oldValue;
    g_status.last_change.new_value = value;
    g_status.last_change.valid = 1U;
    g_status.revision++;
    return 1U;
}

void VisionPitchTuning_RestoreDefault(void)
{
    g_status.params = default_params();
    g_status.last_change.parameter = VISION_PITCH_TUNING_PARAM_DEFAULT;
    g_status.last_change.old_value = 0U;
    g_status.last_change.new_value = 0U;
    g_status.last_change.valid = 1U;
    g_status.revision++;
}

const char *VisionPitchTuning_ParamName(VisionPitchTuningParam parameter)
{
    switch (parameter) {
        case VISION_PITCH_TUNING_PARAM_DEADBAND:
            return "DB";
        case VISION_PITCH_TUNING_PARAM_KP:
            return "KP";
        case VISION_PITCH_TUNING_PARAM_MAX_SPEED:
            return "MS";
        case VISION_PITCH_TUNING_PARAM_FILTER_ALPHA:
            return "AL";
        case VISION_PITCH_TUNING_PARAM_TIMEOUT:
            return "TO";
        case VISION_PITCH_TUNING_PARAM_DEFAULT:
            return "DEF";
        case VISION_PITCH_TUNING_PARAM_NONE:
        default:
            return "NONE";
    }
}
