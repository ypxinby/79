#include "vision_yaw_tuning.h"

static VisionYawTuningStatus g_status;

static VisionYawTuningParams default_params(void)
{
    VisionYawTuningParams params = {
        .deadband_px = VISION_YAW_TUNING_DEFAULT_DEADBAND_PX,
        .kp_x1000 = VISION_YAW_TUNING_DEFAULT_KP_X1000,
        .max_speed_deg_s_x10 =
            VISION_YAW_TUNING_DEFAULT_MAX_SPEED_X10,
        .observation_timeout_ms =
            VISION_YAW_TUNING_DEFAULT_TIMEOUT_MS
    };

    return params;
}

static uint8_t value_is_valid(VisionYawTuningParam parameter,
    uint16_t value)
{
    switch (parameter) {
        case VISION_YAW_TUNING_PARAM_DEADBAND:
            return (value <= 100U) ? 1U : 0U;
        case VISION_YAW_TUNING_PARAM_KP:
            return (value <= 1000U) ? 1U : 0U;
        case VISION_YAW_TUNING_PARAM_MAX_SPEED:
            return ((value >= 10U) && (value <= 240U)) ? 1U : 0U;
        case VISION_YAW_TUNING_PARAM_TIMEOUT:
            return ((value >= 50U) && (value <= 5000U)) ? 1U : 0U;
        default:
            return 0U;
    }
}

static uint16_t *parameter_address(VisionYawTuningParam parameter)
{
    switch (parameter) {
        case VISION_YAW_TUNING_PARAM_DEADBAND:
            return &g_status.params.deadband_px;
        case VISION_YAW_TUNING_PARAM_KP:
            return &g_status.params.kp_x1000;
        case VISION_YAW_TUNING_PARAM_MAX_SPEED:
            return &g_status.params.max_speed_deg_s_x10;
        case VISION_YAW_TUNING_PARAM_TIMEOUT:
            return &g_status.params.observation_timeout_ms;
        default:
            return 0;
    }
}

void VisionYawTuning_Init(void)
{
    g_status.params = default_params();
    g_status.last_change = (VisionYawTuningLastChange){0};
    g_status.revision = 0U;
}

void VisionYawTuning_GetSnapshot(VisionYawTuningParams *params)
{
    if (params != 0) {
        *params = g_status.params;
    }
}

const VisionYawTuningStatus *VisionYawTuning_GetStatus(void)
{
    return &g_status;
}

uint8_t VisionYawTuning_Set(VisionYawTuningParam parameter,
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

void VisionYawTuning_RestoreDefault(void)
{
    g_status.params = default_params();
    g_status.last_change.parameter = VISION_YAW_TUNING_PARAM_DEFAULT;
    g_status.last_change.old_value = 0U;
    g_status.last_change.new_value = 0U;
    g_status.last_change.valid = 1U;
    g_status.revision++;
}

const char *VisionYawTuning_ParamName(VisionYawTuningParam parameter)
{
    switch (parameter) {
        case VISION_YAW_TUNING_PARAM_DEADBAND:
            return "DB";
        case VISION_YAW_TUNING_PARAM_KP:
            return "KP";
        case VISION_YAW_TUNING_PARAM_MAX_SPEED:
            return "MS";
        case VISION_YAW_TUNING_PARAM_TIMEOUT:
            return "TO";
        case VISION_YAW_TUNING_PARAM_DEFAULT:
            return "DEF";
        case VISION_YAW_TUNING_PARAM_NONE:
        default:
            return "NONE";
    }
}
