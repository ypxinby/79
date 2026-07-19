#ifndef GIMBAL_VISION_ADAPTER_H
#define GIMBAL_VISION_ADAPTER_H

#include <stdint.h>

#include "gimbal_tracker.h"
#include "vision_protocol.h"

typedef enum {
    GIMBAL_VISION_ADAPTER_WAITING = 0,
    GIMBAL_VISION_ADAPTER_TARGET,
    GIMBAL_VISION_ADAPTER_NO_TARGET,
    GIMBAL_VISION_ADAPTER_SOURCE_ERROR
} GimbalVisionAdapterState;

typedef struct {
    uint8_t output_available;
    GimbalVisionAdapterState state;
    uint32_t conversion_count;
    uint32_t conversion_error_count;
    uint32_t receiver_accepted_frame_count;
    uint32_t session_id;
    uint32_t local_receive_timestamp_ms;
    VisionTargetPacket source_packet;
    GimbalTargetObservation observation;
} GimbalVisionAdapterFeedback;

void GimbalVisionAdapter_Init(void);
void GimbalVisionAdapter_Update(void);
const GimbalTargetObservation *GimbalVisionAdapter_GetObservation(void);
const GimbalVisionAdapterFeedback *GimbalVisionAdapter_GetFeedback(void);

#endif
