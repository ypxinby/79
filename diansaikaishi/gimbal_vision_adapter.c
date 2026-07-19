#include "gimbal_vision_adapter.h"

#include "vision_receiver.h"

static GimbalVisionAdapterFeedback g_feedback;
static uint32_t g_lastAcceptedFrameCount;

static uint32_t tracker_timestamp_from_local(uint32_t localTimestampMs)
{
    /* Tracker reserves timestamp 0 for the P23 key simulation path. */
    return (localTimestampMs == 0U) ? 1U : localTimestampMs;
}

static uint8_t source_target_is_valid(
    const VisionReceiverObservation *source)
{
    if ((source->packet.frame_width == 0U) ||
        (source->packet.frame_height == 0U) ||
        (source->packet.target_center_x >= source->packet.frame_width) ||
        (source->packet.target_center_y >= source->packet.frame_height)) {
        return 0U;
    }

    return 1U;
}

void GimbalVisionAdapter_Init(void)
{
    g_feedback = (GimbalVisionAdapterFeedback){0};
    g_feedback.state = GIMBAL_VISION_ADAPTER_WAITING;
    g_lastAcceptedFrameCount = 0U;
}

void GimbalVisionAdapter_Update(void)
{
    const VisionReceiverStatus *receiverStatus =
        VisionReceiver_GetStatus();
    const VisionReceiverObservation *source =
        VisionReceiver_GetObservation();
    GimbalTargetObservation output;

    if ((source->available == 0U) ||
        (receiverStatus->accepted_frame_count == g_lastAcceptedFrameCount)) {
        return;
    }

    g_lastAcceptedFrameCount = receiverStatus->accepted_frame_count;
    output = (GimbalTargetObservation){0};
    output.sequence = source->packet.sequence;
    output.timestamp_ms = tracker_timestamp_from_local(
        source->local_receive_timestamp_ms);

    g_feedback.output_available = 1U;
    g_feedback.conversion_count++;
    g_feedback.receiver_accepted_frame_count =
        receiverStatus->accepted_frame_count;
    g_feedback.session_id = source->packet.session_id;
    g_feedback.local_receive_timestamp_ms =
        source->local_receive_timestamp_ms;
    g_feedback.source_packet = source->packet;

    if (source->target_valid == 0U) {
        output.valid = 0U;
        output.error_x_px = 0;
        output.error_y_px = 0;
        g_feedback.state = GIMBAL_VISION_ADAPTER_NO_TARGET;
    } else if (source_target_is_valid(source) != 0U) {
        int32_t errorX = (int32_t)source->packet.target_center_x -
            (int32_t)(source->packet.frame_width / 2U);
        int32_t errorY = (int32_t)source->packet.target_center_y -
            (int32_t)(source->packet.frame_height / 2U);

        output.valid = 1U;
        output.error_x_px = (int16_t)errorX;
        output.error_y_px = (int16_t)errorY;
        g_feedback.state = GIMBAL_VISION_ADAPTER_TARGET;
    } else {
        output.valid = 0U;
        output.error_x_px = 0;
        output.error_y_px = 0;
        g_feedback.conversion_error_count++;
        g_feedback.state = GIMBAL_VISION_ADAPTER_SOURCE_ERROR;
    }

    g_feedback.observation = output;
}

const GimbalTargetObservation *GimbalVisionAdapter_GetObservation(void)
{
    return &g_feedback.observation;
}

const GimbalVisionAdapterFeedback *GimbalVisionAdapter_GetFeedback(void)
{
    return &g_feedback;
}
