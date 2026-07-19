#ifndef VISION_RECEIVER_H
#define VISION_RECEIVER_H

#include <stdint.h>

#include "vision_protocol.h"

typedef enum {
    VISION_RECEIVER_EVENT_WAITING = 0,
    VISION_RECEIVER_EVENT_TARGET,
    VISION_RECEIVER_EVENT_NO_TARGET,
    VISION_RECEIVER_EVENT_NEW_SESSION,
    VISION_RECEIVER_EVENT_DUPLICATE,
    VISION_RECEIVER_EVENT_OLD_SEQUENCE,
    VISION_RECEIVER_EVENT_LENGTH_ERROR,
    VISION_RECEIVER_EVENT_CRC_ERROR,
    VISION_RECEIVER_EVENT_VERSION_ERROR,
    VISION_RECEIVER_EVENT_TYPE_ERROR,
    VISION_RECEIVER_EVENT_RESERVED_ERROR,
    VISION_RECEIVER_EVENT_FLAGS_ERROR,
    VISION_RECEIVER_EVENT_FIELD_ERROR
} VisionReceiverEvent;

typedef struct {
    uint8_t available;
    uint8_t target_valid;
    uint32_t local_receive_timestamp_ms;
    VisionTargetPacket packet;
} VisionReceiverObservation;

typedef struct {
    volatile uint32_t rx_byte_count;
    volatile uint32_t ring_overflow_count;
    uint32_t discarded_byte_count;
    uint32_t resync_count;
    uint32_t parsed_frame_count;
    uint32_t accepted_frame_count;
    uint32_t target_frame_count;
    uint32_t no_target_frame_count;
    uint32_t length_error_count;
    uint32_t crc_error_count;
    uint32_t version_error_count;
    uint32_t type_error_count;
    uint32_t reserved_error_count;
    uint32_t flags_error_count;
    uint32_t field_error_count;
    uint32_t duplicate_count;
    uint32_t old_sequence_count;
    uint32_t session_change_count;
    uint32_t last_valid_packet_time_ms;
    uint32_t session_id;
    uint16_t last_sequence;
    uint8_t session_initialized;
    uint8_t sequence_initialized;
    VisionReceiverEvent last_event;
} VisionReceiverStatus;

void VisionReceiver_Init(void);
void VisionReceiver_PushByteFromIsr(uint8_t byte);
uint16_t VisionReceiver_Process(uint32_t localTimeMs,
    uint16_t maxBytesToProcess);
const VisionReceiverStatus *VisionReceiver_GetStatus(void);
const VisionReceiverObservation *VisionReceiver_GetObservation(void);
uint32_t VisionReceiver_GetProtocolErrorCount(void);

#endif
