#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

/* Simplified wired K230 protocol:
 * @B,seq,time,valid,state,pos,pred,v,conf,measured*CS\n
 * CS is the two-digit hexadecimal XOR of every byte after '@' and before
 * '*'. Numeric position and velocity fields use millimetres and mm/s. */
#define VISION_PROTOCOL_LINE_MAX_LENGTH       (96U)
#define VISION_PROTOCOL_STATE_MAX_LENGTH      (12U)

#define VISION_FLAG_TARGET_VALID              (1U << 0)
#define VISION_FLAG_HAS_CONFIDENCE            (1U << 3)

typedef struct {
    uint8_t flags;
    uint32_t session_id;
    uint16_t sequence;
    uint32_t source_timestamp_ms;
    uint16_t frame_width;
    uint16_t frame_height;
    uint16_t target_center_x;
    uint16_t target_center_y;
    uint16_t confidence;
    uint16_t target_id;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_width;
    uint16_t bbox_height;
} VisionTargetPacket;

typedef struct {
    uint16_t sequence;
    uint32_t source_timestamp_ms;
    int16_t position_mm;
    int16_t predicted_position_mm;
    int16_t velocity_mm_s;
    uint16_t confidence;
    uint8_t target_valid;
    uint8_t measured;
    char state[VISION_PROTOCOL_STATE_MAX_LENGTH];
} VisionBallAsciiPacket;

typedef enum {
    VISION_PROTOCOL_PARSE_OK = 0,
    VISION_PROTOCOL_PARSE_ARGUMENT_ERROR,
    VISION_PROTOCOL_PARSE_MAGIC_ERROR,
    VISION_PROTOCOL_PARSE_LENGTH_ERROR,
    VISION_PROTOCOL_PARSE_CRC_ERROR,
    VISION_PROTOCOL_PARSE_VERSION_ERROR,
    VISION_PROTOCOL_PARSE_TYPE_ERROR,
    VISION_PROTOCOL_PARSE_RESERVED_ERROR,
    VISION_PROTOCOL_PARSE_FLAGS_ERROR,
    VISION_PROTOCOL_PARSE_FIELD_ERROR
} VisionProtocolParseResult;

uint8_t VisionProtocol_XorChecksum(const uint8_t *data,
    uint16_t length);
VisionProtocolParseResult VisionProtocol_ParseBallAsciiLine(
    const uint8_t *line, uint16_t length,
    VisionBallAsciiPacket *packet);

#endif
