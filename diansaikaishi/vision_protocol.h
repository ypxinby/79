#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_PROTOCOL_MAGIC_0              (0xA5U)
#define VISION_PROTOCOL_MAGIC_1              (0x5AU)
#define VISION_PROTOCOL_VERSION_V1           (0x01U)
#define VISION_PROTOCOL_TYPE_TARGET_REPORT   (0x01U)
#define VISION_PROTOCOL_PAYLOAD_LENGTH_V1    (30U)
#define VISION_PROTOCOL_FRAME_LENGTH_V1      (40U)

#define VISION_FLAG_TARGET_VALID             (1U << 0)
#define VISION_FLAG_HAS_BBOX                 (1U << 1)
#define VISION_FLAG_HAS_TARGET_ID            (1U << 2)
#define VISION_FLAG_HAS_CONFIDENCE           (1U << 3)
#define VISION_FLAG_SOURCE_RESTART           (1U << 4)
#define VISION_FLAG_RESERVED_MASK            (0xE0U)

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

uint16_t VisionProtocol_Crc16CcittFalse(const uint8_t *data,
    uint16_t length);
VisionProtocolParseResult VisionProtocol_ParseV1TargetFrame(
    const uint8_t frame[VISION_PROTOCOL_FRAME_LENGTH_V1],
    VisionTargetPacket *packet);

#endif
