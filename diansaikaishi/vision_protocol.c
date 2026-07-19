#include "vision_protocol.h"

#include <stddef.h>

#define VISION_PROTOCOL_MAX_IMAGE_SIZE (8192U)
#define VISION_PROTOCOL_MAX_CONFIDENCE (1000U)

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
        ((uint16_t)data[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

uint16_t VisionProtocol_Crc16CcittFalse(const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        uint8_t bit;

        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static uint8_t fields_are_valid(const VisionTargetPacket *packet)
{
    uint8_t targetValid;
    uint8_t hasBbox;
    uint8_t hasTargetId;
    uint8_t hasConfidence;

    if ((packet->session_id == 0U) ||
        (packet->frame_width == 0U) ||
        (packet->frame_width > VISION_PROTOCOL_MAX_IMAGE_SIZE) ||
        (packet->frame_height == 0U) ||
        (packet->frame_height > VISION_PROTOCOL_MAX_IMAGE_SIZE)) {
        return 0U;
    }

    targetValid = ((packet->flags & VISION_FLAG_TARGET_VALID) != 0U);
    hasBbox = ((packet->flags & VISION_FLAG_HAS_BBOX) != 0U);
    hasTargetId = ((packet->flags & VISION_FLAG_HAS_TARGET_ID) != 0U);
    hasConfidence =
        ((packet->flags & VISION_FLAG_HAS_CONFIDENCE) != 0U);

    if (targetValid == 0U) {
        if ((packet->flags & (VISION_FLAG_HAS_BBOX |
             VISION_FLAG_HAS_TARGET_ID |
             VISION_FLAG_HAS_CONFIDENCE)) != 0U) {
            return 0U;
        }
        if ((packet->target_center_x != 0xFFFFU) ||
            (packet->target_center_y != 0xFFFFU) ||
            (packet->confidence != 0U) ||
            (packet->target_id != 0xFFFFU) ||
            (packet->bbox_x != 0U) ||
            (packet->bbox_y != 0U) ||
            (packet->bbox_width != 0U) ||
            (packet->bbox_height != 0U)) {
            return 0U;
        }
        return 1U;
    }

    if ((packet->target_center_x >= packet->frame_width) ||
        (packet->target_center_y >= packet->frame_height)) {
        return 0U;
    }

    if (hasConfidence != 0U) {
        if (packet->confidence > VISION_PROTOCOL_MAX_CONFIDENCE) {
            return 0U;
        }
    } else if (packet->confidence != 0U) {
        return 0U;
    }

    if (hasTargetId != 0U) {
        if (packet->target_id == 0xFFFFU) {
            return 0U;
        }
    } else if (packet->target_id != 0xFFFFU) {
        return 0U;
    }

    if (hasBbox != 0U) {
        if ((packet->bbox_width == 0U) ||
            (packet->bbox_height == 0U) ||
            ((uint32_t)packet->bbox_x + packet->bbox_width >
                packet->frame_width) ||
            ((uint32_t)packet->bbox_y + packet->bbox_height >
                packet->frame_height)) {
            return 0U;
        }
    } else if ((packet->bbox_x != 0U) ||
        (packet->bbox_y != 0U) ||
        (packet->bbox_width != 0U) ||
        (packet->bbox_height != 0U)) {
        return 0U;
    }

    return 1U;
}

VisionProtocolParseResult VisionProtocol_ParseV1TargetFrame(
    const uint8_t frame[VISION_PROTOCOL_FRAME_LENGTH_V1],
    VisionTargetPacket *packet)
{
    uint16_t storedCrc;
    uint16_t computedCrc;

    if ((frame == NULL) || (packet == NULL)) {
        return VISION_PROTOCOL_PARSE_ARGUMENT_ERROR;
    }
    if ((frame[0] != VISION_PROTOCOL_MAGIC_0) ||
        (frame[1] != VISION_PROTOCOL_MAGIC_1)) {
        return VISION_PROTOCOL_PARSE_MAGIC_ERROR;
    }
    if (read_u16_le(&frame[6]) != VISION_PROTOCOL_PAYLOAD_LENGTH_V1) {
        return VISION_PROTOCOL_PARSE_LENGTH_ERROR;
    }

    storedCrc = read_u16_le(&frame[38]);
    computedCrc = VisionProtocol_Crc16CcittFalse(frame, 38U);
    if (storedCrc != computedCrc) {
        return VISION_PROTOCOL_PARSE_CRC_ERROR;
    }
    if (frame[2] != VISION_PROTOCOL_VERSION_V1) {
        return VISION_PROTOCOL_PARSE_VERSION_ERROR;
    }
    if (frame[3] != VISION_PROTOCOL_TYPE_TARGET_REPORT) {
        return VISION_PROTOCOL_PARSE_TYPE_ERROR;
    }
    if (frame[5] != 0U) {
        return VISION_PROTOCOL_PARSE_RESERVED_ERROR;
    }
    if ((frame[4] & VISION_FLAG_RESERVED_MASK) != 0U) {
        return VISION_PROTOCOL_PARSE_FLAGS_ERROR;
    }

    packet->flags = frame[4];
    packet->session_id = read_u32_le(&frame[8]);
    packet->sequence = read_u16_le(&frame[12]);
    packet->source_timestamp_ms = read_u32_le(&frame[14]);
    packet->frame_width = read_u16_le(&frame[18]);
    packet->frame_height = read_u16_le(&frame[20]);
    packet->target_center_x = read_u16_le(&frame[22]);
    packet->target_center_y = read_u16_le(&frame[24]);
    packet->confidence = read_u16_le(&frame[26]);
    packet->target_id = read_u16_le(&frame[28]);
    packet->bbox_x = read_u16_le(&frame[30]);
    packet->bbox_y = read_u16_le(&frame[32]);
    packet->bbox_width = read_u16_le(&frame[34]);
    packet->bbox_height = read_u16_le(&frame[36]);

    if (fields_are_valid(packet) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }

    return VISION_PROTOCOL_PARSE_OK;
}
