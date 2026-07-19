#include "vision_receiver.h"

#include <string.h>

#define VISION_RX_RING_SIZE        (512U)
#define VISION_RX_RING_MASK        (VISION_RX_RING_SIZE - 1U)

#if ((VISION_RX_RING_SIZE & VISION_RX_RING_MASK) != 0U)
#error VISION_RX_RING_SIZE must be a power of two
#endif

static volatile uint8_t g_ring[VISION_RX_RING_SIZE];
static volatile uint16_t g_ringHead;
static volatile uint16_t g_ringTail;
static uint8_t g_frame[VISION_PROTOCOL_FRAME_LENGTH_V1];
static uint8_t g_frameSize;
static VisionReceiverStatus g_status;
static VisionReceiverObservation g_observation;

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
        ((uint16_t)data[1] << 8));
}

static uint8_t ring_pop(uint8_t *byte)
{
    uint16_t tail = g_ringTail;

    if (tail == g_ringHead) {
        return 0U;
    }

    *byte = g_ring[tail];
    g_ringTail = (uint16_t)((tail + 1U) & VISION_RX_RING_MASK);
    return 1U;
}

static void resync_candidate(void)
{
    uint8_t index;
    uint8_t oldSize = g_frameSize;

    g_status.resync_count++;
    for (index = 1U; (uint8_t)(index + 1U) < oldSize; index++) {
        if ((g_frame[index] == VISION_PROTOCOL_MAGIC_0) &&
            (g_frame[index + 1U] == VISION_PROTOCOL_MAGIC_1)) {
            uint8_t retained = (uint8_t)(oldSize - index);

            memmove(g_frame, &g_frame[index], retained);
            g_frameSize = retained;
            g_status.discarded_byte_count += index;
            return;
        }
    }

    if ((oldSize != 0U) &&
        (g_frame[oldSize - 1U] == VISION_PROTOCOL_MAGIC_0)) {
        g_frame[0] = VISION_PROTOCOL_MAGIC_0;
        g_frameSize = 1U;
        g_status.discarded_byte_count += (uint32_t)(oldSize - 1U);
    } else {
        g_frameSize = 0U;
        g_status.discarded_byte_count += oldSize;
    }
}

static void count_parse_error(VisionProtocolParseResult result)
{
    switch (result) {
        case VISION_PROTOCOL_PARSE_LENGTH_ERROR:
            g_status.length_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_LENGTH_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_CRC_ERROR:
            g_status.crc_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_CRC_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_VERSION_ERROR:
            g_status.version_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_VERSION_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_TYPE_ERROR:
            g_status.type_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_TYPE_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_RESERVED_ERROR:
            g_status.reserved_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_RESERVED_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_FLAGS_ERROR:
            g_status.flags_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_FLAGS_ERROR;
            break;
        case VISION_PROTOCOL_PARSE_FIELD_ERROR:
            g_status.field_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_FIELD_ERROR;
            break;
        default:
            g_status.field_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_FIELD_ERROR;
            break;
    }
}

static void accept_packet(const VisionTargetPacket *packet,
    uint32_t localTimeMs, uint8_t newSession)
{
    uint8_t targetValid =
        ((packet->flags & VISION_FLAG_TARGET_VALID) != 0U);

    g_status.session_id = packet->session_id;
    g_status.last_sequence = packet->sequence;
    g_status.session_initialized = 1U;
    g_status.sequence_initialized = 1U;
    g_status.last_valid_packet_time_ms = localTimeMs;
    g_status.accepted_frame_count++;

    g_observation.available = 1U;
    g_observation.target_valid = targetValid;
    g_observation.local_receive_timestamp_ms = localTimeMs;
    g_observation.packet = *packet;

    if (targetValid != 0U) {
        g_status.target_frame_count++;
    } else {
        g_status.no_target_frame_count++;
    }

    if (newSession != 0U) {
        g_status.last_event = VISION_RECEIVER_EVENT_NEW_SESSION;
    } else if (targetValid != 0U) {
        g_status.last_event = VISION_RECEIVER_EVENT_TARGET;
    } else {
        g_status.last_event = VISION_RECEIVER_EVENT_NO_TARGET;
    }
}

static void handle_packet(const VisionTargetPacket *packet,
    uint32_t localTimeMs)
{
    if (g_status.session_initialized == 0U) {
        accept_packet(packet, localTimeMs, 1U);
        return;
    }

    if (packet->session_id != g_status.session_id) {
        g_status.session_change_count++;
        g_status.sequence_initialized = 0U;
        accept_packet(packet, localTimeMs, 1U);
        return;
    }

    if (g_status.sequence_initialized == 0U) {
        accept_packet(packet, localTimeMs, 0U);
        return;
    }

    {
        int16_t delta =
            (int16_t)(uint16_t)(packet->sequence - g_status.last_sequence);

        if (delta > 0) {
            accept_packet(packet, localTimeMs, 0U);
        } else if (delta == 0) {
            g_status.duplicate_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_DUPLICATE;
        } else {
            g_status.old_sequence_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_OLD_SEQUENCE;
        }
    }
}

static void process_complete_frame(uint32_t localTimeMs)
{
    VisionTargetPacket packet;
    VisionProtocolParseResult result =
        VisionProtocol_ParseV1TargetFrame(g_frame, &packet);

    g_status.parsed_frame_count++;
    if (result == VISION_PROTOCOL_PARSE_OK) {
        g_frameSize = 0U;
        handle_packet(&packet, localTimeMs);
    } else {
        count_parse_error(result);
        resync_candidate();
    }
}

static void consume_byte(uint8_t byte, uint32_t localTimeMs)
{
    if (g_frameSize == 0U) {
        if (byte == VISION_PROTOCOL_MAGIC_0) {
            g_frame[0] = byte;
            g_frameSize = 1U;
        } else {
            g_status.discarded_byte_count++;
        }
        return;
    }

    if (g_frameSize == 1U) {
        if (byte == VISION_PROTOCOL_MAGIC_1) {
            g_frame[1] = byte;
            g_frameSize = 2U;
        } else if (byte == VISION_PROTOCOL_MAGIC_0) {
            g_status.discarded_byte_count++;
        } else {
            g_frameSize = 0U;
            g_status.discarded_byte_count += 2U;
        }
        return;
    }

    g_frame[g_frameSize++] = byte;

    if (g_frameSize == 8U) {
        if (read_u16_le(&g_frame[6]) !=
            VISION_PROTOCOL_PAYLOAD_LENGTH_V1) {
            count_parse_error(VISION_PROTOCOL_PARSE_LENGTH_ERROR);
            resync_candidate();
        }
    } else if (g_frameSize == VISION_PROTOCOL_FRAME_LENGTH_V1) {
        process_complete_frame(localTimeMs);
    }
}

void VisionReceiver_Init(void)
{
    g_ringHead = 0U;
    g_ringTail = 0U;
    g_frameSize = 0U;
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_observation, 0, sizeof(g_observation));
    g_status.last_event = VISION_RECEIVER_EVENT_WAITING;
}

void VisionReceiver_PushByteFromIsr(uint8_t byte)
{
    uint16_t head = g_ringHead;
    uint16_t next = (uint16_t)((head + 1U) & VISION_RX_RING_MASK);

    g_status.rx_byte_count++;
    if (next == g_ringTail) {
        g_status.ring_overflow_count++;
        return;
    }

    g_ring[head] = byte;
    g_ringHead = next;
}

uint16_t VisionReceiver_Process(uint32_t localTimeMs,
    uint16_t maxBytesToProcess)
{
    uint16_t processed = 0U;
    uint8_t byte;

    while ((processed < maxBytesToProcess) && (ring_pop(&byte) != 0U)) {
        consume_byte(byte, localTimeMs);
        processed++;
    }

    return processed;
}

const VisionReceiverStatus *VisionReceiver_GetStatus(void)
{
    return &g_status;
}

const VisionReceiverObservation *VisionReceiver_GetObservation(void)
{
    return &g_observation;
}

uint32_t VisionReceiver_GetProtocolErrorCount(void)
{
    return g_status.length_error_count +
        g_status.crc_error_count +
        g_status.version_error_count +
        g_status.type_error_count +
        g_status.reserved_error_count +
        g_status.flags_error_count +
        g_status.field_error_count;
}
