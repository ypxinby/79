#include "vision_receiver.h"

#include <limits.h>
#include <string.h>

#include "app_features.h"

#define VISION_RX_RING_SIZE        (512U)
#define VISION_RX_RING_MASK        (VISION_RX_RING_SIZE - 1U)

#if ((VISION_RX_RING_SIZE & VISION_RX_RING_MASK) != 0U)
#error VISION_RX_RING_SIZE must be a power of two
#endif

static volatile uint8_t g_ring[VISION_RX_RING_SIZE];
static volatile uint16_t g_ringHead;
static volatile uint16_t g_ringTail;
static uint8_t g_line[VISION_PROTOCOL_LINE_MAX_LENGTH];
static uint16_t g_lineSize;
static uint8_t g_discardUntilNewline;
static uint32_t g_lastSourceTimestampMs;
static uint32_t g_localSessionId;
static VisionReceiverStatus g_status;
static VisionReceiverObservation g_observation;
static VisionBallPositionObservation g_ballPositionObservation;

static uint32_t magnitude_i32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }
    return (uint32_t)(-(value + 1)) + 1U;
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
        case VISION_PROTOCOL_PARSE_MAGIC_ERROR:
            g_status.discarded_byte_count += g_lineSize;
            g_status.field_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_FIELD_ERROR;
            break;
        default:
            g_status.field_error_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_FIELD_ERROR;
            break;
    }
}

static void populate_legacy_observation(
    const VisionBallAsciiPacket *packet, uint32_t localTimeMs,
    uint32_t sessionId)
{
    VisionTargetPacket *legacy = &g_observation.packet;

    memset(legacy, 0, sizeof(*legacy));
    legacy->flags = (packet->target_valid != 0U) ?
        (VISION_FLAG_TARGET_VALID | VISION_FLAG_HAS_CONFIDENCE) : 0U;
    legacy->session_id = sessionId;
    legacy->sequence = packet->sequence;
    legacy->source_timestamp_ms = packet->source_timestamp_ms;
    legacy->frame_width = BALANCE_BALL_PHYSICAL_SPAN_MM;
    legacy->frame_height = 1U;
    legacy->target_center_x = (packet->target_valid != 0U) ?
        (uint16_t)packet->position_mm : 0xFFFFU;
    legacy->target_center_y = (packet->target_valid != 0U) ? 0U : 0xFFFFU;
    legacy->confidence = (packet->target_valid != 0U) ?
        packet->confidence : 0U;
    legacy->target_id = 0xFFFFU;
    g_observation.available = 1U;
    g_observation.target_valid = packet->target_valid;
    g_observation.local_receive_timestamp_ms = localTimeMs;
}

static void accept_packet(const VisionBallAsciiPacket *packet,
    uint32_t localTimeMs, uint8_t newSession)
{
    int16_t safePredictedPositionMm = packet->predicted_position_mm;

    /* Prediction is not used by the current controller, so an out-of-range
     * optional pred may safely fall back to the real position. */
    if (magnitude_i32(safePredictedPositionMm) >
        BALANCE_BALL_PROTOCOL_POSITION_LIMIT_MM) {
        safePredictedPositionMm = packet->position_mm;
    }

    g_status.session_id = g_localSessionId;
    g_status.last_sequence = packet->sequence;
    g_status.session_initialized = 1U;
    g_status.sequence_initialized = 1U;
    g_status.last_valid_packet_time_ms = localTimeMs;
    g_status.last_accepted_packet_time_ms = localTimeMs;
    g_status.accepted_frame_count++;
    g_lastSourceTimestampMs = packet->source_timestamp_ms;

    populate_legacy_observation(packet, localTimeMs, g_localSessionId);

    g_ballPositionObservation.available = 1U;
    g_ballPositionObservation.target_valid = packet->target_valid;
    g_ballPositionObservation.measured = packet->measured;
    g_ballPositionObservation.local_receive_timestamp_ms = localTimeMs;
    g_ballPositionObservation.update_count++;
    g_ballPositionObservation.session_id = g_localSessionId;
    g_ballPositionObservation.sequence = packet->sequence;
    /* Preserve the received value for diagnostics even when validity flags
     * prevent it from entering control. */
    g_ballPositionObservation.position_mm = packet->position_mm;
    g_ballPositionObservation.predicted_position_mm =
        safePredictedPositionMm;
    g_ballPositionObservation.reported_velocity_mm_s =
        packet->velocity_mm_s;
    g_ballPositionObservation.axis_span_mm =
        BALANCE_BALL_PHYSICAL_SPAN_MM;
    g_ballPositionObservation.confidence = packet->confidence;
    memcpy(g_ballPositionObservation.state, packet->state,
        sizeof(g_ballPositionObservation.state));

    if (packet->target_valid != 0U) {
        g_status.target_frame_count++;
    } else {
        g_status.no_target_frame_count++;
    }
    if (newSession != 0U) {
        g_status.last_event = VISION_RECEIVER_EVENT_NEW_SESSION;
    } else if (packet->target_valid != 0U) {
        g_status.last_event = VISION_RECEIVER_EVENT_TARGET;
    } else {
        g_status.last_event = VISION_RECEIVER_EVENT_NO_TARGET;
    }
}

static void handle_packet(const VisionBallAsciiPacket *packet,
    uint32_t localTimeMs)
{
    int16_t sequenceDelta;

    /* Keep state as an uninterpreted diagnostic string, as agreed with the
     * K230 group. Only the numeric valid/measured pair is enforced. */
    if (packet->target_valid != packet->measured) {
        g_status.semantic_error_count++;
        g_status.invalid_valid_measured_pair_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_SEMANTIC_ERROR;
        return;
    }

    if ((packet->target_valid != 0U) &&
        (magnitude_i32(packet->position_mm) >
            BALANCE_BALL_PROTOCOL_POSITION_LIMIT_MM)) {
        count_parse_error(VISION_PROTOCOL_PARSE_FIELD_ERROR);
        return;
    }
    /* K230 velocity is now a formal feedback signal. A measured frame with
     * an impossible velocity must be rejected as a whole; silently changing
     * it to zero would create a false speed error and a large actuator kick. */
    if ((packet->target_valid != 0U) &&
        (packet->measured != 0U) &&
        (magnitude_i32(packet->velocity_mm_s) >
            BALANCE_BALL_MAX_REPORTED_SPEED_MM_S)) {
        count_parse_error(VISION_PROTOCOL_PARSE_FIELD_ERROR);
        return;
    }

    if (g_status.session_initialized == 0U) {
        g_localSessionId = 1U;
        accept_packet(packet, localTimeMs, 1U);
        return;
    }

    /* A K230 restart resets its monotonic timestamp. Treat that as a new
     * local session so sequence zero is accepted immediately. */
    if (packet->source_timestamp_ms < g_lastSourceTimestampMs) {
        if (g_localSessionId != UINT32_MAX) {
            g_localSessionId++;
        }
        g_status.session_change_count++;
        g_status.sequence_initialized = 0U;
        accept_packet(packet, localTimeMs, 1U);
        return;
    }

    if (g_status.sequence_initialized == 0U) {
        accept_packet(packet, localTimeMs, 0U);
        return;
    }
    sequenceDelta = (int16_t)(uint16_t)(packet->sequence -
        g_status.last_sequence);
    if (sequenceDelta > 0) {
        accept_packet(packet, localTimeMs, 0U);
    } else if (sequenceDelta == 0) {
        g_status.duplicate_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_DUPLICATE;
    } else {
        g_status.old_sequence_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_OLD_SEQUENCE;
    }
}

static void process_complete_line(uint32_t localTimeMs)
{
    VisionBallAsciiPacket packet;
    VisionProtocolParseResult result;

    g_status.parsed_frame_count++;
    result = VisionProtocol_ParseBallAsciiLine(g_line, g_lineSize,
        &packet);
    if (result == VISION_PROTOCOL_PARSE_OK) {
        handle_packet(&packet, localTimeMs);
    } else {
        count_parse_error(result);
    }
    g_lineSize = 0U;
}

static void consume_byte(uint8_t byte, uint32_t localTimeMs)
{
    if (byte == (uint8_t)'\r') {
        return;
    }
    if (byte == (uint8_t)'\n') {
        if (g_discardUntilNewline != 0U) {
            g_discardUntilNewline = 0U;
            g_lineSize = 0U;
            return;
        }
        if (g_lineSize != 0U) {
            process_complete_line(localTimeMs);
        }
        return;
    }
    if (g_discardUntilNewline != 0U) {
        g_status.discarded_byte_count++;
        return;
    }
    if ((byte < 0x20U) || (byte > 0x7EU)) {
        g_status.field_error_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_FIELD_ERROR;
        g_status.discarded_byte_count += g_lineSize + 1U;
        g_lineSize = 0U;
        g_discardUntilNewline = 1U;
        return;
    }
    if (byte == (uint8_t)'@') {
        if (g_lineSize != 0U) {
            g_status.discarded_byte_count += g_lineSize;
            g_status.resync_count++;
            g_status.last_event = VISION_RECEIVER_EVENT_DISCARDED;
        }
        g_line[0] = byte;
        g_lineSize = 1U;
        return;
    }
    if (g_lineSize == 0U) {
        g_status.discarded_byte_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_DISCARDED;
        return;
    }
    if (g_lineSize >= (VISION_PROTOCOL_LINE_MAX_LENGTH - 1U)) {
        g_status.length_error_count++;
        g_status.last_event = VISION_RECEIVER_EVENT_LENGTH_ERROR;
        g_status.discarded_byte_count += g_lineSize;
        g_lineSize = 0U;
        g_discardUntilNewline = 1U;
        return;
    }
    g_line[g_lineSize++] = byte;
}

void VisionReceiver_Init(void)
{
    g_ringHead = 0U;
    g_ringTail = 0U;
    g_lineSize = 0U;
    g_discardUntilNewline = 0U;
    g_lastSourceTimestampMs = 0U;
    g_localSessionId = 0U;
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_observation, 0, sizeof(g_observation));
    memset(&g_ballPositionObservation, 0,
        sizeof(g_ballPositionObservation));
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

void VisionReceiver_RecordUartOverrunFromIsr(void)
{
    g_status.uart_overrun_count++;
}

void VisionReceiver_RecordUartFramingErrorFromIsr(void)
{
    g_status.uart_framing_error_count++;
}

void VisionReceiver_RecordUartParityErrorFromIsr(void)
{
    g_status.uart_parity_error_count++;
}

void VisionReceiver_RecordUartBreakErrorFromIsr(void)
{
    g_status.uart_break_error_count++;
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

const VisionBallPositionObservation *
    VisionReceiver_GetBallPositionObservation(void)
{
    return &g_ballPositionObservation;
}

uint32_t VisionReceiver_GetProtocolErrorCount(void)
{
    return g_status.length_error_count +
        g_status.crc_error_count +
        g_status.version_error_count +
        g_status.type_error_count +
        g_status.reserved_error_count +
        g_status.flags_error_count +
        g_status.field_error_count +
        g_status.semantic_error_count;
}
