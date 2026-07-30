#include "vision_protocol.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define VISION_PROTOCOL_FIELD_COUNT       (10U)
#define VISION_PROTOCOL_NUMERIC_SCALE     (1000)

typedef struct {
    const uint8_t *data;
    uint16_t length;
} VisionField;

static int8_t hex_value(uint8_t value)
{
    if ((value >= (uint8_t)'0') && (value <= (uint8_t)'9')) {
        return (int8_t)(value - (uint8_t)'0');
    }
    if ((value >= (uint8_t)'A') && (value <= (uint8_t)'F')) {
        return (int8_t)(value - (uint8_t)'A' + 10U);
    }
    if ((value >= (uint8_t)'a') && (value <= (uint8_t)'f')) {
        return (int8_t)(value - (uint8_t)'a' + 10U);
    }
    return -1;
}

static uint8_t parse_u32(const VisionField *field, uint32_t maximum,
    uint32_t *value)
{
    uint32_t result = 0U;
    uint16_t index;

    if ((field->length == 0U) || (value == NULL)) {
        return 0U;
    }
    for (index = 0U; index < field->length; index++) {
        uint8_t digit = field->data[index];

        if ((digit < (uint8_t)'0') || (digit > (uint8_t)'9')) {
            return 0U;
        }
        digit = (uint8_t)(digit - (uint8_t)'0');
        if (result > ((maximum - digit) / 10U)) {
            return 0U;
        }
        result = result * 10U + digit;
    }
    *value = result;
    return 1U;
}

/* Parse a decimal number into value*1000 without using floating point.
 * Up to three fractional digits are accepted; missing digits are padded. */
static uint8_t parse_decimal_x1000(const VisionField *field,
    int32_t *scaled_value)
{
    uint16_t index = 0U;
    uint8_t negative = 0U;
    uint8_t decimal_seen = 0U;
    uint8_t fractional_digits = 0U;
    uint8_t digit_seen = 0U;
    int64_t whole = 0;
    int32_t fraction = 0;
    int64_t scaled;

    if ((field->length == 0U) || (scaled_value == NULL)) {
        return 0U;
    }
    if ((field->data[index] == (uint8_t)'-') ||
        (field->data[index] == (uint8_t)'+')) {
        negative = (field->data[index] == (uint8_t)'-') ? 1U : 0U;
        index++;
        if (index >= field->length) {
            return 0U;
        }
    }

    for (; index < field->length; index++) {
        uint8_t value = field->data[index];

        if (value == (uint8_t)'.') {
            if (decimal_seen != 0U) {
                return 0U;
            }
            decimal_seen = 1U;
            continue;
        }
        if ((value < (uint8_t)'0') || (value > (uint8_t)'9')) {
            return 0U;
        }
        digit_seen = 1U;
        value = (uint8_t)(value - (uint8_t)'0');
        if (decimal_seen == 0U) {
            if (whole > ((INT32_MAX - value) / 10)) {
                return 0U;
            }
            whole = whole * 10 + value;
        } else {
            if (fractional_digits >= 3U) {
                return 0U;
            }
            fraction = fraction * 10 + value;
            fractional_digits++;
        }
    }
    if (digit_seen == 0U) {
        return 0U;
    }
    while (fractional_digits < 3U) {
        fraction *= 10;
        fractional_digits++;
    }
    scaled = whole * VISION_PROTOCOL_NUMERIC_SCALE + fraction;
    if (negative != 0U) {
        scaled = -scaled;
    }
    if ((scaled > INT32_MAX) || (scaled < INT32_MIN)) {
        return 0U;
    }
    *scaled_value = (int32_t)scaled;
    return 1U;
}

static uint8_t scaled_to_i16(int32_t scaled, int16_t *value)
{
    int32_t rounded;

    if (value == NULL) {
        return 0U;
    }
    if (scaled >= 0) {
        rounded = (scaled + 500) / VISION_PROTOCOL_NUMERIC_SCALE;
    } else {
        rounded = (scaled - 500) / VISION_PROTOCOL_NUMERIC_SCALE;
    }
    if ((rounded > INT16_MAX) || (rounded < INT16_MIN)) {
        return 0U;
    }
    *value = (int16_t)rounded;
    return 1U;
}

static uint8_t state_is_valid(const VisionField *field)
{
    uint16_t index;

    if ((field->length == 0U) ||
        (field->length >= VISION_PROTOCOL_STATE_MAX_LENGTH)) {
        return 0U;
    }
    for (index = 0U; index < field->length; index++) {
        uint8_t value = field->data[index];
        uint8_t alphanumeric = (uint8_t)(
            ((value >= (uint8_t)'0') && (value <= (uint8_t)'9')) ||
            ((value >= (uint8_t)'A') && (value <= (uint8_t)'Z')) ||
            ((value >= (uint8_t)'a') && (value <= (uint8_t)'z')));

        if ((alphanumeric == 0U) && (value != (uint8_t)'_') &&
            (value != (uint8_t)'-')) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t parse_confidence(const VisionField *field,
    uint16_t *confidence)
{
    int32_t scaled;
    int32_t result;

    if ((parse_decimal_x1000(field, &scaled) == 0U) || (scaled < 0)) {
        return 0U;
    }
    /* Accept either 0.000..1.000 or the integer-style 0..1000 form. */
    if (scaled <= 1000) {
        result = scaled;
    } else {
        result = (scaled + 500) / 1000;
    }
    if (result > 1000) {
        return 0U;
    }
    *confidence = (uint16_t)result;
    return 1U;
}

uint8_t VisionProtocol_XorChecksum(const uint8_t *data,
    uint16_t length)
{
    uint8_t checksum = 0U;
    uint16_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        checksum ^= data[index];
    }
    return checksum;
}

VisionProtocolParseResult VisionProtocol_ParseBallAsciiLine(
    const uint8_t *line, uint16_t length,
    VisionBallAsciiPacket *packet)
{
    VisionField fields[VISION_PROTOCOL_FIELD_COUNT];
    uint16_t field_start;
    uint16_t star_index = 0U;
    uint16_t index;
    uint8_t field_count = 0U;
    int8_t checksum_high;
    int8_t checksum_low;
    uint8_t stored_checksum;
    uint8_t computed_checksum;
    uint32_t unsigned_value;
    int32_t scaled_value;

    if ((line == NULL) || (packet == NULL)) {
        return VISION_PROTOCOL_PARSE_ARGUMENT_ERROR;
    }
    if ((length < 18U) || (length >= VISION_PROTOCOL_LINE_MAX_LENGTH)) {
        return VISION_PROTOCOL_PARSE_LENGTH_ERROR;
    }
    if ((line[0] != (uint8_t)'@') ||
        (line[1] != (uint8_t)'B') ||
        (line[2] != (uint8_t)',')) {
        return VISION_PROTOCOL_PARSE_MAGIC_ERROR;
    }
    for (index = 3U; index < length; index++) {
        if (line[index] == (uint8_t)'*') {
            star_index = index;
            break;
        }
    }
    if ((star_index == 0U) || ((star_index + 3U) != length)) {
        return VISION_PROTOCOL_PARSE_LENGTH_ERROR;
    }
    checksum_high = hex_value(line[star_index + 1U]);
    checksum_low = hex_value(line[star_index + 2U]);
    if ((checksum_high < 0) || (checksum_low < 0)) {
        return VISION_PROTOCOL_PARSE_CRC_ERROR;
    }
    stored_checksum = (uint8_t)(((uint8_t)checksum_high << 4) |
        (uint8_t)checksum_low);
    computed_checksum = VisionProtocol_XorChecksum(&line[1],
        (uint16_t)(star_index - 1U));
    if (stored_checksum != computed_checksum) {
        return VISION_PROTOCOL_PARSE_CRC_ERROR;
    }

    field_start = 1U;
    for (index = 1U; index <= star_index; index++) {
        if ((index == star_index) || (line[index] == (uint8_t)',')) {
            if ((field_count >= VISION_PROTOCOL_FIELD_COUNT) ||
                (index == field_start)) {
                return VISION_PROTOCOL_PARSE_FIELD_ERROR;
            }
            fields[field_count].data = &line[field_start];
            fields[field_count].length =
                (uint16_t)(index - field_start);
            field_count++;
            field_start = (uint16_t)(index + 1U);
        }
    }
    if ((field_count != VISION_PROTOCOL_FIELD_COUNT) ||
        (fields[0].length != 1U) ||
        (fields[0].data[0] != (uint8_t)'B')) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }

    memset(packet, 0, sizeof(*packet));
    if ((parse_u32(&fields[1], UINT16_MAX, &unsigned_value) == 0U)) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    packet->sequence = (uint16_t)unsigned_value;
    if (parse_u32(&fields[2], UINT32_MAX, &packet->source_timestamp_ms) ==
        0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if ((parse_u32(&fields[3], 1U, &unsigned_value) == 0U)) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    packet->target_valid = (uint8_t)unsigned_value;
    if (state_is_valid(&fields[4]) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    memcpy(packet->state, fields[4].data, fields[4].length);
    packet->state[fields[4].length] = '\0';

    if (parse_decimal_x1000(&fields[5], &scaled_value) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (scaled_to_i16(scaled_value, &packet->position_mm) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (parse_decimal_x1000(&fields[6], &scaled_value) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (scaled_to_i16(scaled_value,
            &packet->predicted_position_mm) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (parse_decimal_x1000(&fields[7], &scaled_value) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (scaled_to_i16(scaled_value, &packet->velocity_mm_s) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (parse_confidence(&fields[8], &packet->confidence) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    if (parse_u32(&fields[9], 1U, &unsigned_value) == 0U) {
        return VISION_PROTOCOL_PARSE_FIELD_ERROR;
    }
    packet->measured = (uint8_t)unsigned_value;
    return VISION_PROTOCOL_PARSE_OK;
}
