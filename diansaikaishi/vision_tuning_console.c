#include "vision_tuning_console.h"

#include <string.h>

#include "vision_pitch_tuning.h"
#include "vision_yaw_tuning.h"

#define TUNING_RX_RING_SIZE (256U)
#define TUNING_RX_RING_MASK (TUNING_RX_RING_SIZE - 1U)
#define TUNING_TX_RING_SIZE (256U)
#define TUNING_TX_RING_MASK (TUNING_TX_RING_SIZE - 1U)
#define TUNING_LINE_SIZE    (64U)
#define TUNING_REPLY_SIZE   (128U)

#if ((TUNING_RX_RING_SIZE & TUNING_RX_RING_MASK) != 0U)
#error TUNING_RX_RING_SIZE must be a power of two
#endif

#if ((TUNING_TX_RING_SIZE & TUNING_TX_RING_MASK) != 0U)
#error TUNING_TX_RING_SIZE must be a power of two
#endif

#define TUNING_TARGET_NONE  (0U)
#define TUNING_TARGET_PITCH (1U)
#define TUNING_TARGET_YAW   (2U)

static volatile uint8_t g_rxRing[TUNING_RX_RING_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static uint8_t g_txRing[TUNING_TX_RING_SIZE];
static uint16_t g_txHead;
static uint16_t g_txTail;
static char g_line[TUNING_LINE_SIZE];
static uint8_t g_lineSize;
static uint8_t g_prefixIndex;
static uint8_t g_prefixTarget;
static uint8_t g_commandTarget;
static uint8_t g_capturing;
static VisionTuningConsoleStatus g_status;

static uint8_t rx_pop(uint8_t *byte)
{
    uint16_t tail = g_rxTail;

    if (tail == g_rxHead) {
        return 0U;
    }

    *byte = g_rxRing[tail];
    g_rxTail = (uint16_t)((tail + 1U) & TUNING_RX_RING_MASK);
    return 1U;
}

static uint16_t tx_free_space(void)
{
    return (uint16_t)((g_txTail - g_txHead - 1U) & TUNING_TX_RING_MASK);
}

static void tx_enqueue(const char *text, uint16_t length)
{
    uint16_t index;

    if (tx_free_space() < length) {
        g_status.tx_overflow_count++;
        return;
    }

    for (index = 0U; index < length; index++) {
        g_txRing[g_txHead] = (uint8_t)text[index];
        g_txHead = (uint16_t)((g_txHead + 1U) & TUNING_TX_RING_MASK);
    }
}

static void append_char(char *buffer, uint16_t *length, char value)
{
    if (*length < (TUNING_REPLY_SIZE - 1U)) {
        buffer[*length] = value;
        (*length)++;
    }
}

static void append_text(char *buffer, uint16_t *length, const char *text)
{
    while (*text != '\0') {
        append_char(buffer, length, *text);
        text++;
    }
}

static void append_uint(char *buffer, uint16_t *length, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));

    while (count != 0U) {
        append_char(buffer, length, digits[--count]);
    }
}

static uint32_t power_of_ten(uint8_t digits)
{
    uint32_t value = 1U;

    while (digits != 0U) {
        value *= 10U;
        digits--;
    }
    return value;
}

static void append_scaled(char *buffer, uint16_t *length, uint16_t value,
    uint8_t fractionalDigits)
{
    uint32_t divisor = power_of_ten(fractionalDigits);
    uint32_t fraction;
    uint32_t place;

    if (fractionalDigits == 0U) {
        append_uint(buffer, length, value);
        return;
    }

    append_uint(buffer, length, (uint32_t)value / divisor);
    append_char(buffer, length, '.');
    fraction = (uint32_t)value % divisor;
    place = divisor / 10U;
    while (place != 0U) {
        append_char(buffer, length,
            (char)('0' + ((fraction / place) % 10U)));
        place /= 10U;
    }
}

static void send_simple_reply(const char *text)
{
    char reply[TUNING_REPLY_SIZE];
    uint16_t length = 0U;

    append_text(reply, &length, text);
    append_text(reply, &length, "\r\n");
    tx_enqueue(reply, length);
}

static uint8_t parse_scaled_value(const char *text,
    uint8_t fractionalDigits, uint16_t *value)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint8_t fractionCount = 0U;
    uint8_t sawDigit = 0U;
    uint8_t sawPoint = 0U;
    uint32_t scaled;

    while (*text != '\0') {
        char ch = *text++;

        if ((ch >= '0') && (ch <= '9')) {
            sawDigit = 1U;
            if (sawPoint == 0U) {
                whole = whole * 10U + (uint32_t)(ch - '0');
            } else {
                if (fractionCount >= fractionalDigits) {
                    return 0U;
                }
                fraction = fraction * 10U + (uint32_t)(ch - '0');
                fractionCount++;
            }
        } else if ((ch == '.') && (sawPoint == 0U) &&
            (fractionalDigits != 0U)) {
            sawPoint = 1U;
        } else {
            return 0U;
        }
    }

    if (sawDigit == 0U) {
        return 0U;
    }

    scaled = whole * power_of_ten(fractionalDigits);
    if (fractionCount < fractionalDigits) {
        fraction *= power_of_ten(
            (uint8_t)(fractionalDigits - fractionCount));
    }
    scaled += fraction;
    if (scaled > 65535U) {
        return 0U;
    }

    *value = (uint16_t)scaled;
    return 1U;
}

static uint8_t split_tokens(char *line, char **tokens, uint8_t maxTokens)
{
    uint8_t count = 0U;

    while (*line != '\0') {
        while (*line == ' ') {
            line++;
        }
        if (*line == '\0') {
            break;
        }
        if (count >= maxTokens) {
            return (uint8_t)(maxTokens + 1U);
        }
        tokens[count++] = line;
        while ((*line != '\0') && (*line != ' ')) {
            line++;
        }
        if (*line == ' ') {
            *line++ = '\0';
        }
    }

    return count;
}

static uint8_t resolve_parameter(const char *name,
    VisionPitchTuningParam *parameter, uint8_t *fractionalDigits)
{
    if (strcmp(name, "DB") == 0) {
        *parameter = VISION_PITCH_TUNING_PARAM_DEADBAND;
        *fractionalDigits = 0U;
    } else if (strcmp(name, "KP") == 0) {
        *parameter = VISION_PITCH_TUNING_PARAM_KP;
        *fractionalDigits = 3U;
    } else if (strcmp(name, "MAXSPD") == 0) {
        *parameter = VISION_PITCH_TUNING_PARAM_MAX_SPEED;
        *fractionalDigits = 1U;
    } else if (strcmp(name, "ALPHA") == 0) {
        *parameter = VISION_PITCH_TUNING_PARAM_FILTER_ALPHA;
        *fractionalDigits = 3U;
    } else if (strcmp(name, "TIMEOUT") == 0) {
        *parameter = VISION_PITCH_TUNING_PARAM_TIMEOUT;
        *fractionalDigits = 0U;
    } else {
        return 0U;
    }
    return 1U;
}

static uint8_t resolve_yaw_parameter(const char *name,
    VisionYawTuningParam *parameter, uint8_t *fractionalDigits)
{
    if (strcmp(name, "DB") == 0) {
        *parameter = VISION_YAW_TUNING_PARAM_DEADBAND;
        *fractionalDigits = 0U;
    } else if (strcmp(name, "KP") == 0) {
        *parameter = VISION_YAW_TUNING_PARAM_KP;
        *fractionalDigits = 3U;
    } else if (strcmp(name, "MAXSPD") == 0) {
        *parameter = VISION_YAW_TUNING_PARAM_MAX_SPEED;
        *fractionalDigits = 1U;
    } else if (strcmp(name, "TIMEOUT") == 0) {
        *parameter = VISION_YAW_TUNING_PARAM_TIMEOUT;
        *fractionalDigits = 0U;
    } else {
        return 0U;
    }
    return 1U;
}

static void send_parameter_reply(VisionPitchTuningParam parameter,
    uint16_t value, uint8_t fractionalDigits)
{
    char reply[TUNING_REPLY_SIZE];
    uint16_t length = 0U;

    append_text(reply, &length, "OK ");
    append_text(reply, &length, VisionPitchTuning_ParamName(parameter));
    append_char(reply, &length, '=');
    append_scaled(reply, &length, value, fractionalDigits);
    append_text(reply, &length, "\r\n");
    tx_enqueue(reply, length);
}

static void send_yaw_parameter_reply(VisionYawTuningParam parameter,
    uint16_t value, uint8_t fractionalDigits)
{
    char reply[TUNING_REPLY_SIZE];
    uint16_t length = 0U;

    append_text(reply, &length, "OK ");
    append_text(reply, &length, VisionYawTuning_ParamName(parameter));
    append_char(reply, &length, '=');
    append_scaled(reply, &length, value, fractionalDigits);
    append_text(reply, &length, "\r\n");
    tx_enqueue(reply, length);
}

static void send_all_parameters(void)
{
    VisionPitchTuningParams params;
    char reply[TUNING_REPLY_SIZE];
    uint16_t length = 0U;

    VisionPitchTuning_GetSnapshot(&params);
    append_text(reply, &length, "PARAM DB=");
    append_uint(reply, &length, params.deadband_px);
    append_text(reply, &length, " KP=");
    append_scaled(reply, &length, params.kp_x1000, 3U);
    append_text(reply, &length, " MAXSPD=");
    append_scaled(reply, &length, params.max_speed_deg_s_x10, 1U);
    append_text(reply, &length, " ALPHA=");
    append_scaled(reply, &length, params.filter_alpha_x1000, 3U);
    append_text(reply, &length, " TIMEOUT=");
    append_uint(reply, &length, params.observation_timeout_ms);
    append_text(reply, &length, " ALPHA_ACTIVE=0 PERSIST=0\r\n");
    tx_enqueue(reply, length);
}

static void send_all_yaw_parameters(void)
{
    VisionYawTuningParams params;
    char reply[TUNING_REPLY_SIZE];
    uint16_t length = 0U;

    VisionYawTuning_GetSnapshot(&params);
    append_text(reply, &length, "PARAM DB=");
    append_uint(reply, &length, params.deadband_px);
    append_text(reply, &length, " KP=");
    append_scaled(reply, &length, params.kp_x1000, 3U);
    append_text(reply, &length, " MAXSPD=");
    append_scaled(reply, &length, params.max_speed_deg_s_x10, 1U);
    append_text(reply, &length, " TIMEOUT=");
    append_uint(reply, &length, params.observation_timeout_ms);
    append_text(reply, &length, " PERSIST=0\r\n");
    tx_enqueue(reply, length);
}

static void execute_line(void)
{
    char *tokens[4];
    uint8_t tokenCount;
    uint8_t index;

    g_line[g_lineSize] = '\0';
    for (index = 0U; index < g_lineSize; index++) {
        if ((g_line[index] >= 'a') && (g_line[index] <= 'z')) {
            g_line[index] = (char)(g_line[index] - 'a' + 'A');
        }
    }

    tokenCount = split_tokens(g_line, tokens, 4U);
    g_status.command_count++;

    if ((tokenCount == 2U) && (strcmp(tokens[0], "GET") == 0) &&
        (strcmp(tokens[1], "PARAM") == 0)) {
        if (g_commandTarget == TUNING_TARGET_YAW) {
            send_all_yaw_parameters();
        } else {
            send_all_parameters();
        }
        g_status.success_count++;
    } else if ((tokenCount == 1U) &&
        (strcmp(tokens[0], "DEFAULT") == 0)) {
        if (g_commandTarget == TUNING_TARGET_YAW) {
            VisionYawTuning_RestoreDefault();
        } else {
            VisionPitchTuning_RestoreDefault();
        }
        send_simple_reply("OK DEFAULT");
        g_status.success_count++;
    } else if ((tokenCount == 1U) &&
        (strcmp(tokens[0], "SAVE") == 0)) {
        send_simple_reply("ERR SAVE_UNSUPPORTED");
        g_status.error_count++;
    } else if ((tokenCount == 3U) &&
        (strcmp(tokens[0], "SET") == 0)) {
        uint8_t fractionalDigits;
        uint16_t value;

        if (g_commandTarget == TUNING_TARGET_YAW) {
            VisionYawTuningParam parameter;

            if (resolve_yaw_parameter(tokens[1], &parameter,
                    &fractionalDigits) == 0U) {
                send_simple_reply("ERR PARAM");
                g_status.error_count++;
            } else if (parse_scaled_value(tokens[2], fractionalDigits,
                    &value) == 0U) {
                send_simple_reply("ERR VALUE");
                g_status.error_count++;
            } else if (VisionYawTuning_Set(parameter, value) == 0U) {
                send_simple_reply("ERR RANGE");
                g_status.error_count++;
            } else {
                send_yaw_parameter_reply(parameter, value,
                    fractionalDigits);
                g_status.success_count++;
            }
        } else {
            VisionPitchTuningParam parameter;

            if (resolve_parameter(tokens[1], &parameter,
                    &fractionalDigits) == 0U) {
                send_simple_reply("ERR PARAM");
                g_status.error_count++;
            } else if (parse_scaled_value(tokens[2], fractionalDigits,
                    &value) == 0U) {
                send_simple_reply("ERR VALUE");
                g_status.error_count++;
            } else if (VisionPitchTuning_Set(parameter, value) == 0U) {
                send_simple_reply("ERR RANGE");
                g_status.error_count++;
            } else {
                send_parameter_reply(parameter, value, fractionalDigits);
                g_status.success_count++;
            }
        }
    } else {
        send_simple_reply("ERR COMMAND");
        g_status.error_count++;
    }
}

static void reset_capture(void)
{
    g_lineSize = 0U;
    g_prefixIndex = 0U;
    g_prefixTarget = TUNING_TARGET_NONE;
    g_commandTarget = TUNING_TARGET_NONE;
    g_capturing = 0U;
}

static void consume_byte(uint8_t byte)
{
    if (g_capturing != 0U) {
        if (byte == '\r') {
            return;
        }
        if (byte == '\n') {
            if (g_lineSize != 0U) {
                execute_line();
            } else {
                send_simple_reply("ERR COMMAND");
                g_status.command_count++;
                g_status.error_count++;
            }
            reset_capture();
            return;
        }
        if ((byte < 0x20U) || (byte > 0x7EU)) {
            reset_capture();
            return;
        }
        if (g_lineSize >= (TUNING_LINE_SIZE - 1U)) {
            send_simple_reply("ERR LINE_TOO_LONG");
            g_status.error_count++;
            reset_capture();
            return;
        }
        g_line[g_lineSize++] = (char)byte;
        return;
    }

    switch (g_prefixIndex) {
        case 0U:
            g_prefixIndex = (byte == (uint8_t)'$') ? 1U : 0U;
            g_prefixTarget = TUNING_TARGET_NONE;
            break;
        case 1U:
            g_prefixIndex = (byte == (uint8_t)'V') ? 2U :
                ((byte == (uint8_t)'$') ? 1U : 0U);
            break;
        case 2U:
            if (byte == (uint8_t)'P') {
                g_prefixTarget = TUNING_TARGET_PITCH;
                g_prefixIndex = 3U;
            } else if (byte == (uint8_t)'Y') {
                g_prefixTarget = TUNING_TARGET_YAW;
                g_prefixIndex = 3U;
            } else {
                g_prefixIndex = (byte == (uint8_t)'$') ? 1U : 0U;
                g_prefixTarget = TUNING_TARGET_NONE;
            }
            break;
        case 3U:
            g_prefixIndex = (byte == (uint8_t)'T') ? 4U :
                ((byte == (uint8_t)'$') ? 1U : 0U);
            if (g_prefixIndex != 4U) {
                g_prefixTarget = TUNING_TARGET_NONE;
            }
            break;
        case 4U:
            if (byte == (uint8_t)' ') {
                g_capturing = 1U;
                g_lineSize = 0U;
                g_commandTarget = g_prefixTarget;
                g_prefixIndex = 0U;
                g_prefixTarget = TUNING_TARGET_NONE;
            } else {
                g_prefixIndex = (byte == (uint8_t)'$') ? 1U : 0U;
                g_prefixTarget = TUNING_TARGET_NONE;
            }
            break;
        default:
            reset_capture();
            break;
    }
}

void VisionTuningConsole_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    reset_capture();
    memset(&g_status, 0, sizeof(g_status));
}

void VisionTuningConsole_PushByteFromIsr(uint8_t byte)
{
    uint16_t head = g_rxHead;
    uint16_t next = (uint16_t)((head + 1U) & TUNING_RX_RING_MASK);

    g_status.rx_byte_count++;
    if (next == g_rxTail) {
        g_status.rx_overflow_count++;
        return;
    }

    g_rxRing[head] = byte;
    g_rxHead = next;
}

uint16_t VisionTuningConsole_Process(uint16_t maxBytesToProcess)
{
    uint16_t processed = 0U;
    uint8_t byte;

    while ((processed < maxBytesToProcess) && (rx_pop(&byte) != 0U)) {
        consume_byte(byte);
        processed++;
    }
    return processed;
}

uint8_t VisionTuningConsole_TryPopTxByte(uint8_t *byte)
{
    if ((byte == 0) || (g_txTail == g_txHead)) {
        return 0U;
    }

    *byte = g_txRing[g_txTail];
    g_txTail = (uint16_t)((g_txTail + 1U) & TUNING_TX_RING_MASK);
    return 1U;
}

const VisionTuningConsoleStatus *VisionTuningConsole_GetStatus(void)
{
    return &g_status;
}
