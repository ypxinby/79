#include "balance_tuning.h"

#include <limits.h>
#include <string.h>

#include "app_features.h"
#include "balance_ball_control.h"
#include "balance_position_control.h"
#include "balance_soft_limits.h"
#include "gimbal_stepper.h"
#include "ti_msp_dl_config.h"
#include "vision_receiver.h"

#if FEATURE_BALANCE_SERIAL_TUNING

#define TUNING_RX_SIZE              (256U)
#define TUNING_RX_MASK              (TUNING_RX_SIZE - 1U)
#define TUNING_TX_SIZE              (2048U)
#define TUNING_TX_MASK              (TUNING_TX_SIZE - 1U)
#define TUNING_LINE_SIZE            (48U)
#define TUNING_RX_PROCESS_BUDGET    (96U)
#define TUNING_TX_PROCESS_BUDGET    (96U)
#define JUSTFLOAT_FLOAT_COUNT       (16U)
#define JUSTFLOAT_FRAME_SIZE        (68U)

#if ((TUNING_RX_SIZE & TUNING_RX_MASK) != 0U) || \
    ((TUNING_TX_SIZE & TUNING_TX_MASK) != 0U)
#error Balance tuning ring sizes must be powers of two
#endif

static volatile uint8_t g_rxRing[TUNING_RX_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static uint8_t g_txRing[TUNING_TX_SIZE];
static volatile uint16_t g_txHead;
static volatile uint16_t g_txTail;
static char g_line[TUNING_LINE_SIZE];
static uint8_t g_lineLength;
static uint8_t g_discardLine;
static volatile BalanceTuningStatus g_status;

static uint16_t tx_free_count(void)
{
    return (uint16_t)((g_txTail - g_txHead - 1U) & TUNING_TX_MASK);
}

static uint8_t tx_enqueue_atomic(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == (const uint8_t *)0) ||
        (length > tx_free_count())) {
        g_status.tx_overflow_count++;
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        g_txRing[g_txHead] = data[index];
        g_txHead = (uint16_t)((g_txHead + 1U) & TUNING_TX_MASK);
    }
    return 1U;
}

static void send_text(const char *text)
{
    (void)tx_enqueue_atomic((const uint8_t *)text,
        (uint16_t)strlen(text));
}

static uint8_t parse_u16(const char *text, uint16_t *value)
{
    uint32_t result = 0U;
    uint8_t digit_seen = 0U;

    if ((text == (const char *)0) ||
        (value == (uint16_t *)0)) {
        return 0U;
    }
    while (*text != '\0') {
        if ((*text < '0') || (*text > '9')) {
            return 0U;
        }
        digit_seen = 1U;
        result = result * 10U + (uint32_t)(*text - '0');
        if (result > UINT16_MAX) {
            return 0U;
        }
        text++;
    }
    if (digit_seen == 0U) {
        return 0U;
    }
    *value = (uint16_t)result;
    return 1U;
}

/* Parse a non-negative decimal into a value scaled by 100. Extra decimal
 * digits are rounded, so both KP=1 and KP=1.000000 are convenient. */
static uint8_t parse_x100(const char *text, int32_t *value)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint8_t fraction_digits = 0U;
    uint8_t round_digit = 0U;
    uint8_t digit_seen = 0U;
    uint8_t decimal_seen = 0U;

    if ((text == (const char *)0) ||
        (value == (int32_t *)0)) {
        return 0U;
    }
    while (*text != '\0') {
        if (*text == '.') {
            if (decimal_seen != 0U) {
                return 0U;
            }
            decimal_seen = 1U;
            text++;
            continue;
        }
        if ((*text < '0') || (*text > '9')) {
            return 0U;
        }
        digit_seen = 1U;
        if (decimal_seen == 0U) {
            whole = whole * 10U + (uint32_t)(*text - '0');
            if (whole > 10000U) {
                return 0U;
            }
        } else if (fraction_digits < 2U) {
            fraction = fraction * 10U + (uint32_t)(*text - '0');
            fraction_digits++;
        } else if (fraction_digits == 2U) {
            round_digit = (uint8_t)(*text - '0');
            fraction_digits++;
        }
        text++;
    }
    if (digit_seen == 0U) {
        return 0U;
    }
    while (fraction_digits < 2U) {
        fraction *= 10U;
        fraction_digits++;
    }
    if (round_digit >= 5U) {
        fraction++;
        if (fraction >= 100U) {
            fraction = 0U;
            whole++;
        }
    }
    if ((whole > (uint32_t)(INT32_MAX / 100)) ||
        ((whole * 100U + fraction) > INT32_MAX)) {
        return 0U;
    }
    *value = (int32_t)(whole * 100U + fraction);
    return 1U;
}

static void append_u32(char *buffer, uint8_t *length,
    uint8_t capacity, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        if (*length < capacity) {
            buffer[(*length)++] = '0';
        }
        return;
    }
    while ((value != 0U) && (count < sizeof(digits))) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while ((count != 0U) && (*length < capacity)) {
        buffer[(*length)++] = digits[--count];
    }
}

static void append_i32(char *buffer, uint8_t *length,
    uint8_t capacity, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (*length < capacity) {
            buffer[(*length)++] = '-';
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    append_u32(buffer, length, capacity, magnitude);
}

static void append_text(char *buffer, uint8_t *length,
    uint8_t capacity, const char *text)
{
    while ((*text != '\0') && (*length < capacity)) {
        buffer[(*length)++] = *text++;
    }
}

static void send_config(void)
{
    BalanceBallControlConfig config;
    char response[128];
    uint8_t length = 0U;

    BalanceBallControl_GetConfig(&config);
    append_text(response, &length, sizeof(response), "CFG,KP=");
    append_i32(response, &length, sizeof(response), config.kp_x100);
    append_text(response, &length, sizeof(response), ",KD=");
    append_i32(response, &length, sizeof(response), config.kd_x100);
    append_text(response, &length, sizeof(response), ",DIR=");
    append_i32(response, &length, sizeof(response), config.tilt_sign);
    append_text(response, &length, sizeof(response), ",MAX=");
    append_i32(response, &length, sizeof(response),
        config.maximum_offset_count);
    append_text(response, &length, sizeof(response), ",SLEW=");
    append_i32(response, &length, sizeof(response),
        config.target_slew_count_per_20ms);
    append_text(response, &length, sizeof(response), ",DB=");
    append_u32(response, &length, sizeof(response),
        config.tracking_deadband_count);
    append_text(response, &length, sizeof(response), ",RG=");
    append_u32(response, &length, sizeof(response),
        config.tracking_reengage_count);
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static uint8_t apply_parameter(const char *name, const char *value)
{
    BalanceBallControlConfig config;
    uint16_t parsed_u16;
    int32_t parsed_x100;

    BalanceBallControl_GetConfig(&config);
    if (strcmp(name, "KP") == 0) {
        if (parse_x100(value, &parsed_x100) == 0U) {
            return 0U;
        }
        config.kp_x100 = parsed_x100;
    } else if (strcmp(name, "KD") == 0) {
        if (parse_x100(value, &parsed_x100) == 0U) {
            return 0U;
        }
        config.kd_x100 = parsed_x100;
    } else if (strcmp(name, "DIR") == 0) {
        if (strcmp(value, "1") == 0) {
            config.tilt_sign = 1;
        } else if (strcmp(value, "-1") == 0) {
            config.tilt_sign = -1;
        } else {
            return 0U;
        }
    } else if (strcmp(name, "MAX") == 0) {
        if (parse_u16(value, &parsed_u16) == 0U) {
            return 0U;
        }
        config.maximum_offset_count = parsed_u16;
    } else if (strcmp(name, "SLEW") == 0) {
        if (parse_u16(value, &parsed_u16) == 0U) {
            return 0U;
        }
        config.target_slew_count_per_20ms = parsed_u16;
    } else if (strcmp(name, "DB") == 0) {
        if (parse_u16(value, &parsed_u16) == 0U) {
            return 0U;
        }
        config.tracking_deadband_count = parsed_u16;
    } else if (strcmp(name, "RG") == 0) {
        if (parse_u16(value, &parsed_u16) == 0U) {
            return 0U;
        }
        config.tracking_reengage_count = parsed_u16;
    } else {
        return 0U;
    }
    return BalanceBallControl_SetConfig(&config);
}

static void process_line(void)
{
    char *equals;

    g_status.rx_line_count++;
    if (strcmp(g_line, "RUN") == 0) {
        if (BalanceBallControl_Enable(0) != 0U) {
            send_text("ACK,RUN\r\n");
        } else {
            send_text("ERR,RUN\r\n");
            g_status.command_error_count++;
        }
        return;
    }
    if ((strcmp(g_line, "STOP") == 0) ||
        (strcmp(g_line, "X") == 0)) {
        BalanceBallControl_Disable();
        send_text("ACK,STOP\r\n");
        return;
    }
    if ((strcmp(g_line, "GET") == 0) ||
        (strcmp(g_line, "?") == 0)) {
        send_config();
        return;
    }
    if (strcmp(g_line, "DEF") == 0) {
        BalanceBallControlRuntime runtime;

        BalanceBallControl_GetSnapshot(&runtime);
        if ((runtime.enable_requested != 0U) ||
            (runtime.tracking_owned != 0U)) {
            send_text("ERR,BUSY\r\n");
            g_status.command_error_count++;
        } else {
            BalanceBallControl_ResetConfig();
            send_text("ACK,DEF\r\n");
            send_config();
        }
        return;
    }

    equals = strchr(g_line, '=');
    if ((equals == (char *)0) || (equals == g_line) ||
        (equals[1] == '\0')) {
        send_text("ERR,FORMAT\r\n");
        g_status.command_error_count++;
        return;
    }
    *equals = '\0';
    if (apply_parameter(g_line, equals + 1) == 0U) {
        send_text("ERR,RANGE_OR_BUSY\r\n");
        g_status.command_error_count++;
        return;
    }
    send_text("ACK,SET\r\n");
    send_config();
}

static void consume_rx_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }
    if (byte == '\n') {
        if ((g_discardLine == 0U) && (g_lineLength != 0U)) {
            g_line[g_lineLength] = '\0';
            process_line();
        } else if (g_discardLine != 0U) {
            send_text("ERR,LENGTH\r\n");
            g_status.command_error_count++;
        }
        g_lineLength = 0U;
        g_discardLine = 0U;
        return;
    }
    if ((byte < 0x20U) || (byte > 0x7EU)) {
        g_discardLine = 1U;
        return;
    }
    if (g_discardLine != 0U) {
        return;
    }
    if (g_lineLength >= (TUNING_LINE_SIZE - 1U)) {
        g_discardLine = 1U;
        return;
    }
    if ((byte >= 'a') && (byte <= 'z')) {
        byte = (uint8_t)(byte - 'a' + 'A');
    }
    g_line[g_lineLength++] = (char)byte;
}

static void enqueue_justfloat(uint32_t now_ms)
{
    BalanceBallControlRuntime ball;
    BalanceBallControlConfig config;
    BalancePositionRuntime position;
    BalanceSoftLimitsRuntime limits;
    GimbalStepperFeedback stepper;
    const VisionBallPositionObservation *vision;
    union {
        float values[JUSTFLOAT_FLOAT_COUNT];
        uint8_t bytes[JUSTFLOAT_FLOAT_COUNT * sizeof(float)];
    } payload;
    uint8_t frame[JUSTFLOAT_FRAME_SIZE];
    uint32_t age_ms;
    uint8_t index;

    BalanceBallControl_GetSnapshot(&ball);
    BalanceBallControl_GetConfig(&config);
    BalancePositionControl_GetSnapshot(&position);
    BalanceSoftLimits_GetSnapshot(&limits);
    GimbalStepper_GetFeedbackSnapshot(&stepper);
    vision = VisionReceiver_GetBallPositionObservation();
    age_ms = (ball.last_measurement_time_ms == 0U) ? UINT32_MAX :
        (now_ms - ball.last_measurement_time_ms);

    payload.values[0] = (float)ball.position_mm;
    payload.values[1] = (float)vision->predicted_position_mm;
    payload.values[2] = (float)ball.velocity_mm_s;
    payload.values[3] = (float)ball.error_mm;
    payload.values[4] = (float)ball.p_output_count;
    payload.values[5] = (float)ball.d_output_count;
    payload.values[6] = (float)ball.pd_output_count;
    payload.values[7] = (float)ball.commanded_offset_count;
    payload.values[8] = (float)limits.current_logical_count;
    payload.values[9] = (float)((int32_t)position.commanded_step_rate_hz *
        (int32_t)stepper.direction);
    payload.values[10] = (float)ball.measurement_valid;
    payload.values[11] = (age_ms == UINT32_MAX) ? -1.0f : (float)age_ms;
    payload.values[12] = (float)config.kp_x100 / 100.0f;
    payload.values[13] = (float)config.kd_x100 / 100.0f;
    payload.values[14] = (float)config.tracking_deadband_count;
    payload.values[15] = (float)ball.state;

    for (index = 0U; index < sizeof(payload.bytes); index++) {
        frame[index] = payload.bytes[index];
    }
    frame[64] = 0x00U;
    frame[65] = 0x00U;
    frame[66] = 0x80U;
    frame[67] = 0x7FU;
    if (tx_enqueue_atomic(frame, sizeof(frame)) != 0U) {
        g_status.telemetry_frame_count++;
    } else {
        g_status.telemetry_drop_count++;
    }
}

void BalanceTuning_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    g_lineLength = 0U;
    g_discardLine = 0U;
    g_status = (BalanceTuningStatus){0};
    NVIC_ClearPendingIRQ(UART_BALANCE_TUNING_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BALANCE_TUNING_INST_INT_IRQN);
}

void BalanceTuning_Process(void)
{
    uint8_t byte;
    uint8_t received = 0U;
    uint8_t transmitted = 0U;

    /* Polling fallback mirrors the K230 UART bring-up path. Reading the data
     * register removes the byte, so ISR and polling cannot duplicate it. */
    while ((received < TUNING_RX_PROCESS_BUDGET) &&
        !DL_UART_Main_isRXFIFOEmpty(UART_BALANCE_TUNING_INST)) {
        byte = DL_UART_Main_receiveData(UART_BALANCE_TUNING_INST);
        g_status.rx_byte_count++;
        consume_rx_byte(byte);
        received++;
    }

    while ((received < TUNING_RX_PROCESS_BUDGET) &&
        (g_rxTail != g_rxHead)) {
        byte = g_rxRing[g_rxTail];
        g_rxTail = (uint16_t)((g_rxTail + 1U) & TUNING_RX_MASK);
        consume_rx_byte(byte);
        received++;
    }
    while ((transmitted < TUNING_TX_PROCESS_BUDGET) &&
        (g_txTail != g_txHead) &&
        !DL_UART_Main_isTXFIFOFull(UART_BALANCE_TUNING_INST)) {
        DL_UART_Main_transmitData(UART_BALANCE_TUNING_INST,
            g_txRing[g_txTail]);
        g_txTail = (uint16_t)((g_txTail + 1U) & TUNING_TX_MASK);
        transmitted++;
    }
}

void BalanceTuning_Update20ms(uint32_t now_ms)
{
    enqueue_justfloat(now_ms);
}

void BalanceTuning_GetStatus(BalanceTuningStatus *status)
{
    if (status != (BalanceTuningStatus *)0) {
        *status = g_status;
    }
}

void UART_BALANCE_TUNING_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(
            UART_BALANCE_TUNING_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(
                    UART_BALANCE_TUNING_INST)) {
                uint8_t byte = DL_UART_Main_receiveData(
                    UART_BALANCE_TUNING_INST);
                uint16_t next = (uint16_t)((g_rxHead + 1U) &
                    TUNING_RX_MASK);

                g_status.rx_byte_count++;
                if (next == g_rxTail) {
                    g_status.rx_overflow_count++;
                } else {
                    g_rxRing[g_rxHead] = byte;
                    g_rxHead = next;
                }
            }
            break;
        default:
            break;
    }
}

#else

static BalanceTuningStatus g_status;

void BalanceTuning_Init(void)
{
    g_status = (BalanceTuningStatus){0};
}

void BalanceTuning_Process(void)
{
}

void BalanceTuning_Update20ms(uint32_t now_ms)
{
    (void)now_ms;
}

void BalanceTuning_GetStatus(BalanceTuningStatus *status)
{
    if (status != (BalanceTuningStatus *)0) {
        *status = g_status;
    }
}

#endif
