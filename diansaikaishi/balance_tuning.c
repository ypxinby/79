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
static uint8_t g_forceDebugSnapshot;
static volatile BalanceTuningStatus g_status;

typedef struct {
    uint8_t initialized;
    BalanceBallControlState controller_state;
    BalanceBallObservationResult observation_result;
    BalancePositionFault position_fault;
    uint32_t rx_byte_count;
    uint32_t last_rx_progress_ms;
    uint32_t ring_overflow_count;
    uint32_t protocol_error_count;
    uint32_t duplicate_count;
    uint32_t old_sequence_count;
    uint32_t rejected_jump_count;
    uint32_t clamp_count;
    BalanceSoftLimitError soft_limit_error;
} BalanceTuningDebugState;

static BalanceTuningDebugState g_debug;

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

static const char *controller_state_text(BalanceBallControlState state)
{
    switch (state) {
        case BALANCE_BALL_STATE_DISABLED: return "DIS";
        case BALANCE_BALL_STATE_WAIT_AXIS: return "AXIS";
        case BALANCE_BALL_STATE_WAIT_VISION: return "VIS";
        case BALANCE_BALL_STATE_ACTIVE: return "ACT";
        case BALANCE_BALL_STATE_RETURN_ZERO: return "ZERO";
        case BALANCE_BALL_STATE_VISION_LOST: return "LOST";
        case BALANCE_BALL_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *observation_result_text(
    BalanceBallObservationResult result)
{
    switch (result) {
        case BALANCE_BALL_OBSERVATION_ACCEPTED: return "ACCEPT";
        case BALANCE_BALL_OBSERVATION_TARGET_INVALID: return "VALID_0";
        case BALANCE_BALL_OBSERVATION_NOT_MEASURED: return "MEASURED_0";
        case BALANCE_BALL_OBSERVATION_STALE: return "STALE_FRAME";
        case BALANCE_BALL_OBSERVATION_POSITION_JUMP: return "POS_JUMP";
        default: return "UNKNOWN";
    }
}

static const char *vision_event_text(VisionReceiverEvent event)
{
    switch (event) {
        case VISION_RECEIVER_EVENT_WAITING: return "WAIT";
        case VISION_RECEIVER_EVENT_TARGET: return "TARGET";
        case VISION_RECEIVER_EVENT_NO_TARGET: return "NO_TARGET";
        case VISION_RECEIVER_EVENT_NEW_SESSION: return "NEW_SESSION";
        case VISION_RECEIVER_EVENT_DUPLICATE: return "DUP";
        case VISION_RECEIVER_EVENT_OLD_SEQUENCE: return "OLD_SEQ";
        case VISION_RECEIVER_EVENT_LENGTH_ERROR: return "LENGTH";
        case VISION_RECEIVER_EVENT_CRC_ERROR: return "CRC";
        case VISION_RECEIVER_EVENT_VERSION_ERROR: return "VERSION";
        case VISION_RECEIVER_EVENT_TYPE_ERROR: return "TYPE";
        case VISION_RECEIVER_EVENT_RESERVED_ERROR: return "RESERVED";
        case VISION_RECEIVER_EVENT_FLAGS_ERROR: return "FLAGS";
        case VISION_RECEIVER_EVENT_FIELD_ERROR: return "FIELD";
        case VISION_RECEIVER_EVENT_DISCARDED: return "DISCARD";
        default: return "UNKNOWN";
    }
}

static const char *position_fault_text(BalancePositionFault fault)
{
    switch (fault) {
        case BALANCE_POSITION_FAULT_NONE: return "NONE";
        case BALANCE_POSITION_FAULT_SOFT_LIMIT: return "LIMIT";
        case BALANCE_POSITION_FAULT_TIMEOUT: return "TIMEOUT";
        case BALANCE_POSITION_FAULT_NO_FEEDBACK: return "NOENC";
        case BALANCE_POSITION_FAULT_FOLLOW_ERROR: return "FOLLOW";
        case BALANCE_POSITION_FAULT_DIRECTION: return "DIR";
        default: return "UNKNOWN";
    }
}

static uint32_t age_from(uint32_t now_ms, uint32_t timestamp_ms)
{
    return (timestamp_ms == 0U) ? UINT32_MAX :
        (now_ms - timestamp_ms);
}

static const char *lost_cause_text(uint32_t now_ms,
    const BalanceBallControlRuntime *ball,
    const VisionReceiverStatus *vision_status)
{
    if (ball->last_observation_result !=
        BALANCE_BALL_OBSERVATION_ACCEPTED) {
        return observation_result_text(ball->last_observation_result);
    }
    if (age_from(now_ms, g_debug.last_rx_progress_ms) >
        BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS) {
        return "UART_NO_BYTES";
    }
    if ((vision_status->last_event ==
            VISION_RECEIVER_EVENT_DUPLICATE) ||
        (vision_status->last_event ==
            VISION_RECEIVER_EVENT_OLD_SEQUENCE)) {
        return "SEQ_REJECT";
    }
    if ((vision_status->last_event ==
            VISION_RECEIVER_EVENT_LENGTH_ERROR) ||
        (vision_status->last_event ==
            VISION_RECEIVER_EVENT_CRC_ERROR) ||
        (vision_status->last_event ==
            VISION_RECEIVER_EVENT_FIELD_ERROR) ||
        (vision_status->last_event ==
            VISION_RECEIVER_EVENT_DISCARDED)) {
        return "PARSE_REJECT";
    }
    if (age_from(now_ms,
        vision_status->last_valid_packet_time_ms) >
        BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS) {
        return "NO_ACCEPTED_FRAME";
    }
    return "MEAS_TIMEOUT";
}

static void send_debug_controller(uint32_t now_ms,
    BalanceBallControlState previous_state,
    const BalanceBallControlRuntime *ball,
    const VisionBallPositionObservation *vision,
    const VisionReceiverStatus *vision_status)
{
    char response[240];
    uint8_t length = 0U;
    uint32_t measurement_age = age_from(now_ms,
        ball->last_measurement_time_ms);
    uint32_t rx_age = age_from(now_ms, g_debug.last_rx_progress_ms);

    append_text(response, &length, sizeof(response), "DBG,CTRL,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",old=");
    append_text(response, &length, sizeof(response),
        controller_state_text(previous_state));
    append_text(response, &length, sizeof(response), ",new=");
    append_text(response, &length, sizeof(response),
        controller_state_text(ball->state));
    if (ball->state == BALANCE_BALL_STATE_VISION_LOST) {
        append_text(response, &length, sizeof(response), ",cause=");
        append_text(response, &length, sizeof(response),
            lost_cause_text(now_ms, ball, vision_status));
    }
    append_text(response, &length, sizeof(response), ",mage=");
    append_i32(response, &length, sizeof(response),
        (measurement_age == UINT32_MAX) ? -1 :
            (int32_t)measurement_age);
    append_text(response, &length, sizeof(response), ",rxage=");
    append_i32(response, &length, sizeof(response),
        (rx_age == UINT32_MAX) ? -1 : (int32_t)rx_age);
    append_text(response, &length, sizeof(response), ",seq=");
    append_u32(response, &length, sizeof(response), vision->sequence);
    append_text(response, &length, sizeof(response), ",valid=");
    append_u32(response, &length, sizeof(response), vision->target_valid);
    append_text(response, &length, sizeof(response), ",meas=");
    append_u32(response, &length, sizeof(response), vision->measured);
    append_text(response, &length, sizeof(response), ",streak=");
    append_u32(response, &length, sizeof(response), ball->valid_streak);
    append_text(response, &length, sizeof(response), ",event=");
    append_text(response, &length, sizeof(response),
        vision_event_text(vision_status->last_event));
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void send_debug_filter(uint32_t now_ms,
    const BalanceBallControlRuntime *ball,
    const VisionBallPositionObservation *vision)
{
    char response[192];
    uint8_t length = 0U;

    append_text(response, &length, sizeof(response), "DBG,FILTER,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",result=");
    append_text(response, &length, sizeof(response),
        observation_result_text(ball->last_observation_result));
    append_text(response, &length, sizeof(response), ",seq=");
    append_u32(response, &length, sizeof(response), vision->sequence);
    append_text(response, &length, sizeof(response), ",valid=");
    append_u32(response, &length, sizeof(response), vision->target_valid);
    append_text(response, &length, sizeof(response), ",meas=");
    append_u32(response, &length, sizeof(response), vision->measured);
    append_text(response, &length, sizeof(response), ",pos=");
    append_i32(response, &length, sizeof(response), vision->position_mm);
    append_text(response, &length, sizeof(response), ",accepted=");
    append_u32(response, &length, sizeof(response),
        ball->accepted_measurement_count);
    append_text(response, &length, sizeof(response), ",jump=");
    append_u32(response, &length, sizeof(response),
        ball->rejected_jump_count);
    append_text(response, &length, sizeof(response), ",hold=");
    append_u32(response, &length, sizeof(response),
        ball->held_invalid_count);
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void send_debug_receiver(uint32_t now_ms,
    const VisionReceiverStatus *status)
{
    char response[224];
    uint8_t length = 0U;

    append_text(response, &length, sizeof(response), "DBG,RX,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",event=");
    append_text(response, &length, sizeof(response),
        vision_event_text(status->last_event));
    append_text(response, &length, sizeof(response), ",bytes=");
    append_u32(response, &length, sizeof(response), status->rx_byte_count);
    append_text(response, &length, sizeof(response), ",parsed=");
    append_u32(response, &length, sizeof(response),
        status->parsed_frame_count);
    append_text(response, &length, sizeof(response), ",accepted=");
    append_u32(response, &length, sizeof(response),
        status->accepted_frame_count);
    append_text(response, &length, sizeof(response), ",len=");
    append_u32(response, &length, sizeof(response),
        status->length_error_count);
    append_text(response, &length, sizeof(response), ",crc=");
    append_u32(response, &length, sizeof(response),
        status->crc_error_count);
    append_text(response, &length, sizeof(response), ",field=");
    append_u32(response, &length, sizeof(response),
        status->field_error_count);
    append_text(response, &length, sizeof(response), ",dup=");
    append_u32(response, &length, sizeof(response),
        status->duplicate_count);
    append_text(response, &length, sizeof(response), ",old=");
    append_u32(response, &length, sizeof(response),
        status->old_sequence_count);
    append_text(response, &length, sizeof(response), ",ovf=");
    append_u32(response, &length, sizeof(response),
        status->ring_overflow_count);
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void send_debug_axis(uint32_t now_ms,
    const BalanceSoftLimitsRuntime *limits,
    const BalancePositionRuntime *position)
{
    char response[192];
    uint8_t length = 0U;

    append_text(response, &length, sizeof(response), "DBG,AXIS,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",zero=");
    append_u32(response, &length, sizeof(response), limits->zero_valid);
    append_text(response, &length, sizeof(response), ",limits=");
    append_u32(response, &length, sizeof(response), limits->limits_valid);
    append_text(response, &length, sizeof(response), ",cal=");
    append_u32(response, &length, sizeof(response),
        limits->calibration_active);
    append_text(response, &length, sizeof(response), ",test=");
    append_u32(response, &length, sizeof(response), limits->test_active);
    append_text(response, &length, sizeof(response), ",osc=");
    append_u32(response, &length, sizeof(response),
        limits->oscillation_active);
    append_text(response, &length, sizeof(response), ",busy=");
    append_u32(response, &length, sizeof(response), position->busy);
    append_text(response, &length, sizeof(response), ",pfault=");
    append_text(response, &length, sizeof(response),
        position_fault_text(position->fault));
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void send_debug_position_fault(uint32_t now_ms,
    const BalancePositionRuntime *position)
{
    char response[192];
    uint8_t length = 0U;

    append_text(response, &length, sizeof(response), "DBG,PCTRL,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",fault=");
    append_text(response, &length, sizeof(response),
        position_fault_text(position->fault));
    append_text(response, &length, sizeof(response), ",target=");
    append_i32(response, &length, sizeof(response), position->target_count);
    append_text(response, &length, sizeof(response), ",current=");
    append_i32(response, &length, sizeof(response), position->current_count);
    append_text(response, &length, sizeof(response), ",err=");
    append_i32(response, &length, sizeof(response),
        position->position_error_count);
    append_text(response, &length, sizeof(response), ",follow=");
    append_i32(response, &length, sizeof(response),
        position->following_error_count);
    append_text(response, &length, sizeof(response), ",noenc=");
    append_u32(response, &length, sizeof(response),
        position->steps_without_feedback);
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void send_debug_limit(uint32_t now_ms,
    const BalanceSoftLimitsRuntime *limits)
{
    char response[176];
    uint8_t length = 0U;

    append_text(response, &length, sizeof(response), "DBG,LIMIT,t=");
    append_u32(response, &length, sizeof(response), now_ms);
    append_text(response, &length, sizeof(response), ",count=");
    append_u32(response, &length, sizeof(response), limits->clamp_count);
    append_text(response, &length, sizeof(response), ",dir=");
    append_i32(response, &length, sizeof(response),
        limits->last_clamp_direction);
    append_text(response, &length, sizeof(response), ",current=");
    append_i32(response, &length, sizeof(response),
        limits->current_logical_count);
    append_text(response, &length, sizeof(response), ",min=");
    append_i32(response, &length, sizeof(response),
        limits->minimum_logical_count);
    append_text(response, &length, sizeof(response), ",max=");
    append_i32(response, &length, sizeof(response),
        limits->maximum_logical_count);
    append_text(response, &length, sizeof(response), ",err=");
    append_u32(response, &length, sizeof(response), limits->error);
    append_text(response, &length, sizeof(response), "\r\n");
    (void)tx_enqueue_atomic((const uint8_t *)response, length);
}

static void enqueue_debug_events(uint32_t now_ms)
{
    BalanceBallControlRuntime ball;
    BalancePositionRuntime position;
    BalanceSoftLimitsRuntime limits;
    const VisionReceiverStatus *vision_status =
        VisionReceiver_GetStatus();
    const VisionBallPositionObservation *vision =
        VisionReceiver_GetBallPositionObservation();
    uint32_t protocol_errors = VisionReceiver_GetProtocolErrorCount();

    BalanceBallControl_GetSnapshot(&ball);
    BalancePositionControl_GetSnapshot(&position);
    BalanceSoftLimits_GetSnapshot(&limits);

    if ((g_debug.initialized == 0U) ||
        (vision_status->rx_byte_count != g_debug.rx_byte_count)) {
        g_debug.last_rx_progress_ms = now_ms;
    }

    if (g_debug.initialized == 0U) {
        g_debug.controller_state = ball.state;
        g_debug.observation_result = ball.last_observation_result;
        g_debug.position_fault = position.fault;
        g_debug.soft_limit_error = limits.error;
        send_debug_controller(now_ms, ball.state, &ball, vision,
            vision_status);
        send_debug_axis(now_ms, &limits, &position);
        g_debug.initialized = 1U;
    } else {
        if (ball.state != g_debug.controller_state) {
            send_debug_controller(now_ms, g_debug.controller_state,
                &ball, vision, vision_status);
            if ((ball.state == BALANCE_BALL_STATE_WAIT_AXIS) ||
                (ball.state == BALANCE_BALL_STATE_FAULT)) {
                send_debug_axis(now_ms, &limits, &position);
            }
            if (ball.state == BALANCE_BALL_STATE_VISION_LOST) {
                send_debug_filter(now_ms, &ball, vision);
                send_debug_receiver(now_ms, vision_status);
            }
            g_debug.controller_state = ball.state;
        }
        if ((ball.last_observation_result !=
                g_debug.observation_result) ||
            (ball.rejected_jump_count !=
                g_debug.rejected_jump_count)) {
            send_debug_filter(now_ms, &ball, vision);
            g_debug.observation_result =
                ball.last_observation_result;
        }
        if (position.fault != g_debug.position_fault) {
            send_debug_position_fault(now_ms, &position);
            g_debug.position_fault = position.fault;
        }
        if ((limits.clamp_count != g_debug.clamp_count) ||
            (limits.error != g_debug.soft_limit_error)) {
            send_debug_limit(now_ms, &limits);
            g_debug.soft_limit_error = limits.error;
        }
        if ((vision_status->ring_overflow_count !=
                g_debug.ring_overflow_count) ||
            (protocol_errors != g_debug.protocol_error_count) ||
            (vision_status->duplicate_count !=
                g_debug.duplicate_count) ||
            (vision_status->old_sequence_count !=
                g_debug.old_sequence_count)) {
            send_debug_receiver(now_ms, vision_status);
        }
    }

    if (g_forceDebugSnapshot != 0U) {
        send_debug_controller(now_ms, ball.state, &ball, vision,
            vision_status);
        send_debug_filter(now_ms, &ball, vision);
        send_debug_receiver(now_ms, vision_status);
        send_debug_axis(now_ms, &limits, &position);
        send_debug_position_fault(now_ms, &position);
        send_debug_limit(now_ms, &limits);
        g_forceDebugSnapshot = 0U;
    }

    g_debug.rx_byte_count = vision_status->rx_byte_count;
    g_debug.ring_overflow_count =
        vision_status->ring_overflow_count;
    g_debug.protocol_error_count = protocol_errors;
    g_debug.duplicate_count = vision_status->duplicate_count;
    g_debug.old_sequence_count = vision_status->old_sequence_count;
    g_debug.rejected_jump_count = ball.rejected_jump_count;
    g_debug.clamp_count = limits.clamp_count;
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
    if ((strcmp(g_line, "LOG") == 0) ||
        (strcmp(g_line, "DBG") == 0)) {
        g_forceDebugSnapshot = 1U;
        send_text("ACK,LOG\r\n");
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
    g_forceDebugSnapshot = 0U;
    g_status = (BalanceTuningStatus){0};
    g_debug = (BalanceTuningDebugState){0};
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
    enqueue_debug_events(now_ms);
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
