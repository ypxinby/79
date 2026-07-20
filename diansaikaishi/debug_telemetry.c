#include "debug_telemetry.h"

#include "app_features.h"
#include "runtime_snapshot.h"

#define DEBUG_TELEMETRY_PERIOD_MS    (100U)
#define DEBUG_TELEMETRY_TX_SIZE      (512U)
#define DEBUG_TELEMETRY_TX_MASK      (DEBUG_TELEMETRY_TX_SIZE - 1U)

#if ((DEBUG_TELEMETRY_TX_SIZE & DEBUG_TELEMETRY_TX_MASK) != 0U)
#error DEBUG_TELEMETRY_TX_SIZE must be a power of two
#endif

static uint8_t g_txRing[DEBUG_TELEMETRY_TX_SIZE];
static uint16_t g_txHead;
static uint16_t g_txTail;
static uint32_t g_lastFrameMs;
static DebugTelemetryStatus g_status;

#if FEATURE_DEBUG_TELEMETRY_VISION_UART
static void tx_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((g_txHead + 1U) &
        DEBUG_TELEMETRY_TX_MASK);

    if (next == g_txTail) {
        g_status.tx_overflow_count++;
        return;
    }
    g_txRing[g_txHead] = byte;
    g_txHead = next;
}

static void append_text(const char *text)
{
    while (*text != '\0') {
        tx_push((uint8_t)*text);
        text++;
    }
}

static void append_u32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        tx_push((uint8_t)'0');
        return;
    }
    while ((value != 0U) && (count < sizeof(digits))) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count != 0U) {
        tx_push((uint8_t)digits[--count]);
    }
}

static void append_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        tx_push((uint8_t)'-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    append_u32(magnitude);
}

static void enqueue_snapshot(const RuntimeSnapshot *snapshot)
{
    append_text("$DBG T=");
    append_u32(snapshot->timestamp_ms);
    append_text(" CS=");
    append_u32((uint32_t)snapshot->car_state);
    append_text(" M=");
    append_u32(snapshot->mission_id);
    append_text(" A=");
    append_u32(snapshot->action_index);
    append_text(" AT=");
    append_u32((uint32_t)snapshot->action_type);
    append_text(" AR=");
    append_u32((uint32_t)snapshot->action_result);
    append_text(" R=");
    append_u32((uint32_t)snapshot->run_mode);
    append_text(" RAW=");
    append_u32(snapshot->track_raw);
    append_text(" E=");
    append_i32(snapshot->track_error);
    append_text(" Y10=");
    append_i32((int32_t)(snapshot->yaw_deg * 10.0f));
    append_text(" G10=");
    append_i32((int32_t)(snapshot->gyro_z_dps * 10.0f));
    append_text(" OBS=");
    append_u32((uint32_t)snapshot->obstacle_state);
    append_text(" SH=");
    append_u32(snapshot->safety_hold ? 1U : 0U);
    append_text(" EH=");
    append_u32(snapshot->external_hold ? 1U : 0U);
    append_text(" AV=");
    append_u32((uint32_t)snapshot->avoidance_state);
    append_text(" F=");
    append_u32((uint32_t)snapshot->fault_code);
    append_text(" FD=");
    append_u32(snapshot->fault_detail);
    append_text(" FC=");
    append_u32(snapshot->fault_context);
    append_text(" YMS=");
    append_u32(snapshot->turn_to_yaw_elapsed_ms);
    append_text(" YE10=");
    append_i32((int32_t)(snapshot->turn_to_yaw_error_deg * 10.0f));
    append_text(" MISS=");
    append_u32(snapshot->app_missed_count);
    append_text(" DROP=");
    append_u32(snapshot->app_drop_count);
    append_text(" OVR=");
    append_u32(snapshot->app_overrun_count);
    append_text(" UO=");
    append_u32(snapshot->uart_overflow_count);
    append_text(" HB=");
    append_u32(snapshot->heartbeat_seen ? 1U : 0U);
    append_text(" WT=");
    append_u32(snapshot->watchdog_tripped ? 1U : 0U);
    append_text(" HW=");
    append_u32(snapshot->hardware_watchdog_enabled ? 1U : 0U);
    append_text(" HA=");
    append_u32(snapshot->heartbeat_age_ms);
    append_text("\r\n");
}
#endif

void DebugTelemetry_Init(void)
{
    g_txHead = 0U;
    g_txTail = 0U;
    g_lastFrameMs = 0U;
    g_status.frame_count = 0U;
    g_status.suppressed_count = 0U;
    g_status.tx_overflow_count = 0U;
}

void DebugTelemetry_Update(uint32_t timestamp_ms)
{
    if ((timestamp_ms - g_lastFrameMs) < DEBUG_TELEMETRY_PERIOD_MS) {
        return;
    }
    g_lastFrameMs = timestamp_ms;

#if FEATURE_DEBUG_TELEMETRY_VISION_UART
    enqueue_snapshot(RuntimeSnapshot_Get());
    g_status.frame_count++;
#else
    /* UART3 is owned by vision/protocol traffic; snapshot remains available. */
    g_status.suppressed_count++;
#endif
}

uint8_t DebugTelemetry_TryPopTxByte(uint8_t *byte)
{
    if ((byte == (uint8_t *)0) || (g_txTail == g_txHead)) {
        return 0U;
    }

    *byte = g_txRing[g_txTail];
    g_txTail = (uint16_t)((g_txTail + 1U) & DEBUG_TELEMETRY_TX_MASK);
    return 1U;
}

const DebugTelemetryStatus *DebugTelemetry_GetStatus(void)
{
    return &g_status;
}
