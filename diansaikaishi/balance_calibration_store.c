#include "balance_calibration_store.h"

#include <stddef.h>
#include <stdint.h>

#include "app_features.h"
#include "ti_msp_dl_config.h"

#if FEATURE_BALANCE_SOFT_LIMITS

#define BALANCE_CALIBRATION_FLASH_ADDRESS       (0x0001FC00UL)
#define BALANCE_CALIBRATION_FLASH_SECTOR_BYTES  (1024U)
#define BALANCE_CALIBRATION_MAGIC               (0x314C4142UL)
#define BALANCE_CALIBRATION_VERSION             (1U)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int32_t low_offset_count;
    int32_t high_offset_count;
    uint32_t counts_per_rev;
    uint32_t steps_per_rev;
    uint32_t sequence;
    uint32_t crc32;
} BalanceCalibrationFlashRecord;

typedef union {
    BalanceCalibrationFlashRecord record;
    uint32_t words[8];
    uint8_t bytes[32];
} BalanceCalibrationFlashBuffer;

typedef char BalanceCalibrationRecordMustBe32Bytes[
    (sizeof(BalanceCalibrationFlashRecord) == 32U) ? 1 : -1];

/* Reserve the final 1 KB main-flash sector so code and constants can never be
 * linked into a sector that is erased at runtime. Reflashing firmware resets
 * this sector; ordinary reset and power cycling preserve the saved record. */
__attribute__((location(BALANCE_CALIBRATION_FLASH_ADDRESS), retain, used,
    aligned(8)))
const uint8_t g_balanceCalibrationReservedSector[
    BALANCE_CALIBRATION_FLASH_SECTOR_BYTES] = {0U};

static uint32_t g_lastSequence;

static uint32_t calibration_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }
    return ~crc;
}

static void calibration_read_record(BalanceCalibrationFlashBuffer *buffer)
{
    const volatile uint32_t *source =
        (const volatile uint32_t *)BALANCE_CALIBRATION_FLASH_ADDRESS;
    uint32_t index;

    for (index = 0U; index < 8U; index++) {
        buffer->words[index] = source[index];
    }
}

static uint8_t calibration_record_is_empty(
    const BalanceCalibrationFlashBuffer *buffer)
{
    uint8_t all_zero = 1U;
    uint8_t all_erased = 1U;
    uint32_t index;

    for (index = 0U; index < 8U; index++) {
        if (buffer->words[index] != 0U) {
            all_zero = 0U;
        }
        if (buffer->words[index] != 0xFFFFFFFFUL) {
            all_erased = 0U;
        }
    }
    return (uint8_t)((all_zero != 0U) || (all_erased != 0U));
}

static uint8_t calibration_record_is_valid(
    const BalanceCalibrationFlashBuffer *buffer)
{
    const BalanceCalibrationFlashRecord *record = &buffer->record;
    int64_t span;

    if ((record->magic != BALANCE_CALIBRATION_MAGIC) ||
        (record->version != BALANCE_CALIBRATION_VERSION) ||
        (record->size != sizeof(BalanceCalibrationFlashRecord)) ||
        (record->counts_per_rev != BALANCE_ENCODER_COUNTS_PER_REV) ||
        (record->steps_per_rev != BALANCE_STEPPER_COMMAND_STEPS_PER_REV) ||
        (record->low_offset_count >= 0) ||
        (record->high_offset_count <= 0) ||
        (record->low_offset_count >= record->high_offset_count)) {
        return 0U;
    }

    span = (int64_t)record->high_offset_count -
        (int64_t)record->low_offset_count;
    if (span < BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS) {
        return 0U;
    }

    return (calibration_crc32(buffer->bytes,
        offsetof(BalanceCalibrationFlashRecord, crc32)) ==
        record->crc32) ? 1U : 0U;
}

BalanceCalibrationFlashStatus BalanceCalibrationStore_Load(
    BalanceCalibrationStoredLimits *limits)
{
    BalanceCalibrationFlashBuffer buffer;

    calibration_read_record(&buffer);
    if (calibration_record_is_empty(&buffer) != 0U) {
        g_lastSequence = 0U;
        return BALANCE_CALIBRATION_FLASH_EMPTY;
    }
    if (calibration_record_is_valid(&buffer) == 0U) {
        g_lastSequence = 0U;
        return BALANCE_CALIBRATION_FLASH_INVALID;
    }

    g_lastSequence = buffer.record.sequence;
    if (limits != (BalanceCalibrationStoredLimits *)0) {
        limits->low_offset_count = buffer.record.low_offset_count;
        limits->high_offset_count = buffer.record.high_offset_count;
        limits->sequence = buffer.record.sequence;
    }
    return BALANCE_CALIBRATION_FLASH_VALID;
}

BalanceCalibrationFlashStatus BalanceCalibrationStore_Save(
    int32_t low_offset_count, int32_t high_offset_count,
    BalanceCalibrationStoredLimits *saved_limits)
{
    __attribute__((aligned(8))) BalanceCalibrationFlashBuffer buffer = {0};
    BalanceCalibrationFlashBuffer verify;
    DL_FLASHCTL_COMMAND_STATUS command_status;
    uint32_t interrupt_state;
    uint32_t offset;

    if ((low_offset_count >= 0) || (high_offset_count <= 0) ||
        (low_offset_count >= high_offset_count) ||
        (((int64_t)high_offset_count - (int64_t)low_offset_count) <
            BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS)) {
        return BALANCE_CALIBRATION_FLASH_WRITE_ERROR;
    }

    buffer.record.magic = BALANCE_CALIBRATION_MAGIC;
    buffer.record.version = BALANCE_CALIBRATION_VERSION;
    buffer.record.size = sizeof(BalanceCalibrationFlashRecord);
    buffer.record.low_offset_count = low_offset_count;
    buffer.record.high_offset_count = high_offset_count;
    buffer.record.counts_per_rev = BALANCE_ENCODER_COUNTS_PER_REV;
    buffer.record.steps_per_rev = BALANCE_STEPPER_COMMAND_STEPS_PER_REV;
    buffer.record.sequence = (g_lastSequence == UINT32_MAX) ?
        1U : (g_lastSequence + 1U);
    buffer.record.crc32 = calibration_crc32(buffer.bytes,
        offsetof(BalanceCalibrationFlashRecord, crc32));

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    DL_FlashCTL_unprotectSector(FLASHCTL,
        BALANCE_CALIBRATION_FLASH_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    command_status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        BALANCE_CALIBRATION_FLASH_ADDRESS,
        DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (command_status == DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        for (offset = 0U; offset < sizeof(buffer); offset += 8U) {
            DL_FlashCTL_unprotectSector(FLASHCTL,
                BALANCE_CALIBRATION_FLASH_ADDRESS + offset,
                DL_FLASHCTL_REGION_SELECT_MAIN);
            command_status =
                DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
                    FLASHCTL,
                    BALANCE_CALIBRATION_FLASH_ADDRESS + offset,
                    &buffer.words[offset / sizeof(uint32_t)]);
            if (command_status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
                break;
            }
        }
    }
    if ((interrupt_state & 1U) == 0U) {
        __enable_irq();
    }

    if (command_status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return BALANCE_CALIBRATION_FLASH_WRITE_ERROR;
    }

    calibration_read_record(&verify);
    if ((calibration_record_is_valid(&verify) == 0U) ||
        (verify.record.low_offset_count != low_offset_count) ||
        (verify.record.high_offset_count != high_offset_count) ||
        (verify.record.sequence != buffer.record.sequence)) {
        return BALANCE_CALIBRATION_FLASH_WRITE_ERROR;
    }

    g_lastSequence = verify.record.sequence;
    if (saved_limits != (BalanceCalibrationStoredLimits *)0) {
        saved_limits->low_offset_count = verify.record.low_offset_count;
        saved_limits->high_offset_count = verify.record.high_offset_count;
        saved_limits->sequence = verify.record.sequence;
    }
    return BALANCE_CALIBRATION_FLASH_VALID;
}

#else

BalanceCalibrationFlashStatus BalanceCalibrationStore_Load(
    BalanceCalibrationStoredLimits *limits)
{
    (void)limits;
    return BALANCE_CALIBRATION_FLASH_EMPTY;
}

BalanceCalibrationFlashStatus BalanceCalibrationStore_Save(
    int32_t low_offset_count, int32_t high_offset_count,
    BalanceCalibrationStoredLimits *saved_limits)
{
    (void)low_offset_count;
    (void)high_offset_count;
    (void)saved_limits;
    return BALANCE_CALIBRATION_FLASH_WRITE_ERROR;
}

#endif
