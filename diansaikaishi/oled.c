#include "oled.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define OLED_I2C_ADDR              (0x3CU)
#define OLED_WIDTH                 (128U)
#define OLED_PAGES                 (8U)
#define OLED_I2C_DELAY_CYCLES      (24U)

static uint8_t g_oledPage;
static uint8_t g_oledColumn;

static void oled_delay(void)
{
    delay_cycles(OLED_I2C_DELAY_CYCLES);
}

static void oled_scl_high(void)
{
    DL_GPIO_disableOutput(GPIO_OLED_PORT, GPIO_OLED_SCL_PIN);
    oled_delay();
}

static void oled_scl_low(void)
{
    DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_OLED_PORT, GPIO_OLED_SCL_PIN);
    oled_delay();
}

static void oled_sda_high(void)
{
    DL_GPIO_disableOutput(GPIO_OLED_PORT, GPIO_OLED_SDA_PIN);
    oled_delay();
}

static void oled_sda_low(void)
{
    DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_OLED_PORT, GPIO_OLED_SDA_PIN);
    oled_delay();
}

static void oled_i2c_start(void)
{
    oled_sda_high();
    oled_scl_high();
    oled_sda_low();
    oled_scl_low();
}

static void oled_i2c_stop(void)
{
    oled_sda_low();
    oled_scl_high();
    oled_sda_high();
}

static void oled_i2c_write_byte(uint8_t value)
{
    for (uint8_t i = 0; i < 8U; i++) {
        if ((value & 0x80U) != 0U) {
            oled_sda_high();
        } else {
            oled_sda_low();
        }

        oled_scl_high();
        oled_scl_low();
        value <<= 1;
    }

    oled_sda_high();
    oled_scl_high();
    oled_scl_low();
}

static void oled_write_command(uint8_t command)
{
    oled_i2c_start();
    oled_i2c_write_byte((uint8_t)(OLED_I2C_ADDR << 1));
    oled_i2c_write_byte(0x00U);
    oled_i2c_write_byte(command);
    oled_i2c_stop();
}

static void oled_begin_data(void)
{
    oled_i2c_start();
    oled_i2c_write_byte((uint8_t)(OLED_I2C_ADDR << 1));
    oled_i2c_write_byte(0x40U);
}

static void oled_end_data(void)
{
    oled_i2c_stop();
}

static const uint8_t *oled_get_glyph(char ch)
{
    static const uint8_t space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t gt[5]    = {0x00, 0x41, 0x22, 0x14, 0x08};
    static const uint8_t zero[5]  = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t one[5]   = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t two[5]   = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const uint8_t three[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const uint8_t four[5]  = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t five[5]  = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const uint8_t six[5]   = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const uint8_t seven[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t eight[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t nine[5]  = {0x06, 0x49, 0x49, 0x29, 0x1E};
    static const uint8_t glyphA[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t glyphB[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t glyphC[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t glyphD[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t glyphE[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t glyphG[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
    static const uint8_t glyphH[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const uint8_t glyphI[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t glyphK[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const uint8_t glyphL[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t glyphM[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const uint8_t glyphN[5] = {0x7F, 0x02, 0x0C, 0x10, 0x7F};
    static const uint8_t glyphO[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t glyphP[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t glyphR[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t glyphS[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t glyphT[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t glyphU[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t glyphV[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const uint8_t glyphW[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};
    static const uint8_t glyphX[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
    static const uint8_t glyphY[5] = {0x07, 0x08, 0x70, 0x08, 0x07};
    static const uint8_t glyphb[5] = {0x7F, 0x48, 0x48, 0x48, 0x30};

    switch (ch) {
        case ' ': return space;
        case ':': return colon;
        case '-': return minus;
        case '>': return gt;
        case '0': return zero;
        case '1': return one;
        case '2': return two;
        case '3': return three;
        case '4': return four;
        case '5': return five;
        case '6': return six;
        case '7': return seven;
        case '8': return eight;
        case '9': return nine;
        case 'A': return glyphA;
        case 'B': return glyphB;
        case 'C': return glyphC;
        case 'D': return glyphD;
        case 'E': return glyphE;
        case 'G': return glyphG;
        case 'H': return glyphH;
        case 'I': return glyphI;
        case 'K': return glyphK;
        case 'L': return glyphL;
        case 'M': return glyphM;
        case 'N': return glyphN;
        case 'O': return glyphO;
        case 'P': return glyphP;
        case 'R': return glyphR;
        case 'S': return glyphS;
        case 'T': return glyphT;
        case 'U': return glyphU;
        case 'V': return glyphV;
        case 'W': return glyphW;
        case 'X': return glyphX;
        case 'Y': return glyphY;
        case 'b': return glyphb;
        default: return space;
    }
}

void OLED_Init(void)
{
    oled_sda_high();
    oled_scl_high();

    oled_write_command(0xAEU);
    oled_write_command(0x20U);
    oled_write_command(0x02U);
    oled_write_command(0xB0U);
    oled_write_command(0xC8U);
    oled_write_command(0x00U);
    oled_write_command(0x10U);
    oled_write_command(0x40U);
    oled_write_command(0x81U);
    oled_write_command(0x7FU);
    oled_write_command(0xA1U);
    oled_write_command(0xA6U);
    oled_write_command(0xA8U);
    oled_write_command(0x3FU);
    oled_write_command(0xA4U);
    oled_write_command(0xD3U);
    oled_write_command(0x00U);
    oled_write_command(0xD5U);
    oled_write_command(0x80U);
    oled_write_command(0xD9U);
    oled_write_command(0xF1U);
    oled_write_command(0xDAU);
    oled_write_command(0x12U);
    oled_write_command(0xDBU);
    oled_write_command(0x40U);
    oled_write_command(0x8DU);
    oled_write_command(0x14U);

    OLED_Clear();
    oled_write_command(0xAFU);
}

void OLED_Clear(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        OLED_SetCursor(page, 0);
        oled_begin_data();
        for (uint8_t column = 0; column < OLED_WIDTH; column++) {
            oled_i2c_write_byte(0x00U);
        }
        oled_end_data();
    }

    OLED_SetCursor(0, 0);
}

void OLED_SetCursor(uint8_t page, uint8_t column)
{
    if (page >= OLED_PAGES) {
        page = OLED_PAGES - 1U;
    }
    if (column >= OLED_WIDTH) {
        column = OLED_WIDTH - 1U;
    }

    g_oledPage = page;
    g_oledColumn = column;

    oled_write_command((uint8_t)(0xB0U | page));
    oled_write_command((uint8_t)(0x00U | (column & 0x0FU)));
    oled_write_command((uint8_t)(0x10U | (column >> 4)));
}

void OLED_PrintChar(char ch)
{
    const uint8_t *glyph = oled_get_glyph(ch);

    if ((g_oledColumn + 6U) > OLED_WIDTH) {
        return;
    }

    oled_begin_data();
    for (uint8_t i = 0; i < 5U; i++) {
        oled_i2c_write_byte(glyph[i]);
    }
    oled_i2c_write_byte(0x00U);
    oled_end_data();

    g_oledColumn += 6U;
}

void OLED_PrintString(const char *text)
{
    while (*text != '\0') {
        OLED_PrintChar(*text);
        text++;
    }
}

void OLED_PrintInt16(int16_t value)
{
    char buffer[7];
    uint8_t index = 0;
    uint16_t magnitude;
    uint16_t divisor = 10000U;
    bool started = false;

    if (value < 0) {
        OLED_PrintChar('-');
        magnitude = (uint16_t)(-value);
    } else {
        magnitude = (uint16_t)value;
    }

    while (divisor > 0U) {
        uint8_t digit = (uint8_t)(magnitude / divisor);
        magnitude = (uint16_t)(magnitude % divisor);

        if ((digit != 0U) || started || (divisor == 1U)) {
            buffer[index] = (char)('0' + digit);
            index++;
            started = true;
        }
        divisor = (uint16_t)(divisor / 10U);
    }

    buffer[index] = '\0';
    OLED_PrintString(buffer);
}

void OLED_PrintUInt16(uint16_t value)
{
    char buffer[6];
    uint8_t index = 5U;

    buffer[index] = '\0';
    do {
        index--;
        buffer[index] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (index != 0U));

    OLED_PrintString(&buffer[index]);
}

void OLED_PrintBinary8(uint8_t value)
{
    for (int8_t bit = 7; bit >= 0; bit--) {
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            OLED_PrintChar('1');
        } else {
            OLED_PrintChar('0');
        }
    }
}

void OLED_PrintBinary7(uint8_t value)
{
    for (int8_t bit = 6; bit >= 0; bit--) {
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            OLED_PrintChar('1');
        } else {
            OLED_PrintChar('0');
        }
    }
}
