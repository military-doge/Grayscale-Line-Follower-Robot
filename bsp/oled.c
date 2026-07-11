#include "oled.h"
#include "oledfont.h"
#include "board.h"
#include <string.h>

uint8_t OLED_GRAM[128][8];

/* ---------- low-level GPIO helpers ---------- */
static void OLED_RST_Clr(void) { DL_GPIO_clearPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN); }
static void OLED_RST_Set(void) { DL_GPIO_setPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN); }
static void OLED_RS_Clr(void)  { DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN); }
static void OLED_RS_Set(void)  { DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN); }
static void OLED_SCLK_Clr(void){ DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN); }
static void OLED_SCLK_Set(void){ DL_GPIO_setPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN); }
static void OLED_SDIN_Clr(void){ DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN); }
static void OLED_SDIN_Set(void){ DL_GPIO_setPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN); }

/* ---------- write one byte to OLED (software SPI) ---------- */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    if (cmd)
        OLED_RS_Set();
    else
        OLED_RS_Clr();
    delay_cycles(10);

    for (uint8_t i = 0; i < 8; i++) {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        delay_cycles(20);
        OLED_SCLK_Set();
        delay_cycles(20);
        dat <<= 1;
    }
    OLED_RS_Set();
}

/* ---------- display on / off ---------- */
void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

/* ---------- refresh GRAM to screen ---------- */
void OLED_Refresh_Gram(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (uint8_t n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}

/* ---------- clear screen ---------- */
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
    OLED_Refresh_Gram();
}

/* ---------- draw a point ---------- */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    if (x > 127 || y > 63) return;

    uint8_t pos = 7 - y / 8;
    uint8_t bx  = y % 8;
    uint8_t temp = 1 << (7 - bx);

    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

/* ---------- show a character (6x12 or 8x16) ---------- */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t y0 = y;
    chr -= ' ';

    for (uint8_t t = 0; t < size; t++) {
        uint8_t temp;
        if (size == 12)
            temp = oled_asc2_1206[chr][t];
        else
            temp = oled_asc2_1608[chr][t];

        for (uint8_t t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/* ---------- power helper ---------- */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/* ---------- show a number ---------- */
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t enshow = 0;
    for (uint8_t t = 0; t < len; t++) {
        uint8_t temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            } else {
                enshow = 1;
            }
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

/* ---------- show a string ---------- */
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p)
{
    while (*p != '\0') {
        if (x > 122) { x = 0; y += 16; }
        if (y > 58)  { y = x = 0; OLED_Clear(); }
        OLED_ShowChar(x, y, *p, 12, 1);
        x += 8;
        p++;
    }
}

/* ---------- set display position ---------- */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte(x & 0x0f, OLED_CMD);
}

/* ---------- initialize OLED ---------- */
void OLED_Init(void)
{
    OLED_RST_Clr();
    delay_ms(120);
    OLED_RST_Set();

    OLED_WR_Byte(0xAE, OLED_CMD); /* display off */
    OLED_WR_Byte(0xD5, OLED_CMD); /* clock div */
    OLED_WR_Byte(80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD); /* multiplex */
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD); /* display offset */
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD); /* start line */
    OLED_WR_Byte(0x8D, OLED_CMD); /* charge pump */
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD); /* memory mode */
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD); /* segment remap */
    OLED_WR_Byte(0xC0, OLED_CMD); /* COM scan direction */
    OLED_WR_Byte(0xDA, OLED_CMD); /* COM pins */
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD); /* contrast */
    OLED_WR_Byte(0xEF, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); /* pre-charge */
    OLED_WR_Byte(0xf1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); /* VCOMH */
    OLED_WR_Byte(0x30, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD); /* display all on resume */
    OLED_WR_Byte(0xA6, OLED_CMD); /* normal display */
    OLED_WR_Byte(0xAF, OLED_CMD); /* display on */

    OLED_Clear();
}
