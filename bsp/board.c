#include "board.h"

/* ---------- SysTick ---------- */
uint32_t Systick_getTick(void)
{
    return SysTick->VAL;
}

/* ---------- microsecond delay (busy-wait on SysTick) ---------- */
void delay_us(uint32_t us)
{
    if (us > SysTickMAX_COUNT / (SysTickFre / 1000000U))
        us = SysTickMAX_COUNT / (SysTickFre / 1000000U);

    uint32_t load = us * (SysTickFre / 1000000U);
    uint32_t start = Systick_getTick();
    uint32_t elapsed = 0;
    uint8_t  wrapped = 0;

    while (1) {
        uint32_t now = Systick_getTick();
        if (now > start)
            wrapped = 1;

        if (wrapped)
            elapsed = start + SysTickMAX_COUNT - now;
        else
            elapsed = start - now;

        if (elapsed >= load)
            break;
    }
}

/* ---------- millisecond delay ---------- */
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
        delay_us(1000);
}

/* ---------- printf redirect via UART ---------- */
#ifndef BOARD_NO_PRINTF

#ifdef UART_0_INST
int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST));
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
    return ch;
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    (void)stream;
    uint16_t len = 0;
    while (s[len] != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, s[len]);
        len++;
    }
    return len;
}
#endif /* UART_0_INST */

#endif /* BOARD_NO_PRINTF */
