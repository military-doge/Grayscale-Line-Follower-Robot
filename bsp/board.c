#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

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

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms)  { delay_ms(ms); }

/* ---------- printf redirect via UART ---------- */
#ifndef BOARD_NO_PRINTF

#ifdef UART_0_INST

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}
#endif

int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

#endif /* UART_0_INST */

#endif /* BOARD_NO_PRINTF */
