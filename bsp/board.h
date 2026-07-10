#ifndef _BOARD_H_
#define _BOARD_H_

#include "ti_msp_dl_config.h"

/* SysTick max count (24-bit) */
#define SysTickMAX_COUNT  0xFFFFFF

/* SysTick frequency (Hz) */
#define SysTickFre        32000000

/* Convert milliseconds/microseconds to SysTick ticks */
#define SysTick_MS(x)     (((SysTickFre) / 1000U) * (uint32_t)(x))
#define SysTick_US(x)     (((SysTickFre) / 1000000U) * (uint32_t)(x))

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

/*
 * If UART_0 is available via SysConfig, declare the printf redirect helper.
 * Applications that do not need printf can define BOARD_NO_PRINTF before including this header.
 */
#ifndef BOARD_NO_PRINTF
#include <stdio.h>
#endif

#endif /* _BOARD_H_ */
