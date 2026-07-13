#ifndef _JY62_H
#define _JY62_H

#include <stdint.h>

/*
 * Fallback UART_1 defines — will be overridden by ti_msp_dl_config.h
 * after SysConfig regeneration.  Remove this block once SysConfig
 * generates the matching defines.
 */
#ifndef UART_1_INST
#define UART_1_INST            UART1
#define UART_1_INST_INT_IRQN   UART1_INT_IRQn
#define UART_1_INST_IRQHandler UART1_IRQHandler
#endif

void JY62_Init(void);
float JY62_Get_AngularVelocityZ(void);
float JY62_Get_Yaw(void);
uint8_t JY62_Is_Data_Ready(void);
void JY62_UART_RX_ISR(uint8_t byte);

#endif
