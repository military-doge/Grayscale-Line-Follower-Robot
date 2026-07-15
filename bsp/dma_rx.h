#ifndef _DMA_RX_H
#define _DMA_RX_H

#include <stdint.h>

typedef void (*dma_rx_byte_cb_t)(uint8_t byte);

void DMA_RX_Init(dma_rx_byte_cb_t rx_callback);
void DMA_RX_Process(void);
uint32_t DMA_RX_GetByteCount(void);

#endif
