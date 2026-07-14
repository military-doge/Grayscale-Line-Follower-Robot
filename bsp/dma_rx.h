#ifndef _DMA_RX_H
#define _DMA_RX_H

#include <stdint.h>

void DMA_RX_Init(void);
void DMA_RX_Process(void);
uint32_t DMA_RX_GetByteCount(void);

#endif
