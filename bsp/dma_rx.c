#include "ti_msp_dl_config.h"
#include "dma_rx.h"
#include "jy62.h"

#define DMA_RX_BUF_SIZE  256

/* DMA buffer must be aligned for DMA access */
static uint8_t  dma_rx_buf[DMA_RX_BUF_SIZE] __attribute__((aligned(32)));
static uint32_t dma_read_idx;
static uint32_t dma_last_remaining;
static uint32_t dma_byte_count;

void DMA_RX_Init(void)
{
    DL_DMA_Config dmaCfg = {
        .trigger       = DMA_UART2_RX_TRIG,
        .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL,
        .transferMode  = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode  = 0,
        .srcWidth      = DL_DMA_WIDTH_BYTE,
        .destWidth     = DL_DMA_WIDTH_BYTE,
        .srcIncrement  = 0,   /* Fixed source: UART RXDATA register */
        .destIncrement = DL_DMA_ADDR_INCREMENT,   /* +1 byte per transfer */
    };

    /* Disable channel first in case SysConfig already configured it */
    DL_DMA_disableChannel(DMA, DMA_CH_0_TRIG);

    /* Init DMA channel 0 for UART2 RX */
    DL_DMA_initChannel(DMA, DMA_CH_0_TRIG, &dmaCfg);

    /* Source = UART2 RX data register (fixed), Dest = buffer */
    DL_DMA_setSrcAddr(DMA, DMA_CH_0_TRIG,
        (uint32_t)&(UART_2_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH_0_TRIG,
        (uint32_t)dma_rx_buf);
    DL_DMA_setTransferSize(DMA, DMA_CH_0_TRIG, DMA_RX_BUF_SIZE);

    /* Enable UART2 -> DMA trigger */
    DL_UART_enableDMAReceiveEvent(UART_2_INST, DL_UART_DMA_INTERRUPT_RX);

    /* Start DMA */
    DL_DMA_enableChannel(DMA, DMA_CH_0_TRIG);

    dma_read_idx      = 0;
    dma_last_remaining = DMA_RX_BUF_SIZE;
    dma_byte_count    = 0;
}

void DMA_RX_Process(void)
{
    uint32_t remaining;
    uint32_t write_idx;

    remaining = DL_DMA_getTransferSize(DMA, DMA_CH_0_TRIG);

    /*
     * DMA fills buffer from index 0 upward.  After transferring N bytes,
     * `remaining` = DMA_RX_BUF_SIZE - N, so the write position is:
     *   write_idx = DMA_RX_BUF_SIZE - remaining
     */
    if (remaining == dma_last_remaining)
        return;  /* No new data */

    /* Handle wrap-around: DMA finished and was restarted */
    if (remaining > dma_last_remaining) {
        /* Process tail of buffer first */
        while (dma_read_idx < DMA_RX_BUF_SIZE) {
            JY62_UART_RX_ISR(dma_rx_buf[dma_read_idx++]);
            dma_byte_count++;
        }
        dma_read_idx = 0;
    }

    dma_last_remaining = remaining;
    write_idx = DMA_RX_BUF_SIZE - remaining;

    /* Process new bytes */
    while (dma_read_idx < write_idx) {
        JY62_UART_RX_ISR(dma_rx_buf[dma_read_idx++]);
        dma_byte_count++;
    }

    /* If buffer nearing full, restart DMA */
    if (remaining < 32) {
        DL_DMA_disableChannel(DMA, DMA_CH_0_TRIG);

        /* Flush any last bytes that arrived during disable */
        write_idx = DMA_RX_BUF_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH_0_TRIG);
        while (dma_read_idx < write_idx) {
            JY62_UART_RX_ISR(dma_rx_buf[dma_read_idx++]);
            dma_byte_count++;
        }

        /* Reset and restart */
        DL_DMA_setDestAddr(DMA, DMA_CH_0_TRIG, (uint32_t)dma_rx_buf);
        DL_DMA_setTransferSize(DMA, DMA_CH_0_TRIG, DMA_RX_BUF_SIZE);
        DL_DMA_enableChannel(DMA, DMA_CH_0_TRIG);

        dma_read_idx      = 0;
        dma_last_remaining = DMA_RX_BUF_SIZE;
    }
}

uint32_t DMA_RX_GetByteCount(void)
{
    return dma_byte_count;
}
