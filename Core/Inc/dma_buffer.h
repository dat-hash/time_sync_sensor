// Put a variable in DMA-reachable RAM:
// example static uint8_t gnss_receive_buffer[512] DMA_BUFFER;

#ifndef DMA_BUFFER_H
#define DMA_BUFFER_H

#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

#endif