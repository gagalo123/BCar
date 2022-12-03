//
// Created by naoki on 19/03/09.
//

#ifndef HAL_DMA_PRINTF_DMA_SCANF_H
#define HAL_DMA_PRINTF_DMA_SCANF_H
#include "dma_ring.h"
#include "run_main.h"
struct dma_scanf_info {
    struct dma_ring_buf rx_ring;
    UART_HandleTypeDef *huart;
};
extern struct dma_scanf_info dsi;

void dma_scanf_init(UART_HandleTypeDef *scanf_huart);
#endif //HAL_DMA_PRINTF_DMA_SCANF_H
