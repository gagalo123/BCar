//
// Created by naoki on 19/03/09.
//

#include "dma_ring.h"
#include "dma_scanf.h"
#include"stm32f1xx_hal_uart.h"
struct dma_scanf_info dsi;

void dma_scanf_init(UART_HandleTypeDef *scanf_huart){
		__HAL_UART_ENABLE_IT(scanf_huart, UART_IT_IDLE); //¿ªÆô¿ÕÏÐÖÐ¶Ï
    dsi.huart = scanf_huart;
    dma_ring_init(&dsi.rx_ring);
    HAL_UART_Receive_DMA(dsi.huart, dsi.rx_ring.buf, dsi.rx_ring.buf_size);
}
