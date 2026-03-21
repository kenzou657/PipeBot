//
// Created by asus on 2026/3/19.
//

#include "bsp_usart_dma.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_it.h"
#include "usart.h"



void StartSerialRxTask(void *argument) {
    // 开启初始化的 DMA 接收
    HAL_UART_Receive_IT(&huart1, &aRxByte, 1);

    for (;;) {
        // 调用状态机解析
        Serial_Parse_Task();

        // 因为 F407 很快，10ms 的延时足够处理几百个字节了
        osDelay(10);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 1. 将收到的字节存入环形缓冲区
        uint16_t next_head = (uart_rx_fifo.head + 1) % RING_BUF_SIZE;

        // 检查缓冲区是否已满（防止覆盖未读数据）
        if (next_head != uart_rx_fifo.tail) {
            uart_rx_fifo.buffer[uart_rx_fifo.head] = aRxByte;
            uart_rx_fifo.head = next_head;
        }

        // 2. 重新开启单字节中断接收
        HAL_UART_Receive_IT(&huart1, &aRxByte, 1);

        // 3. (可选) 释放信号量，通知任务有新数据可读
        // osSemaphoreRelease(Sem_RxDataHandle);
    }
}

