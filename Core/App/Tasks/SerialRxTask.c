//
// Created by asus on 2026/3/19.
//

#include "bsp_usart_dma.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_it.h"
#include "usart.h"

void StartSerialRxTask(void *argument) {
    UART_Rx_Init(); // 内部开启 HAL_UARTEx_ReceiveToIdle_DMA
    ProtocolFrame_t decoded_frame;
    osSemaphoreRelease(Sem_RxCompleteHandle);

    for (;;) {
        // 等待信号量唤醒（不消耗 CPU）
        if (osSemaphoreAcquire(Sem_RxCompleteHandle, osWaitForever) == osOK) {

            // 一次性处理缓冲区内可能存在的所有帧
            Protocol_Parse_Stream(g_rx_raw_buf, g_actual_rx_len);
            Protocol_Handle_Payload(&decoded_frame);

            // 处理完成后重置接收长度
            g_actual_rx_len = 0;
        }
    }
}

/**
 * @brief 串口事件回调函数 (由 IDLE 中断或 DMA 满中断触发)
 * @param Size 当前缓冲区中有效数据的长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        g_actual_rx_len = Size;

        // 释放信号量，唤醒 StartRxTask
        osSemaphoreRelease(Sem_RxCompleteHandle);

        /* 关键：因为我们用的是 HAL_UART_Receive_DMA，
         * 每次 IDLE 触发后，建议重启 DMA 以重置计数器 (NDTR)
         */
        HAL_UART_AbortReceive(huart);
        UART_Rx_Init();
    }
}

