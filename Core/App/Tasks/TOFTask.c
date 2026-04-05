//
// Created by asus on 2026/3/30.
//

#include <stdio.h>

#include "cmsis_os2.h"
#include "Types/LEDType.h"
#include "bsp_tof050f.h"
#include "bsp_usart_dma.h"
#include "usart.h"

uint16_t disL = 0, disR = 0;

void StartTOFTask(void *argument) {
    UART_Device_Init(&huart3, g_tofl_buf, sizeof(g_tofl_buf)); // 内部开启 HAL_UARTEx_ReceiveToIdle_DMA
    UART_Device_Init(&huart4, g_tofr_buf, sizeof(g_tofr_buf)); // 内部开启 HAL_UARTEx_ReceiveToIdle_DMA

    osSemaphoreRelease(Sem_TOFLRxCompleteHandle);
    osSemaphoreRelease(Sem_TOFRRxCompleteHandle);

    ProtocolFrame_t feedbackFrame;
    feedbackFrame.head = 0x55;
    feedbackFrame.id   = 0x06; // 测距传感器
    feedbackFrame.len  = 0x06;
    feedbackFrame.tail = 0xBB;

    for (;;) {
        // 等待信号量唤醒（不消耗 CPU）
        if (osSemaphoreAcquire(Sem_TOFLRxCompleteHandle, osWaitForever) == osOK) {

            // 一次性处理缓冲区内可能存在的所有帧
            // 处理 USART3 的 TOF
            if (TOF_Parse(g_tofl_buf, g_tofl_rx_len, TOF_LEFT_ADDR, &disL) == 0) {
                feedbackFrame.data[0] = (uint8_t)(disL >> 8);
                feedbackFrame.data[1] = (uint8_t)(disL & 0xFF);
            }

            // 处理完成后重置接收长度
            g_tofl_rx_len = 0;
        }

        // 等待信号量唤醒（不消耗 CPU）
        if (osSemaphoreAcquire(Sem_TOFRRxCompleteHandle, osWaitForever) == osOK) {

            // 一次性处理缓冲区内可能存在的所有帧
            // 处理 UART4 的 TOF
            if (TOF_Parse(g_tofr_buf, g_tofr_rx_len, TOF_RIGHT_ADDR, &disR) == 0) {
                feedbackFrame.data[2] = (uint8_t)(disR >> 8);
                feedbackFrame.data[3] = (uint8_t)(disR & 0xFF);
            }

            // 处理完成后重置接收长度
            g_tofr_rx_len = 0;
        }

        feedbackFrame.data[4] = 0;
        feedbackFrame.data[5] = 0;

        osMessageQueuePut(UART_TxQueueHandle, &feedbackFrame, 0, 0);
    }
}
