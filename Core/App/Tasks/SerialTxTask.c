//
// Created by asus on 2026/3/19.
//

#include "bsp_usart_dma.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "Types/LEDType.h"

void StartSerialTxTask(void *argument) {
    ProtocolFrame_t txFrame;

    // 初始化时释放一次信号量，表示串口空闲
    osSemaphoreRelease(Sem_TxCompleteHandle);

    for(;;) {
        // 1. 阻塞等待队列：有任务想发数据吗？
        if (osMessageQueueGet(UART_TxQueueHandle, &txFrame, NULL, osWaitForever) == osOK) {

            // 2. 阻塞等待硬件：上一帧发完了吗？
            osSemaphoreAcquire(Sem_TxCompleteHandle, osWaitForever);

            // 3. 计算校验 (对前9字节求和)
            txFrame.checksum = Protocol_Calculate_Checksum((uint8_t*)&txFrame, 9);

            // 4. 执行发送 (注意：只发送协议要求的11字节，忽略最后的reserved)
            // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&txFrame, 11);
        }
    }
}

// DMA发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        osSemaphoreRelease(Sem_TxCompleteHandle);
    }
}
