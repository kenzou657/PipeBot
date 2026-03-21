//
// Created by asus on 2026/3/19.
//

#ifndef PIPEBOT_BSP_USART_DMA_H
#define PIPEBOT_BSP_USART_DMA_H
#include <stdint.h>

#include "Types/LEDType.h"
#include "main.h"

#define RING_BUF_SIZE 512

extern PID_Params_t MotorL_PID;
extern PID_Params_t MotorR_PID;
extern RingBuffer_t uart_rx_fifo;
extern uint8_t aRxByte;

uint8_t Protocol_Calculate_Checksum(uint8_t *pData, uint16_t len);
void Serial_Parse_Task(void);
void Process_Valid_Frame(uint8_t *packet);
// void Process_Serial_Data(uint8_t *pData, uint16_t len);

#endif //PIPEBOT_BSP_USART_DMA_H