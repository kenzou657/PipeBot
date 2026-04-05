//
// Created by asus on 2026/3/19.
//

#ifndef PIPEBOT_BSP_USART_DMA_H
#define PIPEBOT_BSP_USART_DMA_H
#include <stdint.h>

#include "Types/LEDType.h"
#include "main.h"

#define FRAME_SIZE 11


extern uint8_t g_rx_raw_buf[FRAME_SIZE * 2];
extern uint16_t g_actual_rx_len;

void UART_Device_Init(UART_HandleTypeDef *huart, uint8_t *pBuf, uint16_t size);
uint8_t Protocol_Calculate_Checksum(uint8_t *pData, uint16_t len);
void Protocol_Parse_Stream(uint8_t *raw_data, uint16_t len);
void Protocol_Handle_Payload(ProtocolFrame_t *frame);
// void Process_Serial_Data(uint8_t *pData, uint16_t len);

#endif //PIPEBOT_BSP_USART_DMA_H