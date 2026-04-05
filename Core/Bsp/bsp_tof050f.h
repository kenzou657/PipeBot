//
// Created by asus on 2026/3/30.
//

#ifndef PIPEBOT_BSP_TOF050F_H
#define PIPEBOT_BSP_TOF050F_H

#include "stm32f4xx_hal.h"
#include "Types/LEDType.h"

#define TOF_FRAME_SIZE 8
#define TOF_LEFT_ADDR 0x02
#define TOF_RIGHT_ADDR 0x03

extern uint8_t g_tofl_buf[TOF_FRAME_SIZE * 2];
extern uint16_t g_tofl_rx_len;

extern uint8_t g_tofr_buf[TOF_FRAME_SIZE * 2];
extern uint16_t g_tofr_rx_len;

int8_t TOF_Parse(uint8_t *raw_buf, uint16_t len, uint8_t target_addr, uint16_t *out_dist);

#endif //PIPEBOT_BSP_TOF050F_H