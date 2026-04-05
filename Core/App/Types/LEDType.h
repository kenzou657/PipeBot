//
// Created by asus on 2026/3/10.
//

#ifndef PIPEBOT_LEDTYPE_H
#define PIPEBOT_LEDTYPE_H


#include "main.h"
// #include "bsp_usart_dma.h"

typedef enum {
    LEDColor_BLUE = 0,
    LEDColor_RED = 1
}LEDColor;

typedef enum {
    LEDState_Off = 0,
    LEDState_On = 1
}LEDState;

typedef struct {
    LEDColor color;
    LEDState state;
}LEDMessage;

typedef struct {
    uint8_t head;       // 0x55
    uint8_t id;         // 标识位
    uint8_t len;        // 0x06 (固定)
    uint8_t data[6];    // 数据位
    uint8_t checksum;   // 校验位
    uint8_t tail;       // 0xBB
    uint8_t reserved;   // 补齐位，凑够12字节对齐
} ProtocolFrame_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;       // 目标值（如目标转速）
    float current;      // 当前值（编码器读到的实际转速）
    float err;          // 当前误差 e(k)
    float last_err;     // 上一次误差 e(k-1)
    float prev_err;     // 上上次误差 e(k-2)
    float output;       // PID 计算出的增量输出
} PID_Params_t;

/* 数据结构体，用于存放解析后的物理量 */
typedef struct {
    float acc[3];   // 加速度
    float gyro[3];  // 角速度
    float angle[3]; // 角度
} JY61P_Data_t;


#endif //PIPEBOT_LEDTYPE_H