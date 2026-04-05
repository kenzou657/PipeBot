//
// Created by asus on 2026/3/19.
//

#include "bsp_usart_dma.h"

#include <string.h>

#include "bsp_motor.h"
#include "usart.h"



/* 底层接收缓冲区，大小设为结构体的 2 倍以上防止溢出 */
uint8_t g_rx_raw_buf[FRAME_SIZE * 2];
uint16_t g_actual_rx_len = 0;


/**
 * @brief  计算累加校验和 (uint8_t)
 * @param  pData: 待计算的数据首地址
 * @param  len: 需要累加的字节长度
 * @return uint8_t: 计算得到的校验值
 */
uint8_t Protocol_Calculate_Checksum(uint8_t *pData, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += pData[i];
    }
    return sum;
}

// 传入串口句柄、缓冲区指针、缓冲区大小
void UART_Device_Init(UART_HandleTypeDef *huart, uint8_t *pBuf, uint16_t size) {
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    // 开启 DMA
    HAL_UART_Receive_DMA(huart, pBuf, size);

    // 手动开启 IDLE 中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // 禁用半传输中断
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief 串口流多帧解析函数
 * @param raw_data 原始字节流缓冲区
 * @param len      本次接收到的总长度 (g_actual_rx_len)
 */
void Protocol_Parse_Stream(uint8_t *raw_data, uint16_t len) {
    ProtocolFrame_t temp_frame;
    uint8_t calculated_sum;

    // 1. 滑动窗口搜索
    for (uint16_t i = 0; i <= (len - FRAME_SIZE); ) {

        // 2. 匹配帧头 0x55 和 帧尾 0xBB
        if (raw_data[i] == 0x55 && raw_data[i + FRAME_SIZE - 1] == 0xBB) {

            // 拷贝到临时结构体进行校验
            memcpy(&temp_frame, &raw_data[i], FRAME_SIZE);

            // 计算前 9 字节校验和 (Head + ID + Len + Data[6])
            calculated_sum = Protocol_Calculate_Checksum((uint8_t *)&temp_frame, 9);

            if (calculated_sum == temp_frame.checksum) {
                /* --- 校验通过：处理本帧业务 --- */
                Protocol_Handle_Payload(&temp_frame);

                /* --- 核心修改：跳过已处理的帧长 --- */
                i += FRAME_SIZE;
                continue; // 直接进入下一轮循环，不执行末尾的 i++
            } else {
                // 校验失败，可能是伪随机数据，i++ 继续寻找真正的帧头
                i++;
            }
        } else {
            // 头尾不匹配，i++ 继续寻找
            i++;
        }
    }
}

/**
 * @brief 根据解析出的 ID 执行对应的小车动作
 */
void Protocol_Handle_Payload(ProtocolFrame_t *frame) {
    switch (frame->id) {
        case 0x01: // 运动控制指令
            // 示例：将 data[0-3] 转换为 float 速度或直接处理
            // Update_Motor_Setpoint(frame->data[0], frame->data[1]);
            break;

        case 0x02: // 传感器采样触发 (如开启管道缺陷检测相机)
            // Trigger_Camera_Capture();
            break;

        case 0x03: // 定位参数校准 (如 IMU 归零)
            // IMU_Calibrate_Zero();
            break;

        case 0x08: // PID 参数配置
            Motor_Update_PID_From_Protocol(frame->data);
            break;

        default:
            // 未知 ID 处理
            break;
    }
}

//
// void Process_Serial_Data(uint8_t *pData, uint16_t len) {
//     // 遍历整个缓冲区，寻找 0x55
//     // 注意：我们要找的是一帧的开头，所以遍历范围是 [0, len - FRAME_LEN]
//     for (int i = 0; i <= (len - FRAME_LEN); i++) {
//         if (pData[i] == 0x55) {
//             // 命中帧头
//
//             // 验证帧尾是否匹配
//             if (pData[i + FRAME_LEN - 1] == 0xBB) {
//                 // 验证校验和
//                 uint8_t sum = Protocol_Calculate_Checksum(&pData[i], 9);
//                 sum_test = sum;
//                 if (sum == pData[i + 9]) {
//                     // --- 校验通过，执行解析 ---
//                     uint8_t msg_id = pData[i + 1];
//
//                     if (msg_id == 0x08) { // PID 参数配置
//                         // 数据位 Data 0~1: Kp, 2~3: Ki, 4~5: Kd (大端模式，放大100倍)
//                         Motor_PID_Config.Kp = (float)((pData[3] << 8) | pData[4]) / 100.0f;
//                         Motor_PID_Config.Ki = (float)((pData[5] << 8) | pData[6]) / 100.0f;
//                         Motor_PID_Config.Kd = (float)((pData[7] << 8) | pData[8]) / 100.0f;
//                     }
//                     else if (msg_id == 0x01) { // 假设 0x01 是速度控制指令
//                         // 这里可以解析目标速度并更新到 MotorL.target_speed
//                     }
//                 }
//             }
//         }
//     }
// }

