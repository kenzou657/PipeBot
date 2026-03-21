//
// Created by asus on 2026/3/19.
//

#include "bsp_usart_dma.h"

PID_Params_t MotorL_PID = {4.1f, 0.15f, 0.0f};
PID_Params_t MotorR_PID = {4.1f, 0.15f, 0.0f};
RingBuffer_t uart_rx_fifo = {0};
uint8_t aRxByte = 0;

uint8_t sum_test;


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

void Serial_Parse_Task(void) {
    static ParseState_t state = STATE_IDLE;
    static uint8_t temp_packet[11]; // 临时存放正在组装的帧
    static uint8_t data_cnt = 0;

    // 只要缓冲区里有数据（读指针不等于写指针），就一直处理
    while (uart_rx_fifo.tail != uart_rx_fifo.head) {
        // 从环形缓冲区取出一个字节
        uint8_t byte = uart_rx_fifo.buffer[uart_rx_fifo.tail];
        uart_rx_fifo.tail = (uart_rx_fifo.tail + 1) % RING_BUF_SIZE;

        switch (state) {
            case STATE_IDLE:
                if (byte == 0x55) { // 发现帧头
                    temp_packet[0] = 0x55;
                    data_cnt = 1;
                    state = STATE_HEADER_OK;
                }
                break;

            case STATE_HEADER_OK:
                temp_packet[data_cnt++] = byte;

                if (data_cnt >= 11) { // 凑够一帧长度
                    // 1. 检查帧尾
                    if (temp_packet[10] == 0xBB) {
                        // 2. 计算校验和
                        uint8_t sum = 0;
                        for (int i = 0; i < 9; i++) sum += temp_packet[i];

                        if (sum == temp_packet[9]) {
                            // --- 校验完全通过！---
                            Process_Valid_Frame(temp_packet); // 这里去解析 PID
                        }
                    }
                    // 无论校验是否通过，处理完 11 字节后都回到空闲态找下一个头
                    state = STATE_IDLE;
                    data_cnt = 0;
                }
                break;
        }
    }
}

void Process_Valid_Frame(uint8_t *packet) {
    // packet[0] 是 0x55 (帧头)
    // packet[1] 是 ID (0x08)
    // packet[2] 是 Length (0x06)

    uint8_t msg_id = packet[1];

    if (msg_id == 0x08) { // 判断是否为 PID 调节指令

        // 1. 数据还原：将两个 uint8 拼接回 uint16，再除以 100 恢复成 float
        // 使用位移操作：高字节左移 8 位 + 低字节
        float new_kp = (float)((packet[3] << 8) | packet[4]) / 100.0f;
        float new_ki = (float)((packet[5] << 8) | packet[6]) / 100.0f;
        float new_kd = (float)((packet[7] << 8) | packet[8]) / 100.0f;

        // 2. 赋值给你的 PID 结构体（假设你有一个全局变量）
        // 注意：这里可以加一个简单的数值保护，防止输入非法超大值
        if (new_kp >= 0 && new_kp < 500.0f) {
            MotorL_PID.Kp = new_kp;
            MotorR_PID.Kp = new_kp;
            MotorL_PID.Ki = new_ki;
            MotorR_PID.Ki = new_ki;
            MotorL_PID.Kd = new_kd;
            MotorR_PID.Kd = new_kd;

        }
    }
    // 如果以后有其他 ID（比如 0x01 控制电机启停），可以继续写 else if
}


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

