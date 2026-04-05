//
// Created by asus on 2026/3/30.
//

#include "bsp_tof050f.h"

/* 底层接收缓冲区，大小设为结构体的 2 倍以上防止溢出 */
uint8_t g_tofl_buf[TOF_FRAME_SIZE * 2];
uint16_t g_tofl_rx_len = 0;

uint8_t g_tofr_buf[TOF_FRAME_SIZE * 2];
uint16_t g_tofr_rx_len = 0;

/**
 * @brief TOF 解析函数
 * @param raw_buf    DMA 接收缓冲区
 * @param len        当前收到的总长度
 * @param target_addr 预期的设备地址 (例如 0x01)
 * @param out_dist   解析出的距离结果
 * @return int8_t    0: 成功, -1: 未找到匹配包头, -2: 长度不足, -3: 校验失败
 */
int8_t TOF_Parse(uint8_t *raw_buf, uint16_t len, uint8_t target_addr, uint16_t *out_dist) {

    // 1. 滑动窗口搜索包头 (即从机地址)
    for (uint16_t i = 0; i <= (len - 7); i++) {

        // 匹配包头（地址）且 匹配功能码 (0x03)
        if (raw_buf[i] == target_addr && raw_buf[i + 1] == 0x03) {

            // 找到包头后，检查剩余长度是否足够一帧 (7字节)
            if ((len - i) < 7) return -2;

            // 解析测量值 (大端转小端)
            *out_dist = (uint16_t)(raw_buf[i + 3] << 8) | raw_buf[i + 4];

            return 0; // 解析成功
        }
    }

    return -1; // 遍历整个缓冲区未找到匹配包头
}
