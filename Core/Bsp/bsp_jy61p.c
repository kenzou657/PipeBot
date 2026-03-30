//
// Created by asus on 2026/3/21.
//

#include "bsp_jy61p.h"


JY61P_Data_t imu_data;

/**
 * @brief 解析从 JY61P 读取的原始缓冲区数据
 * @param pDest 目标结构体指针
 * @param pSrc  指向 rx_buf 的原始数据指针
 */
void JY61P_Decode(JY61P_Data_t *pDest, uint8_t *pSrc) {
    // 1. 解析加速度 (Register 0x34-0x36)
    // 利用 (int16_t) 强制转换处理负数补码
    pDest->acc[0] = (float)((int16_t)(pSrc[1] << 8 | pSrc[0])) / Q_BASE * ACC_RANGE;
    pDest->acc[1] = (float)((int16_t)(pSrc[3] << 8 | pSrc[2])) / Q_BASE * ACC_RANGE;
    pDest->acc[2] = (float)((int16_t)(pSrc[5] << 8 | pSrc[4])) / Q_BASE * ACC_RANGE;

    // 2. 解析角速度 (Register 0x37-0x39)
    pDest->gyro[0] = (float)((int16_t)(pSrc[7] << 8 | pSrc[6])) / Q_BASE * GYRO_RANGE;
    pDest->gyro[1] = (float)((int16_t)(pSrc[9] << 8 | pSrc[8])) / Q_BASE * GYRO_RANGE;
    pDest->gyro[2] = (float)((int16_t)(pSrc[11] << 8 | pSrc[10])) / Q_BASE * GYRO_RANGE;

    // 3. 解析角度 (Register 0x3D-0x3F)
    // 注意：如果是连续读取，偏移量需根据你的读取起始地址计算
    pDest->angle[0] = (float)((int16_t)(pSrc[19] << 8 | pSrc[18])) / Q_BASE * ANGLE_RANGE;
    pDest->angle[1] = (float)((int16_t)(pSrc[21] << 8 | pSrc[20])) / Q_BASE * ANGLE_RANGE;
    pDest->angle[2] = (float)((int16_t)(pSrc[23] << 8 | pSrc[22])) / Q_BASE * ANGLE_RANGE;
}

