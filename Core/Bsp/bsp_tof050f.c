//
// Created by asus on 2026/3/30.
//

#include "bsp_tof050f.h"

extern I2C_HandleTypeDef hi2c2; // 使用你的硬件 I2C2

/* 传感器全局状态 */
TOF_Sensor_t sensor_L = {TOF_ADDR_LEFT, 0, 0};
TOF_Sensor_t sensor_R = {TOF_ADDR_RIGHT, 0, 0};

/**
 * @brief 向映射寄存器写入 16 位数据
 */
HAL_StatusTypeDef TOF_WriteMappedReg16(uint8_t dev_addr, uint16_t reg_addr, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);   // 高位在前
    buf[1] = (uint8_t)(value & 0xFF);
    return HAL_I2C_Mem_Write(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, buf, 2, 100);
}

/**
 * @brief 从 0x0010 寄存器读取 16 位距离值
 */
uint16_t TOF_ReadDistance(uint8_t dev_addr) {
    uint8_t buf[2] = {0};
    // 厂家映射模块通常将 8 位测量值扩展为 16 位返回
    if (HAL_I2C_Mem_Read(&hi2c2, dev_addr, REG_DIST_VALUE, I2C_MEMADD_SIZE_16BIT, buf, 2, 100) == HAL_OK) {
        return (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return 0xFFFF; // 读取失败返回特殊值
}

