//
// Created by asus on 2026/3/30.
//

#ifndef PIPEBOT_BSP_TOF050F_H
#define PIPEBOT_BSP_TOF050F_H

#include "stm32f4xx_hal.h"
#include "Types/LEDType.h"

/* 设备 8 位 I2C 地址  */
#define TOF_ADDR_LEFT       0x52
#define TOF_ADDR_RIGHT      0x54

/* 厂家映射寄存器地址 */
#define REG_RESET           0x0001  // 重启:写入 0x1000
#define REG_DIST_VALUE      0x0010  // 测量结果: 2字节只读 (单位 mm)

/* 函数声明 */
HAL_StatusTypeDef TOF_WriteMappedReg16(uint8_t dev_addr, uint16_t reg_addr, uint16_t value);
uint16_t          TOF_ReadDistance(uint8_t dev_addr);

extern TOF_Sensor_t sensor_L;
extern TOF_Sensor_t sensor_R;

#endif //PIPEBOT_BSP_TOF050F_H