//
// Created by asus on 2026/3/21.
//

#ifndef PIPEBOT_BSP_JY61P_H
#define PIPEBOT_BSP_JY61P_H

#include "Types/LEDType.h"

/* JY61P 寄存器定义 */
#define JY61P_ADDR      (0x50 << 1) // 硬件地址左移一位
#define SAVE            0x00
#define AX              0x34        // 加速度 X 轴起始地址
#define GX              0x37        // 角速度 X 轴起始地址
#define AngleX          0x3d        // 角度 X 轴起始地址

/* JY61P 换算常量定义 */
#define ACC_RANGE      16.0f     // 加速度量程 ±16g
#define ANGLE_RANGE    180.0f    // 角度范围 ±180°
#define GYRO_RANGE     2000.0f   // 角速度量程 ±2000°/s
#define Q_BASE         32768.0f  // 2^15，用于 16 位有符号整型归一化

extern JY61P_Data_t imu_data;

void JY61P_Decode(JY61P_Data_t *pDest, uint8_t *pSrc);

#endif //PIPEBOT_BSP_JY61P_H