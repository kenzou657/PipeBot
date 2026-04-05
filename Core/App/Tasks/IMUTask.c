//
// Created by asus on 2026/3/21.
//

#include <stdio.h>

#include "bsp_jy61p.h"
#include "cmsis_os2.h"
#include "i2c.h"

void StartIMUTask(void *argument) {
    uint8_t i2c_buf[24];
    osStatus_t status;

    for(;;) {
        // 1. 发起非阻塞 I2C 读取请求
        // 从 AX(0x34) 开始读 24 字节，涵盖了 Acc, Gyro, Angle
        if (HAL_I2C_Mem_Read_IT(&hi2c1, JY61P_ADDR, AX, I2C_MEMADD_SIZE_8BIT, i2c_buf, 24) == HAL_OK) {

            // 2. 阻塞当前任务，等待 I2C 中断完成传输
            // osWaitForever 会让 CPU 此时去执行别的任务（如电机控制）
            status = osSemaphoreAcquire(Sem_I2CHandle, osWaitForever);

            if (status == osOK) {
                // 3. 传输完成后，调用刚才封装的函数进行数据打包
                JY61P_Decode(&imu_data, i2c_buf);
                // printf("%d,%d,%d\n", (int)imu_data.angle[0], (int)imu_data.angle[1], (int)imu_data.angle[2]);

                // 【提示】在这里你可以添加定位算法代码，比如：
                // Update_Odometer_Logic(&imu_data);
            }
        } else {
            // 如果 I2C 忙或报错，尝试重置或稍微延时
            osDelay(1);
        }

        // 4. 控制采样频率为 100Hz
        osDelay(10);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // 释放信号量，通知 IMU 任务数据已就绪
        osSemaphoreRelease(Sem_I2CHandle);
    }
}

/* 错误处理回调：防止线缆松动导致的 I2C 挂死 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // 可以在此处清除错误标志位或尝试重置 I2C 外设
    }
}
