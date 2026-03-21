//
// Created by asus on 2026/3/17.
//

#include <stdint.h>
#include <stdio.h>

#include "bsp_motor.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Types/LEDType.h"

void StartMotorTask(void *argument) {
    /* 1. 获取当前系统节拍数作为基准 */
    uint32_t tick = osKernelGetTickCount();

    ProtocolFrame_t feedbackFrame;
    feedbackFrame.head = 0x55;
    feedbackFrame.id   = 0x02; // 速度反馈
    feedbackFrame.len  = 0x06;
    feedbackFrame.tail = 0xBB;

    // Move_Forward(200);

    /* 2. 设定周期 */
    const uint32_t frequency = (uint32_t)(SAMPLE_TIME * 1000.0f);

    for (;;) {
        osStatus_t status = osDelayUntil(tick + frequency);

        if (status == osOK) {
            tick += frequency; // 手动更新基准时间，确保下一个周期的起点准确
        }

        // 读取编码器
        Motor_Read_Encoder();
        Move_Forward(180);
        printf("%d,%d\n", MotorL.velocity, MotorR.velocity);



        // 运动学解算并打包
        Pack_Kinematics_To_Frame(&feedbackFrame, MotorL.velocity, MotorR.velocity);





        osMessageQueuePut(UART_TxQueueHandle, &feedbackFrame, 0, 0);

    }
}
