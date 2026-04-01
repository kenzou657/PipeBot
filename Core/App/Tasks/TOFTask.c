//
// Created by asus on 2026/3/30.
//

#include <stdio.h>

#include "cmsis_os2.h"
#include "Types/LEDType.h"
#include "bsp_tof050f.h"



void StartTOFTask(void *argument) {
    for(;;) {
        // 3. 直接读取映射后的结果寄存器
        uint16_t distL = TOF_ReadDistance(sensor_L.i2c_addr);
        uint16_t distR = TOF_ReadDistance(sensor_R.i2c_addr);

        // 4. 更新全局数据并进行简单的有效性判断
        if (distL != 0xFFFF) {
            sensor_L.distance_mm = distL;
            sensor_L.is_online = 1;
        } else {
            sensor_L.is_online = 0; // 掉线处理
        }

        if (distR != 0xFFFF) {
            sensor_R.distance_mm = distR;
            sensor_R.is_online = 1;
        } else {
            sensor_R.is_online = 0;
        }

        /* 此处可插入弯道控制逻辑（方案二：八字布局）：
           int error = sensor_L.distance_mm - sensor_R.distance_mm;
           Apply_Differential_Drive(error);
        */

        printf("%d,%d\n", sensor_L.distance_mm, sensor_R.distance_mm);

        // 5. 任务调度周期建议与传感器更新周期匹配
        osDelay(100);
    }
}
