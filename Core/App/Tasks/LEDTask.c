//
// Created by asus on 2026/3/10.
//

#include "bsp_motor.h"
#include "bsp_usart_dma.h"
#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
#include "Types/LEDType.h"

void StartLEDTask(void *argument) {

    for (;;) {
        LEDMessage* msg;
        osMessageQueueGet(LEDQueueHandle, &msg, 0, osWaitForever);
        switch (msg -> color) {
            case LEDColor_RED:
                HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, msg -> state ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
            case LEDColor_BLUE:
                HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, msg -> state ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
        }
        vPortFree(msg);

    }
}
