//
// Created by asus on 2026/3/10.
//

#include <stdlib.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "Types/LEDType.h"

unsigned char Key_Scan(void)
{
    unsigned char unKey_Val = 0;

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
        unKey_Val = 1;

    return unKey_Val;
}



void StartKeyTask(void *argument) {

    unsigned char ucKey_Val, unKey_Down, ucKey_Up, ucKey_Old;
    LEDState state = LEDState_Off;

    ucKey_Old = Key_Scan();

    for (;;) {

        ucKey_Val = Key_Scan();
        unKey_Down = ucKey_Val & (ucKey_Old ^ ucKey_Val);
        ucKey_Up = ~ucKey_Val & (ucKey_Old ^ ucKey_Val);
        ucKey_Old = ucKey_Val;

        if(unKey_Down == 1)
        {
            state = !state;
            LEDMessage* msg = pvPortMalloc(sizeof(LEDMessage));
            msg->color = LEDColor_RED;
            msg->state = state;
            osMessageQueuePut(LEDQueueHandle, &msg, 0, 0);
        }

        osDelay(10);
    }
}
