#include "Task_init.h"
#include "Dev_Buzzer.h"
#include "System_DataPool.h"
#include "stdint.h"
#include <stddef.h>

/**
 * @brief      LED BUZZER
 * @param[in]  None
 * @retval     None
 */
void Task_LED(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(5);  // --- 5MS 

    // static uint8_t init_flag = true;

    for(;;)
    {
        if(Infantry.Get_FPS() == 500)
        {
            LED.BLN_Ctrl();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

