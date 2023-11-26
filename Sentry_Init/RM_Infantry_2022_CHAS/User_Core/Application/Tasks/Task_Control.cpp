#include "Task_Control.h"
#include "Task_Init.h"
#include "System_DataPool.h"
#include "Dev_LED.h"

/**
 * @brief      总控任务
 * @param[in]  None
 * @retval     None
 */
void Task_Control(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 

    static uint8_t led_cnt = 0;

    for(;;)
    {
        led_cnt++;

        // CTRL_DR16.LeverMode_Update();
        // CTRL_DR16.CTRLSource_Update();

        Infantry.Control();

        if((led_cnt%=50) == 0)
        {   //--- 任务正常运行流水灯
            Waterfall_LED();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

