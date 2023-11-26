#include "Task_init.h"
#include "System_DataPool.h"


/**
 * @brief      ×Ü¿ØÈÎÎñ
 * @param[in]  None
 * @retval     None
 */
void Task_Control(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 

    for(;;)
    {
        Infantry.Control();

        if(Infantry.Get_FPS() == 500)
        {
            LED.BLN_Ctrl();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
