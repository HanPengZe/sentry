#include "Task_Monitor.h"
#include "Task_Init.h"
#include "System_DataPool.h"
#include "Dev_OLED.h"

/**
 * @brief      设备检测任务
 * @param[in]  None
 * @retval     None
 */
void Task_DevicesMonitor(void *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(20);  // --- 200MS

    static uint16_t monitor_cnt = 0;

    for(;;)
    {
#if USE_DeviceMonitor
        if(((++monitor_cnt)%=10)==0) //--- 200ms
        {
            DevicesMonitor.Devices_Detec();
        }
#endif


        // OLED_Button();
        // OLED_ShowMessage(OLED_Page);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

