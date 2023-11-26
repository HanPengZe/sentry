#include "Task_Init.h"
#include "DJI_AIMU.h"
#include "System_DataPool.h"
#include "IMU_Compensate.h"


/**
  * @brief      IMU数据采样任务
  * @param[in]  None
  * @retval     None
  */
void Task_DataSampling(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS

    for (;;)
    {
        if (IMU_Init_Condition == 1)
        {
            IMU_CompensateFUN.IMU_GetData_Compensate();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

