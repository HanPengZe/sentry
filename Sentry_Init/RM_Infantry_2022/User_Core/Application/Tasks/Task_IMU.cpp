#include "Task_init.h"
#include "BMI088_EKF/INS_EKF_task.h"
#include <FreeRTOS.h>
#include <task.h>
#include "cmsis_os.h"
#include "Devices_Monitor.h"
#include "System_DataPool.h"


/**
 * @brief      IMUÈÎÎñ
 * @param[in]  None
 * @retval     None
 */
void Task_IMU_Process(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 1MS

    INS_EKF_Init();

    for(;;)
    {
        INS_EKF_Task();
        
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
