#include "Task_init.h"
#include "System_DataPool.h"

/**
 * @brief      视觉通信任务
 * @param[in]  None
 * @retval     None
 */
void Task_VisionSend(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);

    for(;;)
    {
			if(Gimbal.init_mode == false)
			{
					Vision.Sentry_SendToPC(&Vision.Sentry_Send_Msg); //--- 等IMU初始化完毕再发数据
			}
//			osDelay(5);
			if(DevicesMonitor.Get_State(COMMU_0X342_MONITOR) != Off_line ||
				DevicesMonitor.Get_State(COMMU_0X343_MONITOR) != Off_line)
			{
				Vision.Radar_SendToPC(&Vision.Radar_Send_Msg);
			}
//			osDelay(5);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}


/**
 * @note  通过队列接收与串口层解耦
 */
void Task_VisionRecv(void *argument)
{
    /* Pre-Load for task */
    // static TickType_t xLastWakeTime_t = xTaskGetTickCount();

	static USART_COB Usart_RxCOB;
    /* Infinite loop */
    for(;;)
	{
		// /* Pass control to the next task */
		// if(xQueueReceive(Vision_QueueHandle,&Usart_RxCOB,portMAX_DELAY) == pdPASS)
		// {
		// 	switch(Usart_RxCOB.port_num)
		// 	{
		// 	case 6:
		// 		memcpy(Vision.Recv_Msg.data,Usart_RxCOB.address,VISION_RX_SIZE);

        //         if(Vision.Get_Mode() == false || Vision.Recv_Msg.Pack.start_Tag != 'S' || Vision.Recv_Msg.Pack.end_Tag != 'E' ||)
        //         {
        //             Vision.Recv_Msg.Pack.yaw = Vision.Recv_Msg.Pack.pit = Vision.Recv_Msg.Pack.depth = 0;
        //             Vision.Update_Flag = false;
        //         }
		// 		break;

		// 	default:
		// 		break;
		// 	}
  		// }

        /*---------------------------------------------------*/
        if(xQueueReceive(Vision_QueueHandle,&Usart_RxCOB,portMAX_DELAY) == pdPASS)
		{
			switch(Usart_RxCOB.port_num)
			{
			case 6:
//				memcpy(Vision.Recv_Msg.data,Usart_RxCOB.address,VISION_RX_SIZE);

//				if(Vision.Get_Mode() == false || Vision.Recv_Msg.Pack.start_Tag != 'S' || Vision.Recv_Msg.Pack.end_Tag != 'E')
//				{
//					Vision.Recv_Msg.Pack.yaw = Vision.Recv_Msg.Pack.pit = /* Vision.Recv_Msg.Pack.depth = */ 0;
//					Vision.Update_Flag = false;
//				}
				break;

			default:
				break;
			}
  		}
	}
}

