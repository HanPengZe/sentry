#include "Task_init.h"
#include "System_DataPool.h"

/**
 * @brief      ����ϵͳ���ݸ���
 * @param[in]  None
 * @retval     None
 */
void Recv_Referee(void *arg)
{
    /* Pre-Load for task */
	static USART_COB* referee_pack;
    static TickType_t xLastWakeTime_t = xTaskGetTickCount();

    /* Infinite loop */
    for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t,1);
        // Sent_Contorl(&huart1);
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
		{
			// Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);		//���²���ϵͳ����
		}
		/* Pass control to the next task */
		
	}
}
