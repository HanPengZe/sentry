/**
 ------------------------------------------------------------------------------
 * @file    Communation.cpp
 * @author  Shake
 * @brief   数据通讯
 * @version V0.1
 * @date    2021-10-14
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

/* Includes ------------------------------------------------------------------*/
#include "Communication.h"
#include "System_DataPool.h"
#include "Referee_System.h"

/**
 * @brief      USART1(DR16) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t DR16_Recv_Callback(uint8_t *buf, uint16_t len)
{
    DR16.DataCapture((DR16_DataPack_Typedef*)buf);
    DevicesMonitor.Update(Frame_DR16);

    CTRL_DR16.LeverMode_Update();
    CTRL_DR16.CTRLSource_Update();
    return 0;
}

/**
 * @brief      CAN1 接收中断回调函数
 * @param[in]  CAN_RxMessage
 * @retval     None
 */
void User_CAN1_RxCpltCallback(CanRxMsg_t *CAN_RxMessage)
{
    if(Can1Recv_QueueHandle == NULL)
    {
        return;
    }
    xQueueSendToBackFromISR(Can1Recv_QueueHandle,CAN_RxMessage,0); //--- Ban on waiting.
    DevicesMonitor.Update(Frame_CAN1);
}


/**
 * @brief      CAN2 接收中断回调函数
 * @param[in]  CAN_RxMessage
 * @retval     None
 */
void User_CAN2_RxCpltCallback(CanRxMsg_t *CAN_RxMessage)
{
    if(Can2Recv_QueueHandle == NULL)
    {
        return;
    }
    xQueueSendToBackFromISR(Can2Recv_QueueHandle,CAN_RxMessage,0); //--- Ban on waiting.
    DevicesMonitor.Update(Frame_CAN2);
}


/**
 * @brief      USART3(Referee) 接受中断回调函数
 * @param[in]  buf
 * @param[in]  len
 * @retval     None
 */
uint32_t User_UART3_RxCpltCallback(uint8_t *buf, uint16_t len)
{
    // static int16_t Judge_DMA_Counter;
    // // Judge_GetMessage(JUDGESYSTEM_PACKSIZE);
    // Judge_DMA_Counter = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    // Judge_GetMessage(389u - Judge_DMA_Counter);
    DevicesMonitor.Update(Frame_REFEREE);
    return 0;
}

uint32_t Referee_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
	static BaseType_t* pxHigherPriorityTaskWoken;
	static USART_COB referee_pack;
    //	static uint8_t temp_buff[256];
	
	if(RefereeRecv_TaskHandle != NULL && ReceiveLen < 256)
    {
		referee_pack.address = Recv_Data;
		referee_pack.len = ReceiveLen;
		xTaskNotifyFromISR(RefereeRecv_TaskHandle, (uint32_t) &referee_pack, eSetValueWithOverwrite, pxHigherPriorityTaskWoken);
        DevicesMonitor.Update(Frame_REFEREE);
	}
  return 0;
	
}
