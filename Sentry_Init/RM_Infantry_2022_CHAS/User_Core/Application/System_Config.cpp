/* Includes ------------------------------------------------------------------*/
#include "System_Config.h"
#include "System_DataPool.h"
#include "Communication.h"
#include "GRWML.h"
#include "tim.h"
#include "IMU_Compensate.h"
#include "DJI_AIMU.h"
#include "Referee_System.h"
#include "BSP_PVD.h"
#include "SerialDebug.h"
#include "Dev_OLED.h"

void User_System_Init(void)
{
	/* DJI-IMU Init */  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	IMU_CompensateFUN.Preserve_temp(/* ADC_FUN.get_temprate() */40.0f); //温度校准
  DJI_IMUFUN.DJI_IMU_Init();

	/* PWR Init */
  PVD_Config();
	
    /* Drivers Init ---------------------*/
	Uart_Init(&huart1,Uart1_Rx_Buff,USART1_RX_BUFFER_SIZE,DR16_Recv_Callback);//DR16
	Uart_Init(&huart3,Uart3_Rx_Buff,USART3_RX_BUFFER_SIZE,Referee_Recv_Callback);	

	CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
	CanFilter_Init(&hcan1);
	CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
	CanFilter_Init(&hcan2);

	/* Referee system Init */
	Referee.Init(&huart3, Get_RefereeTime);	

	/* Upper Computor Uart Init */
	SerialDebug.Uart_Init(&huart6);

	/* RTOS Resources Init --------------*/
	Can1Recv_QueueHandle = xQueueCreate(60, sizeof(CanRxMsg_t));
	Can2Recv_QueueHandle = xQueueCreate(40, sizeof(CanRxMsg_t));
	Referee_QueueHandle = xQueueCreate(10,sizeof(USART_COB));

	// HAL_Delay(2000);

	// Vision.Init();

	/* OLED 初始化 */
	oled_init();
}



