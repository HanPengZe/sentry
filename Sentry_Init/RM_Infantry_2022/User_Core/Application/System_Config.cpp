/* Includes ------------------------------------------------------------------*/
#include "System_Config.h"
#include "System_DataPool.h"
#include "Communication.h"
#include "tim.h"
//#include "Referee_System.h"
#include "BSP_PVD.h"


#include "Dev_Buzzer.h"
#include "SerialDebug.h"
#include "Dev_OLED.h"
#include "BSP_ADC.h"
#include "spi.h"
#include "bsp_dwt.h"


void User_System_Init(void)
{
	
	LED.Init();

	/* dwt delay init */
	DWT_Init(168);

	/* DJI-C IMU Init */  
	// cali_param_init();
	BMI088_Init(&hspi1);

	/* PWR Init */
  	PVD_Config();

	/* 使用基准电压来校准ADC电压采样 */
    init_vrefint_reciprocal();
	
    /* Drivers Init ---------------------*/
	Uart_Init(&huart1,Vision.Radar_Recv_Msg.data,VISION_Radar_RX_SIZE,Vision_Radar_Recv_Callback);//底盘
	Uart_Init(&huart3,Uart3_Rx_Buff,USART3_RX_BUFFER_SIZE,DR16_Recv_Callback);//DR16 
	Uart_Init(&huart6,Vision.Sentry_Recv_Msg.data,VISION_Sentry_RX_SIZE,Vision_Sentry_Recv_Callback);//自瞄

	CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
	CanFilter_Init(&hcan1);
	CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
	CanFilter_Init(&hcan2);

	// Referee.Init(&huart3, Get_RefereeTime);	

	/* 上位机串口注册 */
	SerialDebug.Uart_Init(&huart1);
	
	/* RTOS Resources Init --------------*/
	Can1Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	Can2Recv_QueueHandle = xQueueCreate(50, sizeof(CanRxMsg_t));
	Vision_QueueHandle = xQueueCreate(15,sizeof(USART_COB));
	// Referee_QueueHandle = xQueueCreate(10,sizeof(USART_COB));

	// HAL_Delay(2000);

//	Vision.Init(&huart1);
//	Shoot.PWM_Init();//哨兵不用
	Buzzer.Init(&htim4, TIM_CHANNEL_3);

	/* OLED 初始化 */
//	oled_init();

}



