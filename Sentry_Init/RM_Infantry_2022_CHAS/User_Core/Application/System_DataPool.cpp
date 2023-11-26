#include "System_DataPool.h"

//用于存储所有需要的变量

/* RTOS Resources ------------------------------------------------------------*/
//Queue
QueueHandle_t Can1Recv_QueueHandle;
QueueHandle_t Can2Recv_QueueHandle;
QueueHandle_t Can1Send_QueueHandle;

QueueHandle_t xQueueDR16Receive_Handle;

QueueHandle_t  Referee_QueueHandle;

TaskHandle_t RefereeRecv_TaskHandle;

/* Other Resources ------------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart1 */
uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */


/* Global Objects ------------------------------------------------------------*/
DR16_Classdef DR16;
CTRL_DR16_classdef CTRL_DR16;
DevicesMonitor_classdef DevicesMonitor;
Robot_classdef Infantry;
Chassis_classdef Chassis;
Gimbal_classdef Gimbal;
// Shoot_classdef Shoot;
referee_Classdef Referee;
// Vision_classdef Vision;
SupCap_classdef SupCap;
PowerMeter_classdef PowerMeter;
Auto_classdef Auto;

