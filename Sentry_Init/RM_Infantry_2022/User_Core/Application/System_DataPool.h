#ifndef USER_SYS_DATAPOOL_H
#define USER_SYS_DATAPOOL_H

/* Includes ------------------------------------------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cmsis_os.h"

#include "can.h"
#include "usart.h"

/* Middlewares & Drivers Support */
#include <stm32f4xx.h>
#include <GRWML.h>

#include "Control_DR16.h"
#include "Devices_Monitor.h"
#include "Control_Robot.h"
#include "Control_Chassis.h"
#include "Control_Gimbal.h"
//#include "Referee_System.h"
//#include "Power_Meter.h"
#include "Dev_IMU.h"
#include "Dev_LED.h"

// #include "INS_task.h"
// #include "../BMI088/calibrate_task.h"


#include "INS_EKF_task.h"
#include "QuaternionEKF.h"
#include "../BMI088_EKF/BMI088driver.h"




/* Macro Definitions ---------------------------------------------------------*/
#define TASK_STACK_SIZE_64    64
#define TASK_STACK_SIZE_128   128
#define TASK_STACK_SIZE_256   256
#define TASK_STACK_SIZE_512   512
#define TASK_STACK_SIZE_1024  1024

#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8



/* Exported types ------------------------------------------------------------*/
enum PID_calctype_e
{
    PID_Outer,
    PID_Inner
};

enum Switchtype_e
{
    OFF,
    ON
};

typedef struct
{
  uint8_t  port_num;
  int16_t  len;
  void*    address;
}USART_COB;

enum SendMotor_ID_e
{
		SendID_Yaw=0,
		SendID_Barrel=0,
		SendID_Pit=1,
		SendID_Reload=1,
		SendID_Fric_L=2,
		SendID_Fric_R=3,
};

/* RTOS Resources ------------------------------------------------------------*/
extern QueueHandle_t Can1Recv_QueueHandle;
extern QueueHandle_t Can2Recv_QueueHandle;
extern QueueHandle_t Can1Send_QueueHandle;
extern QueueHandle_t Referee_QueueHandle;
extern QueueHandle_t Vision_QueueHandle;

extern TaskHandle_t RefereeRecv_TaskHandle;

extern TaskHandle_t Visionrecv_TaskHandle;
extern TaskHandle_t DR16recv_TaskHandle;

extern osMessageQId xQueueCAN1_RecvHandle;
extern osMessageQId xQueueCAN2_RecvHandle;

/* HAL Handlers --------------------------------------------------------------*/
// extern CAN_HandleTypeDef hcan1;
// extern CAN_HandleTypeDef hcan2;

// extern UART_HandleTypeDef huart1;
// extern UART_HandleTypeDef huart3;
// extern UART_HandleTypeDef huart6;

#define USART1_RX_BUFFER_SIZE 32
#define USART2_RX_BUFFER_SIZE 64
#define USART3_RX_BUFFER_SIZE 256
#define USART4_RX_BUFFER_SIZE 256
#define USART5_RX_BUFFER_SIZE 512
// #define USART6_RX_BUFFER_SIZE 1024
#define USART6_RX_BUFFER_SIZE 32

extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
extern uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];  
extern uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];     


/* Global Objects ------------------------------------------------------------*/
extern DR16_Classdef DR16;
extern CTRL_DR16_classdef CTRL_DR16;
extern DevicesMonitor_classdef DevicesMonitor;
extern Robot_classdef Infantry;
extern Chassis_classdef Chassis;
extern Gimbal_classdef Gimbal;
extern Shoot_classdef Shoot; 
// extern referee_Classdef Referee;
extern Vision_classdef Vision;
extern IMU_classdef DJI_CIMU;
extern LED_classdef LED;


#endif
