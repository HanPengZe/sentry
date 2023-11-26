#include "Task_Init.h"
#include "BSP_CAN.h"
#include "System_DataPool.h"
#include "Dev_Buzzer.h"

/**
  * @brief      创建用户任务
  * @param[in]  None
  * @return     None
  */
 void System_Tasks_Init(void)
{
    Infantry.State = Robot_Initializing;

    //--- CAN1 接收任务
    xTaskCreate(Task_CAN1_Recv, "CAN1_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

    //--- CAN2 接收任务
    xTaskCreate(Task_CAN2_Recv, "CAN2_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

    //--- 设变检测任务
    xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

    //--- IMU数据采样任务
    xTaskCreate(Task_DataSampling, "IMUSampling", TASK_STACK_SIZE_256, NULL, PriorityNormal, NULL);

//    //--- RM客户端UI任务
//    xTaskCreate(Task_RMClientUI, "RMClient_UI", TASK_STACK_SIZE_256, NULL, PriorityAboveNormal, NULL);

//    //--- 上位机任务（目前不需要）
    xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

//    //--- 裁判系统数据读取任务（目前不需要）
//    xTaskCreate(Task_RefereeRecv, "Referee", TASK_STACK_SIZE_512, NULL, PriorityAboveNormal, &RefereeRecv_TaskHandle);

    //--- 总控制任务(放最后创建)
    xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- 初始化完毕 机器人允许运作
    Infantry.State = Robot_Activating;

	  // Buzzer.Music_Play(SuperMario, sizeof(SuperMario));
    
}


