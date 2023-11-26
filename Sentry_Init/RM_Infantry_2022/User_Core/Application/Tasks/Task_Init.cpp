#include "Task_Init.h"
#include "System_DataPool.h"
// #include "BMI088/INS_task.h"
// #include "calibrate_task.h"
#include "Dev_Buzzer.h"
#include "BMI088_EKF/INS_EKF_task.h"


/**
  * @brief      创建用户系统任务
  * @param[in]  None
  * @return     None
  */
void System_Tasks_Init(void)
{
//		taskENTER_CRITICAL();
    Infantry.State = Robot_Initializing;

    //--- CAN1 接收任务
    xTaskCreate(Task_CAN1_Recv, "CAN1_Recv", TASK_STACK_SIZE_256, NULL,PriorityHigh, NULL);

    //--- CAN2 接收任务
    xTaskCreate(Task_CAN2_Recv, "CAN2_Recv", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

//    //--- 设备检测任务
//    xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);

    //--- IMU数据读取任务
    // xTaskCreate((TaskFunction_t)INS_task, "IMU_Update", TASK_STACK_SIZE_512, NULL, PriorityAboveNormal, NULL);
    xTaskCreate(Task_IMU_Process, "IMU_Update", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- IMU校准任务
    // xTaskCreate((TaskFunction_t)calibrate_task, "IMU_Calibrate", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

    //--- 调试任务
//    xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

    //--- 视觉数据发送任务
    xTaskCreate(Task_VisionSend, "VisionSend", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- 视觉数据读取任务
    // xTaskCreate(Task_VisionRecv, "VisionRecv", TASK_STACK_SIZE_256, NULL, PriorityHigh, &Visionrecv_TaskHandle);
    
    //--- LED Buzzer任务
    // xTaskCreate(Task_LED, "LED_Buzzer", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

    //--- 裁判系统数据读取任务
    // xTaskCreate(Recv_Referee, "Referee", TASK_STACK_SIZE_256, NULL, PriorityAboveNormal, &RefereeRecv_TaskHandle);

    //--- 总控制任务(放最后创建)
    xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_1024, NULL, PriorityHigh, NULL);

	//--- 初始化完毕 机器人允许运作
    Infantry.State = Robot_Activating;

	//--- DR16数据读取任务
    xTaskCreate(Task_DR16_Recv, "DR16", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, &DR16recv_TaskHandle);


    // //--- 初始化完毕 机器人允许运作
    // Infantry.State = Robot_Activating;

//	taskEXIT_CRITICAL();  
}

