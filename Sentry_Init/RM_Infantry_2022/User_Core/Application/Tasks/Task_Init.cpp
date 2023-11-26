#include "Task_Init.h"
#include "System_DataPool.h"
// #include "BMI088/INS_task.h"
// #include "calibrate_task.h"
#include "Dev_Buzzer.h"
#include "BMI088_EKF/INS_EKF_task.h"


/**
  * @brief      �����û�ϵͳ����
  * @param[in]  None
  * @return     None
  */
void System_Tasks_Init(void)
{
//		taskENTER_CRITICAL();
    Infantry.State = Robot_Initializing;

    //--- CAN1 ��������
    xTaskCreate(Task_CAN1_Recv, "CAN1_Recv", TASK_STACK_SIZE_256, NULL,PriorityHigh, NULL);

    //--- CAN2 ��������
    xTaskCreate(Task_CAN2_Recv, "CAN2_Recv", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

//    //--- �豸�������
//    xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, NULL);

    //--- IMU���ݶ�ȡ����
    // xTaskCreate((TaskFunction_t)INS_task, "IMU_Update", TASK_STACK_SIZE_512, NULL, PriorityAboveNormal, NULL);
    xTaskCreate(Task_IMU_Process, "IMU_Update", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- IMUУ׼����
    // xTaskCreate((TaskFunction_t)calibrate_task, "IMU_Calibrate", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

    //--- ��������
//    xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

    //--- �Ӿ����ݷ�������
    xTaskCreate(Task_VisionSend, "VisionSend", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- �Ӿ����ݶ�ȡ����
    // xTaskCreate(Task_VisionRecv, "VisionRecv", TASK_STACK_SIZE_256, NULL, PriorityHigh, &Visionrecv_TaskHandle);
    
    //--- LED Buzzer����
    // xTaskCreate(Task_LED, "LED_Buzzer", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

    //--- ����ϵͳ���ݶ�ȡ����
    // xTaskCreate(Recv_Referee, "Referee", TASK_STACK_SIZE_256, NULL, PriorityAboveNormal, &RefereeRecv_TaskHandle);

    //--- �ܿ�������(����󴴽�)
    xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_1024, NULL, PriorityHigh, NULL);

	//--- ��ʼ����� ��������������
    Infantry.State = Robot_Activating;

	//--- DR16���ݶ�ȡ����
    xTaskCreate(Task_DR16_Recv, "DR16", TASK_STACK_SIZE_128, NULL, PriorityAboveNormal, &DR16recv_TaskHandle);


    // //--- ��ʼ����� ��������������
    // Infantry.State = Robot_Activating;

//	taskEXIT_CRITICAL();  
}

