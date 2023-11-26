#include "Task_Init.h"
#include "BSP_CAN.h"
#include "System_DataPool.h"
#include "Dev_Buzzer.h"

/**
  * @brief      �����û�����
  * @param[in]  None
  * @return     None
  */
 void System_Tasks_Init(void)
{
    Infantry.State = Robot_Initializing;

    //--- CAN1 ��������
    xTaskCreate(Task_CAN1_Recv, "CAN1_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

    //--- CAN2 ��������
    xTaskCreate(Task_CAN2_Recv, "CAN2_Receive", TASK_STACK_SIZE_256, NULL, PriorityHigh, NULL);

    //--- ���������
    xTaskCreate(Task_DevicesMonitor, "DevicesMonitor", TASK_STACK_SIZE_128, NULL, PriorityNormal, NULL);

    //--- IMU���ݲ�������
    xTaskCreate(Task_DataSampling, "IMUSampling", TASK_STACK_SIZE_256, NULL, PriorityNormal, NULL);

//    //--- RM�ͻ���UI����
//    xTaskCreate(Task_RMClientUI, "RMClient_UI", TASK_STACK_SIZE_256, NULL, PriorityAboveNormal, NULL);

//    //--- ��λ������Ŀǰ����Ҫ��
    xTaskCreate(Task_Debug, "Debug", TASK_STACK_SIZE_64, NULL, PriorityAboveNormal, NULL);

//    //--- ����ϵͳ���ݶ�ȡ����Ŀǰ����Ҫ��
//    xTaskCreate(Task_RefereeRecv, "Referee", TASK_STACK_SIZE_512, NULL, PriorityAboveNormal, &RefereeRecv_TaskHandle);

    //--- �ܿ�������(����󴴽�)
    xTaskCreate(Task_Control, "Control", TASK_STACK_SIZE_512, NULL, PriorityHigh, NULL);

    //--- ��ʼ����� ��������������
    Infantry.State = Robot_Activating;

	  // Buzzer.Music_Play(SuperMario, sizeof(SuperMario));
    
}


