#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Private function prototypes -----------------------------------------------*/

void System_Tasks_Init(void);

void Task_CAN1_Recv(void *argument);
void Task_CAN2_Recv(void *argument);
void Task_CAN1_Transmit(void *argument);
void Task_Control(void *argument);
void Task_DevicesMonitor(void *argument);
void Task_DataSampling(void *argument);
void Task_Debug(void *argument);
void Recv_Referee(void *arg);
void Task_LED(void *argument);
void Task_VisionSend(void *argument);
void Task_VisionRecv(void *argument);
void Task_IMU_Process(void *argument);
void Task_DR16_Recv(void *argument);

#ifdef __cplusplus
}
#endif

#endif
