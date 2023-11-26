#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#ifdef __cplusplus
extern "C"
{
#endif

void System_Tasks_Init(void);

void Task_CAN1_Recv(void *argument);
void Task_CAN2_Recv(void *argument);
void Task_Control(void *argument);
void Task_DevicesMonitor(void *argument);
void Task_DataSampling(void *argument);
void Task_Debug(void *argument);
void Task_RefereeRecv(void *arg);
void Task_RMClientUI(void *argument);
void Task_DataSampling(void *argument);

#ifdef __cplusplus
}
#endif

#endif
