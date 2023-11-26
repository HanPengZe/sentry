#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "GRWML.h"
#include "System_DataPool.h"
#include "DR16.h"
#include "Motor.h"
#include "main.h"

#ifdef __cplusplus
extern "C"{
#endif

/*------------------------------ System Handlers ------------------------------*/

	
	
/*------------------------------Function prototypes ---------------------------*/   
uint32_t DR16_Recv_Callback(uint8_t *buf, uint16_t len);
uint32_t User_UART3_RxCpltCallback(uint8_t *buf, uint16_t len);
uint32_t Referee_Recv_Callback(uint8_t* Recv_Data, uint16_t ReceiveLen);
void User_CAN1_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);
void User_CAN2_RxCpltCallback(CanRxMsg_t *CAN_RxMessage);


#ifdef __cplusplus
}
#endif

#endif
