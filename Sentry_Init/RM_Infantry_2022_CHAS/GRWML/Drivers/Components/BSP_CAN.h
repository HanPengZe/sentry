#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef  __cplusplus
extern "C" {
#endif
    /* Includes ------------------------------------------------------------------*/
#if defined(USE_HAL_DRIVER)
#if defined(STM32F427xx) || defined(STM32F429xx)
#include <stm32f4xx_hal.h>
#endif
#if defined(STM32F103xx)
#include <stm32f1xx_hal.h>
#endif
#if defined(STM32H750xx)
#include <stm32h7xx_hal.h>
#endif	
#endif
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define CAN_LINE_BUSY  0
#define CAN_SUCCESS    1


/* Exported types ------------------------------------------------------------*/
typedef struct CanRxMsg_t
{
	uint8_t Data[8];
	uint8_t CANx;
	CAN_RxHeaderTypeDef Header; 
}CanRxMsg_t;

typedef struct CanTxMsg_t
{
	uint8_t Data[8];
	uint8_t CANx;
	CAN_RxHeaderTypeDef Header; 
}CanTxMsg_t;

/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void CAN_Init(CAN_HandleTypeDef* hcan, void (*pFunc)(CanRxMsg_t*));
void CanFilter_Init(CAN_HandleTypeDef* hcan);
uint8_t CANx_SendData(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t* pData, uint16_t Len);



#ifdef  __cplusplus
}
#endif

#endif
