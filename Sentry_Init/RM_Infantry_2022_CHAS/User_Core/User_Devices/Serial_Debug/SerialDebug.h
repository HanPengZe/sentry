#ifndef _UPPER_COM_H_ 
#define _UPPER_COM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "usart.h"
/* Exported types ------------------------------------------------------------*/
class SerialDebug_classdef
{
private:
    uint8_t ANO_SendBuff[35] = {0};
    int32_t ANO_UserBuff[8] = {0};

    unsigned char Mini_Buffer[42] = {0}; // 数据发送缓冲区

    UART_HandleTypeDef *Send_Uart; // 外部句柄
public:
    SerialDebug_classdef();

    void Uart_Init(UART_HandleTypeDef *_huart);

    /* ANO 上位机*/
    void ANO_DataShow(float data1, float data2, float data3, float data4);
    /* MiniBalance 上位机 */
    void Mini_Float2Byte(float *target, unsigned char *buf, unsigned char beg);
    void Mini_addData(float Data, unsigned char Channel);
    void Mini_Datashow(int ChannelAmount);
    unsigned char Mini_DataGenerate(unsigned char Channel_Number);



};

extern SerialDebug_classdef SerialDebug;


#endif
