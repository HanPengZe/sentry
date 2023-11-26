/**
 ------------------------------------------------------------------------------
 * @file    UpperComputer.cpp
 * @author  Shake
 * @brief   ��λ��Ӧ��
 * @version V0.1
 * @date    2021-11
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "SerialDebug.h"
#include "usart.h"
/* Private macros ------------------------------------------------------------*/
#define USE_USART_DMA  1	/* �Ƿ�ʹ��DMA���� */
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
SerialDebug_classdef SerialDebug;
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
/**
 * @brief      Initialize Class
 * @param[in]  None
 * @retval     None
 */
SerialDebug_classdef::SerialDebug_classdef()
{
   
}

/**
 * @brief      ���ڳ�ʼ��
 * @param[in]  _huart
 * @retval     None
 */
void SerialDebug_classdef::Uart_Init(UART_HandleTypeDef *_huart)
{
	Send_Uart = _huart;
}

/**
 * @brief      ANO��λ������������ʾ
 * @param[in]  data 
 * @retval     None
 */
void SerialDebug_classdef::ANO_DataShow(float data1, float data2, float data3, float data4)
{
	int8_t data_sum = 0;
	int i = 0, cout = 0;
	
	ANO_UserBuff[0] = data1;
	ANO_UserBuff[1] = data2;
	ANO_UserBuff[2] = data3;
	ANO_UserBuff[3] = data4;
	ANO_SendBuff[cout++] = 0xAA;
	ANO_SendBuff[cout++] = 0x01;
	ANO_SendBuff[cout++] = 0xAF;
	ANO_SendBuff[cout++] = 0xF1;
	ANO_SendBuff[cout++] = 0;
	ANO_SendBuff[cout++] = ANO_UserBuff[0] >> 24;
	ANO_SendBuff[cout++] = ((ANO_UserBuff[0] >> 16) & 0x00ff);
	ANO_SendBuff[cout++] = ((ANO_UserBuff[0] >> 8) & 0x0000ff);
	ANO_SendBuff[cout++] = ANO_UserBuff[0];
	ANO_SendBuff[cout++] = ANO_UserBuff[1] >> 24;
	ANO_SendBuff[cout++] = ((ANO_UserBuff[1] >> 16) & 0x00ff);
	ANO_SendBuff[cout++] = ((ANO_UserBuff[1] >> 8) & 0x0000ff);
	ANO_SendBuff[cout++] = ANO_UserBuff[1];
	ANO_SendBuff[cout++] = ANO_UserBuff[2] >> 24;
	ANO_SendBuff[cout++] = ((ANO_UserBuff[2] >> 16) & 0x00ff);
	ANO_SendBuff[cout++] = ((ANO_UserBuff[2] >> 8) & 0x0000ff);
	ANO_SendBuff[cout++] = ANO_UserBuff[2];
	ANO_SendBuff[cout++] = ANO_UserBuff[3] >> 24;
	ANO_SendBuff[cout++] = ((ANO_UserBuff[3] >> 16) & 0x00ff);
	ANO_SendBuff[cout++] = ((ANO_UserBuff[3] >> 8) & 0x0000ff);
	ANO_SendBuff[cout++] = ANO_UserBuff[3];

	ANO_SendBuff[4] = cout - 5;
	for (i = 0; i < cout; i++)
	{
		data_sum += ANO_SendBuff[i];
	}
	ANO_SendBuff[cout++] = data_sum;

#if USE_USART_DMA
	HAL_UART_Transmit_DMA(Send_Uart, ANO_SendBuff, cout);
#else
	for (i = 0; i < cout; i++)
	{
		while ((Send_Uart->Instance->SR & 0X40) == 0);

		Send_Uart->Instance->DR = ANO_SendBuff[i];
	}
#endif
}


/**
 * @brief      �������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ
 * @param[in]  target Ŀ�굥��������
 * @param[in]  buf ��д������
 * @param[in]  beg ָ��������ڼ���Ԫ�ؿ�ʼд��
 * @retval     None
 */
void SerialDebug_classdef::Mini_Float2Byte(float *target, unsigned char *buf, unsigned char beg)
{
    unsigned char *point;
	point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}


/**
 * @brief      ��������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
 * @param[in]  Data Data��ͨ������
 * @param[in]  Channel ѡ��ͨ��(1-10)
 * @retval     None
 */
void SerialDebug_classdef::Mini_addData(float Data, unsigned char Channel)
{
    if ((Channel > 10) || (Channel == 0)) return;  //ͨ����������10�����0��ֱ����������ִ�к���
	else
	{
		switch (Channel)
		{
		case 1:  Mini_Float2Byte(&Data, Mini_Buffer, 1); break;
		case 2:  Mini_Float2Byte(&Data, Mini_Buffer, 5); break;
		case 3:  Mini_Float2Byte(&Data, Mini_Buffer, 9); break;
		case 4:  Mini_Float2Byte(&Data, Mini_Buffer, 13); break;
		case 5:  Mini_Float2Byte(&Data, Mini_Buffer, 17); break;
		case 6:  Mini_Float2Byte(&Data, Mini_Buffer, 21); break;
		case 7:  Mini_Float2Byte(&Data, Mini_Buffer, 25); break;
		case 8:  Mini_Float2Byte(&Data, Mini_Buffer, 29); break;
		case 9:  Mini_Float2Byte(&Data, Mini_Buffer, 33); break;
		case 10: Mini_Float2Byte(&Data, Mini_Buffer, 37); break;
		}
	}
}


/**
 * @brief      ���� DataScopeV1.0 ����ȷʶ���֡��ʽ
 * @param[in]  Channel_Number ��Ҫ���͵�ͨ������
 * @retval     None
 */
unsigned char SerialDebug_classdef::Mini_DataGenerate(unsigned char Channel_Number)
{
    if ((Channel_Number > 10) || (Channel_Number == 0)) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
	else
	{
		Mini_Buffer[0] = '$';  //֡ͷ

		switch (Channel_Number)
		{
		case 1:   Mini_Buffer[5] = 5; return  6;
		case 2:   Mini_Buffer[9] = 9; return 10;
		case 3:   Mini_Buffer[13] = 13; return 14;
		case 4:   Mini_Buffer[17] = 17; return 18;
		case 5:   Mini_Buffer[21] = 21; return 22;
		case 6:   Mini_Buffer[25] = 25; return 26;
		case 7:   Mini_Buffer[29] = 29; return 30;
		case 8:   Mini_Buffer[33] = 33; return 34;
		case 9:   Mini_Buffer[37] = 37; return 38;
		case 10:  Mini_Buffer[41] = 41; return 42;
		}
	}
	return 0;
}

void SerialDebug_classdef::Mini_Datashow(int ChannelAmount)
{
    int Send_Count = Mini_DataGenerate(ChannelAmount);

    // for (int i = 0; i < Send_Count; i++)
	// {
	// 	while ((USART6->SR & 0X40) == 0);

	// 	huart6.Instance->DR = Mini_Buffer[i];
	// }

	/*ѭ��Send_Count�ν�DataScope_OutPut_Buffer[i]�е����ݷ��ͳ�ȥ��
	�������һ�ε�ǰ֡�������ϴ���*/
#if USE_USART_DMA
	HAL_UART_Transmit_DMA(Send_Uart, Mini_Buffer, Send_Count);
#else
	HAL_UART_Transmit(Send_Uart, Mini_Buffer, Send_Count, 0xFF);
#endif

}
