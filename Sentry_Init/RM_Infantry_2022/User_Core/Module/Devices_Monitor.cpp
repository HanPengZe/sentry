/**
 ------------------------------------------------------------------------------
 * @file    Devices_Monitor.cpp
 * @author  Shake
 * @brief   �豸״̬���
 * @version V0.1
 * @date    2021-10-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

/* Includes ------------------------------------------------------------------*/
#include "Devices_Monitor.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private macros ------------------------------------------------------------*/
uint16_t DevicesMonitor_classdef::Critical_Val[] = CRITICAL_VAL_INIT;
#undef CRITICAL_VAL_INIT

uint32_t DevicesMonitor_classdef::Devices_Frame = 0xFFFFFFFF;

/**
 * @brief      �豸֡�ʼ�⺯��
 * @param[in]  None
 * @retval     None
 */
void DevicesMonitor_classdef::Devices_Detec()
{
    for(uint8_t i = 0; i < FrameCount_NUM; i++)
    {
        if(Frame_Detec(FrameCounter[i],Critical_Val[i]))
        {
            Devices_Frame &= ~(uint32_t)(1<<i);
        }
        else
        {
            Devices_Frame |= (uint32_t)(1<<i);
        }
        FrameCounter[i] = 0;
    }
}


/**
 * @brief      �豸֡�ʸ���
 * @param[in]  device
 * @retval     None
 */
void DevicesMonitor_classdef::Update(FrameType_e device)
{
    FrameCounter[device]++;
}



/**
 * @brief      ����豸֡���Ƿ���������Ҫ��
 * @param[in]  counter  ����ֵ
 * @param[in]  critical �ٽ�ֵ
 * @retval     0:����  1:��֡
 */
uint8_t DevicesMonitor_classdef::Frame_Detec(uint16_t counter, uint16_t critical)
{
    return counter>=critical? 0:1;
}

/**
 * @brief      ��ȡ�豸����״̬
 * @param[in]  device
 * @retval     state  0:����  1:�쳣/����
 */
uint8_t DevicesMonitor_classdef::Get_State(uint32_t device)
{
    if(device>1<<FrameCount_NUM-2) //--- ���ָ���豸����������豸
    {
        return ((Devices_Frame&device)==device?0:1);
    }
    else    //--- ���ָ���豸
    {
        return !(Devices_Frame&device);
    }
}

/**
 * @brief      ��ȡ�豸֡��
 * @param[in]  time
 * @param[in]  FPS
 * @retval     None
 */
void DevicesMonitor_classdef::Get_FPS(WorldTime_t *time, uint16_t *FPS)
{
    time->Now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	*FPS = FPS_Calc(time->Now - time->Pre);
	time->Pre = time->Now;
}
