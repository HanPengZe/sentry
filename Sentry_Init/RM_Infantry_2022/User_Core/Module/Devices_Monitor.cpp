/**
 ------------------------------------------------------------------------------
 * @file    Devices_Monitor.cpp
 * @author  Shake
 * @brief   设备状态检测
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
 * @brief      设备帧率检测函数
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
 * @brief      设备帧率更新
 * @param[in]  device
 * @retval     None
 */
void DevicesMonitor_classdef::Update(FrameType_e device)
{
    FrameCounter[device]++;
}



/**
 * @brief      检测设备帧率是否满足在线要求
 * @param[in]  counter  计数值
 * @param[in]  critical 临界值
 * @retval     0:正常  1:掉帧
 */
uint8_t DevicesMonitor_classdef::Frame_Detec(uint16_t counter, uint16_t critical)
{
    return counter>=critical? 0:1;
}

/**
 * @brief      获取设备在线状态
 * @param[in]  device
 * @retval     state  0:正常  1:异常/掉线
 */
uint8_t DevicesMonitor_classdef::Get_State(uint32_t device)
{
    if(device>1<<FrameCount_NUM-2) //--- 检测指定设备以外的其他设备
    {
        return ((Devices_Frame&device)==device?0:1);
    }
    else    //--- 检测指定设备
    {
        return !(Devices_Frame&device);
    }
}

/**
 * @brief      获取设备帧率
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
