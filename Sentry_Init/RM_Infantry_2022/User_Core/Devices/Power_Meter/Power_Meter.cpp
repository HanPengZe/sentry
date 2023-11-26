/**
 ------------------------------------------------------------------------------
 * @file    Power_Meter.cpp
 * @author  Shake
 * @brief   ¹¦ÂÊ¼Æ
 * @version V1.0
 * @date    2021-12
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "Power_Meter.h"

PowerMeter_t Power_Meter;

void PowerMeter_Update(uint8_t recv_data[])
{
    Power_Meter.Voltage = (float)(((recv_data[1]<<8)|recv_data[0]) / 100.0f);
    Power_Meter.Current = (float)(((recv_data[3]<<8)|recv_data[2]) / 100.0f);
    Power_Meter.Power = Power_Meter.Voltage * Power_Meter.Current;
}


