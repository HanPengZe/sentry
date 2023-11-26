/**
 ------------------------------------------------------------------------------
 * @file    Power_Meter.cpp
 * @author  Shake
 * @brief   功率计
 * @version V1.0
 * @date    2021-12
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

#include "Power_Meter.h"
#include <string.h>

// PowerMeter_t Power_Meter;

/**
  * @brief  功率计数据更新
  * @param	can_rx_data
  * @Note   这个是淘宝买的
  * @retval None
  */
// void PowerMeter_Update(uint8_t recv_data[])
// {
//     Power_Meter.Voltage = (float)(((recv_data[1]<<8)|recv_data[0]) / 100.0f);
//     Power_Meter.Current = (float)(((recv_data[3]<<8)|recv_data[2]) / 100.0f);
//     Power_Meter.Power = Power_Meter.Voltage * Power_Meter.Current;
// }

PowerMeter_classdef::PowerMeter_classdef()
{
  
}


/**
  * @brief  功率计数据更新
  * @param	can_rx_data
  * @retval None
  */
void PowerMeter_classdef::Update(uint8_t can_rx_data[])
{
#if INA226_Self
    memcpy(Self_RecvData.data, can_rx_data, 8);
#else
    RecvData.data[0] = can_rx_data[1];
    RecvData.data[1] = can_rx_data[0];
    RecvData.data[2] = can_rx_data[3];
    RecvData.data[3] = can_rx_data[2];
    RecvData.data[4] = can_rx_data[5];
    RecvData.data[5] = can_rx_data[4];
    RecvData.data[6] = can_rx_data[7];
    RecvData.data[7] = can_rx_data[6];
#endif

}

/**
  * @brief  获取底盘功率 W
  * @param	None
  * @retval cap data
  */
float PowerMeter_classdef::Get_Power_Val()
{
#if INA226_Self
  return Get_Shunt_Current()*Get_voltageVal();
#else
	return (float)RecvData.Pack.Power_Val/1000;
#endif
}
/**
  * @brief  获取底盘电压 V
  * @param	None
  * @retval cap data
  */
float PowerMeter_classdef::Get_voltageVal()
{
#if INA226_Self
  return (float)Self_RecvData.Pack.voltageVal/1000;
#else
	return (float)RecvData.Pack.voltageVal/1000;
#endif
}
/**
  * @brief  获取底盘电流 A
  * @param	None
  * @retval cap data
  */
float PowerMeter_classdef::Get_Shunt_Current()
{
#if INA226_Self
  return (float)Self_RecvData.Pack.Shunt_Current/1000;
#else
	return (float)RecvData.Pack.Shunt_Current/1000;
#endif
}
/**
  * @brief  获取底盘分流电压 mV
  * @param	None
  * @retval cap data
  */
float PowerMeter_classdef::Get_Shunt_voltage()
{
#if INA226_Self
  return NULL;
#else
  return (float)RecvData.Pack.Shunt_voltage/1000;
#endif	
}


