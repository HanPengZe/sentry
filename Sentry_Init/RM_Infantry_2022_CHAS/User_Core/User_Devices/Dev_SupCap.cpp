/**
  ******************************************************************************
  * @file     Supercapacitor.c
  * @author   Shake
  * @version  V2.0
  * @brief    超级电容模块
  ******************************************************************************
  */

#include "Dev_SupCap.h"
#include "System_DataPool.h"
#include "Power_Meter.h"



SupCap_classdef::SupCap_classdef()
{
	
}

/**
  * @brief  底盘超级电容控制函数
  * @param	None
  * @retval None
  */
void SupCap_classdef::Control()
{
	Process();

	Send_Msg(&hcan1);
}


uint8_t charge_flag; //--- 超电用完后充电标志位
void SupCap_classdef::Process()
{
	// ChassisPower_Limit = 0;

	//--- 底盘失能或电容离线
	if (Infantry.Get_ChassisMode() == CHAS_DisableMode || DevicesMonitor.Get_State(SUPCAP_MONITOR) == Off_line)
	{
		State = SupCap_OFF;
		Main_Switch(SupCap_OFF, Charging_OFF, Power_NotSupply);
		return;
	}
	else
	{
		State = SupCap_ON;
		SendData.charge_enable = SupCap_ON;
	}

	//

	if (Chassis.Get_PowerLimit() < 45)
    {
        ChassisPower_Limit = 45;
    }
    else if (Chassis.Get_PowerLimit() > 120)
    {
        ChassisPower_Limit = 120;
    }
    else
    {
        ChassisPower_Limit = Chassis.Get_PowerLimit();
    }

	//---充能功率——剩余功率
	if(Is_Output() == true && Get_Cell() > 40)
	{
		//--- 边放边充，放电的时候满功率充电
		Charging_Power = ChassisPower_Limit;
	}
	else
	{
		if(DevicesMonitor.Get_State(POWERMETER_MONITOR) == Off_line)
		{
			//--- 功率计掉线则使用裁判系统的数据
			Charging_Power = ChassisPower_Limit - Chassis.Get_Power();
		}
		else
		{
			Charging_Power = ChassisPower_Limit - PowerMeter.Get_Power_Val();
		}
	}

	//充能功率限幅
	if (Charging_Power > ChassisPower_Limit + 5) // 底盘输出功率
	{
		Charging_Power = ChassisPower_Limit;
	}
	else if(Charging_Power < 0)
	{
		Charging_Power = 0.0f;
	}

	//缓存功率限制充能功率
	if(Chassis.Get_PowerBuffer() < 60 && Chassis.Get_PowerBuffer() > 55)
	{
		Charging_Power -= 5;
	}
	else if(Chassis.Get_PowerBuffer() <= 55)
	{
		Charging_Power = 0.0f;
		// SendData.is_cap_output = false;
	}
	ChargeControl(Charging_Power);

	//--- 用完后等充到50再允许再次使用
	if(charge_flag == true && Get_Cell() > 50)
	{
		charge_flag = false;
	}
	
	if(Infantry.Write_Msg[Cap_Ctrl] == true && Get_Cell() > 40 && Is_CanbeUse() != false)
	{
		if(charge_flag == false)
		{
			SupplySwitch(Power_Supply); //--- 放电
		}
	}
	else
	{
		if(Get_Cell() <= 40)
		{
			charge_flag = true;
		}
		SupplySwitch(Power_NotSupply);
	}

}

/**
  * @brief  电容状态开关控制
  * @param	Ctrl Charge Supply
  * @retval None
  */
void SupCap_classdef::Main_Switch(bool Ctrl, bool Charge, bool Supply)
{
	State = Ctrl;
	SendData.charge_enable = Charge;
	SendData.is_cap_output = Supply;
}

/**
  * @brief  电容数据更新
  * @param	can_rx_data
  * @retval None
  */
void SupCap_classdef::Update(uint8_t can_rx_data[])
{
	memcpy(RecvData.data, can_rx_data, 8);
}

/**
  * @brief  电容数据发送
  * @param	None
  * @retval None
  */
void SupCap_classdef::Send_Msg(CAN_HandleTypeDef *hcan)
{
	uint8_t Data[8];
	memcpy(Data, SendData.data, 8);
	CANx_SendData(hcan, SCCM_SEND_ID, Data, 8);
}


/**
  * @brief  超级电容充电功率
  * @param	Charging_Power
  * @retval None
  */
void SupCap_classdef::ChargeControl(float Charging_Power)
{
	SendData.charge_power = Charging_Power;
}

/**
  * @brief  超级电容充电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_classdef::ChargeSwitch(bool Switch)
{
	SendData.charge_enable = Switch;
}

/**
  * @brief  超级电容放电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_classdef::SupplySwitch(bool Switch)
{
	SendData.is_cap_output = Switch;
}

/**
  * @brief  获取电容相关信息
  * @param	None
  * @retval cap data
  */
uint8_t SupCap_classdef::Get_Cell()
{
	return RecvData.cap_cell;
}

/**
  * @brief  获取能否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_classdef::Is_CanbeUse()
{
	return RecvData.Is_enable;
}

/**
  * @brief  获取是否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_classdef::Is_Output()
{
	return SendData.is_cap_output;
}

