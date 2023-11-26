#ifndef SUPERCAPACITOR_H
#define SUPERCAPACITOR_H

#pragma anon_unions

#include <stdint.h>
#include "BSP_CAN.h"
#include <stdbool.h>


#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

#define SupCap_ON 1
#define SupCap_OFF 0
#define Charging_ON 1
#define Charging_OFF 0
#define Power_Supply 1
#define Power_NotSupply 0

typedef union
{
	uint8_t data[8];
	struct
	{
		float chassis_power;  /* ���̹��ʣ���λ��W */
		uint8_t chassis_buff; /* ���̹��ʻ��� */
		uint8_t Is_enable;    /* ���ݿ��Խ������ */
		uint8_t cap_cell;     /* ����ʣ�����������ָ��� */
	};
} SCCM_RecvData_u;

typedef union
{
	uint8_t data[8];
	struct
	{
		float charge_power;    /* ��繦�ʣ���λ��W ,��Χ 0-80W */
		uint8_t charge_enable; /* ���ʹ�� */
		uint8_t is_cap_output; /* ʹ�õ��ݹ��� */
	};
} SCCM_SendData_u;



class SupCap_classdef
{
private:

public:
	SupCap_classdef();
	SCCM_RecvData_u RecvData;
	SCCM_SendData_u SendData;

	uint8_t Data[8];

	uint8_t State;
	uint16_t ChassisPower_Limit;
	float Charging_Power;      // ���Ĺ���

	void Control();
	void Process();
	void Update(uint8_t can_rx_data[]);
	void Send_Msg(CAN_HandleTypeDef *hcan);
	void Main_Switch(bool Ctrl, bool Charge, bool Supply);

    void ChargeControl(float Charging_Power);
    void ChargeSwitch(bool Switch);
    void SupplySwitch(bool Switch);

	uint8_t Get_Cell();
	bool Is_CanbeUse();
	bool Is_Output();


};


#endif
