#ifndef _CONTROL_ROBOT_H_
#define _CONTROL_ROBOT_H_

#include "DR16.h"
#include "Control_Chassis.h"
#include "Control_Gimbal.h"
#include "Control_Shoot.h"
#include "Control_Vision.h"

//�����˹���״̬
enum RobotState_e
{
	Robot_Initializing, //��ʧ�ܽ��뵽����״̬�Ĺ��ɽ׶Σ�����ʱ���ʼ���Ľ׶Σ��������豸���߼��
	Robot_Disability,	//ʧ��״̬��Ӧ�Խ�������Ϳ�������
	Robot_Activating	//��������״̬
};



class Robot_classdef
{
public:
    RobotState_e State;
    DR16Status_Typedef Ctrl_Source;
    CHAS_CtrlMode_e Chassis_Mode;
    CHAS_CtrlMode_e Chassis_PreMode;
    Gimbal_CtrlMode_e Gimbal_Mode;
    Attack_CtrlMode_e Attack_Mode;
    Fric_CtrlMode_e Fric_Mode;
    VisionMode_e Vision_Mode;

    uint8_t ID;          /*<! ������ID */
    uint8_t PC_State;    /*<! PC״̬ */
    uint8_t SendData[8]; /*<! ���ͨ�Ŵ洢���� */
    uint8_t Write_Msg[2][8];
    uint8_t Reflash_UI;  /*<! UIˢ�� */
    uint8_t ResetFlag = false;   /*<! ������־λ */
    uint8_t Chas_Warning;   /*<! ����״̬���� */
    uint16_t FPS;
    WorldTime_t TIME;

		uint8_t DetectColor;
		uint8_t God_State[5];
		uint8_t CanShootNum[2];
		uint8_t HurtArmor_NoZero;
		uint8_t ShootRecNum,lastShootRecNum,mul_num;
		uint16_t TrueShootRecNum;
		uint8_t CoolBuff;
		uint8_t Enemy_Sentry_God_State;
		
    Robot_classdef();
    void Enable();
    void Disable();
    void Reset();
    void Control();
    void CtrlSource_Switch(DR16Status_Typedef mode);
    void Set_ChassisMode(CHAS_CtrlMode_e mode);
    void Set_GimbalMode(Gimbal_CtrlMode_e mode);
    void Set_AttackMode(Attack_CtrlMode_e mode);
    void Set_VisionMode(VisionMode_e mode);

    void Gimbal_Ctrl();

    void SendMsgtoCHAS();
    void RefereeMsg_Write(uint16_t id, uint8_t can_rx_data[]);

    uint8_t Get_ID();
		uint8_t Get_DetectColor();
    uint16_t Get_FPS();

    RobotState_e Get_State();
    DR16Status_Typedef Get_CtrlSource();
    CHAS_CtrlMode_e Get_ChassisMode();
    CHAS_CtrlMode_e Get_ChassisPreMode();
    Gimbal_CtrlMode_e Get_GimbalMode();
    Attack_CtrlMode_e Get_AttackMode();
    Fric_CtrlMode_e Get_FricMode();
    VisionMode_e Get_VisionMode();
    
};


#endif
