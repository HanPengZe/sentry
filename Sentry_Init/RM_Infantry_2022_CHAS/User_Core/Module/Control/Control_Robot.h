#ifndef _CONTROL_ROBOT_H_
#define _CONTROL_ROBOT_H_


#include "Control_Chassis.h"
#include "Control_Gimbal.h"
#include "Devices_Monitor.h"

//�����˹���״̬
enum RobotState_e
{
	Robot_Initializing, //��ʧ�ܽ��뵽����״̬�Ĺ��ɽ׶Σ�����ʱ���ʼ���Ľ׶Σ��������豸���߼��
	Robot_Disability,	//ʧ��״̬��Ӧ�Խ�������Ϳ�������
	Robot_Activating	//��������״̬
};

/* ���ͨ���������� */
enum Commu_e
{
    Vision_State = 0,
    Fric_State = 1,
    Uphill_Mode = 2,
    Robot_Reset = 3,
    Cap_Ctrl = 4,
    Attack_Ctrl = 5,
    Mag_Ctrl = 6,
    UI_Reflash = 7
};

/* �Ӿ�ģʽ */
enum VisionMode_e
{
	Vision_Disable = 0,		// �Ӿ�ʧ��	
	Vision_Default = 1,		// Ĭ������(�Ӿ����治��Ԥ��)
	Vision_BIGWindmill = 2, // ��糵
	// Vision_Sentry = 3,	// �ڱ�
	// Vision_BASE = 4,		// ����
	// Vision_Top = 5,		// ����
	// Vision_Record = 6	// ¼��
	Vision_Forcast = 6		// �Ӿ�Ԥ��

};



class Robot_classdef
{
public:
    RobotState_e State;
    DR16Status_Typedef Ctrl_Source;
    CHAS_CtrlMode_e Chassis_Mode;
    CHAS_CtrlMode_e Chassis_PreMode;
    Gimbal_CtrlMode_e Gimbal_Mode;
    // Attack_CtrlMode_e Attack_Mode;
    // Fric_CtrlMode_e Fric_Mode;
    VisionMode_e Vision_Mode;

    uint8_t Vision_switch;
    uint8_t Vision_Depth;
    uint8_t Write_Msg[8];
    uint8_t RefereeShootData[8];
		uint8_t RefereePosData[8];
		uint8_t RefereeHpData[8];

    uint8_t Shoot_Freq;
    uint8_t Onebullet_heat = 10; /*<! �����ӵ�������(С����) */

    uint16_t FPS;
    WorldTime_t TIME;

    Robot_classdef();
    void Enable();
    void Disable();
    void Reset();
    void Control();
    void CtrlSource_Switch(DR16Status_Typedef mode);
    void Set_ChassisMode(CHAS_CtrlMode_e mode);
    void Set_GimbalMode(Gimbal_CtrlMode_e mode);
    // void Set_AttackMode(Attack_CtrlMode_e mode);

    void Gimbal_Ctrl();

    void RefereeMsg_Send_Shoot(uint8_t *data);
    void RefereeMsg_Send_Pos(uint8_t *data);
    void RefereeMsg_Send_HP(uint8_t *data);
    void WriteMsgFromGimbal(uint8_t can_rx_data[]);

    void ShootMsg_Process(float bullet_speed);

    uint8_t Get_ID();
    uint8_t Get_VisionDepth();
    uint8_t ShootFreq_Calc();    /**<! ��Ƶ���� */
    // uint16_t Get_GunHeat();      /**<! ǹ�ܵ�ǰ���� */
    // uint16_t Get_CoolingLimit(); /**<! ǹ����ȴ���� */
    // uint16_t Get_CoolingRate();  /**<! ǹ����ȴ���� */

    uint16_t Get_SentryGunHeat1();
    uint16_t Get_SentryGunHeat2();
    uint16_t Get_SentryMapX();
    uint16_t Get_SentryMapY();
    uint8_t Get_SentryKeyBoard();
    uint8_t Get_SentryEnemyColor();

    RobotState_e Get_State();
    DR16Status_Typedef Get_CtrlSource();
    CHAS_CtrlMode_e Get_ChassisMode();
    Gimbal_CtrlMode_e Get_GimbalMode();
    // Attack_CtrlMode_e Get_AttackMode();
    // Fric_CtrlMode_e Get_FricMode();
    VisionMode_e Get_VisionMode();
    
};

extern uint32_t Get_RefereeTime();

#endif
