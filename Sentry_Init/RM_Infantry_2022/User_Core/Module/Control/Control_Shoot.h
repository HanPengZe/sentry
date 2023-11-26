#ifndef _CONTROL_SHOOT_H_
#define _CONTROL_SHOOT_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"



enum FricMotor_ID_e
{
    //����ǹ��Ϊ׼
    Fric_L,
    Fric_R
};

enum Attack_CtrlMode_e
{
    Attack_Disable,		//ʧ�ܹ���
	Attack_15,	        //�ֶ���׼ - ����
	Attack_18,	        //�ֶ���׼ - ����
	Attack_22,	        //�ֶ���׼ - �и���
	Attack_30,	        //�ֶ���׼ - ����
	Attack_BigWindmill, //���ģʽ
	Attack_Auto,		//�Զ����
	Attack_Zero,
	Attack_Unload		//�˵�
	
};

//Ħ����ģʽ
enum Fric_CtrlMode_e
{
	Fric_Disable, // �ر�Ħ����
	Fric_Zero, // �ر�Ħ��
	Fric_Unload,
	Fric_15m_s,	  // 15m/s
	Fric_18m_s,	  // 18m/s
	Fric_22m_s,   // 22m/s
	Fric_30m_s    // 30m/s
};

/*���̲���*/
typedef struct
{
	uint8_t Began;		    // ��ʼ����
	uint8_t Began2;		    // ��ʼ����
	uint8_t Stuck_Flag;		// ������־λ
	int8_t Motor_Direction;	// ���ת������  1 -> ��  | -1 -> ��
	int16_t Stuck_Duration;	// ����������ʱ��
    uint16_t Num;           // ����װ����
    uint16_t CanShoot_Num;  // �������/
	uint16_t CriticalVal;	// ���̵Ľ����ٶ�ֵ->�����ٽ�ֵ
	int32_t Init_Angle;	// ��ʼλ��
	int32_t Launched_Num;	// �Ѿ�������ӵ���
	int32_t Target_Angle;   // ��������Ŀ��ֵ
	int32_t Re_TargetAngle; // ���������תĿ��ֵ

} Reload_t;

class Shoot_classdef
{
private:
		
    Reload_t Reload;          /*<! ���̲��� */
	
	int16_t Fric_Speed;       /*<! Ħ����ת�� */
	uint16_t Fric_Output;	  /*<! Ħ������� */

	int32_t Card_Target_Angle;

    void ReloadPID_Calc(float tar_angle); /*<! ���̵��PID���� */
    void BarrelPID_Calc(float tar_angle); /*<! ���̵��PID���� */
    void FricPID_Calc(float tar_speed);   /*<! Ħ���ֵ��PID���� */
		void CardPID_Calc(float tar_angle);   /*<! �������PID���� */

public:
		//ǹ�ܡ�Pit������Ħ����
		Motor_M3508 Send_Motor[4] = {Motor_M3508(5), Motor_M3508(6),Motor_M3508(7),Motor_M3508(8)};  /*<! ��һ�����ڻ�ǹ�ܣ��ڶ���������̨Pit����������Ħ���ֵ�� */
    Motor_M3508 Fric_Motor[2] = {Motor_M3508(7), Motor_M3508(8)};   /*<! Ħ���ֵ�� */
    Motor_M2006 Reload_Motor = Motor_M2006(6);                      /*<! ���̵�� */
    Motor_M2006 Barrel_Motor = Motor_M2006(5);                      /*<! ǹ�ܵ�� */
		Motor_M2006 Card_Motor = Motor_M2006(1);                        /*<! ������� */

    PositionPID Reload_PID[2];   /*<! ���̵�� λ��ʽPID */
		PositionPID Barrel_PID[2];	 /*<! ǹ�ܵ�� λ��ʽPID */
		PositionPID Card_PID[2];		 /*<! ������� λ��ʽPID */
    IncrementPID Fric_PID[2];     /*<! Ħ���� ����ʽPID */

    bool Mag_Switch;          /*<! ���ֿ��ر�־λ */

    uint8_t Onebullet_heat = 10; /*<! �����ӵ�������(С����) */

    uint8_t Rage_Mode;        /*<! ��ģʽ,������������ */
    uint8_t ContLaunch;       /*<! ������־λ */
		
		uint8_t ShootID;
		uint8_t Turn_Bar_ID;

    uint8_t PowerState;
    uint16_t SpeedLimit;
    uint16_t CoolingRate;
    uint16_t CoolingLimit;
    uint16_t GunHeat;

    uint16_t Mshoot_Cnt,Ashoot_Cnt; /* <! ������ʱ */
    uint16_t Shoot_Freq=10;        /*<! ��Ƶ */

    float BulletSpeed;      /*<! ����ϵͳ�ӵ��ٶ� */
    float Fric_SpeedOffset[4] = {0}; /*<! Ħ�����ٶȲ�������ֵ */
    float Fric_TempOffset[4] = {0};  /*<! Ħ�����¶Ȳ�������ֵ */
		
		uint16_t CanShoot_Num[2]={24,24};//�ڱ�240
		uint32_t ShootStartTiming[2],ShootOverTiming[2],ShootTime[2];
		uint32_t turnShoot_startTiming,turnShoot_overTiming,turnShoot_Time;
		uint32_t ShootNumTurnTiming,ShootNumTurnTime1,ShootNumTurnTime2;
		

    Shoot_classdef();

    void Control();          /*<! ������� */
    void PWM_Init();         /*<! PWM ��ʼ�� */
    void Devices_Init();     /*<! ģ���ʼ�� */
    void Fric_Process();     /*<! Ħ���ִ��� */
		uint8_t ShootFreq_Calc();
    void Reload_Calc();      /*<! ���̼��� */
    void Barrel_Calc();      /*<! ǹ�ܼ��� */
    void Reload_Reset();     /*<! �������� */
    void Set_ReloadNum(uint16_t num);   /*<! ���÷��䵯������ */
    void Mag_Ctrl(uint8_t state);       /*<! ���ֿ��� */
    void Laser_Ctrl(bool state);/*<! ������� */
    void Set_FricOut();         /*<! ����Ħ������� */

    void FricSpeed_Adapt(); /*<! Ħ����ת�ٵ��� */

    void AttackMode_Ctrl();    /*<! ����ģʽ���� */

    bool Judge_ReloadStall(); /*<! �ж��Ƿ��ת */

    void ContShoot();
    
    bool Get_ShooterPower();     /*<! ��ȡ����ģ���Դ״̬ */
    uint16_t Get_CoolingLimit(); /*<! ��ȡǹ����������ֵ */
    uint16_t Get_CoolingRate();  /*<! ��ȡǹ����ȴ���� */
    uint16_t Get_SpeedLimit();   /*<! ��ȡǹ���������� */
    uint16_t Get_GunHeat();      /*<! ��ȡǹ�ܵ�ǰ���� */
    float Get_GunSpeed();        /*<! ��ȡ��ǰ���� */


};

extern uint8_t biubiubiu;
extern uint8_t biubiubiu_flag;

#endif
