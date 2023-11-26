#ifndef _CONTROL_GIMBAL_H_
#define _CONTROL_GIMBAL_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"
/* Exported types ------------------------------------------------------------*/
#define GM6020_MAX_OUT		29999
#define ENCODER_ANGLE_RATIO 22.7527f 
#define GIMBAL_YAW_CENTRE	2019    	// Yaw ���ĵ�
#define GIMBAL_YAW_RECENTRE 6115	// Yaw �������ĵ�


// ������ײ�����Ӧ��ֵ���ݵ���İ�װ��ʽ����-------------*/
#define GIMBAL_PIT_MIN		3800	 // Pit ����������λ 
#define GIMBAL_PIT_MAX		5100  	 // Pit ���븩����λ

/* --- ��̨����ģʽ -----------------------------------------------------------*/
enum Gimbal_CtrlMode_e
{
	Gimbal_DisableMode,	// ʧ��
    Gimbal_NormalMode,	// ��ͨ��תģʽ
	Gimbal_SupplyMode,  // ����ģʽ
    Gimbal_LockMode,    // ��סģʽ
};

/* --- ��̨�������ģʽ---------------------------------------------------------*/
enum GimbalMotor_Ctrl_e
{
	Mech,	 // ��е�Ƕ�
	A_IMU,   // A��������
	C_IMU,	 // C��������
	V_Mech,	 // �Ӿ�PID
	V_AIMU,  // �Ӿ�PID
	V_CIMU   // �Ӿ�PID
};

/* --- ��̨��� ID ------------------------------------------------------------*/
enum GimbalMotor_type_e
{
	Yaw,
	Pit
};

/* --- ��̨Ŀ��ֵ���� -----------------------------------------------------------*/
typedef struct
{
	float Mech_Yaw;
	float Mech_Pit;
	float AIMU_Yaw;
	float AIMU_Pit;
	float CIMU_Yaw;
	float CIMU_Pit;
}Gimbal_TarData_t;

/* --- ��̨��ǰֵ���� -----------------------------------------------------------*/
enum Gimbal_CurData_e
{
	Yaw_MechAngle,
	Yaw_MechSpeed,
	Yaw_AIMUAngle,
	Yaw_AIMUSpeed,
	Yaw_CIMUAngle,
	Yaw_CIMUSpeed,

	Pit_MechAngle,
	Pit_MechSpeed,
	Pit_AIMUAngle,
	Pit_AIMUSpeed,
	Pit_CIMUAngle,
	Pit_CIMUSpeed,
};



class Gimbal_classdef
{
private:
	// float YawRCCtrl_Coe;
public:
	Gimbal_classdef();
	Gimbal_CtrlMode_e Mode;
	GimbalMotor_Ctrl_e YawMode;
	GimbalMotor_Ctrl_e PitMode;
	Motor_GM6020 Motor[2] = {Motor_GM6020(5), Motor_GM6020(2)};

	PositionPID Gimbal_YawPID[6][2];  /*<! ��̨�����ģʽPID */
	PositionPID Gimbal_PitPID[6][2];  /*<! ��̨�����ģʽPID */

	Gimbal_TarData_t Target;

	float YawRCCtrl_Coe;

	float Target_Yaw;
	float Target_Pit;
	float TargetAngle[2];

	void Control();
	void Process();

	void Pit_AngleLimit(); 		/*<! Pit����Ƕ����� */
	void CHAS_SeparateLimit(bool Switch, float *yawparam);					   /*<! ��̨����̷������� */
	void Set_InitAngle();	   /*<! ���ó�ʼ�Ƕ� */
	void Set_YawCtrlMode(GimbalMotor_Ctrl_e mode); /*<! ����Yaw�����ģʽ */
	void Set_PitCtrlMode(GimbalMotor_Ctrl_e mode); /*<! ����Pit�����ģʽ */
	void TargetAngle_Update(float Yawparam, float Pitparam);
	void Motor_PIDCalc();

	float Get_YawDeviateC(float cur_angle);      /*<! ��ȡYaw���ƫ���е�Ƕ� */
	float Get_YawDeviateReC(float cur_angle);      /*<! ��ȡYaw���ƫ�뷴���е�Ƕ� */
	float Get_PitDeviateL();  /*<! ��ȡPit���ƫ����С�޷�ֵ�Ƕ� */
	float Get_PitDeviateH();  /*<! ��ȡPit���ƫ������޷�ֵ�Ƕ� */
	float *Get_TargetAngle(GimbalMotor_type_e type); /*<! ��ȡ���ڼ����Ŀ����� */

};


#endif
