#ifndef _CONTROL_GIMBAL_H_
#define _CONTROL_GIMBAL_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"
/* Exported types ------------------------------------------------------------*/
#define GM6020_MAX_OUT		29999
#define ENCODER_ANGLE_RATIO 22.7527f 
#define GIMBAL_YAW_CENTRE	2019    	// Yaw 中心点
#define GIMBAL_YAW_RECENTRE 6115	// Yaw 反向中心点


// 顶部与底部的相应数值根据电机的安装方式而定-------------*/
#define GIMBAL_PIT_MIN		3800	 // Pit 编码仰角限位 
#define GIMBAL_PIT_MAX		5100  	 // Pit 编码俯角限位

/* --- 云台控制模式 -----------------------------------------------------------*/
enum Gimbal_CtrlMode_e
{
	Gimbal_DisableMode,	// 失能
    Gimbal_NormalMode,	// 普通运转模式
	Gimbal_SupplyMode,  // 补给模式
    Gimbal_LockMode,    // 锁住模式
};

/* --- 云台电机控制模式---------------------------------------------------------*/
enum GimbalMotor_Ctrl_e
{
	Mech,	 // 机械角度
	A_IMU,   // A板陀螺仪
	C_IMU,	 // C板陀螺仪
	V_Mech,	 // 视觉PID
	V_AIMU,  // 视觉PID
	V_CIMU   // 视觉PID
};

/* --- 云台电机 ID ------------------------------------------------------------*/
enum GimbalMotor_type_e
{
	Yaw,
	Pit
};

/* --- 云台目标值数据 -----------------------------------------------------------*/
typedef struct
{
	float Mech_Yaw;
	float Mech_Pit;
	float AIMU_Yaw;
	float AIMU_Pit;
	float CIMU_Yaw;
	float CIMU_Pit;
}Gimbal_TarData_t;

/* --- 云台当前值数据 -----------------------------------------------------------*/
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

	PositionPID Gimbal_YawPID[6][2];  /*<! 云台电机各模式PID */
	PositionPID Gimbal_PitPID[6][2];  /*<! 云台电机各模式PID */

	Gimbal_TarData_t Target;

	float YawRCCtrl_Coe;

	float Target_Yaw;
	float Target_Pit;
	float TargetAngle[2];

	void Control();
	void Process();

	void Pit_AngleLimit(); 		/*<! Pit电机角度限制 */
	void CHAS_SeparateLimit(bool Switch, float *yawparam);					   /*<! 云台与底盘分离限制 */
	void Set_InitAngle();	   /*<! 设置初始角度 */
	void Set_YawCtrlMode(GimbalMotor_Ctrl_e mode); /*<! 设置Yaw轴控制模式 */
	void Set_PitCtrlMode(GimbalMotor_Ctrl_e mode); /*<! 设置Pit轴控制模式 */
	void TargetAngle_Update(float Yawparam, float Pitparam);
	void Motor_PIDCalc();

	float Get_YawDeviateC(float cur_angle);      /*<! 获取Yaw电机偏离中点角度 */
	float Get_YawDeviateReC(float cur_angle);      /*<! 获取Yaw电机偏离反向中点角度 */
	float Get_PitDeviateL();  /*<! 获取Pit电机偏离最小限幅值角度 */
	float Get_PitDeviateH();  /*<! 获取Pit电机偏离最大限幅值角度 */
	float *Get_TargetAngle(GimbalMotor_type_e type); /*<! 获取用于计算的目标参数 */

};


#endif
