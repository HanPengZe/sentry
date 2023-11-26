#ifndef _CONTROL_GIMBAL_H_
#define _CONTROL_GIMBAL_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"
#include "Robot_Config.h"

/* Exported types ------------------------------------------------------------*/
#define GM6020_MAX_OUT		29999
#define ENCODER_ANGLE_RATIO 22.7527f 

#if ROBOT_ID == INFANTRY_2022_SWERVE_1

#define GIMBAL_YAW_CENTRE	4775    	// Yaw 中心点
#define GIMBAL_YAW_RECENTRE 670			// Yaw 反向中心点
// 顶部与底部的相应数值根据电机的安装方式而定-------------*/
#define GIMBAL_PIT_MIN		-100   // Pit 编码仰角限位
#define GIMBAL_PIT_MAX		1060   // Pit 编码俯角限位
#define GIMBAL_PIT_CENTRE 	675

#elif ROBOT_ID == INFANTRY_2022_SWERVE_2

#define GIMBAL_YAW_CENTRE	700    	// Yaw 中心点
#define GIMBAL_YAW_RECENTRE 4800	// Yaw 反向中心点
// 顶部与底部的相应数值根据电机的安装方式而定-------------*/
#define GIMBAL_PIT_MIN		1910	 // Pit 编码仰角限位 
#define GIMBAL_PIT_MAX		3120  	 // Pit 编码俯角限位
#define GIMBAL_PIT_CENTRE 	2660

#elif ROBOT_ID == SRNTRY_2023_OMNI

#define GIMBAL_YAW_CENTRE	6788    	// Yaw 中心点
#define GIMBAL_YAW_RECENTRE 2692	// Yaw 反向中心点
//3,438 7,534
// 顶部与底部的相应数值根据电机的安装方式而定-------------*/
#define GIMBAL_PIT_MIN		3800	 // Pit 编码仰角限位 
#define GIMBAL_PIT_MAX		5100  	 // Pit 编码俯角限位
#define GIMBAL_PIT_CENTRE 	4450

#endif
/* --- 云台控制模式 -----------------------------------------------------------*/
enum Gimbal_CtrlMode_e
{
	Gimbal_DisableMode,	// 失能
	Gimbal_NormalMode,	// 普通运转模式
	Gimbal_SupplyMode,  // 补给模式
	Gimbal_LockMode,    // 锁住模式
	Gimbal_PatrolMode,      // 巡逻
	Gimbal_PCFollowMode,       // PC控制(跟随)
	Gimbal_PCMode       // PC控制(自瞄)
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
enum Gimbal_type_e
{
	Yaw,
	Pit,
	Rol
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


typedef struct
{
	float Angle[3];       /*<! IMU 角度 */
    float Pre_Angle[3];   /*<! 上一次角度 */
    float TotalAngle[3];  /*<! IMU 总角度*/
    float Gyro[3];        /*<! IMU 角速度 */
	float OffsetGyro[3];  /*<! 解算后的角速度 */
    int32_t Cnt[3]; 	  /*<! 累计圈数 */
}IMU_Data_t;



class Gimbal_classdef
{
private:
	// float YawRCCtrl_Coe;
public:
	Gimbal_classdef();
	Gimbal_CtrlMode_e Mode; /*<! 云台运作模式 */
	GimbalMotor_Ctrl_e YawMode; /*<! Yaw 控制模式 */
	GimbalMotor_Ctrl_e PitMode; /*<! Pit 控制模式 */
	// GimbalMotor_Ctrl_e Pre_YawMode; /*<! Yaw 上一次控制模式 */
	// GimbalMotor_Ctrl_e Pre_PitMode; /*<! Yaw 上一次控制模式 */
	
	Motor_M3508 Send_Motor[4] = {Motor_M3508(5), Motor_M3508(6), Motor_M3508(7), Motor_M3508(8)};
	Motor_GM6020 Motor[2] = {Motor_GM6020(1), Motor_GM6020(2)};

	PositionPID YawPID[6][2];  /*<! 云台电机各模式PID */
	PositionPID PitPID[6][2];  /*<! 云台电机各模式PID */

	Gimbal_TarData_t Target;

	IMU_Data_t CIMU;  /*<! C板陀螺仪 */

	uint8_t init_mode = true;
	uint16_t init_cnt;
	
	int Enemy_VanishTime;

	float YawRCCtrl_Coe;

	float Target_Yaw;
	float Target_Pit;
	float TargetAngle[2];

	float Windmill_Offset[2];

	// float CIMU_Angle[3];
	// float CIMU_Speed[3];

	void Control();

	void Pit_AngleLimit(); 		/*<! Pit电机角度限制 */
	void CHAS_SeparateLimit(bool Switch, float *yawparam);	/*<! 云台与底盘分离限制 */
	void Set_InitAngle();	   /*<! 设置初始角度 */
	void Set_YawCtrlMode(GimbalMotor_Ctrl_e mode); /*<! 设置Yaw轴控制模式 */
	void Set_PitCtrlMode(GimbalMotor_Ctrl_e mode); /*<! 设置Pit轴控制模式 */
	void TargetAngle_Update(float Yawparam, float Pitparam);
	void Motor_PIDCalc();

	void IMU_Update(float *angle, float *gyro); /*<! IMU数据更新 */

	float Get_YawDeviateC(float cur_angle);      /*<! 获取Yaw电机偏离中点角度 */
	float Get_YawDeviateReC(float cur_angle);    /*<! 获取Yaw电机偏离反向中点角度 */
	float Get_PitDeviateL();  /*<! 获取Pit电机偏离最小限幅值角度 */
	float Get_PitDeviateH();  /*<! 获取Pit电机偏离最大限幅值角度 */

	float Get_IMUAngle(Gimbal_type_e type);        /*<! 获取IMU实际角度 */
    float Get_IMUTotalAngle(Gimbal_type_e type);   /*<! 获取IMU累计角度 */
    float Get_IMUGyro(Gimbal_type_e type);         /*<! 获取IMU角速度 */		

	float Get_CurrentAngle(Gimbal_type_e type); /*<! 获取用于计算的测量角度 */
	float *Get_TargetAngle(Gimbal_type_e type); /*<! 获取用于计算的目标参数 */

};


#endif
