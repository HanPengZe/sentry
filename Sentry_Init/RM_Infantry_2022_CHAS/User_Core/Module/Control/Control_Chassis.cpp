/**
 ------------------------------------------------------------------------------
 * @file    Control_Chassis.cpp
 * @author  Shake
 * @brief   底盘控制
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

/* Includes ------------------------------------------------------------------*/
#include "Control_Chassis.h"
#include "System_DataPool.h"
#include "arm_math.h"
#include "Control_Robot.h"
#include <math.h>
#include "Robot_Config.h"
#include "Control_Gimbal.h"
#include "DJI_AIMU.h"
#include "kalman_Filter.h"
// #include "Chassis_Power.h"
// #include "Power_Limit.h"

CHAS_Power_classdef CHAS_Power;

/* Private define ------------------------------------------------------------*/
#define USE_M3508_OVERHEAT_PROTECT    1 //--- 是否启用3508电机过热保护

#define USE_TEST_POWERLIMIT           0

#define CHASSIS_MAX_SPEED 9000  // 底盘驱动轮最大速度
#define CHASSIS_MAX_VW    9000  // 底盘旋转最大速度
#define CHASSIS_SPEED_L   5500
#define CHASSIS_SPEED_M   6500
#define CHASSIS_SPEED_H   4000

//--- 转向轮零点角度
#define RF_INITANGLE 149.32f
#define LF_INITANGLE -95.48f
#define LB_INITANGLE 34.89f
#define RB_INITANGLE -33.55f

#if ROBOT_ID == INFANTRY_2022_SWERVE_1
float RUD_InitAngle[4] = {149.32f, -95.48f, 34.89f, -33.55f};
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
float RUD_InitAngle[4] = {5.4f, -118.65f, 150.82f, 33.48f};
#elif ROBOT_ID == INFANTRY_2022_SWERVE_3
float RUD_InitAngle[4] = {0.0f};
#elif ROBOT_ID == SRNTRY_2023_OMNI
float RUD_InitAngle[4] = {0.0f};
#endif

float Radius = 1.0f;  // 圆心距

extKalman_t Kalman_CHASFollow_Speed;

/* Private function declarations ---------------------------------------------*/
float Acclerate(float x , float k);
float Decclerate(float x , float k);

/**
 * @brief      初始化
 * @param[in]  None
 * @retval     None
 */
Chassis_classdef::Chassis_classdef()
{
    /*--- 驱动轮 PID -------------------------------------------------------------------------*/
    for(auto &m : DRV_PID)
    {
        //--- kp,ki,kd,ki_max,out_max,dt
        m.SetPIDParam(12.0f, 0.5f, 0.0f, 5000, 15000, 1.0f);
        m.I_SeparThresh = 9000;
    }

#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || ROBOT_ID == INFANTRY_2022_SWERVE_2
    /*--- 转向轮 PID -------------------------------------------------------------------------*/
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        RUD_PID[i][PID_Outer].SetPIDParam(18.0f, 0.0f, 0.0f, 2000, 1000, 0.002f);
        RUD_PID[i][PID_Outer].I_SeparThresh = 20;
        RUD_PID[i][PID_Inner].SetPIDParam(50.0f, 100.0f, 0.0f, 1000, 29999, 0.002f);
        RUD_PID[i][PID_Inner].I_SeparThresh = 100;
    }
        RUD_PID[3][PID_Outer].SetPIDParam(15.0f, 0.0f, 0.0f, 2000, 1000, 0.002f);
        RUD_PID[3][PID_Outer].I_SeparThresh = 20;
        RUD_PID[3][PID_Inner].SetPIDParam(50.0f, 100.0f, 0.0f, 1000, 29999, 0.002f);
        RUD_PID[3][PID_Inner].I_SeparThresh = 100;
    /*--- 转向轮参数校准、初始化----------------------------------------------------------------*/
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        RUD_Param[i].Init_angle = RUD_Param[i].Total_angle = RUD_InitAngle[i];

        RUD_Param[i].Turns_cnt = 0;
        RUD_Param[i].TarTurns_cnt = 0;
        RUD_Param[i].Turns_flag = 0;
        RUD_Param[i].Target_angle = 0;
    }
#elif ROBOT_ID == SRNTRY_2023_OMNI
#endif
    /*--- 跟随模式 PID -----------------------------------------------------------------------*/
    Follow_PID[PID_Outer].SetPIDParam(5500.0f/* 4500.0f */, 1.0f, 0.0f, 2000, 4000, 0.002f);//7200
    Follow_PID[PID_Outer].I_SeparThresh = 1000;
    Follow_PID[PID_Inner].SetPIDParam(0.45f, 0.1f, 0.0f, 2000, 5000, 0.002f);  //0.45  0.1
    Follow_PID[PID_Inner].I_SeparThresh = 9000;

    VxVy_Coe = 1.0f;
    ReF_Flag = false;


    //--- 底盘跟随模式的双环模式的内环测量值滤波
    KalmanCreate(&Kalman_CHASFollow_Speed, 1, 100);
}


/**
 * @brief      底盘总控制函数
 * @param[in]  None
 * @retval     None
 */
void Chassis_classdef::Control()
{

    if(Mode != CHAS_LockMode)
    {
        Process(Target_Vx, Target_Vy, Target_Vw);
//        Process(CTRL_DR16.Get_ExptVx(),CTRL_DR16.Get_ExptVy(),CTRL_DR16.Get_ExptVw());
    }
    else
    {
        Process(0,0,0);
    }

    // 驱动轮 发送电流值
    MotorMsgSend(&hcan1, DRV_Motor);
		
//    MotorMsgSend(&hcan2, DRV_Motor);
//#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 ||INFANTRY_2022_SWERVE_3
//    // 转向轮 发送电流值
//    MotorMsgSend(&hcan2, RUD_Motor);
//#endif

}


/**
 * @brief      底盘数据处理
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @retval     None
 */
#if USE_TEST_POWERLIMIT
float drv_tempcurrent[4];
#else
int16_t drv_tempcurrent[4];
#endif
int16_t rud_tempcurrent[4];
uint8_t Move_flag;
int16_t ssssspeed[3];
float Ramp_Vy, Ramp_Vx, Ramp_Vw;


#if ROBOT_ID == INFANTRY_2022_SWERVE_1
float ACCCCC_VAL = 5.0f, DECCCCC_VAL = 8.5f;
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
float ACCCCC_VAL = 17.0f, DECCCCC_VAL = 25.0f;/*10.0f*/
#elif ROBOT_ID == SRNTRY_2023_OMNI
float ACCCCC_VAL = 17.0f, DECCCCC_VAL = 25.0f;/*10.0f*/
#endif
void Chassis_classdef::Process(float Vx, float Vy, float Vw)
{
    //--- Yaw轴中心点
    // Gimbal.Motor[Yaw].encoder_offset = GIMBAL_YAW_CENTRE;

    //--- 根据功率设置底盘最大速度
    Set_MaxSpeed();

    //--- 底盘失能或状态异常——纯底盘不用340
    if(Mode == CHAS_DisableMode)// || DevicesMonitor.Get_State(COMMU_0X340_MONITOR) == Off_line)
    {
        Vx = Vy = Vw = 0.0f;
        Infantry.Chassis_Mode = Mode = CHAS_DisableMode;

        for(uint8_t i = 0 ; i < 4 ; i++)
        {
            DRV_Motor[i].Out = 0;
            RUD_Motor[i].Out = 0;

            DRV_PID[i].Reset();
            RUD_PID[i][PID_Outer].Reset();
            RUD_PID[i][PID_Inner].Reset();
        }
        Infantry.Chassis_PreMode = CHAS_DisableMode;

        return;
    }

#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || ROBOT_ID == INFANTRY_2022_SWERVE_2
    //--- 转向轮的总CAN掉线了则设置速度为0，防止底盘疯转
    if(DevicesMonitor.Get_State(0xF<<7))
    {
        Vx = Vy = Vw = 0;
    }
#elif ROBOT_ID == SRNTRY_2023_OMNI
#endif
		
    if(Mode != CHAS_SpinMode)
    {
        //--- 非小陀螺模式使用速度斜坡
        Drv_Slow(&Ramp_Vx, Vx, 4.0f, ACCCCC_VAL, DECCCCC_VAL);
        Drv_Slow(&Ramp_Vy, Vy, 4.0f, ACCCCC_VAL, DECCCCC_VAL);
    }
    else
    {
        Ramp_Vx = Get_PowerLimit()<80?Vx*0.8f:Vx;
        Ramp_Vy = Get_PowerLimit()<80?Vy*0.8f:Vy;
    }

    /*Low Pass Filter*/
    // Ramp_Vx = Vx_LPF.f(Ramp_Vx);
    // Ramp_Vy = Vy_LPF.f(Ramp_Vy);
    // Vw = Vw_LPF.f(Vw);
    Ramp_Vw = KalmanFilter(&Kalman_CHASFollow_Speed, Vw);


#if ROBOT_ID == INFANTRY_2021
    /* 麦轮解算 --------------------------*/
    Mecanum_Solve(Vx, Vy, Vw, Cal_Speed);
#elif ROBOT_ID == SRNTRY_2023_OMNI
    /* 全向解算 --------------------------*/
    Omni_Solve(Ramp_Vx, Ramp_Vy, Ramp_Vw, Cal_Speed);
#elif ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 || INFANTRY_2022_SWERVE_3
    /* 舵轮解算 --------------------------*/
    Rudder_Solve(Ramp_Vx, Ramp_Vy, Ramp_Vw, Cal_Speed);

#endif
		
    //--- PID计算
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        DRV_PIDCalc(i, Cal_Speed[i], DRV_Motor[i].getSpeed());
#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 || INFANTRY_2022_SWERVE_3
        RUD_PIDCalc(i, RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
#elif ROBOT_ID == SRNTRY_2023_OMNI
#endif	
    }

    //--- 爬坡功率分配处理
    // Uphill_Process((int16_t *)drv_tempcurrent, 4);

#if USE_RM_Referee //--- 接入裁判系统进行功率限制

#if USE_TEST_POWERLIMIT
    //--- Test
    //@note 有初步的限制效果了(但好像又限的太过了)，但是跟随归中的最后一下会卡顿一下
    //--- 广工的功率算法
    CHAS_Power.Test_NewLimit(drv_tempcurrent, rud_tempcurrent, 4);
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        DRV_Motor[i].Out = drv_tempcurrent[i];
        RUD_Motor[i].Out = rud_tempcurrent[i];
    }
#else
    CHAS_Power.Limit(drv_tempcurrent, 4);
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        DRV_Motor[i].Out = drv_tempcurrent[i];
    }
#endif /* end of USE_TEST_POWERLIMIT */
    
#endif /* end of USE_RM_Referee */
}


/**
 * @brief      全向移动速度分解
 * @param[in]  Vx
 * @param[in]  Vy
 * @retval     None
 */
void Chassis_classdef::Speed_Decompose(const float angle, float *Vx, float *Vy)
{
    float tempVx = *Vx;
    
    *Vx = *Vx * arm_cos_f32(angle) - *Vy * arm_sin_f32(angle);
	*Vy = *Vy * arm_cos_f32(angle) + tempVx * arm_sin_f32(angle);
}

/**
 * @brief      麦轮解算
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @param[in]  cal_speed 
 * @retval     None
 */
void Chassis_classdef::Mecanum_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float tempSpeed[4];
	float MaxSpeed = 0.0f;
	float Param = 1.0f;

	if (Infantry.Chassis_Mode != CHAS_FollowMode && Infantry.Chassis_Mode != CHAS_ReFollowMode)
	{
		VxVy_Coe = 1.0f;
	}

#if USE_RM_Referee
	//速度限制
    Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));

#else
	// 速度限制
	Constrain(&Vx, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-CHASSIS_MAX_SPEED ), (int16_t)(CHASSIS_MAX_SPEED));
#endif

	//四轮速度分解
	tempSpeed[0] = Vx - Vy + Vw;
	tempSpeed[1] = Vx + Vy + Vw;
	tempSpeed[2] = -Vx + Vy + Vw;
	tempSpeed[3] = -Vx - Vy + Vw;

	//寻找最大速度
	for (uint8_t i = 0; i < 4; i++)
	{
		if (fabs(tempSpeed[i]) > MaxSpeed)
		{
			MaxSpeed = fabs(tempSpeed[i]);
		}
	}

	//速度分配
	if (MaxSpeed > Vw_Limit)
	{
		Param = (float)Vw_Limit / MaxSpeed;
	}

	cal_speed[0] = tempSpeed[0] * Param;
	cal_speed[1] = tempSpeed[1] * Param;
	cal_speed[2] = tempSpeed[2] * Param;
	cal_speed[3] = tempSpeed[3] * Param;
}

/**
 * @brief      全向轮解算
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @param[in]  cal_speed 
 * @retval     None
 */
uint8_t No_move_flag = false;
uint8_t spin_flag;
uint16_t Brake_cnt;//--- 刹车
void Chassis_classdef::Omni_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
	float MaxSpeed = 0.0f;
	float Param = 1.0f;
	static uint16_t No_move_cnt = 0;

	if (Infantry.Chassis_Mode != CHAS_FollowMode && Infantry.Chassis_Mode != CHAS_ReFollowMode)
	{
		VxVy_Coe = 1.0f;
	}

#if USE_RM_Referee
	//速度限制
    Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));
#else
	// 速度限制
	Constrain(&Vx, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-CHASSIS_MAX_SPEED ), (int16_t)(CHASSIS_MAX_SPEED));
#endif

    //--- TODO 因为陀螺仪零漂会产生小幅度的Vw速度导致这里的InitAngle会来回跳变(暂时用个低级的方法解决)√

    //--- TODO 还有就是45度刹车可能由于车的重心较高，高速运动的时候会刹的比较猛会出现前倾
    //---      一个思路是不同的功率给不同的斜坡
    //---      另一个是刹车的时候先保持原本的角度，先让前轮失能，等车稳定后再呈45度归中
    //---      再一个是用高级点的加速曲线？

    if(Vx == 0 && Vy == 0)
    {
        No_move_flag = true;
        if(abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if(No_move_cnt < 500/* 500 */) //--- 静止后1000ms期间不允许底盘跟随
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
        
    }
    else
    {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        Move_flag = false;
    }
    else
    {
        if(No_move_flag != true)
        {
            Brake_cnt = 0;
        }
        Move_flag = true;
        if(abs(Vw)>100)
        {
            spin_flag = true;
        }
    }
	//四轮速度分解
	cal_speed[RF_201] = Vx - Vy + Vw;
	cal_speed[LF_202] = Vx + Vy + Vw;
	cal_speed[LB_203] = -Vx + Vy + Vw;
	cal_speed[RB_204] = -Vx - Vy + Vw;

    if(Infantry.Write_Msg[Uphill_Mode] == true)
    {
        cal_speed[RF_201] *= 0.5f;
        cal_speed[LF_202] *= 0.5f;
    }

	//寻找最大速度
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(cal_speed[i]) > MaxSpeed)
		{
			MaxSpeed = abs(cal_speed[i]);
		}
	}

	//速度分配
	if (MaxSpeed > Vw_Limit)
	{
		Param = (float)Vw_Limit / MaxSpeed;
	}

	cal_speed[RF_201] *= Param;
	cal_speed[LF_202] *= Param;
	cal_speed[LB_203] *= Param;
	cal_speed[RB_204] *= Param;
}

/**
 * @brief      舵轮解算
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @param[in]  cal_speed 
 * @retval     None
 */
void Chassis_classdef::Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{

    float Param = 1.0f;
    float MaxSpeed = 0.0f;

    static float const theta = atan(1.0/1.0);
    static uint8_t compare_flag = true;
    static uint16_t No_move_cnt = 0;

    // if((sqrt(float(Vx*Vx + Vy*Vy))) < 1000)
	// {
	// 	if(abs(Vw) >= 100/* 2000 */)
	// 	{
	// 		compare_flag = true;
	// 	}
	// 	else if(abs(Vw)<100)
	// 	{
	// 		if(abs(Vw)<100)
	// 		{
	// 			Vw = 0;
	// 			compare_flag = false;
	// 		}
	// 		else if(compare_flag == false)
	// 		{
	// 			Vw = 0;
	// 		}
	// 		else
	// 		{}
	// 	}
	// 	else
	// 	{}
	// }

    VxVy_Coe = 1.0f;

    
    /* 速度限制 ---------------------------------------------------------------------------------------*/
#if USE_RM_Referee
    Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));
#else
	Constrain(&Vx, (int16_t)-(CHASSIS_MAX_SPEED*VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED*VxVy_Coe));
	Constrain(&Vy, (int16_t)-(CHASSIS_MAX_SPEED*VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED*VxVy_Coe));
	Constrain(&Vw, (int16_t)-(CHASSIS_MAX_SPEED), (int16_t)(CHASSIS_MAX_SPEED));
#endif

    ssssspeed[0] = Vx;
    ssssspeed[1] = Vy;
    ssssspeed[2] = Vw;
    
    /* 三轴速度解算转向轮角度 ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);

    if(Vx == 0 && Vy == 0)
    {
        if(abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if(No_move_cnt < 500/* 500 */) //--- 静止后1000ms期间不允许底盘跟随
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
        
    }
    else
    {
        No_move_cnt = 0;
    }

    /* 驱动轮 速度解算 ---------------------------------------------------------------------------------*/
    cal_speed[RF_201] = -sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
    cal_speed[LF_202] =  sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[LB_203] =  sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[RB_204] = -sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));

    if(Infantry.Write_Msg[Uphill_Mode] == true)
    {
        cal_speed[RF_201] *= 0.5f;
        cal_speed[LF_202] *= 0.5f;
    }

    // 寻找最大速度
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(cal_speed[i]) > MaxSpeed)
		{
			MaxSpeed = abs(cal_speed[i]);
		}
	}

	// 速度分配  
	if (MaxSpeed > VxVy_Limit)
	{
		Param = (float)VxVy_Limit / MaxSpeed;
	}

	cal_speed[RF_201] *= Param;
	cal_speed[LF_202] *= Param;
	cal_speed[LB_203] *= Param;
	cal_speed[RB_204] *= Param;


    /* 计算转向轮角度当前值 ----------------------------------------------------------------------------*/
    RUDTotalAngle_Calc(RUD_Motor+RF_205, RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+LF_206, LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+LB_207, LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+RB_208, RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    /* 计算转向轮角度目标值 ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        //--- 转向轮 劣弧旋转
        RUD_Param[i].Target_angle = Turn_InferiorArc(i, RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
    }

}

/**
 * @brief      三轴速度解算舵角度
 * @param[in]  None
 * @retval     None
 */
void Chassis_classdef::RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw)
{
    static float const theta = atan(1.0/1.0);
    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;

    //--- TODO 因为陀螺仪零漂会产生小幅度的Vw速度导致这里的InitAngle会来回跳变(暂时用个低级的方法解决)√

    //--- TODO 还有就是45度刹车可能由于车的重心较高，高速运动的时候会刹的比较猛会出现前倾
    //---      一个思路是不同的功率给不同的斜坡
    //---      另一个是刹车的时候先保持原本的角度，先让前轮失能，等车稳定后再呈45度归中
    //---      再一个是用高级点的加速曲线？

    if(Vx == 0 && Vy == 0)
    {
        No_move_flag = true;
        if(abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if(No_move_cnt < 500) //--- 静止后1000ms期间不允许底盘跟随
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
    }
    else
    {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        Move_flag = false;

        if(Brake_cnt < 500)
        {
            Brake_cnt++;

            //--- 在上一帧不是静止的时候产生一个Vw的速度，为了不让他目标角度有一个归零的动作
            last_vw = (spin_flag == true ? 50 : 0);

            //--- 使用上一次的目标速度来保存上一次的目标角度
            RUD_Param[RF_205].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[LF_206].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[LB_207].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[RB_208].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
        }
        else
        {
            spin_flag = false;
            //--- 45度归中
            RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205]-45;
            RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206]+45;
            RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207]-45;
            RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208]+45;

            //--- 目标角度归零
            for(uint8_t i = 0 ; i < 4 ; i++)
            {
                RUD_Param[i].Target_angle = 0;
            }
        }

    }
    else
    {
        if(No_move_flag != true)
        {
            Brake_cnt = 0;
        }
        Move_flag = true;

        //--- 解除45度归中
        RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205];
        RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206];
        RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207];
        RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208];

        //--- 有目标速度的时候才进行舵轮解算的计算
        RUD_Param[RF_205].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[LF_206].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[LB_207].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[RB_208].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);

        if(abs(Vw)>100)
        {
            spin_flag = true;
        }

        //--- 无目标速度的时候不使用上一次角度来保存是因为跟随模式下IMU静止的瞬间会产生轻微的Vw速度
        last_vx = Vx;
        last_vy = 0;
        last_vw = 0;

    }

    //--- 更改为保存经过劣弧处理的目标角度(现在暂时不用这个上一帧的角度)
    // for(uint8_t i = 0 ; i < 4 ; i++) 
    // {
    //     RUD_Param[i].PreTar_angle = RUD_Param[i].Target_angle;
    // }

}

/**
 * @brief      转向轮当前总角度计算
 * @param[in]  rudder_motor
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */
void Chassis_classdef::RUDTotalAngle_Calc(Motor_GM6020* rudder_motor , int8_t motor_num , int8_t reset , uint8_t opposite)
{   
    float Cur_encoder = rudder_motor->getencoder()/8192*360;
    static float Pre_encoder[4] = {0};

    if(Cur_encoder - Pre_encoder[motor_num] > 180)
    {
        RUD_Param[motor_num].Turns_cnt--;
    }
    else if(Cur_encoder - Pre_encoder[motor_num] < -180)
    {
        RUD_Param[motor_num].Turns_cnt++;
    }
    Pre_encoder[motor_num] = Cur_encoder;

    if(reset == true)  //---圈数清零
    {
        RUD_Param[motor_num].Turns_cnt = 0;
    }

    if(opposite == false)   //---是否反向
    {
        RUD_Param[motor_num].Total_angle = Cur_encoder + RUD_Param[motor_num].Turns_cnt*360;
    }
    else if(opposite == true)
    {
        RUD_Param[motor_num].Total_angle = -Cur_encoder - RUD_Param[motor_num].Turns_cnt*360;
    }

}


/**
 * @brief      转向轮目标角度计算
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes < 100)
	{
		if(*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if(*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}
	recursiveTimes--;
}

float error;
void Chassis_classdef::RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = RUD_Param[motor_num].Target_angle;
    static float Pre_Target[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};
    static float Pre_TempTarget[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};

    static float temp_pre = 0;  //--- 保存劣弧旋转的上一次未经过零处理的目标角度

    // if(Cur_Target - Pre_Target[motor_num] > 180)
    // {
    //     RUD_Param[motor_num].TarTurns_cnt--;
    // }
    // else if(Cur_Target - Pre_Target[motor_num] < -180)
    // {
    //     RUD_Param[motor_num].TarTurns_cnt++;
    // }
    // Pre_Target[motor_num] = Cur_Target;
    //
    // if(reset == true)   //--- 圈数清零
    // {
    //     RUD_Param[motor_num].TarTurns_cnt = 0;
    // }
    //
    // if(opposite == false)    //---是否反向
    // {
    //     RUD_Param[motor_num].Target_angle = Cur_Target + \
    //                                         RUD_Param[motor_num].TarTurns_cnt*360 + \
    //                                         RUD_Param[motor_num].Init_angle + \
    //                                         RUD_Param[motor_num].Turns_flag*180;
    // }
    // else if(opposite == true)
    // {
    //     RUD_Param[motor_num].Target_angle = -Cur_Target - \
    //                                         RUD_Param[motor_num].TarTurns_cnt*360 - \
    //                                         RUD_Param[motor_num].Init_angle - \
    //                                         RUD_Param[motor_num].Turns_flag*180;
    // }
    
    /*--------------------------------*/
    //--- 将目标角度与当前角度换算到同一个周期中
    RUD_Param[motor_num].TarTurns_cnt = (int32_t)(RUD_Param[motor_num].Total_angle/180.0f) - (int32_t)(RUD_Param[motor_num].Total_angle/360.0f);

    RUD_Param[motor_num].Target_angle = RUD_Param[motor_num].TarTurns_cnt*360.0f + Cur_Target + RUD_Param[motor_num].Init_angle;

    temp_pre = Cur_Target;

    //---- 当前目标角度和当前角度的误差
    error = RUD_Param[motor_num].Target_angle - RUD_Param[motor_num].Total_angle;

    //--- 误差限幅
    // Constrain(&error, -180.0f, 180.0f); //--- 用这种限幅方法好像不太行
    AngleLimit(&error);

    //--- 如果角度误差大于90度则将速度反向并将目标角度叠加180度
    if(fabs(error) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500) /* && Mode != CHAS_SpinMode */)
    {
        //--- 目标值叠加半个周期
        RUD_Param[motor_num].Target_angle += 180.0f;

        temp_pre += 180.0f;

        //--- 驱动轮反转
        Cal_Speed[motor_num] = -Cal_Speed[motor_num];
        //--- 确保目标角度和当前角度处于同一个周期
        if(RUD_Param[motor_num].Target_angle > RUD_Param[motor_num].TarTurns_cnt*360.0f + 180.0f)
        {
            RUD_Param[motor_num].Target_angle -= 360.0f;
        }
        else if(RUD_Param[motor_num].Target_angle < RUD_Param[motor_num].TarTurns_cnt*360.0f - 180.0f)
        {
            RUD_Param[motor_num].Target_angle += 360.0f;
        }
    }
    /*--------------------------------*/
    Pre_TempTarget[motor_num] = RUD_Param[motor_num].Target_angle;

    RUD_Param[motor_num].PreTar_angle = temp_pre;  //--- 保存劣弧旋转的上一次未经过零处理的目标角度

}

/**
 * @brief      计算最小偏差,确定底盘跟随方向
 * @param[in]  target
 * @param[in]  current
 * @retval     Error(弧度制)
 */
float Chassis_classdef::Get_MinDeviation(int32_t target, int32_t current)
{
    int32_t Error = target - current;

	if (Error > 4096)
	{
		Error -= 8191;
	}
	else if (Error < -4096)
	{
		Error += 8191;
	}
	return ((float)Error / ENCODER_ANGLE_RATIO * (PI/180));
}


/**
 * @brief      计算最小偏差，使转向轮保持劣弧旋转
 * @param[in]  target
 * @param[in]  current
 * @retval     target(角度制)
 */
float Chassis_classdef::Turn_InferiorArc(uint8_t motor, float target, float current)
{
    float Error = target - current;

    if(Error > 180.0f)
	{
		return (target - 360.0f);
	}
	else if(Error < -180.0f)
	{
		return (target + 360.0f);
	}
	else
	{
		return target;
	}
}

/**
 * @brief      判断驱动轮是否需要反转
 * @param[in]  target
 * @param[in]  current
 * @retval     None
 */
float Error;
void Chassis_classdef::Judge_DRV_Dir(uint8_t motor, float target, float current)
{   
    
    Error = RUD_Param[motor].Target_angle - RUD_Param[motor].Total_angle;

    Constrain(&Error, -180.0f, 180.0f);

    if(fabs(Error) > 90.0f && Move_flag == true /* && Spin_flag == false *//* && Spin_flag == false && Movecount_flag == 0 */ /* && Movecount_flag != 0 */)
    {
        RUD_Param[motor].Target_angle += 180.0f;

        if(RUD_Param[motor].Target_angle > (RUD_Param[motor].Turns_cnt * 360.0f + 180.0f))
        {
            RUD_Param[motor].Target_angle -= 360.0f;
        }
        else if(RUD_Param[motor].Target_angle < (RUD_Param[motor].Turns_cnt * 360.0f - 180.0f))
        {
            RUD_Param[motor].Target_angle += 360.0f;
        }

        Cal_Speed[motor] = -Cal_Speed[motor];
        
    }

}


/**
 * @brief      设置最大速度限制
 * @param[in]  None
 * @retval     None
 */
void Chassis_classdef::Set_MaxSpeed()
{
#if USE_RM_Referee
    if (SupCap.SendData.is_cap_output == OFF || DevicesMonitor.Get_State(SUPCAP_MONITOR) == Off_line)
	{
		if (Get_PowerLimit() < 60 /* && Get_PowerLimit() != 0 */)
		{
			// VxVy_Limit = 4700.0f/* Chassis_SpeedNormal */;
			// Vw_Limit =  4700.0f ;

            VxVy_Limit = 1500.0f; // 老年人模式，摆摊用
			Vw_Limit =  4500.0f ;

            Set_RudMaxOut(12000);
		}
		else if(Get_PowerLimit() == 60)
		{
			VxVy_Limit = CHASSIS_SPEED_L;
			Vw_Limit = 5500.0f;

            Set_RudMaxOut(13000);
		}
		else if (Get_PowerLimit() > 60 && Get_PowerLimit() <= 80)
		{
			VxVy_Limit = CHASSIS_SPEED_M;
			Vw_Limit = CHASSIS_SPEED_M;

            Set_RudMaxOut(18000);
		}
		else if (Get_PowerLimit() > 80 /* && Get_PowerLimit() <= 120 */)
		{
            if(Get_PowerLimit()>=200) //--- 比赛准备阶段的200W功率按80W来算
            {
                VxVy_Limit = 7500.0f;
								Vw_Limit = 7500.0f;
            }
            else
            {
                VxVy_Limit = CHASSIS_SPEED_H;
                Vw_Limit = CHASSIS_SPEED_H;
            }

            Set_RudMaxOut(29999);
		}

		if(/* Infantry.Write_Msg[4] == ON || */ Get_Power() == 65535)  // --- 没有电容且或者无限功率
		{
			VxVy_Limit = CHASSIS_SPEED_H;
			Vw_Limit = CHASSIS_SPEED_H;
		}
	}
    else
	{
		//--- 开电容的时候要加个上坡的特别处理
        VxVy_Limit = 8000;
        Vw_Limit = 8000;
// #if ROBOT_ID == INFANTRY_2022_SWERVE_2
        Set_RudMaxOut(13999);
// #endif
	}
#else
    // if(Low speed mode == true)
	// {
	// 	VxVy_Limit = 3000.0f;
	// 	Vw_Limit = 3000.0f;
	// }
	// else
	// {
		VxVy_Limit = CHASSIS_SPEED_H;
		Vw_Limit = CHASSIS_SPEED_H;
	// }
#endif

    Constrain(&Target_Vx, (int16_t)(-VxVy_Limit), (int16_t)VxVy_Limit);
    Constrain(&Target_Vy, (int16_t)(-VxVy_Limit), (int16_t)VxVy_Limit);
    Constrain(&Target_Vw, (int16_t)(-Vw_Limit), (int16_t)Vw_Limit);

}

/**
  * @brief  	跟随模式
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
float temp_spin;
uint16_t ramp_count = 0;
void Chassis_classdef::Follow_Ctrl(float *Vx, float *Vy, float *Vw)
{
    Follow_PID[PID_Outer].Target = *Vw;
    Follow_PID[PID_Outer].Current = imu_Export.gz;
    *Vw = Follow_PID[PID_Outer].Cal();
}

/**
  * @brief  	反向跟随模式
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::ReFollow_Ctrl(float *Vx, float *Vy, float *Vw)
{
    temp_spin = 0;

    // 反向跟随过度----------------------------------------------------------
    if(ReF_Flag == true)
    {
        ReF_Cnt++;

        Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()-(GIMBAL_YAW_CENTRE-4096)/ENCODER_ANGLE_RATIO), Vx, Vy);

        if (fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE + 4066) <= 50.0f)
		{
			ReF_Flag = false;
		}
		if (ReF_Cnt > 200 && ReF_Flag != false) // --- 超时未转换则直接转换
		{
			ReF_Flag = false;
			ReF_Cnt = 0;
		}
    }
    // 反向跟随过度 END----------------------------------------------------
    else
    {
        ReF_Cnt = 0;

        //--- 反向跟随的中心点为底盘尾部中心点
        if(fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE + 4096) > 5)
        {
            // follow pid calc
            Follow_PID[PID_Outer].Target = Get_MinDeviation(Gimbal.Motor[Yaw].get_totalencoder(), GIMBAL_YAW_CENTRE+4096);
            Follow_PID[PID_Outer].Current = 0;

            // *Vz += KalmanFilter(&Kalman_CHASFollow_Speed, Follow_PID[PID_Inner].Cal());
            *Vw += Follow_PID[PID_Inner].Cal();
        }

        //--- 转弯时对Vx和Vy进行限制，在功率限制的情况下可以更好转弯
        if(fabs(*Vw) > 1200) 
        {
            VxVy_Coe = ((CHASSIS_MAX_VW - fabs(*Vw) + 1200) * (CHASSIS_MAX_VW - fabs(*Vw) + 1200)) / (CHASSIS_MAX_VW * CHASSIS_MAX_VW);
            Constrain(&VxVy_Coe, 0.2f, 1.0f);
        }
        else
        {
            VxVy_Coe = 1.0f;
        }

        
    }

    Infantry.Chassis_PreMode = CHAS_ReFollowMode;
}

/**
  * @brief  	小陀螺模式
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
float test_Vz = 7000;
void Chassis_classdef::Spin_Ctrl(float *Vx, float *Vy, float *Vw)
{

    Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()), Vx, Vy);

    // *Vx *= 0.7f;
    // *Vy *= 0.7f;

    switch(Infantry.Ctrl_Source)
    {
    case CTRL_RC:
        temp_spin = RAMP_Output(test_Vz, temp_spin, 20.0f);
        *Vw = temp_spin;
        break;

    case CTRL_PC:
        temp_spin = RAMP_Output(7000, temp_spin, 20.0f);
        *Vw = temp_spin;
        break;
    }


    Infantry.Chassis_PreMode = CHAS_SpinMode;
}


/**
  * @brief  	扭腰模式
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::Swing_Ctrl(float *Vx, float *Vy, float *Vw)
{
    //--- 舵轮没必要用这个玩意
}

/**
  * @brief  	自动追踪装甲板模式(娱乐用)
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::AotuTrack_Ctrl(float *Vx, float *Vy, float *Vz)
{
    //---平时展览用或者其他的小测试用
}


/**
  * @brief  	无跟随模式(纯底盘运动)
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::NotFollwe_Ctrl(float *Vx, float *Vy, float *Vw)
{
    *Vx = *Vx;
    *Vy = *Vy;
    *Vw = *Vw;

    Infantry.Chassis_PreMode = CHAS_NotFollowMode;
}


/**
 * @brief      底盘驱动电机增量式PID计算
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @note       加入过热保护
 * @retval     None
 */
void Chassis_classdef::DRV_PIDCalc(uint8_t motor, float target, float current)
{
    DRV_PID[motor].Target = target;
    DRV_PID[motor].Current = current;
    DRV_Motor[motor].Out = DRV_PID[motor].Cal();
#if USE_M3508_OVERHEAT_PROTECT
    OverHeat_Protect(motor); //--- 过热保护
    DRV_Motor[motor].Out *= overHeat_Coe[motor];
#endif
    drv_tempcurrent[motor] = DRV_Motor[motor].Out; // 保存当前值用于功率限制的计算
}

/**
 * @brief      底盘转向电机位置式PID计算
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @retval     None
 */
void Chassis_classdef::RUD_PIDCalc(uint8_t motor, float target, float current)
{
    RUD_PID[motor][PID_Outer].Target = target;
    RUD_PID[motor][PID_Outer].Current = current;

    RUD_PID[motor][PID_Inner].Target = RUD_PID[motor][PID_Outer].Cal();
    RUD_PID[motor][PID_Inner].Current = RUD_Motor[motor].getSpeed();
    RUD_Motor[motor].Out = RUD_PID[motor][PID_Inner].Cal();

    rud_tempcurrent[motor] = RUD_Motor[motor].Out; // 保存当前值用于功率限制的计算
}

/**
 * @brief      底盘驱动电机数据更新
 * @param[in]  header
 * @param[in]  can_rx_data
 * @retval     ID
 */
uint8_t Chassis_classdef::DRV_Motors_Update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
{
    uint8_t counter = Frame_CHAS_DRV0;

    //--- auto类型的方式
    for (auto& m : DRV_Motor)
    {
        if (m.CheckID(header->StdId))
        {
            m.update(can_rx_data);
            DevicesMonitor.FrameCounter[counter]++;
            return m.ID;
        }
       counter++;
    }
   
    return 0;
}




/**
 * @brief      获取裁判底盘相关裁判系统信息
 * @param[in]  None
 * @retval     Referee data
 */
float Chassis_classdef::Get_Power()
{
    return Referee.PowerHeatData.chassis_power; // 底盘功率
}
uint16_t Chassis_classdef::Get_PowerLimit()
{
    return Referee.GameRobotState.max_chassis_power; // 底盘最大功率
}
uint16_t Chassis_classdef::Get_PowerBuffer()
{
    return Referee.PowerHeatData.chassis_power_buffer; // 底盘缓冲功率
}
uint8_t Chassis_classdef::IsMove()
{
    return Infantry.Write_Msg[2];
}



void Chassis_classdef::Set_RudMaxOut(uint16_t maxout)
{
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        RUD_PID[i][PID_Inner].Out_Max = maxout;
    }
}

/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_classdef::Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{

    if(abs(*rec) - abs(target) < 0)//加速时
    {
        if(abs(*rec) > 10)
        {
            slow_Inc = slow_Inc * Accval;//速度提起来的时候增大到5倍
        }
    }
    
    if(abs(*rec) - abs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//减速时放大15倍
    }
    if(abs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}


/**
 * @brief      底盘加速曲线
 * @param[in]  Vx Vy
 * @retval     None
 */
void Chassis_classdef::AcclerateCurve(float *Vx, float *Vy)
{
    Acc_Param.linnerSpeed = sqrt((*Vx) * (*Vx) + (*Vy) * (*Vy));
    if ((fabs(Acc_Param.linnerSpeed) - fabs(Acc_Param.linnerSpeedLast)) > 3000) //当加速超过一定阈值时使用曲线
    {
        Acc_Param.accelerating = 1;
		Acc_Param.decelerating = 0;
        Acc_Param.accCnt = 0;
    }
    if ((fabs(Acc_Param.linnerSpeed) - fabs(Acc_Param.linnerSpeedLast)) < -3000) //当减速超过一定阈值时使用曲线
    {
        Acc_Param.decelerating = 1;
		Acc_Param.accelerating = 0;
        Acc_Param.accCnt = 0;
    }
	/*加速曲线*/
    if (Acc_Param.accelerating == 1)
    {
        Acc_Param.accCnt += 0.005;
        Acc_Param.accKp = Acclerate(Acc_Param.accCnt, Acc_Param.accK);
        if (Acc_Param.accKp > 0.999f)
        {
            Acc_Param.accelerating = 0;
        }
    }
    else if(Acc_Param.decelerating != 1)
    {
        Acc_Param.accKp = 1;
        Acc_Param.accCnt = 0;
    }
	/*减速曲线*/
    if (Acc_Param.decelerating == 1)
    {
        Acc_Param.accCnt += 0.005;
        Acc_Param.accKp = Decclerate(Acc_Param.accCnt,20);
        if ( Acc_Param.accKp < 0.01f)
        {
            Acc_Param.decelerating = 0;
        }
    }
    else if(Acc_Param.accelerating != 1)
    {
        Acc_Param.accKp = 1;
        Acc_Param.accCnt = 0;
    }
	/*×系数*/
	if(Acc_Param.accelerating == 1)
	{
		*Vx *= Acc_Param.accKp;
		*Vy *= Acc_Param.accKp;
	}
	else if(Acc_Param.decelerating == 1)
	{
		*Vx = Acc_Param.deceleRecodeSpeed[0] * Acc_Param.accKp;
		*Vy = Acc_Param.deceleRecodeSpeed[1] * Acc_Param.accKp;
	}
	if(Acc_Param.decelerating != 1)
	{
		Acc_Param.deceleRecodeSpeed[0] = *Vx;
		Acc_Param.deceleRecodeSpeed[1] = *Vy;
	}
    Acc_Param.linnerSpeedLast = Acc_Param.linnerSpeed;
}
/**
 * @description: 公式原型 y = 1/(1+e^(-k(x-2/k)))  当k = 4.2 , x = 1时 y = 0.9
 */
float Natural_num = 2.718281828f;//--- 自然数e
float Acclerate(float x , float k)
{
	float y;
	k = 4.2 / k;
	if(k == 0)
	{
		return 1;
	}
	y = 1/(1+powf(Natural_num,(-k*(x-2/k))));
	return y;
}
/**
 * @description: 公式原型 y =1/(1+e^(?k(-(x-(5/k))?2/k) ) )
 * @note         当k = 10 , x = 0.8时 y = 0.01
 */
float Decclerate(float x , float k)
{
	float y;
	if(k == 0)
	{
		return 1;
	}
	y = 1/(1+powf(Natural_num,(-k*(-x+3/k))));
	return y;
}

/**
 * @brief      电机过热输出保护
 * @param[in]  motor ID
 * @retval     None
 */
void Chassis_classdef::OverHeat_Protect(uint8_t motor)
{
    static float M3508_max_temp = 125.0f; //--- 3508绕组最高允许温度为125
    static float critical_temp = M3508_max_temp*0.7f; //--- 临界温度87.5，当前温度超过该临界值输出开始下降
    static float warning_temp = M3508_max_temp*0.8f;  //--- 过热保护截止温度100，当前温度高于此温度时电机停止输出

    overHeat_Coe[motor] = (float)(1.0f - ((DRV_Motor[motor].getTempature() - critical_temp) / (warning_temp - critical_temp)));
    Constrain(&overHeat_Coe[motor], 0.0f, 1.0f);
}


/**
 * @brief      爬坡模式处理(驱动轮功率分配)
 * @param[in]  motorCurrent wheelNum
 * @note       感觉在此基础上可以再加速度限制来限制目标值的输出
 * @note       放到功率限制里面去了
 * @retval     None
 */
void Chassis_classdef::Uphill_Process(int16_t *DRV_Current, uint8_t amount)
{
    int32_t FBwheel_IN[2] = {0, 0}; //--- 前后轮输入电流
    float FBWheel_coe[2][2] = {{0,0}, {0,0}}; //--- 前后轮电流占比
    uint8_t i = 0;

    if (Mode == CHAS_FollowMode && Infantry.Write_Msg[Uphill_Mode]==true && abs(Target_Vy)>1000 && imu.rol < 170.0f) //--- 如果处于上坡模式且有Vy速度
    {
        //--- 计算前轮原始电流
        for (i = 0 ; i < amount/2 ; i++)
        {
            FBwheel_IN[0] += abs(DRV_Current[i]);
        }
        //--- 计算前轮电流占比
        for (i = 0 ; i < amount/2 ; i++)
        {
            FBWheel_coe[0][i] = ((float)(DRV_Current[i])) / ((float)(FBwheel_IN[0]));
        }
        //--- 需要分配后轮的功率 挖一部分给后轮(测试值70%)
        int32_t temp_FWheel = (int32_t)FBwheel_IN[0] * 0.7f;
        FBwheel_IN[0] -= temp_FWheel;

        //--- 计算分配后，前轮该有的电流
        for (i = 0 ; i < amount/2 ; i++)
        {
            DRV_Current[i] = (int16_t)(FBwheel_IN[0] * FBWheel_coe[0][i]);
        }

        //--- 计算后轮原本的输入电流
        for (i = amount/2 ; i < amount ; i++)
        {
            FBwheel_IN[1] += abs(DRV_Current[i]);
        }
        //--- 计算后轮原本的输入百分比
        for (i = amount/2 ; i < amount ; i++)
        {
            FBWheel_coe[1][i] = ((float)(DRV_Current[i])) / ((float)(FBwheel_IN[1]));
        }
        //--- 给后轮加上从前轮挖过来的电流
        FBwheel_IN[1] += temp_FWheel;
        //--- 计算分配后，后轮最终该有的电流
        for (i = amount/2 ; i < amount ; i++)
        {
            DRV_Current[i] = (int16_t)(FBwheel_IN[1] * FBWheel_coe[1][i]);
        }
    }
}

