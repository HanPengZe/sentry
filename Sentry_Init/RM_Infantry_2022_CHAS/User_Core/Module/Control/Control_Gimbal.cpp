/**
 ------------------------------------------------------------------------------
 * @file    Control_Gimbal.cpp
 * @author  Shake
 * @brief   ��̨����
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "Control_Gimbal.h"
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "Robot_Config.h"
#include "arm_math.h"
#include "DJI_AIMU.h"

/**
 * @brief      ��ʼ��
 * @param[in]  None
 * @retval     None
 */
Gimbal_classdef::Gimbal_classdef()
{
    /*-------------------------------------------------
		6020���泯��Ĳ����������Ƕȸ����ٶȻ���
		6020���泯�ҵĲ����������Ƕ������ٶȻ���
	--------------------------------------------------*/
    //--- kp,ki,kd,ki_max,out_max,dt
    Gimbal_YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    Gimbal_YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    Gimbal_YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    Gimbal_YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    Gimbal_YawPID[A_IMU][PID_Outer].SetPIDParam(-15.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    Gimbal_YawPID[A_IMU][PID_Outer].I_SeparThresh = 1000;
    Gimbal_YawPID[A_IMU][PID_Inner].SetPIDParam(16.0f, 100.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);
    Gimbal_YawPID[A_IMU][PID_Inner].I_SeparThresh = 5000;

    Gimbal_PitPID[Mech][PID_Outer].SetPIDParam(13.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    Gimbal_PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    Gimbal_PitPID[Mech][PID_Inner].SetPIDParam(10.0f, 200.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);
    Gimbal_PitPID[Mech][PID_Inner].I_SeparThresh = 5000;

    /*--- Angle Init -------------------------------------------------------------------------*/
    Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
    Target.Mech_Pit = Motor[Pit].get_totalencoder();
    Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;//

    //--- Yaw��ң�ؿ������ϵ��
    YawRCCtrl_Coe = /* 0.05f */ 0.0015f;

    //--- ������̨��ʼģʽ
    Set_YawCtrlMode(A_IMU);
    Set_PitCtrlMode(Mech);

}


/**
 * @brief      ��̨�ܿغ���
 * @param[in]  None
 * @retval     None
 */
void Gimbal_classdef::Control()
{
    // if(YawMode == A_IMU && DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    // {
    //     Set_YawCtrlMode(Mech);
    // }

    //--- Ŀ��Ƕȸ���
    TargetAngle_Update(CTRL_DR16.Get_ExptYaw(), CTRL_DR16.Get_ExptPit());

    //--- ���ݴ���
    Process();

    //--- ���͵���ֵ
#if ROBOT_ID == INFANTRY_2021
    MotorMsgSend(&hcan1, Motor);
#elif ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 ||INFANTRY_2022_SWERVE_3
    MotorMsgSend(&hcan2, Motor[Yaw]);
    // MotorMsgSend(&hcan1, Motor[Pit]);
#endif

}

/**
 * @brief      ��̨���ݴ���
 * @param[in]  Yawparam
 * @param[in]  Pitparam
 * @retval     None
 */
void Gimbal_classdef::Process()
{
    Gimbal.Motor[Pit].encoder_offset = 0;

    if(Infantry.Get_GimbalMode() == Gimbal_DisableMode || Infantry.Get_State() != Robot_Activating || DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    {
        //set init angle...
        Set_InitAngle();

        Motor[Yaw].Out = 0;
        Motor[Pit].Out = 0;
        return;
    }

    //--- PID����
    Motor_PIDCalc();

}


/**
 * @brief      ��ȡ�����Ŀ��Ƕ�
 * @param[in]  type
 * @retval     Target Angle
 */
float *Gimbal_classdef::Get_TargetAngle(GimbalMotor_type_e type)
{
    static float *return_angle[2];

    switch(type)
    {
    case Yaw:   //--- Yaw ------------------------
        switch(YawMode)
        {
        case Mech: return_angle[Yaw] = &Target.Mech_Yaw;
            break;
        case A_IMU: return_angle[Yaw] = &Target.AIMU_Yaw/* *ENCODER_ANGLE_RATIO */;
            break;
        case C_IMU: return_angle[Yaw] = &Target.CIMU_Yaw;
            break;
        }
        break;

    case Pit:   //--- Pit ------------------------
        switch(PitMode)
        {
        case Mech: return_angle[Pit] = &Target.Mech_Pit;
            break;
        case A_IMU: return_angle[Pit] = &Target.AIMU_Pit;
            break;
        case C_IMU: return_angle[Pit] = &Target.CIMU_Pit;
            break;
        }
        break;
    }
    return return_angle[type];
}

/**
 * @brief      ����Ŀ��Ƕ�
 * @param[in]  Yawparam Pitparam
 * @retval     None
 */
void Gimbal_classdef::TargetAngle_Update(float Yawparam, float Pitparam)
{
    // --- �Ӿ���Ŀ��ʱ�������ֶ�ֵ
    // if(Infantry.Get_VisionMode() != Vision_Disable /* && Vision.Get_Mode() != false */ && DevicesMonitor.Get_State(VISION_MONITOR) != Off_line)
    // {
    //     if(Vision.Get_Mode() != false)
    //     {
    //         Yawparam = Pitparam = 0;
    //     }

    //     Vision.KalmanFilter_Ctrl(VISION_ON, Get_TargetAngle(Yaw), Get_TargetAngle(Pit));
        
    //     // Pit_AngleLimit();
    //     // return;
    // }
    // else
    // {
    //     Vision.KalmanFilter_Ctrl(VISION_OFF, Get_TargetAngle(Yaw), Get_TargetAngle(Pit));
    // }

    /*--- Yaw Ŀ��Ƕȸ��� -----------------------------------------*/
    CHAS_SeparateLimit(ON, &Yawparam); // ����RCģʽ����̨����̵ķ���Ƕ�


    /*--- ʵʱ������������ģʽ�ĳ�ʼ�Ƕ�ֵ,��ֹģʽ�л�ʱը�� ---------*/
    switch(YawMode)
    {
    case Mech: Target.Mech_Yaw += Yawparam * 30.0f;
               Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO; 
        break;
    case A_IMU: Target.AIMU_Yaw += Yawparam * ENCODER_ANGLE_RATIO;//
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    case C_IMU: Target.CIMU_Yaw += Yawparam;
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
                Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;
        break;
    }

    /*--- Pit Ŀ��Ƕȸ��� -----------------------------------------*/
    switch(PitMode)
    {
    case Mech: Target.Mech_Pit += Pitparam;
               Target.AIMU_Pit = imu_Export.Total_Pit * ENCODER_ANGLE_RATIO;
        break;
    case A_IMU: Target.AIMU_Pit += Pitparam;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
        break;
    case C_IMU: Target.CIMU_Pit += Pitparam;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
                Target.AIMU_Pit = imu_Export.Total_Pit * ENCODER_ANGLE_RATIO;
        break;
    }

    /*--- Pit �Ƕ����� ---------------------------------------------*/
    Pit_AngleLimit();
}


/**
 * @brief      ��̨���λ��ʽPID����
 * @param[in]  None
 * @retval     None
 */
void Gimbal_classdef::Motor_PIDCalc()
{
    
    switch(YawMode)
    {
    case Mech:
        Gimbal_YawPID[Mech][PID_Outer].Current = Motor[Yaw].get_totalencoder();
        Gimbal_YawPID[Mech][PID_Inner].Current = Motor[Yaw].getSpeed();
        break;

    case A_IMU:
        Gimbal_YawPID[A_IMU][PID_Outer].Current = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;
        Gimbal_YawPID[A_IMU][PID_Inner].Current = imu_Export.gz;
        break;

    case C_IMU:

        break;
    }

    switch(PitMode)
    {
    case Mech:
        Gimbal_PitPID[Mech][PID_Outer].Current = Motor[Pit].get_totalencoder();
        Gimbal_PitPID[Mech][PID_Inner].Current = imu_Export.gy/* Motor[Pit].getSpeed() */;
        break;

    case A_IMU:
        
        break;

    case C_IMU:

        break;
    }
    /*--- Yaw PID Calc --------------------------------------------------------------------------*/
    Gimbal_YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
    Gimbal_YawPID[YawMode][PID_Inner].Target = Gimbal_YawPID[YawMode][PID_Outer].Cal();
    Motor[Yaw].Out = Gimbal_YawPID[YawMode][PID_Inner].Cal();

    /*--- Pit PID Calc --------------------------------------------------------------------------*/
    Gimbal_PitPID[PitMode][PID_Outer].Target = *Get_TargetAngle(Pit);
    Gimbal_PitPID[PitMode][PID_Inner].Target = Gimbal_PitPID[PitMode][PID_Outer].Cal();
    Motor[Pit].Out = Gimbal_PitPID[PitMode][PID_Inner].Cal();

}


/**
 * @brief      ����Yaw��������ģʽ
 * @param[in]  mode
 * @retval     None
 */
void Gimbal_classdef::Set_YawCtrlMode(GimbalMotor_Ctrl_e mode)
{
    switch (mode)
    {
    case Mech:
        YawMode = Mech;
        Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;
        // Target.CIMU_Yaw = CIMU..
        break;

    case A_IMU:
        // if(mode == A_IMU && DevicesMonitor.Get_State(GIMBAL_AIMU_MONITOR) == Off_line)
        // {
        //     Set_YawCtrlMode(Mech);
        // }
        // else
        // {
            YawMode = A_IMU;
            Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
            // Target.CIMU_Yaw = CIMU..
        // }
        break;

    case C_IMU:
        // if(mode == C_IMU && DevicesMonitor.Get_State(GIMBAL_CIMU_MONITOR) == Off_line)
        // {
        //     Set_YawCtrlMode(Mech);
        // }
        // else
        // {
            YawMode = C_IMU;
            Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
            Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;
        // }
        break;
    }
}


/**
 * @brief      ����Pit��������ģʽ
 * @param[in]  mode
 * @retval     None
 */
void Gimbal_classdef::Set_PitCtrlMode(GimbalMotor_Ctrl_e mode)
{
    switch (mode)
    {
    case Mech:
        PitMode = Mech;
        // Target.AIMU_Yaw = AIMU..
        // Target.CIMU_Yaw = CIMU..
        break;

    case A_IMU:
        // if(mode == A_IMU && DevicesMonitor.Get_State(GIMBAL_AIMU_MONITOR) == Off_line)
        // {
        //     Set_PitCtrlMode(Mech);
        // }
        // else
        // {
            PitMode = A_IMU;
            Target.AIMU_Pit = imu_Export.Total_Pit * ENCODER_ANGLE_RATIO;
            // Target.CIMU_Pit = CIMU..
        // }
        break;

    case C_IMU:
        // if(mode == C_IMU && DevicesMonitor.Get_State(GIMBAL_CIMU_MONITOR) == Off_line)
        // {
        //     Set_PitCtrlMode(Mech);
        // }
        // else
        // {
            PitMode = C_IMU;
            Target.Mech_Pit = Motor[Pit].get_totalencoder();
            Target.AIMU_Pit = imu_Export.Total_Pit * ENCODER_ANGLE_RATIO;
        // }
        break;
    }
}




/**
 * @brief      ���ó�ʼĿ��Ƕ�
 * @param[in]  yaw_init
 * @param[in]  pit_init
 * @retval     None
 */
void Gimbal_classdef::Set_InitAngle()
{
    switch(YawMode)
    {
    case Mech: Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    case A_IMU: Target.AIMU_Yaw = imu_Export.Total_Yaw * ENCODER_ANGLE_RATIO;//
        break;
    case C_IMU: /* Target.CIMU_Yaw = *yaw_init; */
        break;
    }

    switch(PitMode)
    {
    case Mech: Target.Mech_Pit = Motor[Pit].get_totalencoder();
        break;
    case A_IMU: Target.AIMU_Pit = imu_Export.Total_Pit * ENCODER_ANGLE_RATIO;
        break;
    case C_IMU: /* Target.CIMU_Pit = *pit_init; */
        break;
    }
}

/**
 * @brief      Pit����Ƕ��޷�
 * @param[in]  None
 * @retval     None
 */
void Gimbal_classdef::Pit_AngleLimit()
{
    static float IMUMode_Limit_L;
    static float IMUMode_Limit_H;

    switch(PitMode)
    {
    case Mech:
        if(Target.Mech_Pit < GIMBAL_PIT_MIN)
        {   
            Target.Mech_Pit = GIMBAL_PIT_MIN;
        }
        else if(Target.Mech_Pit > GIMBAL_PIT_MAX)
        {
            Target.Mech_Pit = GIMBAL_PIT_MAX;
        }
        break;

    case A_IMU:
        //--- ������ģʽ������ֵ��Ϊ�ο� ӳ�䵽IMU�ǶȽ��нǶ�����
        IMUMode_Limit_L = imu_Export.Total_Pit - fabs(Get_PitDeviateL());
        IMUMode_Limit_H = imu_Export.Total_Pit + fabs(Get_PitDeviateH());
        if(Target.AIMU_Pit > IMUMode_Limit_H)
        {
            Target.AIMU_Pit = IMUMode_Limit_H;
        }
        else if(Target.AIMU_Pit < IMUMode_Limit_L)
        {
            Target.AIMU_Pit = IMUMode_Limit_L;
        }
        break;

    case C_IMU:
        //--- ������ģʽ������ֵ��Ϊ�ο� ӳ�䵽IMU�ǶȽ��нǶ�����
        // IMUMode_Limit_L = IMU Total - fabs(Get_PitDeviateL());
        // IMUMode_Limit_H = IMU Total + fabs(Get_PitDeviateH());
        if(Target.CIMU_Pit > IMUMode_Limit_H)
        {
            Target.CIMU_Pit = IMUMode_Limit_H;
        }
        else if(Target.CIMU_Pit < IMUMode_Limit_L)
        {
            Target.CIMU_Pit = IMUMode_Limit_L;
        }
        break;
    }
}


/**
 * @brief      ��ȡ��ǰλ����Yaw�����ĵ��ƫ��Ƕ�
 * @param[in]  cur_encoder
 * @retval     Off angle(������)
 */
float Gimbal_classdef::Get_YawDeviateC(float cur_encoder)
{
    // return ((cur_encoder - GIMBAL_YAW_CENTRE) / ENCODER_ANGLE_RATIO * (PI/180));
    return (cur_encoder * (PI/180));
}
/**
 * @brief      ��ȡ��ǰλ����Yaw�ᷴ�����ĵ��ƫ��Ƕ�
 * @param[in]  cur_encoder
 * @retval     Off angle(������)
 */
float Gimbal_classdef::Get_YawDeviateReC(float cur_encoder)
{
    return ((cur_encoder - GIMBAL_YAW_CENTRE - 4096) / ENCODER_ANGLE_RATIO * (PI/180));
}

/**
 * @brief      ��ȡPit���ƫ���޷�ֵ�ĽǶ�
 * @param[in]  None
 * @retval     deviate encoder
 */
float Gimbal_classdef::Get_PitDeviateH()
{
    return ((Motor[Pit].get_totalencoder()>GIMBAL_PIT_MAX)? 0 : Motor[Pit].get_totalencoder()-GIMBAL_PIT_MAX);
}
float Gimbal_classdef::Get_PitDeviateL()
{
    return ((Motor[Pit].get_totalencoder()<GIMBAL_PIT_MIN)? 0 : Motor[Pit].get_totalencoder()-GIMBAL_PIT_MIN);
}

/**
 * @brief      ����ң��ģʽ�µ���̨����̵ķ���Ƕ�
 * @param[in]  Switch
 * @param[in]  yawparam
 * @retval     None
 */
float xxxx = 0.0015; //0.05
float aaaa = 0.03f; //5
float bbbb = 180.0f;
void Gimbal_classdef::CHAS_SeparateLimit(bool Switch, float *yawparam)
{

    if(Switch == OFF || Infantry.Get_CtrlSource() != CTRL_RC || (Infantry.Get_ChassisMode() != CHAS_FollowMode && Infantry.Get_ChassisMode() != CHAS_ReFollowMode))
    {
        return;
    }

    if((fabs(Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE)> 1000) && *yawparam != 0)
    {
        YawRCCtrl_Coe -= (aaaa / bbbb);

        if(fabs(YawRCCtrl_Coe) < aaaa / (bbbb-10.0f))
        {
            YawRCCtrl_Coe = 0;
        }
    }
    else
    {
        YawRCCtrl_Coe = xxxx;
    }

    // if(fabs(Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE) > 1000)
    // {
    //     Target.AIMU_Yaw -= temp_param;
    // }

}


