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


//--- ����ģʽ
#define Gimbal_Debug  0

float YawVision_Ki = 5.5f;

extKalman_t Kalman_PitSpeed;
extKalman_t Kalman_PitOut;


/**
 * @brief      ��ʼ��
 * @param[in]  None
 * @retval     None
 */
Gimbal_classdef::Gimbal_classdef()
{
    //--- kp,ki,kd,ki_max,out_max,dt

#if ROBOT_ID == INFANTRY_2022_SWERVE_1
    /* Yaw ����Ƕ� */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw A��IMU */
    YawPID[A_IMU][PID_Outer].SetPIDParam(-15.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[A_IMU][PID_Outer].I_SeparThresh = 1000;
    YawPID[A_IMU][PID_Inner].SetPIDParam(16.0f, 100.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);
    YawPID[A_IMU][PID_Inner].I_SeparThresh = 5000;

    /* Yaw C��IMU */
    YawPID[C_IMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);// V 1.0
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    YawPID[C_IMU][PID_Inner].SetPIDParam(370.0f, 600.0f, 0.0f, 12000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Yaw ����C��IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 100, 2000, 0.002f);  //1.1  1.0   //fu 0.8
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
    YawPID[V_CIMU][PID_Inner].SetPIDParam(370.0f, 600.0f, 0.0f, 12000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Pit ����Ƕ� */
    PitPID[Mech][PID_Outer].SetPIDParam(0.75f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(300.0f, 500.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit �������Ƕ� */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C��IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(1.3f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75  
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(300.0f, 500.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
    /* Yaw ����Ƕ� */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw C��IMU */
    // YawPID[C_IMU][PID_Outer].SetPIDParam(1.2f, 0.0f, 0.0f, 2000, 15000, 0.002f);//
    // YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    // YawPID[C_IMU][PID_Inner].SetPIDParam(310.0f, 600.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//
    // YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    YawPID[C_IMU][PID_Outer].SetPIDParam(0.65f, 0.0f, 0.0f, 100, 2500, 0.002f);//0.78
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 80;
    YawPID[C_IMU][PID_Inner].SetPIDParam(500.0f, 600.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Yaw ����C��IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(0.65f, 0.0f, 0.0f, 100, 2000, 0.002f); 
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
    YawPID[V_CIMU][PID_Inner].SetPIDParam(500.0f, 600.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Pit ����Ƕ� */
    PitPID[Mech][PID_Outer].SetPIDParam(0.68f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(180.0f, 500.0f, 0.0f, 4000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 500;

    /* Pit �������Ƕ� */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C��IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(220.0f, 500.0f, 0.0f, 5000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;


    /* Pit ���� C��IMU */
    PitPID[V_CIMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);
    PitPID[V_CIMU][PID_Outer].I_SeparThresh = 500;
    PitPID[V_CIMU][PID_Inner].SetPIDParam(220.0f, 500.0f, 0.0f, 8000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[V_CIMU][PID_Inner].I_SeparThresh = 200;
#elif ROBOT_ID == SRNTRY_2023_OMNI
    /* Yaw ����Ƕ� */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw C��IMU */
    // YawPID[C_IMU][PID_Outer].SetPIDParam(1.2f, 0.0f, 0.0f, 2000, 15000, 0.002f);//
    // YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    // YawPID[C_IMU][PID_Inner].SetPIDParam(310.0f, 600.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//
    // YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    YawPID[C_IMU][PID_Outer].SetPIDParam(0.65f, 0.0f, 0.0f, 100, 2500, 0.002f);//0.78
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 80;
    YawPID[C_IMU][PID_Inner].SetPIDParam(500.0f, 600.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Yaw ����C��IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(0.8f, 0.0f, 0.0f, 100, 2000, 0.002f); 
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
    YawPID[V_CIMU][PID_Inner].SetPIDParam(300.0f, 0.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Pit ����Ƕ� */
    PitPID[Mech][PID_Outer].SetPIDParam(0.68f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(180.0f, 500.0f, 0.0f, 4000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 500;

    /* Pit �������Ƕ� */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C��IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(0.9f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(170.0f, 400.0f, 0.0f, 5000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;


    /* Pit ���� C��IMU */
    PitPID[V_CIMU][PID_Outer].SetPIDParam(0.8f, 0.0f, 0.0f, 2000, 15000, 0.002f);
    PitPID[V_CIMU][PID_Outer].I_SeparThresh = 500;
    PitPID[V_CIMU][PID_Inner].SetPIDParam(200.0f, 300.0f, 0.0f, 8000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[V_CIMU][PID_Inner].I_SeparThresh = 200;
#endif

    /*--- Angle Init -------------------------------------------------------------------------*/
    Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
    Target.Mech_Pit = Motor[Pit].get_totalencoder();
    // Target.AIMU_Yaw = 8888 * ENCODER_ANGLE_RATIO;//
    Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
    Target.CIMU_Pit = Get_IMUTotalAngle(Rol) * ENCODER_ANGLE_RATIO;

    //--- Yaw��ң�ؿ������ϵ��
    YawRCCtrl_Coe = 0.0009f;

    //--- ������̨��ʼģʽ
    Set_YawCtrlMode(C_IMU);
#if ROBOT_ID == INFANTRY_2022_SWERVE_1
    Set_PitCtrlMode(Mech);
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
    Set_PitCtrlMode(C_IMU);
    KalmanCreate(&Kalman_PitOut, 1, 80);
#elif ROBOT_ID == SRNTRY_2023_OMNI
    Set_PitCtrlMode(C_IMU);//�����ڱ���������ΪC_IMUȫ��
    KalmanCreate(&Kalman_PitOut, 1, 80);
#endif
    KalmanCreate(&Kalman_PitSpeed, 1, 80);
}


/**
 * @brief      ��̨�ܿغ���
 * @param[in]  None
 * @retval     None
 */
uint8_t zero_init = false;
void Gimbal_classdef::Control()
{
    //--- IMU���ݸ���
#if USE_IMU_EKF
    IMU_Update(QEKF_INS.angle, QEKF_INS.Gyro);
    init_cnt = 500;
#else
    IMU_Update(INS_angle, INS_gyro);
    init_cnt = 1300;
#endif

    //--- �ȴ�IMU��ʼ�����
    if(init_mode)
    {
        if(Get_IMUAngle(Yaw) != 0 || Get_IMUAngle(Pit) != 0 || Get_IMUAngle(Rol) != 0)
        {
            init_cnt++;
            if(--init_cnt != 0) //--- �ȴ�IMU�ȶ� 800*2ms�����ȶ�ʱ��+500*2ms������ʱ��
            {
                Set_InitAngle();
                init_mode = false;
            }
        }
        return;
    }

#if ROBOT_ID == INFANTRY_2022_SWERVE_1 
    //--- ���ݴ���
    //--- �����Pit��λ����Ĵ���
    if(zero_init == false)
    {
        if(Motor[Pit].encoder_offset > 7910)
        {
            Motor[Pit].encoder_offset = 8191;
        }
        else
        {
            Motor[Pit].encoder_offset = 0;
        }
        zero_init = true;
    }
#else
    Motor[Pit].encoder_offset = 0;
#endif

    if(Infantry.Get_GimbalMode() == Gimbal_DisableMode || Infantry.Get_State() != Robot_Activating || DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    {
        //set init angle...
        Set_InitAngle();
        
        Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out = 0;
				Shoot.Send_Motor[SendID_Pit].Out = Motor[Pit].Out = 0;
			
        return;
    }

    //--- Ŀ��Ƕȸ���
    TargetAngle_Update(CTRL_DR16.Get_ExptYaw(), CTRL_DR16.Get_ExptPit());

    //--- PID����
    Motor_PIDCalc();

    //--- ���͵���ֵ
#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 || INFANTRY_2022_SWERVE_3
//    MotorMsgSend(&hcan1, Motor);
#endif

}


/**
 * @brief      ��ȡ�����Ŀ��Ƕ�
 * @param[in]  type
 * @retval     The point of Target Angle
 */
float *Gimbal_classdef::Get_TargetAngle(Gimbal_type_e type)
{
    static float *return_angle[2];

    switch(type)
    {
    case Yaw:   //--- Yaw ------------------------
        switch(YawMode)
        {
        case Mech: return_angle[Yaw] = &Target.Mech_Yaw;
            break;
        case A_IMU: return_angle[Yaw] = &Target.AIMU_Yaw;
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
 * @brief      ��ȡ����Ĳ����Ƕ�
 * @param[in]  type
 * @retval     Current Angle 
 * @note       ���ص��Ǽ���ʱ������
 */
float Gimbal_classdef::Get_CurrentAngle(Gimbal_type_e type)
{
    static float return_angle[2];

    switch(type)
    {
    case Yaw:   //--- Yaw ------------------------
        switch(YawMode)
        {
        case Mech: return_angle[Yaw] = Motor[Yaw].get_totalencoder();
            break;
        case C_IMU: return_angle[Yaw] = Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO;
            break;
        }
        break;

    case Pit:   //--- Pit ------------------------
        switch(PitMode)
        {
        case Mech: return_angle[Pit] = Motor[Pit].get_totalencoder();
            break;
        case C_IMU: return_angle[Pit] = Get_IMUTotalAngle(Rol)*ENCODER_ANGLE_RATIO;
            break;
        }
        break;
    }

    return return_angle[type];
}

/**
 * @brief      ����Ŀ��Ƕ�
 * @param[in]  Yawparam Pitparam
 * @note       �������ֵ�ỻ�ɺ͵����ͬ������
 * @retval     None
 */
float YawDebug_target;
float PitDebug_target;
float YawDebug_tar_cs=0.03;
float YawDebug_tar_cs_turn=1.5;
float PitDebug_target_cs;
//int Enemy_VanishTime;
void Gimbal_classdef::TargetAngle_Update(float Yawparam, float Pitparam)
{
    // --- �Ӿ���Ŀ��ʱ�������ֶ�ֵ�����ǵð��Ӿ��������߼���
    if((Infantry.Get_GimbalMode() == Gimbal_PCMode || Infantry.Get_GimbalMode() == Gimbal_PatrolMode))//&& DevicesMonitor.Get_State(VISION_MONITOR) != Off_line)
    {
        if(Vision.Get_Enemy() != false)
        {
            Yawparam = Pitparam = 0;
						Target.CIMU_Yaw =Vision.Tar_Yaw_Vision;
						Target.CIMU_Pit =Vision.Tar_Pit_Vision;
						Enemy_VanishTime = 0;
        }
				else
				{
					switch(Vision.Radar_Gimbal_Mode)
					{
						case 0:Yawparam=0;Pitparam=0;break;
						case 1:
							// Ѳ��
							if(Infantry.Get_GimbalMode() == Gimbal_PatrolMode)
							{
								Enemy_VanishTime++;
								if(Enemy_VanishTime>300)
								{
									Yawparam = -0.0009*300;
									Target.CIMU_Pit = 180* ENCODER_ANGLE_RATIO;
									Pitparam = 0;
								}
							}
						break;
							
						case 2:
							// ��ǰ��ս
							if(Infantry.Get_GimbalMode() == Gimbal_PatrolMode)
							{
								Enemy_VanishTime++;
								if(Enemy_VanishTime>300)
								{
									Yawparam = -0.001*300;
									if(Enemy_VanishTime>=1300)
									{
										Target.CIMU_Pit = 160* ENCODER_ANGLE_RATIO;
										if(Enemy_VanishTime>=2600){Enemy_VanishTime=100;}
									}
									else
									{
										Target.CIMU_Pit = 145 * ENCODER_ANGLE_RATIO;
									}
								}
								Pitparam = 0;
							}
						break;
					}
				}
    }
    else if(Infantry.Get_GimbalMode() == Gimbal_PCFollowMode)
    {
			if(Vision.Radar_Recv_Msg.Pack.chassis_lx == 0 && Vision.Radar_Recv_Msg.Pack.chassis_ly == 0 && Vision.Radar_Recv_Msg.Pack.chassis_az == 0)
			{}
			else
			{
				Vision.Tar_Yaw_Radar = Target.CIMU_Yaw;
				YawDebug_tar_cs=abs(Vision.ForTurn_Yaw_Radar)/30*YawDebug_tar_cs_turn;
				if(Vision.ForTurn_Yaw_Radar>YawDebug_tar_cs)
				{
					Vision.Tar_Yaw_Radar+=YawDebug_tar_cs*ENCODER_ANGLE_RATIO;
				}
				else if(Vision.ForTurn_Yaw_Radar<-YawDebug_tar_cs)
				{
					Vision.Tar_Yaw_Radar-=YawDebug_tar_cs*ENCODER_ANGLE_RATIO;
				}
				else
				{
					Vision.Tar_Yaw_Radar += Vision.ForTurn_Yaw_Radar*ENCODER_ANGLE_RATIO;
				}
				Target.CIMU_Yaw = Vision.Tar_Yaw_Radar;
			}
    }
    else
    {
        // Shoot.ContLaunch = false;
    }



    /*--- ʵʱ������������ģʽ�ĳ�ʼ�Ƕ�ֵ,��ֹģʽ�л�ʱը�� ---------*/
    switch(YawMode)
    {
    case Mech: Target.Mech_Yaw += Yawparam * 30.0f;
               Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        break;
    case A_IMU: Target.AIMU_Yaw += Yawparam * ENCODER_ANGLE_RATIO;//
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
                Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        break;
    case C_IMU: Target.CIMU_Yaw += -(Yawparam * ENCODER_ANGLE_RATIO);
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    }

    /*--- Pit Ŀ��Ƕȸ��� -----------------------------------------*/
    switch(PitMode)
    {
    case Mech: Target.Mech_Pit += Pitparam;
               Target.CIMU_Pit = Get_IMUTotalAngle(Rol) * ENCODER_ANGLE_RATIO;
        break;
    case A_IMU: Target.AIMU_Pit += Pitparam;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
                Target.CIMU_Pit = Get_IMUTotalAngle(Rol) * ENCODER_ANGLE_RATIO;
        break;
    case C_IMU: Target.CIMU_Pit += Pitparam*1.5f;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
        break;
    }

    /*--- Pit �Ƕ����� ---------------------------------------------*/
    Pit_AngleLimit();
}



/**
 * @brief      ��̨���λ��ʽPID����
 * @param[in]  None
 * @note       �������ֵ�ỻ��ɺ͵����ͬ������
 * @retval     None
 */
float pid_yawnow_sxxs=1.6;
float pid_pitnow_sxxs=0.7;
float debug_yaw_kp = 0.8f;
float debug_pit_kp = 0.6f;
void Gimbal_classdef::Motor_PIDCalc()
{

    static float temp_wind_offset_yaw = 0.0f;
    static float temp_wind_offset_pit = 0.0f;
    
		//��ǰֵ��ȡ
    switch(YawMode)
    {
    case Mech:
        YawPID[Mech][PID_Outer].Current = Motor[Yaw].get_totalencoder();
        YawPID[Mech][PID_Inner].Current = Motor[Yaw].getSpeed();
        break;

    case A_IMU:
        // YawPID[A_IMU][PID_Outer].Current = 8888 * ENCODER_ANGLE_RATIO;
        // YawPID[A_IMU][PID_Inner].Current = 9999;
        break;

    case C_IMU:
    case V_CIMU:
        YawPID[C_IMU][PID_Outer].Current = YawPID[V_CIMU][PID_Outer].Current = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        // YawPID[C_IMU][PID_Inner].Current = YawPID[V_CIMU][PID_Inner].Current = CIMU.OffsetGyro[Yaw]/*  Get_IMUGyro(Yaw) *//* temp_gz */;

        YawPID[C_IMU][PID_Inner].Current = YawPID[V_CIMU][PID_Inner].Current = Get_IMUGyro(Yaw);//*pid_yawnow_sxxs/*  Get_IMUGyro(Yaw) *//* temp_gz */;
        break;
    }

    switch(PitMode)
    {
    case Mech:
    case V_Mech:
        PitPID[Mech][PID_Outer].Current = PitPID[V_Mech][PID_Outer].Current = Motor[Pit].get_totalencoder();
        // PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = Get_IMUGyro(Rol)/* Motor[Pit].getSpeed() */;
        // PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = KalmanFilter(&Kalman_PitSpeed, /* Get_IMUGyro(Rol) */INS_OffsetGyro[1]*(180/PI));

        PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = Get_IMUGyro(Rol);
        break;

    case A_IMU: 
        break;

    case C_IMU:
    case V_CIMU:
        PitPID[C_IMU][PID_Outer].Current = PitPID[V_CIMU][PID_Outer].Current = Get_IMUTotalAngle(Rol) * ENCODER_ANGLE_RATIO;
        // PitPID[C_IMU][PID_Inner].Current = PitPID[V_CIMU][PID_Inner].Current = /* Motor[Pit].getSpeed() */KalmanFilter(&Kalman_PitSpeed, /* Get_IMUGyro(Rol) */INS_OffsetGyro[1]*(180/PI));

        PitPID[C_IMU][PID_Inner].Current = PitPID[V_CIMU][PID_Inner].Current = Get_IMUGyro(Rol)*pid_pitnow_sxxs;
        break;
    }
    /*--- Yaw PID Calc --------------------------------------------------------------------------*/
    // YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
    // YawPID[YawMode][PID_Inner].Target = YawPID[YawMode][PID_Outer].Cal();
    // Motor[Yaw].Out = YawPID[YawMode][PID_Inner].Cal();

    /*--- Pit PID Calc --------------------------------------------------------------------------*/
    if(Infantry.Get_GimbalMode() == Gimbal_PCMode || Infantry.Get_GimbalMode() == Gimbal_PatrolMode)//[5][0]/[5][1]  V_CIMU�Ӿ�����pid
    {
        /*--- Yaw PID Calc --------------------------------------------------------------------------*/
        YawPID[YawMode+3][PID_Outer].Target = *Get_TargetAngle(Yaw) + temp_wind_offset_yaw;
        YawPID[YawMode+3][PID_Inner].Target = YawPID[YawMode+3][PID_Outer].Cal();
//				if(abs(YawPID[YawMode+3][PID_Outer].Error)<180*ENCODER_ANGLE_RATIO)
//				{
//					YawPID[YawMode+3][PID_Inner].Current *= (abs(YawPID[YawMode+3][PID_Outer].Error)/(180*ENCODER_ANGLE_RATIO)*0.7+1.3);
//				}
//				else{YawPID[YawMode+3][PID_Inner].Current *= 2;}
        Motor[Yaw].Out = YawPID[YawMode+3][PID_Inner].Cal();

        /*--- Pit PID Calc --------------------------------------------------------------------------*/
        PitPID[PitMode+3][PID_Outer].Target = *Get_TargetAngle(Pit) + temp_wind_offset_pit;
        PitPID[PitMode+3][PID_Inner].Target = PitPID[PitMode+3][PID_Outer].Cal();
        Motor[Pit].Out = PitPID[PitMode+3][PID_Inner].Cal() /* + (3.5873f*PitPID[PitMode+3][PID_Outer].Current) */;

    }
    else																																														//[2][0]/[2][1]  C_IMU����Pid
    {
        /*--- Yaw PID Calc --------------------------------------------------------------------------*/
        YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
        YawPID[YawMode][PID_Inner].Target = YawPID[YawMode][PID_Outer].Cal();
        Motor[Yaw].Out = YawPID[YawMode][PID_Inner].Cal();

        /*--- Pit PID Calc --------------------------------------------------------------------------*/
        PitPID[PitMode][PID_Outer].Target = *Get_TargetAngle(Pit);
        PitPID[PitMode][PID_Inner].Target = PitPID[PitMode][PID_Outer].Cal();
        Motor[Pit].Out = PitPID[PitMode][PID_Inner].Cal() /* + (3.5873f*PitPID[PitMode][PID_Outer].Current) */;

        // YawPID[YawMode+3][PID_Outer].Reset();
        // YawPID[YawMode+3][PID_Inner].Reset();
        // PitPID[PitMode+3][PID_Outer].Reset();
        // PitPID[PitMode+3][PID_Inner].Reset();
    }
		
//		if(Chassis.Mode == CHAS_SpinMode)
//		{
//			
//			if(Chassis.Get_TargetVw()>9000)
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out - 10000;
//			}
//			if(Chassis.Get_TargetVw()<-9000)
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out + 10000;
//			}
//			else
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out -	Chassis.Get_TargetVw()/9000*10000;
//		
//			}
//		}
//		else
//		{
//			Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out;
//		}
		Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out;
		Shoot.Send_Motor[SendID_Pit].Out = Motor[Pit].Out;//Pitch�����shoot�ĵ���ĵ���Ľ���idһ�����Դ������shoot����ͳһ����
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
        // Target.AIMU_Yaw = 8888 * ENCODER_ANGLE_RATIO;
        Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw);
        break;

    case A_IMU:
        YawMode = A_IMU;
        Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;

    case C_IMU:
        YawMode = C_IMU;
        Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
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
        Target.CIMU_Pit = Get_IMUTotalAngle(Rol);
        break;

    case A_IMU:
        PitMode = A_IMU;
        break;

    case C_IMU:
        PitMode = C_IMU;
        Target.Mech_Pit = Motor[Pit].get_totalencoder();
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
    case A_IMU: Target.AIMU_Yaw = 8888;//
        break;
    case C_IMU: Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO;
        break;
    }

    switch(PitMode)
    {
    case Mech: Target.Mech_Pit = (init_mode == true?GIMBAL_PIT_CENTRE:Motor[Pit].get_totalencoder());
        break;
    case A_IMU: Target.AIMU_Pit = 7777;
        break;
    case C_IMU: Target.CIMU_Pit = (init_mode == true?Get_IMUTotalAngle(Rol)-(Motor[Pit].get_totalencoder()-GIMBAL_PIT_CENTRE):Get_IMUTotalAngle(Rol) * ENCODER_ANGLE_RATIO);
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
    static float Limit_Min;
    static float Limit_Max;

    switch(PitMode)
    {
    case Mech:
        Limit_Min = GIMBAL_PIT_MIN;
        Limit_Max = GIMBAL_PIT_MAX;
        break;

    case C_IMU:
        //--- ������ģʽ������ֵ��Ϊ�ο� ӳ�䵽IMU�ǶȽ��нǶ�����
        Limit_Min = Get_IMUTotalAngle(Rol)*ENCODER_ANGLE_RATIO - fabs(Get_PitDeviateL());
        Limit_Max = Get_IMUTotalAngle(Rol)*ENCODER_ANGLE_RATIO + fabs(Get_PitDeviateH());
        break;
    }

    if(*Get_TargetAngle(Pit) > Limit_Max)
    {
        *Get_TargetAngle(Pit) = Limit_Max;
    }
    else if(*Get_TargetAngle(Pit) < Limit_Min)
    {
        *Get_TargetAngle(Pit) = Limit_Min;
    }
}


/**
 * @brief      ��ȡ��ǰλ����Yaw�����ĵ��ƫ��Ƕ�
 * @param[in]  cur_encoder
 * @retval     Off angle(������)
 */
float Gimbal_classdef::Get_YawDeviateC(float cur_encoder)
{
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
    static float Separate_diff = 0;

    if(Switch == OFF || Infantry.Get_CtrlSource() != CTRL_RC || (Infantry.Get_ChassisMode() != CHAS_FollowMode && Infantry.Get_ChassisMode() != CHAS_ReFollowMode))
    {
        return;
    }

    Separate_diff = Gimbal.Motor[Yaw].getAngle();

    if(Separate_diff > 300)
    {
        Separate_diff -= 360.0f;
    }
    else if(Separate_diff < -300)
    {
        Separate_diff += 360.0f;
    }

    if(fabs(Separate_diff) > 65.0f && *yawparam != 0)
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

    // if(fabs(Motor[Yaw].getencoder() - GIMBAL_YAW_CENTER) > 1000)
    // {
    //     Target.AIMU_Yaw -= temp_param;
    // }
		
}


/**
 * @brief      ��ȡIMU�Ƕ�(�Ƕ���)
 * @param[in]  type
 * @retval     Angle
 */
void Gimbal_classdef::IMU_Update(float *angle, float *gyro)
{
#if USE_IMU_EKF

    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- �Ƕ�
        CIMU.Angle[i] = angle[i] + 180; //--- (0 ~ 360��)

        //--- ���㴦��,�ۼƽǶ�
        if(CIMU.Angle[i] - CIMU.Pre_Angle[i] < -180)
        {
            CIMU.Cnt[i]++;
        }
        else if(CIMU.Angle[i] - CIMU.Pre_Angle[i] > 180)
        {
            CIMU.Cnt[i]--;
        }

        CIMU.TotalAngle[i] = CIMU.Angle[i] + CIMU.Cnt[i] * 360.0f;
        CIMU.Pre_Angle[i] = CIMU.Angle[i];
    }
    //--- ���ٶ�
    CIMU.Gyro[0] = gyro[2]*(180/PI);
    CIMU.Gyro[1] = gyro[0]*(180/PI);
    CIMU.Gyro[2] = LPF_Gyro_y*(180/PI);//gyro[1]*(180/PI);


#else
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- �Ƕ�
        CIMU.Angle[i] = angle[i]*(180/PI) + 180; //--- ����ת��Ϊ��(0 ~ 360��)
        //--- ���ٶ�
        CIMU.Gyro[i] = gyro[2-i]*(180/PI);
        //--- �����Ľ��ٶ� Yaw
        CIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);

        //--- ���㴦��,�ۼƽǶ�
        if(CIMU.Angle[i] - CIMU.Pre_Angle[i] < -180)
        {
            CIMU.Cnt[i]++;
        }
        else if(CIMU.Angle[i] - CIMU.Pre_Angle[i] > 180)
        {
            CIMU.Cnt[i]--;
        }

        CIMU.TotalAngle[i] = CIMU.Angle[i] + CIMU.Cnt[i] * 360.0f;
        CIMU.Pre_Angle[i] = CIMU.Angle[i];
    }
#endif

}

/**
 * @brief      ��ȡIMU�Ƕ�(�Ƕ���)
 * @param[in]  type
 * @retval     Angle
 */
float Gimbal_classdef::Get_IMUAngle(Gimbal_type_e type)
{
    return CIMU.Angle[type];
}

/**
 * @brief      ��ȡIMU�ۼƽǶ�(�Ƕ���)
 * @param[in]  type
 * @retval     TotalAngle
 */
float Gimbal_classdef::Get_IMUTotalAngle(Gimbal_type_e type)
{
    return CIMU.TotalAngle[type];
}

/**
 * @brief      ��ȡIMU���ٶ�
 * @param[in]  type
 * @retval     Gyro
 */
float Gimbal_classdef::Get_IMUGyro(Gimbal_type_e type)
{
    return CIMU.Gyro[type];
}




