/**
 ------------------------------------------------------------------------------
 * @file    Control_Chassis.cpp
 * @author  Shake
 * @brief   ���̿���
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
#include "kalman_Filter.h"
// #include "Chassis_Power.h"

// CHAS_Power_classdef CHAS_Power;

/* Private define ------------------------------------------------------------*/
#define USE_RUDDER_RESET 1

#define CHASSIS_MAX_SPEED 9000  // ��������������ٶ�
#define CHASSIS_MAX_VW    8000  // ������ת����ٶ�
#define CHASSIS_SPEED_L   5000
#define CHASSIS_SPEED_M   6000
#define CHASSIS_SPEED_H   9000

float Radius = 1.0f;  // Բ�ľ�

extKalman_t Kalman_CHASFollow_Speed;

/* Private function declarations ---------------------------------------------*/
float Acclerate(float x , float k);
float Decclerate(float x , float k);

/**
 * @brief      ��ʼ��
 * @param[in]  None
 * @retval     None
 */
Chassis_classdef::Chassis_classdef()
{
    /*--- ����ģʽ PID -----------------------------------------------------------------------*/
    Follow_PID[PID_Outer].SetPIDParam(2800.0f/* 6500.0f */, 0.0f, 0.0f, 2000, 6000/* 4500 */, 0.002f);//3200 5000
    Follow_PID[PID_Outer].I_SeparThresh = 1000;
		Follow_PID[PID_Inner].SetPIDParam(0.65f, 0.0f, 0.0f, 2000, 5500, 0.002f);  //0.45  0.1
    Follow_PID[PID_Inner].I_SeparThresh = 9000;
//    Follow_PID[PID_Outer].SetPIDParam(4800.0f/* 6500.0f */, 0.0f, 0.0f, 2000, 6000/* 4500 */, 0.002f);//3200 5000
//    Follow_PID[PID_Outer].I_SeparThresh = 1000;
//    Follow_PID[PID_Inner].SetPIDParam(0.65f, 0.0f, 0.0f, 2000, 5500, 0.002f);  //0.45  0.1
//    Follow_PID[PID_Inner].I_SeparThresh = 9000;

    Swing_PID[PID_Outer].SetPIDParam(-5200.0f/* 4500.0f */, 1.0f, 0.0f, 2000, 4500, 0.002f);//7200
    Swing_PID[PID_Outer].I_SeparThresh = 1000;

    AutoTrack_PID[PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 0, 5500, 0.002f);
    AutoTrack_PID[PID_Outer].I_SeparThresh = 1000;

    VxVy_Coe = 1.0f;
    ReF_Flag = false;

    //--- ���̸���ģʽ��˫��ģʽ���ڻ�����ֵ�˲�
    KalmanCreate(&Kalman_CHASFollow_Speed, 1, 20);

}


/**
 * @brief      �����ܿ��ƺ���
 * @param[in]  None
 * @retval     None
 */
float xy_jiasudu=200;
void Chassis_classdef::Control()
{
    // SupCap Ctrl...
		if(Infantry.Chassis_Mode == CHAS_SentryPCMode)
		{
			switch(Vision.Radar_Chassis_Mode)
			{
				case 0:
					 Process(Vision.Radar_Recv_Msg.Pack.chassis_ly*xy_jiasudu, \
					Vision.Radar_Recv_Msg.Pack.chassis_lx*xy_jiasudu,\
					0);
				break;
				
				case 1:
					 Process(Vision.Radar_Recv_Msg.Pack.chassis_ly*xy_jiasudu, \
					Vision.Radar_Recv_Msg.Pack.chassis_lx*xy_jiasudu,\
					4500);
				break;
				
				case 2:
					 Process(Vision.Radar_Recv_Msg.Pack.chassis_ly*xy_jiasudu, \
					Vision.Radar_Recv_Msg.Pack.chassis_lx*xy_jiasudu,\
					9000);
				break;
			}
		}
    else if(Infantry.Chassis_Mode != CHAS_LockMode)
    {
        Process(CTRL_DR16.Get_ExptVx(), CTRL_DR16.Get_ExptVy(), CTRL_DR16.Get_ExptVw());
    }
    else
    {
        Process(0,0,0);
        Infantry.Chassis_PreMode = CHAS_LockMode;
    }

}


/**
 * @brief      �������ݴ���
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @retval     None
 */
int16_t drv_tempcurrent[4];
int16_t rud_tempcurrent[4];
uint8_t Move_flag;
void Chassis_classdef::Process(float Vx, float Vy, float Vw)
{
    // static float Ramp_Vy = 0, Ramp_Vx = 0;

    //--- Yaw�����ĵ�
    Gimbal.Motor[Yaw].encoder_offset = GIMBAL_YAW_CENTRE;
    // Gimbal.Motor[Yaw].encoder_offset = (Infantry.Get_ChassisMode() == CHAS_ReFollowMode?GIMBAL_YAW_RECENTRE:GIMBAL_YAW_CENTRE);

    //--- ���õ�������ٶ�
    // Set_MaxSpeed();

    //--- ����״̬�쳣
    if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line || Infantry.Get_ChassisMode() == CHAS_DisableMode || Gimbal.init_mode != false)
    {
        Target_Vx = Target_Vy = Target_Vw = 0;
        Infantry.Chassis_PreMode = CHAS_DisableMode;
        return;
    }

    if(Infantry.Get_ChassisMode() != CHAS_SpinMode)
    {
        // AcclerateCurve(&Vx, &Vy); //--- �������� ������
    }

    //--- ����ģʽѡ��
    switch(Infantry.Get_ChassisMode())
    {
			case CHAS_SentryPCMode:
//				Sentry_Follow_Ctrl(&Vx, &Vy, &Vw);
				Sentry_Spin_Ctrl(&Vx, &Vy, &Vw);
			break;
			
    case CHAS_FollowMode: //--- ����ģʽ
        Follow_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_ReFollowMode: //--- �������ģʽ
        ReFollow_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_SpinMode: //--- С����
    case CHAS_sinSpinMode:
        Spin_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_SwingMode: //---Ť��
        Swing_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_AutoTrackMode: //---׷��װ�װ�ģʽ
        AutoTrack_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_NotFollowMode: //---�޸���ģʽ
        NotFollwe_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_LockMode:
        break;
		
    case CHAS_SentryFollowMode: //--- �������ģʽ
//		SentryFollow_Ctrl(&Vx, &Vy, &Vw);
        break;

    case CHAS_SentrySpinMode: //--- С����
//        SentrySpin_Ctrl(&Vx, &Vy, &Vw);
        break;
		

    }

    /*Low Pass Filter*/
    // Vx = Vx_LPF.f(Vx);
    // Vy = Vy_LPF.f(Vy);
    // Vw = Vw_LPF.f(Vw);
    // if(Infantry.Get_ChassisMode() != CHAS_SpinMode)
    // {
    //     AcclerateCurve(&Vx, &Vy); //--- �������� ������
    // }

    Target_Vx = Vx;
    Target_Vy = Vy;
    Target_Vw = Vw;

    // Target_Vx = Vx;
    // Target_Vy = Vy;
    // Target_Vw = Vw;

}


/**
 * @brief      ȫ���ƶ��ٶȷֽ�
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
 * @brief      ���ֽ���
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
		// SupCap.SwerveOffset_Flag = false;
	}

#if USE_RM_Referee
	//�ٶ�����
    Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));

#else
	// �ٶ�����
	Constrain(&Vx, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-CHASSIS_MAX_SPEED * VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-CHASSIS_MAX_SPEED ), (int16_t)(CHASSIS_MAX_SPEED));
#endif

	//�����ٶȷֽ�
	tempSpeed[0] = Vx - Vy + Vw;
	tempSpeed[1] = Vx + Vy + Vw;
	tempSpeed[2] = -Vx + Vy + Vw;
	tempSpeed[3] = -Vx - Vy + Vw;

	//Ѱ������ٶ�
	for (uint8_t i = 0; i < 4; i++)
	{
		if (fabs(tempSpeed[i]) > MaxSpeed)
		{
			MaxSpeed = fabs(tempSpeed[i]);
		}
	}

	//�ٶȷ���
	if (MaxSpeed > VxVy_Limit)
	{
		Param = (float)VxVy_Limit / MaxSpeed;
	}

	cal_speed[0] = tempSpeed[0] * Param;
	cal_speed[1] = tempSpeed[1] * Param;
	cal_speed[2] = tempSpeed[2] * Param;
	cal_speed[3] = tempSpeed[3] * Param;
}


/**
 * @brief      ���ֽ���
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @param[in]  cal_speed 
 * @retval     None
 */
uint16_t Movecount_flag;
void Chassis_classdef::Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float MaxSpeed;
    float Param = 1.0f;

    static float const theta = atan(1.0/1.0);
    // static uint8_t compare_flag = true;

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
    // if(abs(Vx) < 250)
    // {
    //     Vx = 0;
    // }
    // if(abs(Vy) < 250)
    // {
    //     Vy = 0;
    // }

    // if(abs(Vw) < 30)
    // {
    //     Vw = 0;
    // }

    
    /* �ٶ����� ---------------------------------------------------------------------------------------*/
#if USE_RM_Referee
    Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
	Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));
#else
	Constrain(&Vx, (int16_t)-(CHASSIS_MAX_SPEED*VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED*VxVy_Coe));
	Constrain(&Vy, (int16_t)-(CHASSIS_MAX_SPEED*VxVy_Coe), (int16_t)(CHASSIS_MAX_SPEED*VxVy_Coe));
	Constrain(&Vw, (int16_t)-(CHASSIS_MAX_SPEED), (int16_t)(CHASSIS_MAX_SPEED));
#endif
    
    if(Vx == 0 && Vy == 0)
    {
        if(abs(Vw) < 80)
        {
            Vw = 0;
        }
    }

    /* �����ٶȽ���ת���ֽǶ� ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);

    /* ������ �ٶȽ��� ---------------------------------------------------------------------------------*/
    cal_speed[RF_201] = -sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
    cal_speed[LF_202] =  sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[LB_203] =  sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[RB_204] = -sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
    

    //Ѱ������ٶ�
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(cal_speed[i]) > MaxSpeed)
		{
			MaxSpeed = abs(cal_speed[i]);
		}
	}

	//�ٶȷ���
	if (MaxSpeed > VxVy_Limit)
	{
		Param = (float)VxVy_Limit / MaxSpeed;
	}

	cal_speed[RF_201] = cal_speed[RF_201] * Param;
	cal_speed[LF_202] = cal_speed[LF_202] * Param;
	cal_speed[LB_203] = cal_speed[LB_203] * Param;
	cal_speed[RB_204] = cal_speed[RB_204] * Param;


    /* ����ת���ֽǶȵ�ǰֵ ----------------------------------------------------------------------------*/
    RUDTotalAngle_Calc(RUD_Motor+RF_205, RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+LF_206, LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+LB_207, LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor+RB_208, RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    /* ����ת���ֽǶ�Ŀ��ֵ ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        // Judge_DRV_Dir(i, RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
        //--- ת���� �ӻ���ת
        RUD_Param[i].Target_angle = Turn_InferiorArc(i, RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
    }

}

/**
 * @brief      �����ٶȽ����Ƕ�
 * @param[in]  None
 * @retval     None
 */
void Chassis_classdef::RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw)
{
    static float const theta = atan(1.0/1.0);

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        Move_flag = false;

        //--- ��45��ʹ��Ĭ��״̬�����ӳ���ת�Ƕ�
        RUD_Param[RF_205].Init_angle = (-45)-152.85;
        RUD_Param[LF_206].Init_angle = (+45)-88.9;
        RUD_Param[LB_207].Init_angle = (-45)+34.27;
        RUD_Param[RB_208].Init_angle = (+45)+151.69;
    }
    else
    {   //--- ת���ֽǶ�,������
        Move_flag = true;

        RUD_Param[RF_205].Init_angle = -152.85;
        RUD_Param[LF_206].Init_angle = -88.9;
        RUD_Param[LB_207].Init_angle = 34.27;
        RUD_Param[RB_208].Init_angle = 151.69;
    }

    //--- ת���ֽǶȽ���
    RUD_Param[RF_205].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)), Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
    RUD_Param[LF_206].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)), Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
    RUD_Param[LB_207].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)), Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
    RUD_Param[RB_208].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)), Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);


    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        RUD_Param[i].PreTar_angle = RUD_Param[i].Target_angle;
        // RUD_Param[i].Target_angle *= (180/PI);
    }

}

/**
 * @brief      ת���ֵ�ǰ�ܽǶȼ���
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

    if(reset == true)  //---Ȧ������
    {
        RUD_Param[motor_num].Turns_cnt = 0;
    }

    if(opposite == false)   //---�Ƿ���
    {
        RUD_Param[motor_num].Total_angle = Cur_encoder + RUD_Param[motor_num].Turns_cnt*360;
    }
    else if(opposite == true)
    {
        RUD_Param[motor_num].Total_angle = -Cur_encoder - RUD_Param[motor_num].Turns_cnt*360;
    }

}


/**
 * @brief      ת����Ŀ��Ƕȼ���
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
    // static float Pre_Target[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};
    static float Pre_TempTarget[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};
    // static int turns_cnt = 0;


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
    // if(reset == true)   //--- Ȧ������
    // {
    //     RUD_Param[motor_num].TarTurns_cnt = 0;
    // }
    //
    // if(opposite == false)    //---�Ƿ���
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
    //--- ��Ŀ��Ƕ��뵱ǰ�ǶȻ��㵽ͬһ��������
    RUD_Param[motor_num].TarTurns_cnt = (int)(RUD_Param[motor_num].Total_angle/180.0f) - (int)(RUD_Param[motor_num].Total_angle/360.0f);
    RUD_Param[motor_num].Target_angle = 360.0f*RUD_Param[motor_num].TarTurns_cnt + Cur_Target + RUD_Param[motor_num].Init_angle;

    //---- ��ǰĿ��ǶȺ͵�ǰ�Ƕȵ����
    error = RUD_Param[motor_num].Target_angle - RUD_Param[motor_num].Total_angle;

    //--- ����޷�
    // Constrain(&error, -180.0f, 180.0f); //--- �������޷����������е�����
    AngleLimit(&error);

    //--- ����Ƕ�������90�����ٶȷ��򲢽�Ŀ��Ƕȵ���180
    if(fabs(error) > 90.0f && Move_flag == true)
    {
        //--- Ŀ��ֵ���Ӱ������
        RUD_Param[motor_num].Target_angle += 180.0f;
        //--- �����ַ�ת
        Cal_Speed[motor_num] = -Cal_Speed[motor_num];
        //--- ȷ��Ŀ��ǶȺ͵�ǰ�Ƕȴ���ͬһ������
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

}

/**
 * @brief      ������Сƫ��,ȷ�����̸��淽��
 * @param[in]  target
 * @param[in]  current
 * @retval     Error(������)
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
 * @brief      ������Сƫ�ʹת���ֱ����ӻ���ת
 * @param[in]  target
 * @param[in]  current
 * @retval     target(�Ƕ���)
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
 * @brief      �ж��������Ƿ���Ҫ��ת
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
 * @brief      ��������ٶ�����
 * @param[in]  None
 * @retval     None
 */
void Chassis_classdef::Set_MaxSpeed()
{
// #if USE_RM_Referee
//     // if (SupCap.SendData.is_cap_output == OFF || SupCap.Monitor.OffLine == true)
// 	// {
// 		if (Referee.GameRobotState.max_chassis_power < 60 /* && Referee.GameRobotState.max_chassis_power != 0 */)
// 		{
// 			VxVy_Limit = 4000.0f/* Chassis_SpeedNormal */;
// 			Vw_Limit =  4500.0f ;
// 		}
// 		else if(Referee.GameRobotState.max_chassis_power == 60)
// 		{
// 			VxVy_Limit = CHASSIS_SPEED_L;
// 			Vw_Limit = 5000.0f;
// 		}
// 		else if (Referee.GameRobotState.max_chassis_power > 60 && Referee.GameRobotState.max_chassis_power <= 80)
// 		{
// 			VxVy_Limit = CHASSIS_SPEED_M;
// 			Vw_Limit = 6000.0f;
// 		}
// 		else if (Referee.GameRobotState.max_chassis_power > 80 /* && Referee.GameRobotState.max_chassis_power <= 120 */)
// 		{
// 			VxVy_Limit = CHASSIS_SPEED_H;
// 			Vw_Limit = 7500.0f;
// 		}

// 		// if(PowerFullyOpen == ON)  // --- ���湦��ȫ��
// 		// {
// 		// 	CHASSIS.Limit.V_Normal = 6000.0f;
// 		// 	CHASSIS.Limit.V_Spin = 6000.0f;
// 		// 	CHASSIS.Limit.V_Swing = 6000.0f;
// 		// }

// 	// }
//     // else
// 	// {
// 	// 	if(Uphill_Flag == true)
// 	// 	{
// 	// 		CHASSIS.Limit.V_Normal = 6500.0f;
// 	// 		CHASSIS.Limit.V_Spin = 6000.0f;
// 	// 		CHASSIS.Limit.V_Swing = 6000.0f;
// 	// 	}
// 	// 	else
// 	// 	{
// 	// 		CHASSIS.Limit.V_Normal = 7000.0f;
// 	// 		CHASSIS.Limit.V_Spin = 7000.0f;
// 	// 		CHASSIS.Limit.V_Swing = 6000.0f;
// 	// 	}
// 	// }
// #else
//     if(Lspeed_Flag == true)
// 	{
// 		VxVy_Limit = 3000.0f;
// 		Vw_Limit = 3000.0f;
// 	}
// 	else
// 	{
// 		VxVy_Limit = 9000.0f;
// 		Vw_Limit = 9000.0f;
// 	}
// #endif
}

/**
  * @brief  	����ģʽ
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
float temp_spin;
uint16_t ramp_count = 0;
uint8_t flag_45;
void Chassis_classdef::Follow_Ctrl(float *Vx, float *Vy, float *Vw)
{
    static uint8_t done_cnt; // ������ɼ�ʱ
    static uint8_t warning_flag = false; // ���̾����־λ
    static float Vw_temp = 0;
    float Vw_tempout = 0;
    float offset_angle = 0.0f;

    temp_spin = 0;

    //--- ��ֹ�ڵ��̴��ڷ��˶�ģʽʱCH3��ס�����л�ģʽʱ�������������
    if((Infantry.Get_ChassisPreMode() == CHAS_LockMode && DR16.Get_LY_Norm() == -660) || warning_flag == true)
    {
        //--- CH3�����������ƣ�Ҳ�����ж�����ҡ��Ϊ���������ƣ�
        if(warning_flag == true && DR16.Get_LY_Norm() == 0)
        {
            warning_flag = false;
            return;
        }

        *Vx = *Vy = *Vw = 0;
        warning_flag = true;

        return;
    }

    // ����������----------------------------------------------------------
    if(ReF_Flag == true)
    {
        ReF_Cnt++;

        Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()), Vx, Vy);

        if (fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE) <= 100.0f)
		{
            if(++done_cnt > 30)
            {
    			ReF_Flag = false;
                done_cnt = 0;
            }
		}
		if (ReF_Cnt > 500 && ReF_Flag != false) // --- ��ʱδת����ֱ��ת��
		{
			ReF_Flag = false;
			ReF_Cnt = 0;
            done_cnt = 0;
		}
    }
    else //--- ��ͨ����ģʽ
    {
        
        ReF_Cnt = 0;

        Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()), Vx, Vy);

        if(fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE) > 5 || NoMiddle_Mode == true)
        {
            // follow pid calc 
            offset_angle = Gimbal.Motor[Yaw].getAngle();
            if(offset_angle > 180)
            {
                offset_angle -= 360.0f;
            }
						else if(offset_angle < -180)
						{
                offset_angle += 360.0f;
						}
//            else if(offset_angle < -250)
//            {
//                offset_angle += 360.0f;
//            }

//            offset_angle = (NoMiddle_Mode == true ? offset_angle+45 : offset_angle);

            // follow pid calc
            // KalmanFilter(&Kalman_CHASFollow_Speed, Gimbal.Motor[Yaw].getAngle());
            Follow_PID[PID_Outer].Target = 0;
            Follow_PID[PID_Outer].Current = Gimbal.Get_YawDeviateC(offset_angle/* Gimbal.Motor[Yaw].getAngle() */);

            // Follow_PID[PID_Inner].Target = Follow_PID[PID_Outer].Cal();;
            // Follow_PID[PID_Inner].Current = Gimbal.CIMU.OffsetGyro[Yaw];
            // Vw_tempout += Follow_PID[PID_Inner].Cal();
            // Vw_tempout += Follow_PID[PID_Outer].Cal();
            Vw_tempout = KalmanFilter(&Kalman_CHASFollow_Speed, Follow_PID[PID_Outer].Cal())/*  + *Vw */;

            if(IsMove() != true && fabs(Vw_tempout) < 50)
            {
                Vw_tempout = 0;
            }

        }

#if ROBOT_ID == INFANTRY_2021
        //--- ת��ʱ��Vx��Vy�������ƣ��ڹ������Ƶ�����¿��Ը���ת��
        if(fabs(*Vw) > 1200)
        {
            VxVy_Coe = ((CHASSIS_MAX_VW - fabs(*Vw) + 1200) * (CHASSIS_MAX_VW - fabs(*Vw) + 1200)) / (CHASSIS_MAX_VW * CHASSIS_MAX_VW);
            Constrain(&VxVy_Coe, 0.2f, 1.0f);
        }
        else
        {
            VxVy_Coe = 1.0f;
        }

#endif
        //С���ݸ�λ��ʱ����һС��б�£���ֹ�����������ĥ�����
        if(Infantry.Get_ChassisPreMode() == CHAS_SpinMode && ramp_count < 300)
        {   
            ramp_count++;
            Vw_temp = RAMP_Output(Vw_tempout, Vw_temp, 15.0f);
            *Vw = Vw_temp;
            return;
        }
    }
    *Vw = Vw_tempout;
    ramp_count = 0;
    Vw_temp = 0;
    Infantry.Chassis_PreMode = CHAS_FollowMode;
}

void Chassis_classdef::Sentry_Follow_Ctrl(float *Vx, float *Vy, float *Vw)
{
    static float Vw_temp = 0;
    float Vw_tempout = 0;
    float offset_angle = 0.0f;
	
		Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()+Vision.Radar_Recv_Msg.Pack.chassis_forward), Vx, Vy);

		if(fabs(Gimbal.Motor[Yaw].getencoder()+Vision.Radar_Recv_Msg.Pack.chassis_forward - GIMBAL_YAW_CENTRE) > 5 || NoMiddle_Mode == true)
		{
				// follow pid calc 
				offset_angle = Gimbal.Motor[Yaw].getAngle()-Vision.Radar_Recv_Msg.Pack.chassis_forward;
				if(offset_angle > 180)
				{
						offset_angle -= 360.0f;
				}
				else if(offset_angle < -180)
				{
						offset_angle += 360.0f;
				}

				Follow_PID[PID_Outer].Target = 0;
				Follow_PID[PID_Outer].Current = Gimbal.Get_YawDeviateC(offset_angle/* Gimbal.Motor[Yaw].getAngle() */);

				Vw_tempout = KalmanFilter(&Kalman_CHASFollow_Speed, Follow_PID[PID_Outer].Cal())/*  + *Vw */;

				if(IsMove() != true && fabs(Vw_tempout) < 50)
				{
						Vw_tempout = 0;
				}

		}


		//С���ݸ�λ��ʱ����һС��б�£���ֹ�����������ĥ�����
		if(Infantry.Get_ChassisPreMode() == CHAS_SpinMode && ramp_count < 300)
		{   
				ramp_count++;
				Vw_temp = RAMP_Output(Vw_tempout, Vw_temp, 15.0f);
				*Vw = Vw_temp;
				return;
		}
    *Vw = Vw_tempout;
    ramp_count = 0;
    Vw_temp = 0;
    Infantry.Chassis_PreMode = CHAS_SentryFollowMode;
}


/**
  * @brief  	�������ģʽ
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
 float offset_angle;
void Chassis_classdef::ReFollow_Ctrl(float *Vx, float *Vy, float *Vw)
{
    static uint8_t done_cnt; //������ɼ�ʱ
    temp_spin = 0;

    // float offset_angle = 0.0f;

    // ����������----------------------------------------------------------
    if(ReF_Flag == true)
    {
        ReF_Cnt++;

        // Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()-(GIMBAL_YAW_CENTRE-4096)/ENCODER_ANGLE_RATIO), Vx, Vy);

        if (fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE + 4066) <= 100.0f)
		{
            if(++done_cnt > 30)
            {
			    ReF_Flag = false;
                done_cnt = 0;
            }
		}
		if (ReF_Cnt > 500 && ReF_Flag != false) // --- ��ʱδת����ֱ��ת��
		{
			ReF_Flag = false;
			ReF_Cnt = 0;
            done_cnt = 0;
		}
    }
    // ���������� END----------------------------------------------------
    else
    {
        ReF_Cnt = 0;

        if(NoMiddle_Mode == true)
        {
            offset_angle += 45.0f;
            flag_45 = true;
        }

        Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()+180), Vx, Vy);

        //--- �����������ĵ�Ϊ����β�����ĵ�
        // if(fabs(Gimbal.Motor[Yaw].getencoder() - GIMBAL_YAW_CENTRE + 4096) > 5)
        // offset_angle = (NoMiddle_Mode == true ? offset_angle+45 : offset_angle);

        if(fabs(Gimbal.Motor[Yaw].getAngle())+180 > 0.22f || NoMiddle_Mode == true)
        {
            flag_45 = false;
            // follow pid calc 
            offset_angle = Gimbal.Motor[Yaw].getAngle()+180;

            offset_angle = (NoMiddle_Mode == true ? offset_angle+45 : offset_angle);

            if(offset_angle > 250)
            {
                offset_angle -= 360.0f;
            }
            else if(offset_angle < -250)
            {
                offset_angle += 360.0f;
            }


            Follow_PID[PID_Outer].Target = Gimbal.Get_YawDeviateC(offset_angle); /* Get_MinDeviation(Gimbal.Motor[Yaw].get_totalencoder()+4096, GIMBAL_YAW_CENTER); */
            Follow_PID[PID_Outer].Current = 0;

            // *Vw += KalmanFilter(&Kalman_CHASFollow_Speed, Follow_PID[PID_Outer].Cal());
            *Vw += Follow_PID[PID_Outer].Cal();
        }

#if ROBOT_ID == INFANTRY_2021
        //--- ת��ʱ��Vx��Vy�������ƣ��ڹ������Ƶ�����¿��Ը���ת��
        if(fabs(*Vw) > 1200) 
        {
            VxVy_Coe = ((CHASSIS_MAX_VW - fabs(*Vw) + 1200) * (CHASSIS_MAX_VW - fabs(*Vw) + 1200)) / (CHASSIS_MAX_VW * CHASSIS_MAX_VW);
            Constrain(&VxVy_Coe, 0.2f, 1.0f);
        }
        else
        {
            VxVy_Coe = 1.0f;
        }
#endif
        
    }

    Infantry.Chassis_PreMode = CHAS_ReFollowMode;
}

/**
  * @brief  	С����ģʽ
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
float debug_movespeed = 800;
float debug_min_vwcoe = 0.6f;
float Vw_coe = 1.0f;
void Chassis_classdef::Spin_Ctrl(float *Vx, float *Vy, float *Vw)
{
    float move_speed = 0.0f;
    // static float Vw_coe = 1.0f;
    // static float sin_max = 0.3f;
    // static float sinOmiga = 0.0131415f;
    // static float Spin_Param = 1.0f;
    static int32_t sinspin_cnt = 0;
    float offset_angle = 0.0f;

    if(Infantry.Get_ChassisPreMode() != CHAS_SpinMode)
    {
        sinspin_cnt = 0;
    }
    sinspin_cnt++;

    if (Get_PowerLimit() < 60)
    {
        VxVy_Limit = 4500.0f;
        Vw_Limit =  4500.0f ;
    }
    else if(Get_PowerLimit() == 60)
    {
        VxVy_Limit = CHASSIS_SPEED_L;
        Vw_Limit = 5000.0f;
    }
    else if (Get_PowerLimit() > 60 && Get_PowerLimit() <= 80)
    {
        VxVy_Limit = CHASSIS_SPEED_M;
        Vw_Limit = 6000.0f;
    }
    else if (Get_PowerLimit() > 80)
    {
        VxVy_Limit = CHASSIS_SPEED_H;
        Vw_Limit = CHASSIS_SPEED_H;
    }

//    if(Cap_switch == ON)//�ڱ�����
//    {
//        Vw_Limit = CHASSIS_SPEED_H;//8500.0f;
//    }


    // if(*Vx == 0 && *Vy == 0)
    // {
    //     *Vx = arm_sin_f32(sinspin_cnt)*2000;
    //     *Vy = arm_cos_f32(sinspin_cnt)*2000;
    //     move_speed = 0;
    // }
    // else
    // {
        move_speed = fabs(*Vx)>fabs(*Vy) ? fabs(*Vx) : fabs(*Vy); //--- ȡ�����ٶ�
    // }

		offset_angle = Gimbal.Motor[Yaw].getAngle();
		for(;offset_angle>180;)
		{
			offset_angle-=360;
		}
		for(;offset_angle<-180;)
		{
			offset_angle+=360;
		}
    Speed_Decompose(Gimbal.Get_YawDeviateC(offset_angle), Vx, Vy);

    switch(Infantry.Get_CtrlSource())
    {
    case CTRL_RC:
        temp_spin = RAMP_Output(*Vw, temp_spin, 20.0f);
        *Vw = temp_spin;
        break;

    case CTRL_PC:
        // *Vw = abs(7000 * sin((double)(xTaskGetTickCount() * portTICK_PERIOD_MS) / 500.0)) + 4000;      // --- ����С����
        // Spin_Param = sin_max * arm_sin_f32(sinOmiga * sinspin_cnt) + 1.1f;
        // *Vw *= Vw_Limit;
        temp_spin = RAMP_Output(-CHASSIS_MAX_VW, temp_spin, 20.0f);
        *Vw = temp_spin;
        break;
    }
    
    //--- С�����ƶ�ʱ���������ٶ�����ȡ�ƶ��ٶ�(Ӧ������Ҫ��)
    if(fabs(move_speed)>500 /* && Cap_switch == OFF */)
    {
        Vw_coe = pow((VxVy_Limit - fabs(move_speed) + debug_movespeed), 2) / pow(VxVy_Limit, 2);
        Constrain(&Vw_coe, debug_min_vwcoe, 1.0f);
    }
    else
    {
        Vw_coe = 1.0f;
    }

    *Vw *= Vw_coe;

    Infantry.Chassis_PreMode = CHAS_SpinMode;
}

void Chassis_classdef::Sentry_Spin_Ctrl(float *Vx, float *Vy, float *Vw)
{
    float move_speed = 0.0f;
    static int32_t sinspin_cnt = 0;
    float offset_angle = 0.0f;

    if(Infantry.Get_ChassisPreMode() != CHAS_SpinMode)
    {
        sinspin_cnt = 0;
    }
    sinspin_cnt++;


		VxVy_Limit = CHASSIS_SPEED_H;
		Vw_Limit = CHASSIS_SPEED_H;


    // if(*Vx == 0 && *Vy == 0)
    // {
    //     *Vx = arm_sin_f32(sinspin_cnt)*2000;
    //     *Vy = arm_cos_f32(sinspin_cnt)*2000;
    //     move_speed = 0;
    // }
    // else
    // {
        move_speed = fabs(*Vx)>fabs(*Vy) ? fabs(*Vx) : fabs(*Vy); //--- ȡ�����ٶ�
    // }
		offset_angle = Gimbal.Motor[Yaw].getAngle();
		for(;offset_angle>180;)
		{
			offset_angle-=360;
		}
		for(;offset_angle<-180;)
		{
			offset_angle+=360;
		}
    Speed_Decompose(Gimbal.Get_YawDeviateC(offset_angle+Vision.Radar_Recv_Msg.Pack.chassis_forward), Vx, Vy);
//    Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()+Vision.Radar_Recv_Msg.Pack.chassis_forward), Vx, Vy);

		temp_spin = RAMP_Output(*Vw, temp_spin, 20.0f);
		*Vw = temp_spin;
    
    //--- С�����ƶ�ʱ���������ٶ�����ȡ�ƶ��ٶ�(Ӧ������Ҫ��)
    if(fabs(move_speed)>500)
    {
        Vw_coe = pow((VxVy_Limit - fabs(move_speed) + debug_movespeed), 2) / pow(VxVy_Limit, 2);
        Constrain(&Vw_coe, debug_min_vwcoe, 1.0f);
    }
    else
    {
        Vw_coe = 1.0f;
    }

    *Vw *= Vw_coe;

    Infantry.Chassis_PreMode = CHAS_SentrySpinMode;
}


/**
  * @brief  	Ť��ģʽ(����û��Ҫ���������)
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::Swing_Ctrl(float *Vx, float *Vy, float *Vw)
{
    // float tempvx = *Vx;
    // float tempvy = *Vy;

    static float SwingAngle, CalAngle, Time;

    if(Infantry.Get_ChassisPreMode() != CHAS_SwingMode)
    {
        Time = 0;
    }

    Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()), Vx, Vy);

    static float const add_time_step = PI/270.0f;

    //--- �����ƶ��ٶȶ�Ӧ��ͬ��Ť���Ƕ�
    SwingAngle = (IsMove() == true ? 70.0f*(PI/180) : 80.0f*(PI/180));

    //--- ������̵�Ŀ��Ƕ�
    CalAngle = SwingAngle * arm_sin_f32(Time);
    //--- ÿ�ε��ӵĽǶ�
    Time += add_time_step;

    if (Time > 2*PI)
	{
		Time -= 2*PI;
	}

    //---- PID����
    Swing_PID[PID_Outer].Target = CalAngle;
    Swing_PID[PID_Outer].Current = (Gimbal.Motor[Yaw].getAngle())*(PI/180);

    *Vw = Swing_PID[PID_Outer].Cal();

    Infantry.Chassis_PreMode = CHAS_SwingMode;
}

/**
  * @brief  	�Զ�׷��װ�װ�ģʽ(������)
  * @param[in]  Vx
  * @param[in]  Vy
  * @param[in]  Vw
  * @retval 	None
  */
void Chassis_classdef::AutoTrack_Ctrl(float *Vx, float *Vy, float *Vw)
{
    float distance = 0.0f;

    distance = KalmanFilter(&VisionDistance_Kalman, Vision.Get_DepthOffset());
    //---ƽʱչ���û���������С������
    if(distance < 0)
    {
        distance = 0;
    }

    if(Infantry.Get_GimbalMode() == Gimbal_PCMode && Vision.Get_Enemy() == true)
    {
        AutoTrack_PID[PID_Outer].Target = distance;
        AutoTrack_PID[PID_Outer].Current = 1000.0f;
        *Vy += AutoTrack_PID[PID_Outer].Cal();
    }
    else
    {
        *Vy = 0.0f;
    }

    Follow_Ctrl(Vx,Vy,Vw);
}


/**
  * @brief  	�޸���ģʽ
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


void Chassis_classdef::Sentry_NotFollwe_Ctrl(float *Vx, float *Vy, float *Vw)
{
    Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()+Vision.Radar_Recv_Msg.Pack.chassis_forward), Vx, Vy);
	
    *Vx = *Vx;
    *Vy = *Vy;
    *Vw = *Vw;

    Infantry.Chassis_PreMode = CHAS_SentryNotFollowMode;
}

/**
 * @brief      ���̵��ݷŵ翪��
 * @param[in]  state
 * @retval     None
 */
void Chassis_classdef::Cap_Ctrl(uint8_t state)
{
    Cap_switch = state;
}




//void Chassis_classdef::SentryFollow_Ctrl(float *Vx, float *Vy, float *Vw)
//{
//	//ȫ��ǰ�����Ƕ�
//	Speed_Decompose(Gimbal.Get_YawDeviateC(Gimbal.Motor[Yaw].getAngle()), Vx, Vy);
//	getencoder
//}

/**
 * @brief      ���������������ʽPID����
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @retval     None
 */
void Chassis_classdef::DRV_PIDCalc(uint8_t motor, float target, float current)
{
    CHAS_DRV_PID[motor].Target = target;
    CHAS_DRV_PID[motor].Current = current;
    DRV_Motor[motor].Out = CHAS_DRV_PID[motor].Cal();

    drv_tempcurrent[motor] = DRV_Motor[motor].Out; // ���浱ǰֵ���ڹ������Ƶļ���
}

/**
 * @brief      ����ת����λ��ʽPID����
 * @param[in]  motor
 * @param[in]  target
 * @param[in]  current
 * @retval     None
 */
void Chassis_classdef::RUD_PIDCalc(uint8_t motor, float target, float current)
{
    CHAS_RUD_PID[motor][PID_Outer].Target = target;
    CHAS_RUD_PID[motor][PID_Outer].Current = current;

    CHAS_RUD_PID[motor][PID_Inner].Target = CHAS_RUD_PID[motor][PID_Outer].Cal();
    CHAS_RUD_PID[motor][PID_Inner].Current = RUD_Motor[motor].getSpeed();
    RUD_Motor[motor].Out = CHAS_RUD_PID[motor][PID_Inner].Cal();

    rud_tempcurrent[motor] = RUD_Motor[motor].Out; // ���浱ǰֵ���ڹ������Ƶļ���
}


/**
 * @brief      �����Ƿ����ƶ�Ŀ���ٶ�
 * @param[in]  None
 * @retval     1�� 0��
 */
uint8_t Chassis_classdef::IsMove()
{
    if((CTRL_DR16.Get_ExptVx() || CTRL_DR16.Get_ExptVy() || CTRL_DR16.Get_ExptYaw()) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_classdef::Drv_Slow(float *rec , float target , float slow_Inc)
{

    if(abs(*rec) - abs(target) < 0)//����ʱ
    {
        if(abs(*rec) > 10)
        {
            slow_Inc = slow_Inc * 5;//�ٶ���������ʱ������5��
        }
    }
    
    if(abs(*rec) - abs(target) > 0)
    {
        slow_Inc = slow_Inc * 10.0f;//����ʱ�Ŵ�10��
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
 * @brief      ���̼�������
 * @param[in]  Vx Vy
 * @retval     None
 */
int16_t accc_temp = 3000;
void Chassis_classdef::AcclerateCurve(float *Vx, float *Vy)
{
    Acc_Param.linnerSpeed = sqrt((*Vx) * (*Vx) + (*Vy) * (*Vy));
    if ((fabs(Acc_Param.linnerSpeed) - fabs(Acc_Param.linnerSpeedLast)) > accc_temp) //�����ٳ���һ����ֵʱʹ������
    {
        Acc_Param.accelerating = 1;
		Acc_Param.decelerating = 0;
        Acc_Param.accCnt = 0;
    }
    if ((fabs(Acc_Param.linnerSpeed) - fabs(Acc_Param.linnerSpeedLast)) < -accc_temp) //�����ٳ���һ����ֵʱʹ������
    {
        Acc_Param.decelerating = 1;
		Acc_Param.accelerating = 0;
        Acc_Param.accCnt = 0;
    }
	/*��������*/
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
	/*��������*/
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
	/*��ϵ��*/
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
 * @description: ��ʽԭ�� y = 1/(1+e^(-k(x-2/k)))  ��k = 4.2 , x = 1ʱ y = 0.9
 */
float Natural_num = 2.718281828f;//--- ��Ȼ��e
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
 * @description: ��ʽԭ�� y =1/(1+e^(?k(-(x-(5/k))?2/k) ) )
 * @note         ��k = 10 , x = 0.8ʱ y = 0.01
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
 * @brief  	    ��ȡ��������Ŀ���ٶ�
 * @param[in]	mode
 * @retval 	    Target speed
 */
int16_t Chassis_classdef::Get_TargetVx()
{
    return Target_Vx;
}
int16_t Chassis_classdef::Get_TargetVy()
{
    return Target_Vy;
}
int16_t Chassis_classdef::Get_TargetVw()
{
    return Target_Vw;
}
int16_t Chassis_classdef::Get_PowerLimit()
{
    return 150;//Power_Limit;
}





