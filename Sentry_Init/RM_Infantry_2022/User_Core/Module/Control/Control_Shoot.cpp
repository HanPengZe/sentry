#include "Robot_Config.h"
#include "Control_Shoot.h"
#include "System_DataPool.h"
#include "tim.h"

// --- ���ο���ʱ������
#define Limit_StuckDuration 120
// --- ���̵������ֵĽǶ�
#define GearTeeth_Angle     95784//(36859*52/20)���Բ��̵�� */ /* 32764 //(8191*36/9)9������*/
// --- ����װ��1��ת������ȡ��Ϊ -1
#define Reload_Dir      1
// --- Ħ���ְ�װ��ʽ ���� 1 ���� -1
#define Fric_Install    1
//--- ��ת������
#define RollBack_Num    2
//--- ��Ƶ
#define SHOOT_FREQ(x)  (uint16_t)(500/(x))

// --- Ħ����ת��
int16_t FricSpeed_15 = 2000;
int16_t FricSpeed_18 = 4300;
int16_t FricSpeed_22 = 5200;                                                 
int16_t FricSpeed_30 = 7400;
int16_t FricSpeed_Unload = 500;

/**
 * @brief  	   
 * @param[in]  None
 * @retval 	   None
 */
uint8_t init_bar_flag;
float init_bar;
Shoot_classdef::Shoot_classdef()
{
    Reload_PID[PID_Outer].SetPIDParam(0.4f, 0.0f, 0.0f, 10000, 10000, 0.002f);
    Reload_PID[PID_Inner].SetPIDParam(11.5f, 10.0f, 0.0f, 500, 9500, 0.002f);
    Reload_PID[PID_Inner].I_SeparThresh = 1000;
	
		Barrel_PID[PID_Outer].SetPIDParam(0.4f, 0.0f, 0.0f, 10000, 10000, 0.002f);//ǹ��
    Barrel_PID[PID_Inner].SetPIDParam(11.5f, 10.0f, 0.0f, 500, 9500, 0.002f);
    Barrel_PID[PID_Inner].I_SeparThresh = 1000;
	
		Card_PID[PID_Outer].SetPIDParam(0.5f, 0.0f, 0.0f, 10000, 10000, 0.002f);
    Card_PID[PID_Inner].SetPIDParam(10.0f, 10.0f, 0.0f, 500, 9500, 0.002f);
    Card_PID[PID_Inner].I_SeparThresh = 1000;

    Fric_PID[Fric_L].SetPIDParam(11.5f, 0.87, 0.0f, 5000, 15000, 1.0f);
    Fric_PID[Fric_L].I_SeparThresh = 9000;

    Fric_PID[Fric_R].SetPIDParam(11.5f, 0.77, 0.0f, 5000, 15000, 1.0f);
    Fric_PID[Fric_R].I_SeparThresh = 9000;

    Reload.CriticalVal = 1300;
		
		Reload.Target_Angle = Reload_Motor.get_totalencoder();
		Card_Target_Angle = Card_Motor.get_totalencoder();
		init_bar = Barrel_Motor.get_totalencoder();
		init_bar_flag = 1;
}



/**
 * @brief  	   �������
 * @param[in]  None
 * @retval 	   None
 */
int BarrelRecovery;
void Shoot_classdef::Control()
{
    static uint16_t Mag_off_cnt = 1;
		//���ֲ�������_�ֶ�����
		if(CanShoot_Num[0] < 24 || CanShoot_Num[1] < 24)
		{
			BarrelRecovery++;
			if(BarrelRecovery>=(int)(63/Infantry.CoolBuff))
			{
				BarrelRecovery=0;
				CanShoot_Num[0]++;
				CanShoot_Num[1]++;
				if(CanShoot_Num[0]>24){CanShoot_Num[0]=24;}
				if(CanShoot_Num[1]>24){CanShoot_Num[1]=24;}
			}
		}
		
		if(Infantry.Get_AttackMode() == Attack_Disable ||//ʧ�ܷ������
			Infantry.Get_FricMode() == Fric_Disable ||\
		Infantry.Get_State() != Robot_Activating ||\
		DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    {
        //set init angle...
				Gimbal.Send_Motor[SendID_Reload].Out = Reload_Motor.Out = 0;
				
				Send_Motor[SendID_Barrel].Out = Barrel_Motor.Out = 0;
				Send_Motor[SendID_Fric_L].Out = Fric_Motor[Fric_L].Out = 0;
				Send_Motor[SendID_Fric_R].Out = Fric_Motor[Fric_R].Out = 0;
			
			  Card_Motor.Out = 0;
			
        return;
    }
		
    Fric_Process();//����Ħ����
		//		ShootFreq_Calc();����ϵͳ���Ʒ�����������Ƶ:Shoot_Freq
    AttackMode_Ctrl();//�ж���Ƶ������������װ��:Reload.Num�� Reload.CanShoot_Num = 10;
    Reload_Calc();//���ɲ���ϵͳ�ж�����(�����������ע�͵�)����ʹ��Reload.Num�ۼӲ��̵�Ŀ��ֵ
		Barrel_Calc();//ǹ�ڳ�ʼ����ǹ���˶�

    //--- ��ֹ���������ǹص��� �ڱ���Ҫ
#if 0
    if(Infantry.Get_CtrlSource() == CTRL_PC)
    {
        if(abs(CTRL_DR16.Get_ExptVx()) > 2000 || abs(CTRL_DR16.Get_ExptVy()) > 2000)
        {
            Mag_off_cnt++;
        }
        else
        {
            Mag_off_cnt = 1;
        }

        if((Mag_off_cnt%=500) == 0)
        {
            Mag_Ctrl(OFF); //--- PCģʽ�˶�״̬��1s��һ��
        }
    }
    else if(Infantry.Get_CtrlSource() == CTRL_RC)
    {
        if(Infantry.Get_ChassisMode() != CHAS_DisableMode && Infantry.Get_ChassisMode() != CHAS_LockMode)
        {
            Mag_Ctrl(OFF);
        }
    }
#endif
//    MotorMsgSend(&hcan2, Send_Motor);
}

/**
 * @brief  	   Ħ���ִ���
 * @param[in]  None
 * @retval 	   None
 */
void Shoot_classdef::Fric_Process()
{
    /*<! �˰汾ֻ�� M3508 Ħ���ֵ������
    ��21������ʼ����ʹ��Snail��� */

    // --- �жϷ���ϵͳ�Ƿ���������
#if USE_RM_Referee
	if(Get_ShooterPower() == OFF)//����ϵͳ���͹���������
    {
        Reload.Num = 0;
        return;
    }
#else
    // if((DevicesMonitor.Get_State(FRIC_L_MONITOR) && DevicesMonitor.Get_State(FRIC_R_MONITOR)) == Off_line)
    // {
    //     Reload.Num = 0;
    //     return;
    // }
#endif

    FricSpeed_Adapt(); //--- �����Ե���

    if(Infantry.Get_CtrlSource() == CTRL_PC && Infantry.Get_AttackMode() != Attack_Disable)
    {
        // Laser_Ctrl(ON);

        if(Infantry.Get_VisionMode() == Vision_BIGWindmill)
        {
            Laser_Ctrl(OFF);  //--- �����������
        }
        else
        {
            Laser_Ctrl(ON);
        }

        switch (Get_SpeedLimit())
        {
        case 15:
            Infantry.Fric_Mode = Fric_15m_s;
            Fric_Speed = FricSpeed_15 + Fric_SpeedOffset[0] + Fric_TempOffset[0];
            break;
        case 18:
            Infantry.Fric_Mode = Fric_18m_s;
            Fric_Speed = FricSpeed_18 + Fric_SpeedOffset[1] + Fric_TempOffset[1];
            break;
        case 22:
            Infantry.Fric_Mode = Fric_22m_s;
            Fric_Speed = FricSpeed_22 + Fric_SpeedOffset[2] + Fric_TempOffset[2];
            break;
        case 30:
        case 75:
            Infantry.Fric_Mode = Fric_30m_s;
            Fric_Speed = FricSpeed_30 + Fric_SpeedOffset[3] + Fric_TempOffset[3];
            break;
        default:
            Infantry.Fric_Mode = Fric_15m_s;
            Fric_Speed = FricSpeed_15 + Fric_SpeedOffset[0] + Fric_TempOffset[0];
            break;
        }
    }
    else if(Infantry.Ctrl_Source == CTRL_RC)
    {
        switch(Infantry.Get_FricMode())
        {
        case Fric_15m_s:
            Fric_Speed = FricSpeed_15 + Fric_SpeedOffset[0] + Fric_TempOffset[0];
            Laser_Ctrl(ON); //--- RC���Ƶ�ʱ���Ħ���ֲ��ؼ�����Ϊ�˲��ÿ�Ħ���ֲ��м���
            break;
        case Fric_18m_s:
            Fric_Speed = FricSpeed_18 + Fric_SpeedOffset[1] + Fric_TempOffset[1];
            Laser_Ctrl(ON);
            break;
        case Fric_22m_s:
            Fric_Speed = FricSpeed_22 + Fric_SpeedOffset[2] + Fric_TempOffset[2];
            Laser_Ctrl(ON);
            break;
        case Fric_30m_s:
            Fric_Speed = FricSpeed_30 + Fric_SpeedOffset[3]+ Fric_TempOffset[3];
            Laser_Ctrl(ON);
            break;
        case Fric_Unload:
            Fric_Speed = FricSpeed_Unload;
            Laser_Ctrl(ON);
            break;
        case Fric_Disable:
            Fric_Speed = 0;
            break;
				case Fric_Zero:
            Fric_Speed = 0;
            break;
        default:
            break;
        }
    }
    else
    {
        Fric_Speed = 0;
        Laser_Ctrl(OFF);
    }

    Fric_Speed = Fric_Speed > 8000 ? 8000 : Fric_Speed;

    //--- Ħ����PID����
    FricPID_Calc(Fric_Speed);

}

/**
 * @brief  	   Ħ����ת�ٵ��� (Test ing)
 * @param[in]  ǹ�ܷ����ٶ� �������� �ݼ�ֵ ����ֵ
 * @note       ��ת�ٵ����Ļ����ϼ����¿ش�����Ϊ�������Է����¶�Խ������ҲԽ��
 * @note       ���Ȳ�15 18 30��22�����ò���
 * @retval 	   None
 */
float Pre_GunSpeed;
float temp_scope = 35;  //����仯��ΧΪ35���϶�
float temp_init = 35;    //��ʼ�¶��趨Ϊ35���϶�

/* 15m/s 18m/s 22m/s 30m/s */
float min_speed[4] = {13.4f, 16.8f, 19.0f, 27.8f};
float max_speed[4] = {14.5f, 17.3f, 21.4f, 29.3f};
float sub_val[4] = {30, 55, 20, 45};
float add_val[4] = {25, 20, 40, 55};
float Fric_Temp_Coe[4] = {-90.0f, -100.0f, -90.0f, -180.0f};
void Shoot_classdef::FricSpeed_Adapt()
{
    static uint8_t speederr_flag = 0;
    float motor_temp = 0;
    uint8_t fricmode = 0;

    if(Infantry.Get_FricMode() == Fric_Unload || Infantry.Get_FricMode() == Fric_Disable)
    {
        return;
    }

    fricmode = Infantry.Get_FricMode()-2;

    //--- �ж������Ƿ���� ��Ϊ����ϵͳ������������float���͵� �������ε����ݼ����������
    if(Get_GunSpeed() != Pre_GunSpeed)
    {
        //----- ���ٲ������� -----//
        if(Get_GunSpeed() < min_speed[fricmode] && Get_GunSpeed() > 10.0f)
        {
            speederr_flag = 1;
        }
        else if(Get_GunSpeed() >= min_speed[fricmode] && Get_GunSpeed() <= max_speed[fricmode])
        {
            speederr_flag = 0;
        }

        if(speederr_flag == 1) //--- ����ƫ��
        {
            speederr_flag = 0;
            Fric_SpeedOffset[fricmode] += add_val[fricmode];
        }
        if(Get_GunSpeed() > max_speed[fricmode]) //--- ����ƫ��
        {
            Fric_SpeedOffset[fricmode] -= sub_val[fricmode];
        }

        //----- ����¿ز��� -----//
        //--- ����Ħ���ֵ������������ƽ���¶�
        motor_temp = ((float)Fric_Motor[Fric_L].getTempature()+(float)Fric_Motor[Fric_R].getTempature()) / 2.0f;
        
        if(motor_temp >= temp_init) //--- С��������
        {
            Fric_TempOffset[fricmode] = (motor_temp - temp_init)/temp_scope * Fric_Temp_Coe[fricmode];
        }
        if(motor_temp < temp_init) //--- û�����¶��ٽ�ֵ
        {
            Fric_TempOffset[fricmode] = 0;
        }
        if(motor_temp > temp_init + temp_scope) //--- ����̫��
        {
            Fric_TempOffset[fricmode] = Fric_Temp_Coe[fricmode];
        }
    }

    Pre_GunSpeed = Get_GunSpeed(); 
}



/**
 * @brief  	    ����ǹ�����ݼ�����Ƶ
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t Shoot_classdef::ShootFreq_Calc()
{
	
    if(Infantry.CanShootNum[Turn_Bar_ID] > 1) //--- ���㵱ǰǹ��ʣ������
    {
        if(Infantry.CanShootNum[Turn_Bar_ID] == 2) //--- �պ�������ֵ-��������
        {
            Shoot_Freq = 8;
        }
        else if(Infantry.CanShootNum[Turn_Bar_ID] <= 4) //--- ʣ����ĳ����ֵ֮�� 4���ӵ�
        {
            Shoot_Freq = (Infantry.CanShootNum[Turn_Bar_ID] / 2) * 10 + 8;
        }
        else
        {
            Shoot_Freq = 40; //---δ�ﵽ�����������䣬��Ƶ���
        }
    }
    else //--- ������պ÷�����ֵ���ӵ�
    {
        Shoot_Freq = 0;
    }

    return Shoot_Freq;
}


/**
 * @brief  	   ���̲������㺯��
 * @param[in]  None
 * @retval 	   None
 */
uint8_t CanShoot_Limit = 4; //--- �ɷ�������ֵ��������͵������
uint8_t KaDanTime, KaDanTimeLimit=100;
uint8_t Turn_Bar_ID;
uint8_t turn_bar_flag;
uint8_t turning_bar_flag;
void Shoot_classdef::Reload_Calc()
{
    Reload_Motor.encoder_offset = 0;

    // --- �жϷ���ϵͳ�Ƿ���������
#if USE_RM_Referee
    if(Get_ShooterPower() == OFF)
    {
        Reload.Num = 0;
        return;
    }
#endif

    if(Infantry.Get_AttackMode() == Attack_Zero || Infantry.Get_AttackMode() == Attack_Disable || DevicesMonitor.Get_State(RELOAD_MONITOR) == Off_line)
    {
        //--- �Ե�ǰλ��Ϊ����λ��
        Reload_Reset();
				
				Reload.Target_Angle = Reload_Motor.get_totalencoder();
        Reload.Began = false;
        Reload.Num = 0;
        return;
    }
//		if(Reload.Target_Angle-Reload_Motor.get_totalencoder()<-GearTeeth_Angle/4)
//		{
//			Reload.Target_Angle+=GearTeeth_Angle;
//			ReloadPID_Calc(Reload.Target_Angle);
//			return;
//		}
		
		
		//�������⡪��ûд��
    //--- ��ִ���ϴα�����Ŀ�����������
    if(Reload.Stuck_Duration > Limit_StuckDuration)
    {
				//��������
				//�����������
				//������ǰ��һ��
        //---- ����ɿ�����������
			Gimbal.Send_Motor[SendID_Reload].Out = Reload_Motor.Out = 0;
			if(abs(Card_Target_Angle-Card_Motor.get_totalencoder())<8192)
			{			
						ShootOverTiming[Turn_Bar_ID]=HAL_GetTick();
						ShootTime[Turn_Bar_ID] = ShootOverTiming[Turn_Bar_ID]-ShootStartTiming[Turn_Bar_ID];
						CanShoot_Num[Turn_Bar_ID]--;
            Reload.Stuck_Flag = false;
            Reload.Began = false;
            Reload.Re_TargetAngle = 0;
            Reload.Stuck_Duration = 0;
						Reload.Num = 0;
						KaDanTime=0;
			}
			CardPID_Calc(Card_Target_Angle);
			return;
    };
	  //--- �з��䵯������
    if(Reload.Num > 0)
    {
			/*-----------------------------------------------------------------------------------------�Ķ�

//				//���ݲ���ϵͳ������ǹ��
				if(Infantry.CanShootNum[Turn_Bar_ID]<3)
				{
					Reload.Num = 0;
				}
				if(turn_bar_flag || turning_bar_flag)
				{
					Reload.Num = 0;
				}
				else if(Infantry.CanShootNum[0]<2 && Infantry.CanShootNum[1]<2)
				{
//					Turn_Bar_ID=0;//��ʱ
					Reload.Num = 0;
				}
				if(ContLaunch == false)
				{
					Reload.Num = 0;
				}
--------------------------------------------------------------------------------------------*/

//				if(CanShoot_Num[Turn_Bar_ID]<3)
//				{
//					Reload.Num = 0;
//				}
//				if(turn_bar_flag)
//				{
//					Reload.Num = 0;
//				}
//				else if(CanShoot_Num[0]<2 && CanShoot_Num[1]<2)
//				{
//					Turn_Bar_ID=0;
//					Reload.Num = 0;
//				}

				
        //--- ��Ƶ�����㷢������
        if(Shoot_Freq <= 0)
        {
            Reload.Num = 0;
            return;
        }
				
				//�Ӿ��������������
				if(Vision.Sentry_Recv_Msg.Pack.auto_aim==0 && Vision.aim_flag == 1)
				{
					Reload.Num = 0;
				}
//				if(Reload.Target_Angle-Reload_Motor.get_totalencoder()>GearTeeth_Angle+16382)
//				{
//					Reload.Target_Angle -= GearTeeth_Angle * (int)((Reload.Target_Angle-Reload_Motor.get_totalencoder())/GearTeeth_Angle);
//				}

        // --- ������δ��ʼִ��
        if(Reload.Began == false && Reload.Num != 0)
        {
						ShootStartTiming[Turn_Bar_ID]=HAL_GetTick();
            // --- ����׼������״̬
            Reload.Began = true;
            Reload_Reset();
            Reload.Init_Angle = Reload_Motor.get_totalencoder();
//            Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;
						Card_Target_Angle+=8191*12;        
				}

        // --- �����������
//        if(abs(Reload_Motor.get_totalencoder() - Reload.Init_Angle) >= Reload.Num * GearTeeth_Angle - 8191)
	
				/*	----------------------------------------------------------------
				
				if(abs(Card_Motor.get_totalencoder() - Card_Target_Angle) <= 8192 && Reload.Began == true && Reload.Began2 == false)
				{
						Reload.Began2 = true;
						Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;


//						ShootOverTiming[Turn_Bar_ID]=HAL_GetTick();
//						ShootTime[Turn_Bar_ID] = ShootOverTiming[Turn_Bar_ID]-ShootStartTiming[Turn_Bar_ID];
////						CanShoot_Num[Turn_Bar_ID]--;
//            Reload.Num = 0;
//            Reload.Began = false;
//            Reload.Stuck_Flag = false;
//            Reload.Re_TargetAngle = 0;
//            Reload.Launched_Num = Reload.Num;
//            Reload.Stuck_Duration = 0;
//						Reload.Began2 = false;
        }
				----------------------------------------------------------*/
				if( Reload.Began == true && Reload.Began2 == false)
				{
						Reload.Began2 = true;
						Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;


//						ShootOverTiming[Turn_Bar_ID]=HAL_GetTick();
//						ShootTime[Turn_Bar_ID] = ShootOverTiming[Turn_Bar_ID]-ShootStartTiming[Turn_Bar_ID];
////						CanShoot_Num[Turn_Bar_ID]--;
//            Reload.Num = 0;
//            Reload.Began = false;
//            Reload.Stuck_Flag = false;
//            Reload.Re_TargetAngle = 0;
//            Reload.Launched_Num = Reload.Num;
//            Reload.Stuck_Duration = 0;
//						Reload.Began2 = false;
        }

        else
        {
            // --- ����ʣ��������ĵ���
            Reload.Launched_Num = abs(Reload_Motor.get_totalencoder() - Reload.Init_Angle) / GearTeeth_Angle;
        }
				
				if(Reload.Began2 == true && Reload.Began == true && abs(Reload_Motor.get_totalencoder() - (Reload.Target_Angle+32764/4)) <= 8192)
				{
						ShootOverTiming[Turn_Bar_ID]=HAL_GetTick();
						ShootTime[Turn_Bar_ID] = ShootOverTiming[Turn_Bar_ID]-ShootStartTiming[Turn_Bar_ID];
//						CanShoot_Num[Turn_Bar_ID]--;
            Reload.Num = 0;
            Reload.Began = false;
            Reload.Stuck_Flag = false;
            Reload.Re_TargetAngle = 0;
            Reload.Launched_Num = Reload.Num;
            Reload.Stuck_Duration = 0;
						Reload.Began2 = false;
				}
				

				
				//�������⡪��ûд��
				// --- �������
//        if(Judge_ReloadStall() == true)
//        {
//            Reload.Stuck_Duration++;
//            if(Reload.Stuck_Duration > Limit_StuckDuration)
//            {
//								Reload.Began = false;
//                Reload.Stuck_Flag = true; // --- �����������һ�ο�ʼ��ת
//                Reload.Motor_Direction = 1; // --- ��ʼִ���״ο�����������
//								//���ݲ��Կ��Ƿ�ͣ��ԭ��
//								Reload.Re_TargetAngle = Reload.Target_Angle-GearTeeth_Angle;
//								Card_Target_Angle+=8191*12;
//                //Reload.Re_TargetAngle = Reload_Motor.get_totalencoder() - GearTeeth_Angle * 2;
//            }
//        }
//        else
//        {
//            if(Reload.Stuck_Duration > 0)
//            {
//                Reload.Stuck_Duration -= 10;
//            }
//            else
//            {
//                Reload.Stuck_Duration = 0;
//            }
//						Reload.Stuck_Duration = 0;
//        }
    }
    else
    {
        Reload.Num = 0;
    }

		
		
#if 0
    //--- ��ִ���ϴα�����Ŀ�������
    if(Reload.Stuck_Duration > Limit_StuckDuration)
    {
        if(Judge_ReloadStall() == true) //--- Ť�ع��󣬱���ִ�з�ת����ʱҲ����
        {
            Reload.Stuck_Duration++;
            //--- ����ʱ��Ϊ 200~300����ת����ı䷽��
            if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 0 && Reload.Motor_Direction == 1)
            {
                Reload.Re_TargetAngle += GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 0; //--- ������ת����
            }
            else if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 1 && Reload.Motor_Direction == 0)
            {
                Reload.Re_TargetAngle -= GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 1; //--- ���з�ת
            }
        }

        //---- ����ɿ�����ת����
        if(abs(Reload_Motor.get_totalencoder() - Reload.Re_TargetAngle) <= GearTeeth_Angle)
        {
            Reload.Stuck_Flag = false;
            Reload.Re_TargetAngle = 0;
            Reload.Stuck_Duration = 0;
        }
        else
        {
            ReloadPID_Calc(Reload.Re_TargetAngle);
            return;
        }
    }
    
    //--- �з��䵯������
    if(Reload.Num > 0)
    {
        // --- ����������״̬
        // if(Reload.CanShoot_Num < CanShoot_Limit && ContLaunch == true)
        // {
        //     Reload.Num = 0;
        //     return;
        // }
        // // --- �����㵥��״̬
        // else if(Reload.CanShoot_Num < (CanShoot_Limit-2) && ContLaunch == false)
        // {
        //     Reload.Num = 0;
        //     return;
        // }

        //--- ��Ƶ�����㷢������
        if(Shoot_Freq <= 0)
        {
            Reload.Num = 0;
            return;
        }

        // --- ������δ��ʼִ��
        if(Reload.Began == false)
        {
            // --- ����׼������״̬
            Reload.Began = true;
            Reload_Reset();
            Reload.Init_Angle = Reload_Motor.get_totalencoder();
            Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;
        }

        // --- �����������
        if(abs(Reload_Motor.get_totalencoder() - Reload.Init_Angle) >= Reload.Num * GearTeeth_Angle - 8191)
        {
            Reload.Num = 0;
            Reload.Began = false;
            Reload.Stuck_Flag = false;
            Reload.Re_TargetAngle = 0;
            Reload.Launched_Num = Reload.Num;
            Reload.Stuck_Duration = 0;
        }
        else
        {
            // --- ����ʣ��������ĵ���
            Reload.Launched_Num = abs(Reload_Motor.get_totalencoder() - Reload.Init_Angle) / GearTeeth_Angle;
        }

        // --- �������
        if(Judge_ReloadStall() == true)
        {
            Reload.Stuck_Duration++;
            if(Reload.Stuck_Duration > Limit_StuckDuration)
            {
                Reload.Stuck_Flag = true; // --- �����������һ�ο�ʼ��ת
                Reload.Motor_Direction = 1; // --- ��ʼִ���״η�ת����
                Reload.Re_TargetAngle = Reload_Motor.get_totalencoder() - GearTeeth_Angle * 2;
            }
        }
        else
        {
            if(Reload.Stuck_Duration > 0)
            {
                Reload.Stuck_Duration -= 10;
            }
            else
            {
                Reload.Stuck_Duration = 0;
            }
						Reload.Stuck_Duration = 0;
        }
    }
    else
    {
        Reload.Num = 0;
    }
#endif

    ReloadPID_Calc(Reload.Target_Angle+32764/4);//������ǲ��̵��
		CardPID_Calc(Card_Target_Angle);//����ǹ��ǰ����ǿ������
}




uint8_t init_bar_GPIO;
float tar_bar,time_bar;
float text_bar=0;
int beganFalse_time;
void Shoot_classdef::Barrel_Calc()
{
	init_bar_GPIO = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
	
//	if((Infantry.Get_AttackMode() == Attack_Disable &&\
//		Infantry.Get_FricMode() == Fric_Disable) ||\
//		DevicesMonitor.Get_State(RELOAD_MONITOR) == Off_line ||\
//		(DevicesMonitor.Get_State(FRIC_L_MONITOR) &&\
//		DevicesMonitor.Get_State(FRIC_R_MONITOR)) == Off_line||\
//		Infantry.Get_State() != Robot_Activating ||\
//		DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
//	{
//			//--- �Ե�ǰλ��Ϊ����λ��
//			tar_bar = Reload_Motor.get_totalencoder();
//			return;
//	} 
//	-1224 129920
	if(init_bar_flag)//��ʼ��
	{
		//�����ж�
		//���ݲ���ϵͳ������ǹ��
//			if(Turn_Bar_ID)
//		{
//			if(Infantry.CanShootNum[1]<3 && Infantry.CanShootNum[0]>9)
//			{
//				turning_bar_flag=1;
//				Turn_Bar_ID=0;
//				turnShoot_startTiming=HAL_GetTick();
//			}
//		}
//		else
//		{
//			if(Infantry.CanShootNum[0]<3 && Infantry.CanShootNum[1]>9)
//			{
//				turning_bar_flag=1;
//				Turn_Bar_ID=1;
//				turnShoot_startTiming=HAL_GetTick();
//			}
//		}
			
			//�Լ���
//		if(Turn_Bar_ID)
//		{
//			if(CanShoot_Num[1]<3 && CanShoot_Num[0]>8)
//			{
//				Turn_Bar_ID=0;
//				turnShoot_startTiming=HAL_GetTick();
//			}
//		}
//		else
//		{
//			if(CanShoot_Num[0]<3 && CanShoot_Num[1]>8)
//			{
//				Turn_Bar_ID=1;
//				turnShoot_startTiming=HAL_GetTick();
//			}
//		}
		

		if(Reload.Began == false)
		{
			beganFalse_time++;
		}
		else
		{
			beganFalse_time=0;
		}
		if(beganFalse_time > 5)
		{
			if(Turn_Bar_ID)
			{
				//��
				if(abs(Reload_Motor.get_totalencoder() - (Reload.Target_Angle+32764/4))<8192)
				{tar_bar = init_bar+132000+text_bar;}
			}
			else
			{
				//�� ��ʼǹ��
				if(abs(Reload_Motor.get_totalencoder() - (Reload.Target_Angle+32764/4))<8192)
				{tar_bar = init_bar-2000+text_bar;}
//				tar_bar = init_bar-4000+text_bar;
			}
		}
	}
	else
	{

	}
	
	if(abs(Barrel_Motor.get_totalencoder()-tar_bar)<12000)
	{
		if(turn_bar_flag==1)
		{
			turnShoot_overTiming=HAL_GetTick();
			turnShoot_Time = turnShoot_overTiming - turnShoot_startTiming;
		}
		turn_bar_flag = 0;
		turning_bar_flag=0;
	}
	else
	{
		turn_bar_flag = 1;
	}
	BarrelPID_Calc(tar_bar);
}

/**
 * @brief  	   ���ģʽ����
 * @param[in]  None
 * @retval 	   None
 */
uint8_t biubiubiu; // һ����n��
uint8_t biubiubiu_flag;
void Shoot_classdef::AttackMode_Ctrl()
{
    Mshoot_Cnt++;
    Ashoot_Cnt++;

    if(Infantry.Get_AttackMode() == Attack_Disable)
    {
        Mshoot_Cnt = 0;
        Ashoot_Cnt = 0;
        return;
    }

    switch(Infantry.Get_AttackMode())
    {
    case Attack_15:
    case Attack_18:
    case Attack_22:
    case Attack_30:
        Ashoot_Cnt = 0;
        // if((Mshoot_Cnt %= SHOOT_FREQ(20)) == 0 && ContLaunch == true)
        // {
        //     Set_ReloadNum(1);
        // }
				#if 0
        if(Rage_Mode == true)
        {
            Shoot_Freq = 20; //--- ��Ѫģʽ�����������
        }
        else if(Infantry.Get_VisionMode() == Vision_BIGWindmill && Shoot_Freq >= 10)
        {  
            //--- ���������Ƶ��С��
            Shoot_Freq = 6;
        }
				#endif

        if(Shoot_Freq == 0)
        {
            Set_ReloadNum(0);
        }
        else if((Mshoot_Cnt %= SHOOT_FREQ(Shoot_Freq)) == 0 && ContLaunch == true)
        {
            Set_ReloadNum(1);
        }
			break;

    case Attack_Auto:
		{
        Mshoot_Cnt = 0;
        if(Vision.IsAutoAim() == false || Infantry.Get_GimbalMode() != Gimbal_PCMode)
        {}

        if(Vision.IsAutoAim() != false)
        {	
            if(biubiubiu_flag == false)
            {
                biubiubiu_flag = true;
                biubiubiu = 1;
            }
        }
        else
        {}

        if ((Ashoot_Cnt%=SHOOT_FREQ(Shoot_Freq)) == 0 && Vision.IsAutoAim() != false && DevicesMonitor.Get_State(VISION_MONITOR) != Off_line)
        {
            if(biubiubiu_flag == true && biubiubiu > 0)
            {
                if(Shoot_Freq >= 2)
                {
                    Set_ReloadNum(1);
                    biubiubiu--;
                }
            }
            else
            {
                biubiubiu_flag = false;
            }
        }
			}
			break;
				
		case Attack_Zero:
			Set_ReloadNum(0);
		break;

    case Attack_Unload:
        if((Mshoot_Cnt%=SHOOT_FREQ(100)) == 0 && ContLaunch == true)
        {
            Set_ReloadNum(1);
        }
        break;
    }

#if USE_RM_Referee
    //--- ���ݵ�ǰ�����ó��ɻ���������
    //--- ��Ѫģʽ��������
    Reload.CanShoot_Num = ((Rage_Mode == true || Infantry.Get_AttackMode() == Attack_Unload) ? 10 : (Get_CoolingLimit() - Get_GunHeat()) / Onebullet_heat);
#else
    Reload.CanShoot_Num = 10;
#endif


}


/**
 * @brief  	   ��������
 * @param[in]  None
 * @retval 	   None
 */
void Shoot_classdef::Reload_Reset()
{
    Gimbal.Send_Motor[SendID_Reload].Out = Reload_Motor.Out = 0;

    // Began_Reload = false;
    // Reload_Num = 0;
    // Reload_Motor.Reset();
//    Reload.Target_Angle = Reload_Motor.get_totalencoder();
}

/**
 * @brief  	   �жϲ��̵���Ƿ��ڿ����ٽ�ֵ
 * @param[in]  None
 * @retval 	   state
 */
bool Shoot_classdef::Judge_ReloadStall()
{
    return(abs(Reload_Motor.getSpeed()) < Reload.CriticalVal);
}

/**
 * @brief  	   ������� ���PID����
 * @param[in]  None
 * @retval 	   None
 * @note       ��ΪĦ���ֺͲ��̵����һ���ı�ʶ�����͵����������ٵ�������һ��MotorMsgSend()�����Ͳ��̵���
 *             ��������Ħ���ֵĵ���һֱ��0��Ŀ�������������
 */
void Shoot_classdef::ReloadPID_Calc(float tar_angle)
{
    Reload_PID[PID_Outer].Target = tar_angle;
    Reload_PID[PID_Outer].Current = Reload_Motor.get_totalencoder();
    Reload_PID[PID_Inner].Target = Reload_PID[PID_Outer].Cal();
    Reload_PID[PID_Inner].Current = Reload_Motor.getSpeed();
	

    Gimbal.Send_Motor[SendID_Reload].Out = Reload_Motor.Out = Reload_PID[PID_Inner].Cal();
}
void Shoot_classdef::BarrelPID_Calc(float tar_angle)
{
    Barrel_PID[PID_Outer].Target = tar_angle;
    Barrel_PID[PID_Outer].Current = Barrel_Motor.get_totalencoder();
    Barrel_PID[PID_Inner].Target = Barrel_PID[PID_Outer].Cal();
    Barrel_PID[PID_Inner].Current = Barrel_Motor.getSpeed();

    Send_Motor[SendID_Barrel].Out = Barrel_Motor.Out = Barrel_PID[PID_Inner].Cal();
}
void Shoot_classdef::CardPID_Calc(float tar_angle)
{
    Card_PID[PID_Outer].Target = tar_angle;
    Card_PID[PID_Outer].Current = Card_Motor.get_totalencoder();
    Card_PID[PID_Inner].Target = Card_PID[PID_Outer].Cal();
    Card_PID[PID_Inner].Current = Card_Motor.getSpeed();

    Card_Motor.Out = Card_PID[PID_Inner].Cal();
}
void Shoot_classdef::FricPID_Calc(float tar_speed)
{
    // --- ע��Ħ���ְ�װ��ʽ
    Fric_PID[Fric_L].Target = tar_speed*Fric_Install;
    Fric_PID[Fric_L].Current = Fric_Motor[Fric_L].getSpeed();
    Send_Motor[SendID_Fric_L].Out = Fric_Motor[Fric_L].Out = Fric_PID[Fric_L].Cal();
		

    Fric_PID[Fric_R].Target = -tar_speed*Fric_Install;
    Fric_PID[Fric_R].Current = Fric_Motor[Fric_R].getSpeed();
    Send_Motor[SendID_Fric_R].Out = Fric_Motor[Fric_R].Out = Fric_PID[Fric_R].Cal();
}

/**
 * @brief  	   ���������
 * @param[in]  state
 * @retval 	   None
 */
void Shoot_classdef::Laser_Ctrl(bool state)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, (GPIO_PinState)state);
}
/**
 * @brief  	   ���ֿ��ؿ���
 * @param[in]  state
 * @retval 	   None
 */
#if ROBOT_ID == INFANTRY_2022_SWERVE_1
uint16_t mag_on = 3800;
uint16_t mag_off = 1300;
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
uint16_t mag_on = 3750;
uint16_t mag_off = 1200;
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
uint16_t mag_on = 3750;
uint16_t mag_off = 1200;
#endif
#if ROBOT_ID == INFANTRY_2022_SWERVE_1 | ROBOT_ID == INFANTRY_2022_SWERVE_2 | ROBOT_ID == INFANTRY_2022_SWERVE_2
void Shoot_classdef::Mag_Ctrl(uint8_t state)
{
    if (state == ON)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mag_on);
		Mag_Switch = ON;
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mag_off);
		Mag_Switch = OFF;
	}
}
#endif
/**
 * @brief  	   PWM��ʼ��
 * @param[in]  state
 * @retval 	   None
 */
void Shoot_classdef::PWM_Init()
{
//    HAL_TIM_Base_Start(&htim10);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mag_off);
}


/**
 * @brief  	   ���õ���װ����
 * @param[in]  num
 * @retval 	   None 
 */
void Shoot_classdef::Set_ReloadNum(uint16_t num)
{
    //--- ��ֹ�����쳣
    if(num < 0)
    {
        Reload.Began = false;
        Reload.Stuck_Flag = false;
        Reload.Re_TargetAngle = 0;
        Reload.Stuck_Duration = 0;
        Reload_Reset();
				Reload.Target_Angle = Reload_Motor.get_totalencoder();
        return;
    }
		else if(num>1)
		{
			num=1;
		}

    if(Reload.Stuck_Flag != true)
    {
//        Reload.Began = false;
//        Reload_Reset();
        Reload.Num = num;
			
    }
}

/**
 * @brief  	   ��ȡ����ϵͳ�����Ϣ
 * @param[in]  None
 * @retval 	   shoot power state
 */
bool Shoot_classdef::Get_ShooterPower()
{
    // return(bool)(Referee.GameRobotState.mains_power_shooter_output);
    return (bool)PowerState;
}

uint16_t Shoot_classdef::Get_SpeedLimit()
{
    // return Referee.GameRobotState.shooter_id1_17mm_speed_limit;
    return 30;//SpeedLimit;
}

uint16_t Shoot_classdef::Get_CoolingLimit()
{
    // return Referee.GameRobotState.shooter_id1_17mm_cooling_limit;
    return CoolingLimit;
}

uint16_t Shoot_classdef::Get_CoolingRate()
{
    // return Referee.GameRobotState.shooter_id1_17mm_cooling_rate;
    return CoolingRate;
}

uint16_t Shoot_classdef::Get_GunHeat()
{
    // return Referee.PowerHeatData.shooter_id1_17mm_heat1;
    return GunHeat;
}

float Shoot_classdef::Get_GunSpeed()
{
    return BulletSpeed;
}


