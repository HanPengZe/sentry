/**
 ------------------------------------------------------------------------------
 * @file    Control_DR16.cpp
 * @author  Shake
 * @brief   DR16 �û�����
 * @version V1.0-
 * @date    2021-10-09
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "Control_DR16.h"
#include "System_DataPool.h"
/* Private macros ------------------------------------------------------------*/
#define DR16DATA_NORMAL   0
#define DR16DATA_ABNORMAL 1
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
/**
 * @brief      Initialize CTRL_DR16 Class
 * @param[in]  None
 * @retval     None
 */
CTRL_DR16_classdef::CTRL_DR16_classdef()
{
    Expt.Target_Vx = 0.0f;
    Expt.Target_Vy = 0.0f;
    Expt.Target_Vw = 0.0f;
    Expt.Target_Yaw = 0.0f;
    Expt.Target_Pit = 0.0f;

    Coe.Yaw_RC = /* 0.07f */0.0015f;
    Coe.Pit_RC = -0.013f;
    Coe.Yaw_PC = 0.005f;
    Coe.Pit_PC = 0.08f;
}

/**
 * @brief      ����ģʽ����
 * @param[in]  None
 * @retval     None
 */
uint8_t Shoot_flag;
uint8_t refollow_flag;
uint8_t Mag_flag;
uint16_t Fric_flag;
uint16_t Reset_cnt;
VisionMode_e visiontemp = Vision_Default;
void CTRL_DR16_classdef::LeverMode_Update(void)
{
    switch(DR16.Get_S1_L())
    {
    case Lever_UP:  // --- ���� -----------------------------------------------
        switch(DR16.Get_S2_R())
        {
        case Lever_UP: // --- PC�˿��ơ����ڱ��Զ�
						if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
            }
						Infantry.Set_GimbalMode(Gimbal_PCMode);
						Infantry.Set_ChassisMode(CHAS_SentryPCMode);
						Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_15); //��Ħ���ֿ���
						Shoot.Rage_Mode = false;
            break;  /* ����-���� END ------------------------------------------*/
				case Lever_MID://�����ڱ��ֶ�/�Զ���
            if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
								Vision.aim_flag = 0;
            }
						
						Infantry.Set_GimbalMode(Gimbal_PCMode);
//            Infantry.Set_ChassisMode(CHAS_FollowMode);
            Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_30); //��Ħ���ֿ���
						//----------------------�Զ���
						if(DR16.Get_DW_Norm() < -550)
						{
							Vision.aim_flag = 1;
						}
						else
						{
							Vision.aim_flag = 0;
						}
						//-------------------------�ֶ���
            if(DR16.Get_DW_Norm() > 0 && DR16.Get_DW_Norm() < 330 && Shoot_flag == 1) //--- ������
            {
                Shoot_flag = 0;
                Shoot.ContLaunch = false;
                Shoot.Set_ReloadNum(1);
            }
            else if(DR16.Get_DW_Norm() == 0)
            {
                Shoot_flag = 1;
                Shoot.Set_ReloadNum(0);
            }

            if(DR16.Get_DW_Norm() >= 630) //--- ��������
            {
                Shoot.ContLaunch = true;
            }
            else
            {
                Shoot.ContLaunch = false;
            }

            break;  /* ����-���� END ------------------------------------------*/

        case Lever_DOWN:	// �� �� �ڱ��� 
            if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
								Vision.aim_flag = 0;
            }

//						Infantry.Set_GimbalMode(Gimbal_PatrolMode);//Ѳ��
//						Infantry.Set_ChassisMode(CHAS_SentryPCMode);//С����
						
						Infantry.Set_GimbalMode(Gimbal_PatrolMode);//Ѳ��
						Infantry.Set_ChassisMode(CHAS_SentryPCMode);//�״�ȫ���ƶ�
						

//						Infantry.Set_GimbalMode(Gimbal_PCFollowMode);//����
//            Infantry.Set_ChassisMode(CHAS_SentryPCMode);//����

//						Infantry.Set_GimbalMode(Gimbal_PatrolMode);
//						Infantry.Set_GimbalMode(Gimbal_PCFollowMode);
//						Infantry.Set_GimbalMode(Gimbal_PCMode);
//            Infantry.Set_ChassisMode(CHAS_LockMode);
//            Infantry.Set_ChassisMode(CHAS_FollowMode);
//						Infantry.Set_ChassisMode(CHAS_SentryPCMode);
            Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_15); //��Ħ���ֿ���
						
//						if(DR16.Get_DW_Norm() < -550)
//						{
//							Vision.aim_flag = 1;
//						}
//						else
//						{
//							Vision.aim_flag = 0;
//						}
						
            break;  /* ����-���� END ------------------------------------------*/
        }
        break;  // ���� END ---------------------------------------------------

    case Lever_MID:  // --- ���� ----------------------------------------------
        Infantry.CtrlSource_Switch(CTRL_RC);
        Infantry.Set_GimbalMode(Gimbal_NormalMode);
				Vision.aim_flag = 0;
        switch(DR16.Get_S2_R())
        {
        case Lever_UP://�������ֶ�����+����
            Infantry.Set_ChassisMode(CHAS_FollowMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);
            //--- ����ʹ��
						Infantry.Set_AttackMode(Attack_15);//���Բ���ʧ��Ħ����


            if (DR16.Get_DW_Norm() > 0 && DR16.Get_DW_Norm() < 330 && Shoot_flag == 1) // --- ������
						{
							Shoot_flag = 0;
							Shoot.ContLaunch = false;
							Shoot.Set_ReloadNum(1);
						}
						else if (DR16.Get_DW_Norm() == 0)
						{
							Shoot_flag = 1;
						}

						if (DR16.Get_DW_Norm() == 660) // --- ��������
						{
							Shoot.ContLaunch = true;
						}
						else
						{
							Shoot.ContLaunch = false;
						}

            break;

        case Lever_MID:
            Infantry.Set_GimbalMode(Gimbal_NormalMode);
            Infantry.Set_AttackMode(Attack_Zero);

            if(DR16.Get_DW_Norm() <= -50) //--- ����С����
            {
                Infantry.Set_ChassisMode(CHAS_SpinMode);
            }
            else
            {
                if(Infantry.Get_ChassisPreMode() != CHAS_ReFollowMode)
                {
                    Infantry.Set_ChassisMode(CHAS_FollowMode);
                }
            }
            break;

        case Lever_DOWN:
            Infantry.Set_ChassisMode(CHAS_LockMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);

//						if(DR16.Get_DW_Norm() < -550 && Shoot_flag == 1) //--- ������
//            {
//                Shoot_flag = 0;
//                Shoot.ContLaunch = false;
//                Shoot.Set_ReloadNum(1);
//            }
//            else if(DR16.Get_DW_Norm() == 0)
//            {
//                Shoot_flag = 1;
//                Shoot.Set_ReloadNum(0);
//            }

            if(DR16.Get_DW_Norm() < -100) //--- �����˵�
            {
                Infantry.Set_AttackMode(Attack_Unload);
                if(DR16.Get_DW_Norm() <= -600)
                {
                    Shoot.ContLaunch = true;
                }
                else
                {
                    Shoot.ContLaunch = false;
                }
            }

						if(DR16.Get_DW_Norm() >=550 && Shoot_flag)//ǹ�ܿ���
						{
							if(Shoot.Turn_Bar_ID==0){Shoot.Turn_Bar_ID=1;Shoot_flag = 0;}
							else if(Shoot.Turn_Bar_ID==1){Shoot.Turn_Bar_ID=0;Shoot_flag = 0;}
							
						}
						else if(DR16.Get_DW_Norm() == 0)
            {
                Shoot_flag = 1;
						}
            break;
        }
        break;  // ���� END ---------------------------------------------------

    case Lever_DOWN: // --- ���� ----------------------------------------------
        Infantry.CtrlSource_Switch(CTRL_OFF);
        Infantry.Disable();
				if(DR16.Get_DW_Norm() >= 550)
				{
					
				}

        // Robot Reset
        if(DR16.Get_DW_Norm() == -660)
        {
            Reset_cnt++;
					
            Infantry.ResetFlag = true;
            if(Reset_cnt == 100)//--- ���ִ���2s����
            {
								Infantry.Reset();
            }
            
        }
        else
        {
            Infantry.ResetFlag = false;
            Reset_cnt = 0;
        }

        break;  // ���� END ---------------------------------------------------

        default:
        break;
    }

}

/**
 * @brief      ����Դ����
 * @param[in]  None
 * @retval     None
 */
void CTRL_DR16_classdef::CTRLSource_Update(void)
{
    switch(Infantry.Get_CtrlSource())
    {
    case CTRL_RC:  //--- ң��ģʽ
        Infantry.Enable();
        if(Infantry.Get_State() == Robot_Activating) //---��ʼ����ϲ���������
        {
            CTRL_DR16.RCCtrl_Update();
        }
        break;

    case CTRL_PC:   //--- ����ģʽ
        Infantry.Enable();
        if(Infantry.Get_State() == Robot_Activating) //---��ʼ����ϲ���������
        {
            CTRL_DR16.PCCtrl_Update();
        }
        break;

    case CTRL_OFF:  //---ȫ��ʧ��
        Infantry.Disable();
        break;
    }
}

/**
 * @brief      RC����ģʽ - ��������Ŀ��ֵ
 * @param[in]  None
 * @retval     None
 */
float test_rxadd = 5.0f;
void CTRL_DR16_classdef::RCCtrl_Update(void)
{
    switch(Infantry.Get_ChassisMode())
    {
    case CHAS_FollowMode:
    case CHAS_ReFollowMode:
        Expt.Target_Vx = DR16.Get_LX_Norm() * 20.0f;
        Expt.Target_Vy = DR16.Get_LY_Norm() * 20.0f;
        // Expt.Target_Vw = DR16.Get_RX_Norm() * (-test_rxadd);
        Expt.Target_Yaw = DR16.Get_RX_Norm() * Gimbal.YawRCCtrl_Coe/* Coe.Yaw_RC */;
        Expt.Target_Pit = DR16.Get_RY_Norm() * Coe.Pit_RC;
        break;

    case CHAS_SpinMode:
		case CHAS_SentryPCMode:
        Expt.Target_Vx = DR16.Get_LX_Norm() * 10.0f;
        Expt.Target_Vy = DR16.Get_LY_Norm() * 10.0f;
        Expt.Target_Vw = (Infantry.Get_CtrlSource() == CTRL_RC ? DR16.Get_DW_Norm()/330*Chassis.Vw_Limit : -8000);
        Expt.Target_Yaw = DR16.Get_RX_Norm() * Gimbal.YawRCCtrl_Coe * 0.5;//0.0025f/* 0.05f */;
        Expt.Target_Pit = DR16.Get_RY_Norm() * Coe.Pit_RC;
        break;

    case CHAS_SwingMode:
        Expt.Target_Vx = DR16.Get_LX_Norm() * 10.0f;
        Expt.Target_Vy = DR16.Get_LY_Norm() * 10.0f;
        Expt.Target_Yaw = DR16.Get_RX_Norm() * 0.002f;
        Expt.Target_Pit = DR16.Get_RY_Norm() * Coe.Pit_RC;
        break;

    case CHAS_LockMode:
    case CHAS_DisableMode:
        Expt.Target_Vx = 0;
        Expt.Target_Vy = 0;
        Expt.Target_Vw = 0;
        Expt.Target_Yaw = DR16.Get_RX_Norm() * Gimbal.YawRCCtrl_Coe/* Coe.Yaw_RC */;
        Expt.Target_Pit = DR16.Get_RY_Norm() * Coe.Pit_RC;
        break;

    case CHAS_NotFollowMode:
        Expt.Target_Vx = DR16.Get_LX_Norm() * 15.0f;
        Expt.Target_Vy = DR16.Get_LY_Norm() * 15.0f;
        Expt.Target_Vw = DR16.Get_RX_Norm() * -15.0f;
        Expt.Target_Yaw = 0;
        Expt.Target_Pit = DR16.Get_RY_Norm() * Coe.Pit_RC;
        break;
    }

}

/**
 * @brief      PC����ģʽ
 * @param[in]  None
 * @retval     None
 */
uint8_t mag_flag;
uint8_t autoshoot_flag;
uint8_t spin_flag;
uint8_t supcap_flag;
float mouse_vw_add = 6.0f;
uint16_t autoshoot_cnt;
// uint8_t refollow_flag;
uint8_t refollow_cnt;
uint8_t nomiddle_flag;

float slop_temp = 30.0f;
void CTRL_DR16_classdef::PCCtrl_Update(void)//�ڱ�����
{
    // static float temp_Vx = 0; 
    // static float temp_Vy = 0;

    refollow_cnt++;

    Expt.Target_Yaw = (float)DR16.Get_MouseX_Norm() * Coe.Yaw_PC;
    Expt.Target_Pit = (float)DR16.Get_MouseY_Norm() * Coe.Pit_PC;
    // Expt.Target_Vw = (float)DR16.Get_MouseY_Norm() * mouse_vw_add;

    Expt.Target_Yaw = MouseX_LPF.f(Expt.Target_Yaw);
    Expt.Target_Pit = MouseY_LPF.f(Expt.Target_Pit);
    // Expt.Target_Vw = Vw_LPF.f(Expt.Target_Vw);

    if(DR16.IsKeyPress(KEY_W,PRESS))
    {
        Chassis.VxVy_Limit = (Infantry.Get_ChassisMode()==CHAS_ReFollowMode?-8500:8500);
        Chassis.VxVy_Limit *= ((DR16.IsKeyPress(KEY_CTRL,PRESS)/* &&autoshoot_cnt>500 */)?0.1f:1.0f);
        Chassis.Drv_Slow(&Expt.Target_Vy, Chassis.VxVy_Limit, slop_temp);


        // if(Infantry.Get_ChassisMode() == CHAS_SpinMode)
        // {
        //     Chassis.Drv_Slow(&Expt.Target_Vy, Chassis.VxVy_Limit, slop_temp);
        // }
        // else
        // {
        //     Expt.Target_Vy = Chassis.VxVy_Limit;
        // }

    }
    else if(DR16.IsKeyPress(KEY_S,PRESS))
    {
        Chassis.VxVy_Limit = (Infantry.Get_ChassisMode()==CHAS_ReFollowMode?-8500:8500);
        Chassis.VxVy_Limit *= ((DR16.IsKeyPress(KEY_CTRL,PRESS)/* &&autoshoot_cnt>500 */)?0.1f:1.0f);
        Chassis.Drv_Slow(&Expt.Target_Vy, -Chassis.VxVy_Limit, slop_temp);


        // if(Infantry.Get_ChassisMode() == CHAS_SpinMode)
        // {
        //     Chassis.Drv_Slow(&Expt.Target_Vy, -Chassis.VxVy_Limit, slop_temp);
        // }
        // else
        // {
        //     Expt.Target_Vy = -Chassis.VxVy_Limit;
        // }
    }
    else
    {
        Expt.Target_Vy = 0;
    }

    if(DR16.IsKeyPress(KEY_A,PRESS))
    {
        Chassis.VxVy_Limit = (Infantry.Get_ChassisMode()==CHAS_ReFollowMode?-8500:8500);
        Chassis.VxVy_Limit *= ((DR16.IsKeyPress(KEY_CTRL,PRESS)/* &&autoshoot_cnt>500 */)?0.1f:1.0f);
        Chassis.Drv_Slow(&Expt.Target_Vx, -Chassis.VxVy_Limit, slop_temp);

        // // Expt.Target_Vx = -Chassis.VxVy_Limit;

        // if(Infantry.Get_ChassisMode() == CHAS_SpinMode)
        // {
        //     Chassis.Drv_Slow(&Expt.Target_Vx, -Chassis.VxVy_Limit, slop_temp);
        // }
        // else
        // {
        //     Expt.Target_Vx = -Chassis.VxVy_Limit;
        // }
    }
    else if(DR16.IsKeyPress(KEY_D,PRESS))
    {
        Chassis.VxVy_Limit = (Infantry.Get_ChassisMode()==CHAS_ReFollowMode?-8500:8500);
        Chassis.VxVy_Limit *= ((DR16.IsKeyPress(KEY_CTRL,PRESS)/* &&autoshoot_cnt>500 */)?0.1f:1.0f);
        Chassis.Drv_Slow(&Expt.Target_Vx, Chassis.VxVy_Limit, slop_temp);

        // Expt.Target_Vx = Chassis.VxVy_Limit;

        // if(Infantry.Get_ChassisMode() == CHAS_SpinMode)
        // {
        //     Chassis.Drv_Slow(&Expt.Target_Vx, Chassis.VxVy_Limit, slop_temp);
        // }
        // else
        // {
        //     Expt.Target_Vx = Chassis.VxVy_Limit;
        // }
    }
    else
    {
        // Chassis.Drv_Slow(&Expt.Target_Vy, 0, 5.0f);
        Expt.Target_Vx = 0;
    }

    // Chassis.AcclerateCurve(&Expt.Target_Vx, &Expt.Target_Vy);
    
    /*--- CTRL+��ϼ� -----------------------------------------------------------------------------------------------------------*/
    if(DR16.IsKeyPress(KEY_CTRL,PRESS)) 
    {
        if(DR16.IsKeyPress(KEY_X,CLICK) /* && refollow_flag == false */)  /*<! X �������ģʽ */
        {
            refollow_flag = true;
            if(Chassis.ReF_Flag == false)
            {
                if(*Gimbal.Get_TargetAngle(Yaw)/ENCODER_ANGLE_RATIO > 0)
                {
                    *Gimbal.Get_TargetAngle(Yaw) -= 180*ENCODER_ANGLE_RATIO;
                }
                else
                {
                    *Gimbal.Get_TargetAngle(Yaw) += 180*ENCODER_ANGLE_RATIO;
                }
            }

            switch(Infantry.Get_ChassisMode())
            {
            case CHAS_FollowMode:
                Chassis.ReF_Flag = true;
                Infantry.Set_ChassisMode(CHAS_ReFollowMode);
                Expt.Target_Yaw = 0;  //--- ���������Ȳ������ֵ
                break;
            case CHAS_ReFollowMode:
                Chassis.ReF_Flag = true;
                Infantry.Set_ChassisMode(CHAS_FollowMode);
                Expt.Target_Yaw = 0;  //--- ���������Ȳ������ֵ
                break;
            default:
                Chassis.ReF_Flag = false;
                break;
            }
        }
        else if(DR16.IsKeyPress(KEY_E,CLICK)) /*<! E Ħ���ֿ��� */
        {
            Infantry.Set_AttackMode(Attack_Disable);
        }
//        else if(DR16.IsKeyPress(KEY_R,CLICK)) /*<! R ���ֿ��� */
//        {
//            Shoot.Mag_Ctrl(OFF);
//        }
        // else if(DR16.IsKeyPress(KEY_Z,CLICK)) /*<! Z ���ģʽ */
        // {
        //     if(Infantry.Get_VisionMode() != Vision_BIGWindmill)
        //     {
        //         Infantry.Set_VisionMode(Vision_BIGWindmill);
        //     }
        //     else
        //     {
        //         Infantry.Set_VisionMode(Vision_Forcast);
        //     }
        // }
        // else if(DR16.IsKeyPress(KEY_V,CLICK)) /*<! ����С����ģʽ */  //--- ��Ϊ�ϲ����Զ����������
        // {
        //     if(Infantry.Get_VisionMode() != Vision_Top)
        //     {
        //         Infantry.Set_VisionMode(Vision_Top);
        //         Infantry.Set_AttackMode(Attack_Auto);
        //     }
        //     else
        //     {
        //         Infantry.Set_VisionMode(Vision_Forcast);
        //         Infantry.Set_AttackMode(Attack_15); //ֻ����AttackMode ������ʧ��ģʽ ������ShootControl�����.
        //     }
        // }
        else
        {
            //--- �ǵùص���
            refollow_flag = false;

            // refollow_cnt = 100;
        }

        // if(DR16.Get_MouseZ_Norm() > 0) //---- ���ֿ��ص���(��������Ϊ���������������ﲻ����)
        // {
        //     Shoot.Mag_Ctrl(ON);
        // }
        // else if(DR16.Get_MouseZ_Norm() < 0)
        // {
        //     Shoot.Mag_Ctrl(OFF);
        // }
    }
    else if(DR16.IsKeyPress(KEY_X,CLICK)) /*<! X ��������λ */
    {
        if(Infantry.Chassis_Mode == CHAS_ReFollowMode)
        {
            Infantry.Set_ChassisMode(CHAS_FollowMode);
        }
    }
    else if(DR16.IsKeyPress(KEY_E,CLICK))
    {
        Infantry.Set_AttackMode(Attack_15); //ֻ����AttackMode ������ʧ��ģʽ ������ShootControl�����.
    }
//    else if(DR16.IsKeyPress(KEY_R,CLICK)) /*<! R ���ֿ��� */
//    {
//        Shoot.Mag_Ctrl(ON);
//    }
    else
    {
        refollow_cnt = 100;
    }
    //---------------------------- CTRL ��ϼ� END ------------------------------------------------------

    if(DR16.IsKeyPress(KEY_Z,LONGPRESS)) /*<! Z ���ģʽ */
    {
        Infantry.Set_VisionMode(Vision_BIGWindmill);
    }
    else
    {
        Infantry.Set_VisionMode(Vision_Forcast);
    }

    if(DR16.IsKeyPress(KEY_SHIFT,PRESS)) //--- SHIFT С����
    {        
        Infantry.Set_ChassisMode(CHAS_SpinMode);
        spin_flag = true;
    }
    else
    {
        if(spin_flag == true)
        {
            Infantry.Set_ChassisMode(CHAS_FollowMode);
            spin_flag = false;
        }
    }

    // if(DR16.IsKeyPress(KEY_CTRL,PRESS) && DR16.IsKeyPress(KEY_C,CLICK)) /*<! CTRL+C ���ݹر� */
    // {
    //     Chassis.Cap_Ctrl(OFF);
    // }
    // else if(DR16.IsKeyPress(KEY_C,CLICK))  /*<! C ���ݿ��� */
    // {
    //     Chassis.Cap_Ctrl(ON);
    // }

    if(DR16.IsKeyPress(KEY_CTRL,PRESS) && DR16.IsKeyPress(KEY_Q,LONGPRESS)) /*<! Q ����ģʽ������ǹ������ */
    {
        Shoot.Rage_Mode = ON;
    }
    else
    {
        Shoot.Rage_Mode = OFF;
    }

    if(DR16.IsKeyPress(KEY_C,LONGPRESS))  /*<! C ���ݿ��� */
    {
        Chassis.Cap_Ctrl(ON);
    }
    else if(DR16.IsKeyPress(KEY_C,CLICK))
    {
        Chassis.Cap_Ctrl(OFF); 
    }

    if(DR16.IsKeyPress(KEY_F, PRESS))
    {
        Chassis.Uphill_Mode = true;
    }
    else
    {
        Chassis.Uphill_Mode = false;
    }

    if(DR16.IsKeyPress(KEY_G, LONGPRESS))  /* 45�� */
    {
        Chassis.NoMiddle_Mode = true;
    }
    else if(DR16.IsKeyPress(KEY_G,CLICK))
    {
        Chassis.NoMiddle_Mode = false;
    }
    else
    {
        nomiddle_flag = false;
    }
    
    // if(DR16.IsKeyPress(KEY_Z,PRESS) && DR16.IsKeyPress(KEY_X,PRESS) && DR16.IsKeyPress(KEY_C,CLICK)) //--- Z+X+C �޸���ģʽ
    // {
    //     if(Infantry.Get_ChassisMode() != CHAS_NotFollowMode)
    //     {
    //         Gimbal.Motor[Yaw].Reset();
    //         Infantry.Set_ChassisMode(CHAS_NotFollowMode);
    //     }
    //     else
    //     {
    //         Infantry.Set_ChassisMode(CHAS_FollowMode);
    //     }
    // }


    if(DR16.IsKeyPress(MOUSE_L,CLICK))  /*<! ���䵯�� */
    {
        Shoot.ContLaunch = false;
        Shoot.Set_ReloadNum(1);
    }
    else if(DR16.IsKeyPress(MOUSE_L,LONGPRESS))
    {
        Shoot.ContLaunch = true;
    }
    else
    {
        Shoot.ContLaunch = false;
    }

    // if(DR16.IsKeyPress(KEY_R,LONGPRESS) && mag_flag == false) /*<! R ���ֿ��� */
    // {
    //     Shoot.Mag_Ctrl(ON);
    //     mag_flag = true;
    // }
    // else if(DR16.IsKeyPress(KEY_R,CLICK))
    // {
    //     Shoot.Mag_Ctrl(OFF);
    //     mag_flag = false;
    // }
    // else
    // {
    //     mag_flag = true;
    // }

    if(DR16.IsKeyPress(MOUSE_R,PRESS))  /*<! ���鿪�� */
    {
        Infantry.Set_GimbalMode(Gimbal_PCMode);
    }
    else
    {
        Infantry.Set_GimbalMode(Gimbal_NormalMode);
    }
    // if(DR16.IsKeyPress(MOUSE_R,PRESS) && DR16.IsKeyPress(KEY_CTRL,PRESS)) /*<! �Զ���� */
    // {
    //     Infantry.Set_VisionMode(Vision_Top); /*<! ����С����ģʽ */
    //     Infantry.Set_AttackMode(Attack_Auto);
    //     autoshoot_flag = true;
    //     autoshoot_cnt = 0;
    // }
    // else
    // {
    //     if(autoshoot_cnt <= 500) //--- �Զ�����ر�1s���������ģʽ��������ֹ�����ص�
    //     {
    //         autoshoot_cnt++;
    //     }
        
    //     if(autoshoot_flag == true)
    //     {
    //         Infantry.Set_VisionMode(Vision_Forcast);
    //         Infantry.Set_AttackMode(Attack_15);
    //         autoshoot_flag = false;
    //     }
    // }

    if(DR16.IsKeyPress(KEY_V,LONGPRESS)) /*<! �Զ���� + ����С����ģʽ */
    {
        Infantry.Set_VisionMode(Vision_Top); /*<! ����С����ģʽ */
        Infantry.Set_AttackMode(Attack_Auto);
    }
    else if(DR16.IsKeyPress(KEY_V,CLICK))
    {
        Infantry.Set_VisionMode(Vision_Forcast);
        if(Infantry.Get_AttackMode() == Attack_Disable)
        {
            Infantry.Set_AttackMode(Attack_15);
        }
    }

    if(Infantry.Get_VisionMode() == Vision_BIGWindmill)  /*<! ���Pit�ֶ����� */
    {
        if(DR16.IsKeyPress(KEY_W,CLICK))
        {
            Gimbal.Windmill_Offset[Pit] -= 0.1f;  //--- �ϵ�0.1��
        }
        else if(DR16.IsKeyPress(KEY_S,CLICK))
        {
            Gimbal.Windmill_Offset[Pit] += 0.1f;  //--- �µ�0.1��
        }

        if(DR16.IsKeyPress(KEY_A,CLICK))
        {
            Gimbal.Windmill_Offset[Yaw] += 0.1f;  //--- �ϵ�0.1��
        }
        else if(DR16.IsKeyPress(KEY_D,CLICK))
        {
            Gimbal.Windmill_Offset[Yaw] -= 0.1f;  //--- �ϵ�0.1��
        }
    }



    if(DR16.IsKeyPress(KEY_B,PRESS)) /*<! UI ˢ�� */
    {
        Infantry.Reflash_UI = true;
    }
    else
    {
        Infantry.Reflash_UI = false;
    }

}

/**
 * @brief      ���ݼ��
 * @param[in]  None
 * @retval     0:Normal  1:Abnormal
 */
int8_t CTRL_DR16_classdef::Data_Monitor(void)
{ 
    if((DR16.Get_S1_L() != Lever_UP && DR16.Get_S1_L() != Lever_MID && DR16.Get_S1_L() != Lever_DOWN && DR16.Get_S1_L() != Lever_NONE) || /*<! �󲦸� */
       (DR16.Get_S2_R() != Lever_UP && DR16.Get_S2_R() != Lever_MID && DR16.Get_S2_R() != Lever_DOWN && DR16.Get_S2_R() != Lever_NONE) || /*<! �Ҳ��� */
       (DR16.Get_RX_Norm() > 660 || DR16.Get_RX_Norm() < 660) ||                                                                    /*<! CH0 */
       (DR16.Get_RY_Norm() > 660 || DR16.Get_RY_Norm() < 660) ||                                                                    /*<! CH1 */
       (DR16.Get_LX_Norm() > 660 || DR16.Get_LX_Norm() < 660) ||                                                                    /*<! CH2 */
       (DR16.Get_LY_Norm() > 660 || DR16.Get_LY_Norm() < 660) ||                                                                    /*<! CH3 */
       (DR16.Get_DW_Norm() > 660 || DR16.Get_DW_Norm() < 660))                                                                      /*<! CH4 */
    {
        return DR16DATA_ABNORMAL;
    }
    else
    {
        return DR16DATA_NORMAL;
    }
}


/**
 * @brief      �����������
 * @param[in]  None
 * @retval     None
 */
void CTRL_DR16_classdef::ExptData_Reset(void)
{
    Expt.Target_Vx = 0;
    Expt.Target_Vy = 0;
    Expt.Target_Vw = 0;
    Expt.Target_Yaw = 0;
    Expt.Target_Pit = 0;
}

/**
 * @brief      ��ȡ�����������
 * @param[in]  None
 * @retval     export data
 */
float CTRL_DR16_classdef::Get_ExptVx()
{
    return Expt.Target_Vx;
}
float CTRL_DR16_classdef::Get_ExptVy()
{
    return Expt.Target_Vy;
}
float CTRL_DR16_classdef::Get_ExptVw()
{
    return Expt.Target_Vw;
}
float CTRL_DR16_classdef::Get_ExptYaw()
{
    return Expt.Target_Yaw;
}
float CTRL_DR16_classdef::Get_ExptPit()
{
    return Expt.Target_Pit;
}
uint8_t CTRL_DR16_classdef::IsAnyOutput()
{
    if((DR16.Get_RX_Norm()||DR16.Get_RY_Norm()||DR16.Get_LX_Norm()||DR16.Get_LY_Norm()||DR16.Get_DW_Norm())==0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

