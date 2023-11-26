/**
 ------------------------------------------------------------------------------
 * @file    Control_DR16.cpp
 * @author  Shake
 * @brief   DR16 用户控制
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
 * @brief      拨杆模式更新
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
    case Lever_UP:  // --- 左上 -----------------------------------------------
        switch(DR16.Get_S2_R())
        {
        case Lever_UP: // --- PC端控制——哨兵自动
						if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
            }
						Infantry.Set_GimbalMode(Gimbal_PCMode);
						Infantry.Set_ChassisMode(CHAS_SentryPCMode);
						Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_15); //让摩擦轮开启
						Shoot.Rage_Mode = false;
            break;  /* 左上-右上 END ------------------------------------------*/
				case Lever_MID://——哨兵手动/自动打弹
            if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
								Vision.aim_flag = 0;
            }
						
						Infantry.Set_GimbalMode(Gimbal_PCMode);
//            Infantry.Set_ChassisMode(CHAS_FollowMode);
            Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_30); //让摩擦轮开启
						//----------------------自动打弹
						if(DR16.Get_DW_Norm() < -550)
						{
							Vision.aim_flag = 1;
						}
						else
						{
							Vision.aim_flag = 0;
						}
						//-------------------------手动打弹
            if(DR16.Get_DW_Norm() > 0 && DR16.Get_DW_Norm() < 330 && Shoot_flag == 1) //--- 单发射
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

            if(DR16.Get_DW_Norm() >= 630) //--- 持续发射
            {
                Shoot.ContLaunch = true;
            }
            else
            {
                Shoot.ContLaunch = false;
            }

            break;  /* 左上-右中 END ------------------------------------------*/

        case Lever_DOWN:	// — — 哨兵打弹 
            if(Infantry.Get_CtrlSource() != CTRL_RC)
            {
                Infantry.CtrlSource_Switch(CTRL_RC);
                Infantry.Set_AttackMode(Attack_Disable);
								Vision.aim_flag = 0;
            }

//						Infantry.Set_GimbalMode(Gimbal_PatrolMode);//巡逻
//						Infantry.Set_ChassisMode(CHAS_SentryPCMode);//小陀螺
						
						Infantry.Set_GimbalMode(Gimbal_PatrolMode);//巡逻
						Infantry.Set_ChassisMode(CHAS_SentryPCMode);//雷达全向移动
						

//						Infantry.Set_GimbalMode(Gimbal_PCFollowMode);//跟随
//            Infantry.Set_ChassisMode(CHAS_SentryPCMode);//跟随

//						Infantry.Set_GimbalMode(Gimbal_PatrolMode);
//						Infantry.Set_GimbalMode(Gimbal_PCFollowMode);
//						Infantry.Set_GimbalMode(Gimbal_PCMode);
//            Infantry.Set_ChassisMode(CHAS_LockMode);
//            Infantry.Set_ChassisMode(CHAS_FollowMode);
//						Infantry.Set_ChassisMode(CHAS_SentryPCMode);
            Infantry.Set_VisionMode(Vision_Forcast);
						Infantry.Set_AttackMode(Attack_15); //让摩擦轮开启
						
//						if(DR16.Get_DW_Norm() < -550)
//						{
//							Vision.aim_flag = 1;
//						}
//						else
//						{
//							Vision.aim_flag = 0;
//						}
						
            break;  /* 左上-右下 END ------------------------------------------*/
        }
        break;  // 左上 END ---------------------------------------------------

    case Lever_MID:  // --- 左中 ----------------------------------------------
        Infantry.CtrlSource_Switch(CTRL_RC);
        Infantry.Set_GimbalMode(Gimbal_NormalMode);
				Vision.aim_flag = 0;
        switch(DR16.Get_S2_R())
        {
        case Lever_UP://无自瞄手动发射+跟随
            Infantry.Set_ChassisMode(CHAS_FollowMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);
            //--- 发射使能
						Infantry.Set_AttackMode(Attack_15);//测试拨盘失能摩擦轮


            if (DR16.Get_DW_Norm() > 0 && DR16.Get_DW_Norm() < 330 && Shoot_flag == 1) // --- 单发射
						{
							Shoot_flag = 0;
							Shoot.ContLaunch = false;
							Shoot.Set_ReloadNum(1);
						}
						else if (DR16.Get_DW_Norm() == 0)
						{
							Shoot_flag = 1;
						}

						if (DR16.Get_DW_Norm() == 660) // --- 连续发射
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

            if(DR16.Get_DW_Norm() <= -50) //--- 拨上小陀螺
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

//						if(DR16.Get_DW_Norm() < -550 && Shoot_flag == 1) //--- 单发射
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

            if(DR16.Get_DW_Norm() < -100) //--- 拨上退弹
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

						if(DR16.Get_DW_Norm() >=550 && Shoot_flag)//枪管控制
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
        break;  // 左中 END ---------------------------------------------------

    case Lever_DOWN: // --- 左下 ----------------------------------------------
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
            if(Reset_cnt == 100)//--- 拨轮打上2s重启
            {
								Infantry.Reset();
            }
            
        }
        else
        {
            Infantry.ResetFlag = false;
            Reset_cnt = 0;
        }

        break;  // 左下 END ---------------------------------------------------

        default:
        break;
    }

}

/**
 * @brief      控制源更新
 * @param[in]  None
 * @retval     None
 */
void CTRL_DR16_classdef::CTRLSource_Update(void)
{
    switch(Infantry.Get_CtrlSource())
    {
    case CTRL_RC:  //--- 遥控模式
        Infantry.Enable();
        if(Infantry.Get_State() == Robot_Activating) //---初始化完毕才允许运作
        {
            CTRL_DR16.RCCtrl_Update();
        }
        break;

    case CTRL_PC:   //--- 键鼠模式
        Infantry.Enable();
        if(Infantry.Get_State() == Robot_Activating) //---初始化完毕才允许运作
        {
            CTRL_DR16.PCCtrl_Update();
        }
        break;

    case CTRL_OFF:  //---全局失能
        Infantry.Disable();
        break;
    }
}

/**
 * @brief      RC控制模式 - 对外设置目标值
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
 * @brief      PC控制模式
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
void CTRL_DR16_classdef::PCCtrl_Update(void)//哨兵不用
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
    
    /*--- CTRL+组合键 -----------------------------------------------------------------------------------------------------------*/
    if(DR16.IsKeyPress(KEY_CTRL,PRESS)) 
    {
        if(DR16.IsKeyPress(KEY_X,CLICK) /* && refollow_flag == false */)  /*<! X 反向跟随模式 */
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
                Expt.Target_Yaw = 0;  //--- 反向跟随过度不予鼠标值
                break;
            case CHAS_ReFollowMode:
                Chassis.ReF_Flag = true;
                Infantry.Set_ChassisMode(CHAS_FollowMode);
                Expt.Target_Yaw = 0;  //--- 反向跟随过度不予鼠标值
                break;
            default:
                Chassis.ReF_Flag = false;
                break;
            }
        }
        else if(DR16.IsKeyPress(KEY_E,CLICK)) /*<! E 摩擦轮开关 */
        {
            Infantry.Set_AttackMode(Attack_Disable);
        }
//        else if(DR16.IsKeyPress(KEY_R,CLICK)) /*<! R 弹仓开关 */
//        {
//            Shoot.Mag_Ctrl(OFF);
//        }
        // else if(DR16.IsKeyPress(KEY_Z,CLICK)) /*<! Z 打符模式 */
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
        // else if(DR16.IsKeyPress(KEY_V,CLICK)) /*<! 击打小陀螺模式 */  //--- 改为合并到自动射击按键里
        // {
        //     if(Infantry.Get_VisionMode() != Vision_Top)
        //     {
        //         Infantry.Set_VisionMode(Vision_Top);
        //         Infantry.Set_AttackMode(Attack_Auto);
        //     }
        //     else
        //     {
        //         Infantry.Set_VisionMode(Vision_Forcast);
        //         Infantry.Set_AttackMode(Attack_15); //只是让AttackMode 不处于失能模式 速射在ShootControl会更新.
        //     }
        // }
        else
        {
            //--- 记得关电容
            refollow_flag = false;

            // refollow_cnt = 100;
        }

        // if(DR16.Get_MouseZ_Norm() > 0) //---- 滚轮开关弹仓(可能是因为采样放在了任务里不灵敏)
        // {
        //     Shoot.Mag_Ctrl(ON);
        // }
        // else if(DR16.Get_MouseZ_Norm() < 0)
        // {
        //     Shoot.Mag_Ctrl(OFF);
        // }
    }
    else if(DR16.IsKeyPress(KEY_X,CLICK)) /*<! X 反向跟随归位 */
    {
        if(Infantry.Chassis_Mode == CHAS_ReFollowMode)
        {
            Infantry.Set_ChassisMode(CHAS_FollowMode);
        }
    }
    else if(DR16.IsKeyPress(KEY_E,CLICK))
    {
        Infantry.Set_AttackMode(Attack_15); //只是让AttackMode 不处于失能模式 速射在ShootControl会更新.
    }
//    else if(DR16.IsKeyPress(KEY_R,CLICK)) /*<! R 弹仓开关 */
//    {
//        Shoot.Mag_Ctrl(ON);
//    }
    else
    {
        refollow_cnt = 100;
    }
    //---------------------------- CTRL 组合键 END ------------------------------------------------------

    if(DR16.IsKeyPress(KEY_Z,LONGPRESS)) /*<! Z 打符模式 */
    {
        Infantry.Set_VisionMode(Vision_BIGWindmill);
    }
    else
    {
        Infantry.Set_VisionMode(Vision_Forcast);
    }

    if(DR16.IsKeyPress(KEY_SHIFT,PRESS)) //--- SHIFT 小陀螺
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

    // if(DR16.IsKeyPress(KEY_CTRL,PRESS) && DR16.IsKeyPress(KEY_C,CLICK)) /*<! CTRL+C 电容关闭 */
    // {
    //     Chassis.Cap_Ctrl(OFF);
    // }
    // else if(DR16.IsKeyPress(KEY_C,CLICK))  /*<! C 电容开启 */
    // {
    //     Chassis.Cap_Ctrl(ON);
    // }

    if(DR16.IsKeyPress(KEY_CTRL,PRESS) && DR16.IsKeyPress(KEY_Q,LONGPRESS)) /*<! Q 暴走模式，无视枪管热量 */
    {
        Shoot.Rage_Mode = ON;
    }
    else
    {
        Shoot.Rage_Mode = OFF;
    }

    if(DR16.IsKeyPress(KEY_C,LONGPRESS))  /*<! C 电容开启 */
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

    if(DR16.IsKeyPress(KEY_G, LONGPRESS))  /* 45度 */
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
    
    // if(DR16.IsKeyPress(KEY_Z,PRESS) && DR16.IsKeyPress(KEY_X,PRESS) && DR16.IsKeyPress(KEY_C,CLICK)) //--- Z+X+C 无跟随模式
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


    if(DR16.IsKeyPress(MOUSE_L,CLICK))  /*<! 发射弹丸 */
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

    // if(DR16.IsKeyPress(KEY_R,LONGPRESS) && mag_flag == false) /*<! R 弹仓开关 */
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

    if(DR16.IsKeyPress(MOUSE_R,PRESS))  /*<! 自瞄开启 */
    {
        Infantry.Set_GimbalMode(Gimbal_PCMode);
    }
    else
    {
        Infantry.Set_GimbalMode(Gimbal_NormalMode);
    }
    // if(DR16.IsKeyPress(MOUSE_R,PRESS) && DR16.IsKeyPress(KEY_CTRL,PRESS)) /*<! 自动射击 */
    // {
    //     Infantry.Set_VisionMode(Vision_Top); /*<! 击打小陀螺模式 */
    //     Infantry.Set_AttackMode(Attack_Auto);
    //     autoshoot_flag = true;
    //     autoshoot_cnt = 0;
    // }
    // else
    // {
    //     if(autoshoot_cnt <= 500) //--- 自动射击关闭1s后才允许静步模式开启，防止按键重叠
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

    if(DR16.IsKeyPress(KEY_V,LONGPRESS)) /*<! 自动射击 + 击打小陀螺模式 */
    {
        Infantry.Set_VisionMode(Vision_Top); /*<! 击打小陀螺模式 */
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

    if(Infantry.Get_VisionMode() == Vision_BIGWindmill)  /*<! 打符Pit手动调整 */
    {
        if(DR16.IsKeyPress(KEY_W,CLICK))
        {
            Gimbal.Windmill_Offset[Pit] -= 0.1f;  //--- 上调0.1度
        }
        else if(DR16.IsKeyPress(KEY_S,CLICK))
        {
            Gimbal.Windmill_Offset[Pit] += 0.1f;  //--- 下调0.1度
        }

        if(DR16.IsKeyPress(KEY_A,CLICK))
        {
            Gimbal.Windmill_Offset[Yaw] += 0.1f;  //--- 上调0.1度
        }
        else if(DR16.IsKeyPress(KEY_D,CLICK))
        {
            Gimbal.Windmill_Offset[Yaw] -= 0.1f;  //--- 上调0.1度
        }
    }



    if(DR16.IsKeyPress(KEY_B,PRESS)) /*<! UI 刷新 */
    {
        Infantry.Reflash_UI = true;
    }
    else
    {
        Infantry.Reflash_UI = false;
    }

}

/**
 * @brief      数据监测
 * @param[in]  None
 * @retval     0:Normal  1:Abnormal
 */
int8_t CTRL_DR16_classdef::Data_Monitor(void)
{ 
    if((DR16.Get_S1_L() != Lever_UP && DR16.Get_S1_L() != Lever_MID && DR16.Get_S1_L() != Lever_DOWN && DR16.Get_S1_L() != Lever_NONE) || /*<! 左拨杆 */
       (DR16.Get_S2_R() != Lever_UP && DR16.Get_S2_R() != Lever_MID && DR16.Get_S2_R() != Lever_DOWN && DR16.Get_S2_R() != Lever_NONE) || /*<! 右拨杆 */
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
 * @brief      输出数据重置
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
 * @brief      获取对外输出数据
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

