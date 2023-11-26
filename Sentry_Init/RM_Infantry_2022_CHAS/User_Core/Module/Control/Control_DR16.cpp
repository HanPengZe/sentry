/**
 ------------------------------------------------------------------------------
 * @file    Control_DR16.cpp
 * @author  Shake
 * @brief   DR16 用户控制
 * @version V0.1
 * @date    2021-10-09
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "Control_DR16.h"
#include "System_DataPool.h"
/* Private macros ------------------------------------------------------------*/
#define LEVER_PERSONAL 1 // 自己用
#define LEVER_STANDARD 2 // 规范用
#define LEVER_MODE     1

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
    Coe.Pit_RC = -0.025f;
    Coe.Pit_PC = -0.025f;
}

/**
 * @brief      拨杆模式更新
 * @param[in]  None
 * @retval     None
 */
uint8_t Reflag;
uint8_t Shoot_Flag;
void CTRL_DR16_classdef::LeverMode_Update(void)
{
#if LEVER_MODE == LEVER_PERSONAL

    switch(DR16.Get_S1_L())
    {
    case Lever_UP:  // --- 左上 -----------------------------------------------
        switch(DR16.Get_S2_R())
        {
        case Lever_UP:
            if(Infantry.Ctrl_Source != CTRL_PC)
            {
                Infantry.Set_GimbalMode(Gimbal_NormalMode);
                Infantry.Set_ChassisMode(CHAS_FollowMode);
            }
            Infantry.CtrlSource_Switch(CTRL_PC);
            break;  /* 左上-右上 END ------------------------------------------*/
        case Lever_MID:

            break;  /* 左上-右中 END ------------------------------------------*/

        case Lever_DOWN:

            // Infantry.Set_AttackMode(Attack_Manual_15);
            // if(DR16.Get_DW_Norm() >= 330)
            // {
            //     Shoot.Set_ReloadNum(1);
            // }

            // if (DR16.Get_DW_Norm() > 0 && DR16.Get_DW_Norm() < 330 && Shoot_Flag == 1) // --- 单发射
			// {
			// 	Shoot_Flag = 0;
			// 	Shoot.ContShoot_Flag = false;
			// 	Shoot.Set_ReloadNum(1);
			// }
			// else if (DR16.Get_DW_Norm() == 0)
			// {
			// 	Shoot_Flag = 1;
			// }

			// if (DR16.Get_DW_Norm() == 660) // --- 连续发射
			// {
			// 	Shoot.ContShoot_Flag = true;
			// }
			// else
			// {
			// 	Shoot.ContShoot_Flag = false;
			// }

            break;  /* 左上-右下 END ------------------------------------------*/
        }
        break;  // 左上 END ---------------------------------------------------

    case Lever_MID:  // --- 左中 ----------------------------------------------
        Infantry.CtrlSource_Switch(CTRL_RC);

        switch(DR16.Get_S2_R())
        {
        case Lever_UP:
            // Infantry.Set_ChassisMode(CHAS_FollowMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);

            // if(DR16.Get_DW_Norm() == 660)
            // {
            //     Infantry.Set_ChassisMode(CHAS_SpinMode);
            //     test_Vz = 7000;
            // }
            /* else  */if(DR16.Get_DW_Norm() <= -50)
            {
                test_Vz = DR16.Get_DW_Norm() * (-15.0f);
                Infantry.Set_ChassisMode(CHAS_SpinMode);
            }
            else
            {
                Infantry.Set_ChassisMode(CHAS_FollowMode);
            }

            break;

        case Lever_MID:
            Infantry.Set_ChassisMode(CHAS_NotFollowMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);
            
            break;

        case Lever_DOWN:
            Infantry.Set_ChassisMode(CHAS_LockMode);
            Infantry.Set_GimbalMode(Gimbal_NormalMode);
            break;
        }
        break;  // 左中 END ---------------------------------------------------

    case Lever_DOWN: // --- 左下 ----------------------------------------------
        Infantry.CtrlSource_Switch(CTRL_OFF);
        // Infantry.Disable();

        // Robot Reset
        break;  // 左下 END ---------------------------------------------------

        default:
        break;
    }

#elif LEVER_MODE == LEVER_STANDARD

#endif
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
int8_t test_rxadd = 6;
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
        Expt.Target_Vx = DR16.Get_LX_Norm() * 10.0f;
        Expt.Target_Vy = DR16.Get_LY_Norm() * 10.0f;
        Expt.Target_Yaw = DR16.Get_RX_Norm() * 0.0025f/* 0.05f */;
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
        Expt.Target_Vw = DR16.Get_RX_Norm() * 15.0f;
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
float M_Vw_Factor = 0.5f;
void CTRL_DR16_classdef::PCCtrl_Update(void)
{
    static float temp_Vx = 0; 
    static float temp_Vy = 0;

    Expt.Target_Yaw = (float)DR16.Get_MouseX_Norm() * 0.05f;
    // Expt.Target_Vw = (float)DR16.Get_MouseX_Norm() * M_Vw_Factor;

    Expt.Target_Yaw = Yaw_LPF.f(Expt.Target_Yaw);
    // Expt.Target_Vw = Vw_LPF.f(Expt.Target_Vw);

    if(DR16.IsKeyPress(KEY_W,PRESS))
    {
        temp_Vy = RAMP_Output(5000/* Chassis.VxVy_Limit */, temp_Vy, 10.0f);
        Expt.Target_Vy = temp_Vy * (Infantry.Chassis_Mode == CHAS_ReFollowMode?-1.0f:1.0f);

    }
    else if(DR16.IsKeyPress(KEY_S,PRESS))
    {
        temp_Vy = RAMP_Output(-5000/* Chassis.VxVy_Limit */, temp_Vy, 10.0f);
        Expt.Target_Vy = temp_Vy * (Infantry.Chassis_Mode == CHAS_ReFollowMode?-1.0f:1.0f); 
    }
    else
    {
        temp_Vy = 0;
        Expt.Target_Vy = 0;
    }

    if(DR16.IsKeyPress(KEY_A,PRESS))
    {
        temp_Vx = RAMP_Output(-5000/* Chassis.VxVy_Limit */, temp_Vx, 10.0f);
        Expt.Target_Vx = temp_Vx * (Infantry.Chassis_Mode == CHAS_ReFollowMode?-1.0f:1.0f); 
    }
    else if(DR16.IsKeyPress(KEY_D,PRESS))
    {
        temp_Vx = RAMP_Output(5000/* Chassis.VxVy_Limit */, temp_Vx, 10.0f);
        Expt.Target_Vx = temp_Vx  * (Infantry.Chassis_Mode == CHAS_ReFollowMode?-1.0f:1.0f); 
    }
    else
    {
        temp_Vx = 0;
        Expt.Target_Vx = 0;
    }

    
    /*--- CTRL+组合键 -----------------------------------------------------------------------------------------------------------*/
    if(DR16.IsKeyPress(KEY_CTRL,PRESS)) 
    {
        if(DR16.IsKeyPress(KEY_Z,CLICK))  /*<! 小陀螺模式 */
        {
            
            if(Infantry.Chassis_Mode != CHAS_SpinMode)
            {
                Infantry.Set_ChassisMode(CHAS_SpinMode);
            }
            else
            {
                Infantry.Set_ChassisMode(CHAS_FollowMode);
            }
        }
        else if(DR16.IsKeyPress(KEY_C,CLICK))  /*<! 反向跟随模式 */
        {
            if(*Gimbal.Get_TargetAngle(Yaw)/ENCODER_ANGLE_RATIO > 0)
            {
                Gimbal.Target.AIMU_Yaw -= 180;
            }
            else
            {
                Gimbal.Target.AIMU_Yaw += 180;
            }

            switch(Infantry.Chassis_Mode)
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
        else if(DR16.IsKeyPress(KEY_F,CLICK)) /*<! 180掉头,非反向跟随模式 */
        {
            if(*Gimbal.Get_TargetAngle(Yaw)/ENCODER_ANGLE_RATIO > 0)
            {
                Gimbal.Target.AIMU_Yaw -= 180;
            }
            else
            {
                Gimbal.Target.AIMU_Yaw += 180;
            }
        }
 
    }
    else if(DR16.IsKeyPress(KEY_C,CLICK)) /*<! 反向跟随归位 */
    {
        if(Infantry.Chassis_Mode == CHAS_ReFollowMode)
        {
            Infantry.Set_ChassisMode(CHAS_FollowMode);
        }
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

