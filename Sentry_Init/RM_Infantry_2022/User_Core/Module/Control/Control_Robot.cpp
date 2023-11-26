/* Includes ------------------------------------------------------------------*/
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "stm32f4xx_hal.h"

uint32_t Get_RefereeTime()
{
	return xTaskGetTickCount()*1000;
}

Robot_classdef::Robot_classdef()
{
    //--- ��ʼ��
	// Referee.Init(&huart3, Get_RefereeTime);
}

/**
 * @brief  	  �������ܿ���
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Control()
{
    Chassis.Control();
    Gimbal.Control();
//    Shoot.Control();

//		Shoot.Send_Motor[0].Out = 0;
		
		//YAW
//		Gimbal.Send_Motor[0].Out = 0;
//		Gimbal.Send_Motor[1].Out = 0;
		Gimbal.Send_Motor[2].Out = 0;//2��3����ò���
		Gimbal.Send_Motor[3].Out = 0;//���Ը�ֵ0
		MotorMsgSend(&hcan1, Gimbal.Send_Motor);//Gimbal[0]���͸���̨Yaw�ᣬGimbal[1]�ǲ���
		MotorMsgSend(&hcan2, Shoot.Send_Motor);//������/*<! ��һ�����ڻ�ǹ�ܣ��ڶ���������̨Pit����������Ħ���ֵ�� *///�����Ӿ�YAW����ע��Pitch��
		MotorMsgSend(&hcan2, Shoot.Card_Motor);

    //--- ������������������
    SendMsgtoCHAS();

    DevicesMonitor.Get_FPS(&Infantry.TIME, &Infantry.FPS);
}

/**
 * @brief  	  ������ʹ��
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Enable()
{
    if (State == Robot_Disability) //--- ����״̬����
    {
        State = Robot_Activating;
    }
    if (State != Robot_Initializing)
    {
        State = Robot_Activating;
    }
}

/**
 * @brief  	  ������ʧ��,״̬��λ
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Disable()
{
    if (Get_State() != Robot_Initializing || Get_State() != Robot_Disability)
    {
        State = Robot_Disability;
    }

    CTRL_DR16.ExptData_Reset();
    Set_ChassisMode(CHAS_DisableMode);
    Set_GimbalMode(Gimbal_DisableMode);
    Set_AttackMode(Attack_Disable);
    Set_VisionMode(Vision_Forcast);

    Gimbal.Windmill_Offset[0] = 0.0f;
    Gimbal.Windmill_Offset[1] = 0.0f;
}

/**
 * @brief  	    ����������(������)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_classdef::Reset()
{   
    //--- оƬ��λ
    __set_FAULTMASK(1);    //�ر������ж�
    HAL_NVIC_SystemReset();//��λ
}

/**
 * @brief  	  �����˿���Դת��
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::CtrlSource_Switch(DR16Status_Typedef mode)
{
    //--- ����ģʽ����
    if (Get_CtrlSource() != mode)
    {
        // M6020 Yaw Reset
        Gimbal.Motor[Yaw].Reset();
    }
    Ctrl_Source = mode;
}

/**
 * @brief  	  ���õ�������ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_ChassisMode(CHAS_CtrlMode_e mode)
{
    if ((Get_ChassisMode() == CHAS_SpinMode || Get_ChassisMode() == CHAS_LockMode) && mode != CHAS_SpinMode)
    {
        // Reset M6020 Yaw
        Gimbal.Motor[Yaw].Reset();
    }
		
		if (Get_ChassisMode() == CHAS_SentryPCMode && mode != CHAS_SentryPCMode)
    {
        // Reset M6020 Yaw
        Gimbal.Motor[Yaw].Reset();
    }

    Chassis_Mode = mode;
}

/**
 * @brief  	  ������̨����ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
    if(mode == Gimbal_PCMode && DevicesMonitor.Get_State(VISION_MONITOR) == Off_line)
    {
        Gimbal_Mode = Gimbal_NormalMode;
        PC_State = Off_line;
        return;
    }
    PC_State = On_line;
    Gimbal_Mode = mode;
}

/**
 * @brief  	    ���÷�������ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Robot_classdef::Set_AttackMode(Attack_CtrlMode_e mode)
{
    if(Get_AttackMode() == Attack_Auto && mode != Attack_Auto)
    {
        //--- ���Զ����ģʽ��ȡ��ʱ�������Զ����������
        Shoot.Set_ReloadNum(0);
    }
		else if(Attack_Mode != mode)
		{
			Vision.aim_flag = 0;
		}

    Attack_Mode = mode;

    switch(Get_AttackMode())
    {
    case Attack_30:
		Fric_Mode = Fric_30m_s;
		break;
	case Attack_18:
		Fric_Mode = Fric_18m_s;
		break;
    case Attack_22:
        Fric_Mode = Fric_22m_s;
        break;
	case Attack_15:
		Fric_Mode = Fric_15m_s;
		break;

	case Attack_Auto:
		switch (Shoot.Get_SpeedLimit())
		{
		case 15:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		case 18:
			Fric_Mode = Fric_18m_s; // --- 18m/s
			break;
        case 22:
        case 25:
            Fric_Mode = Fric_22m_s; // --- 22m/s
            break;
		case 30:
        case 75:
			Fric_Mode = Fric_30m_s; // --- 30m/s
			break;

		default:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		}
		break;

	case Attack_Unload:
		Fric_Mode = Fric_Unload;
		break;

	case Attack_Disable:
		Fric_Mode = Fric_Disable;
		Shoot.Rage_Mode = false;
		break;
	
	case Attack_Zero:
		Fric_Mode = Fric_Zero;
		Shoot.Rage_Mode = false;
	break;

	default:
		Fric_Mode = Fric_Disable;
		Shoot.Rage_Mode = false;
		break;
	
	
    }
}

/**
 * @brief  	    �����Ӿ�����ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Robot_classdef::Set_VisionMode(VisionMode_e mode)
{
    PC_State = (DevicesMonitor.Get_State(VISION_MONITOR) == Off_line?Off_line:On_line);

    Vision_Mode = mode;

    switch(mode)
    {
    case Vision_Default:
    case Vision_Forcast:
    case Vision_Top:
        if(Infantry.Get_ChassisMode() == CHAS_LockMode && Infantry.Get_CtrlSource() == CTRL_PC)
        {
            //--- PCģʽ�˳����ģʽʱ�л�����
            Infantry.Set_ChassisMode(CHAS_FollowMode);
        }
        break;
    case Vision_BIGWindmill:
        //--- ������̲���
        Infantry.Set_ChassisMode(CHAS_LockMode);
        break;
    default:
        break;
    }
}

/**
 * @brief  	    ���ͨ�� ��������
 * @param[in]	mode
 * @note        0x340
 * @retval 	    None
 */
void Robot_classdef::SendMsgtoCHAS()
{
    static uint8_t test_cnt = 0;
    test_cnt++;

    //--- 0x340

    //--- �ϲ����ؼ�����ĵ����ٶ�
    SendData[0] = Chassis.Get_TargetVx() >> 8;
    SendData[1] = Chassis.Get_TargetVx();

    SendData[2] = Chassis.Get_TargetVy() >> 8;
    SendData[3] = Chassis.Get_TargetVy();

    SendData[4] = Chassis.Get_TargetVw() >> 8;
    SendData[5] = Chassis.Get_TargetVw();

    SendData[6] = 0;
		SendData[7] = 0;

    //--- ����ģʽ(bit 0-3)
    SendData[6] |= Infantry.Get_ChassisMode()&0x0F;
    //--- �Ӿ�ģʽ(bit 4-6)
    SendData[6] |= (Infantry.Get_VisionMode()&0x07)<<4;

    if(test_cnt == 1)
    {
        //--- Data[6] bit7 ��ʶ�� 0:UI���豸��־λ
        SendData[6] |= 0;
    
        //--- ģ����״̬
        SendData[7] |= (DevicesMonitor.Get_State(VISION_MONITOR) == Off_line?1<<0:0);
        SendData[7] |= ((DevicesMonitor.Get_State(FRIC_L_MONITOR)||DevicesMonitor.Get_State(FRIC_R_MONITOR)||DevicesMonitor.Get_State(RELOAD_MONITOR)) == Off_line?1<<1:0);

        //--- ��������ģʽ
        SendData[7] |= Chassis.Uphill_Mode == true?1<<2:0;
    
        //--- ������������
        SendData[7] |= ResetFlag==true?1<<3:0;
    
        //--- ���ݿ���
        SendData[7] |= Chassis.Cap_switch==ON?1<<4:0;
        //--- Ħ����״̬
        SendData[7] |= Infantry.Get_AttackMode()==Attack_Disable?0:1<<5;
        //--- ���ֿ���
        SendData[7] |= Shoot.Mag_Switch==OFF?0:1<<6;
        //--- UIˢ��
        SendData[7] |= Reflash_UI<<7;
    }
    else if((test_cnt%=2)==0)
    {
        //--- Data[6] bit7 ��ʶ�� 1:�Ӿ����
        SendData[6] |= 1<<7;

        SendData[7] = (uint8_t)(Vision.Get_DepthOffset()/100);
    }

    CANx_SendData(&hcan1, 0x340, SendData, 8);

}

/**
 * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
 * @param[in]  id, can_rx_data
 * @retval     None
 */
void Robot_classdef::RefereeMsg_Write(uint16_t id, uint8_t can_rx_data[])
{
    switch(id)
    {
    case 0x341:
        uint8_t temp[4];
        float *speed;       //--- �ӵ����ٶ�
        temp[0] = can_rx_data[0];
        temp[1] = can_rx_data[1];
        temp[2] = can_rx_data[2];
        temp[3] = can_rx_data[3];
        speed = (float*)(&temp);
        Shoot.BulletSpeed = *speed;
		
				//��ǰ�����µĿɷ�������
				CanShootNum[0] = can_rx_data[4];
				CanShootNum[1] = can_rx_data[5];
		
				HurtArmor_NoZero = can_rx_data[6];

        DetectColor = (can_rx_data[7]>>7)&1;   //--- �з���ɫ
        Shoot.PowerState = (can_rx_data[7]>>6)&1; //---�����Դ״̬
        Shoot.ShootID = (can_rx_data[7]>>5)&1;  //--- ǹ��ID
				God_State[4] = (can_rx_data[7]>>4)&1;  //--- 5�Ų���
				God_State[3] = (can_rx_data[7]>>3)&1;  //--- 4�Ų���
				God_State[2] = (can_rx_data[7]>>2)&1;  //--- 3�Ų���
				God_State[1] = (can_rx_data[7]>>1)&1;  //--- 2�Ź���
				God_State[0] = (can_rx_data[7]>>0)&1;  //--- 1��Ӣ��
			break;
				
			case 0x342:
				Vision.Radar_Send_Msg.Pack.game_progress = can_rx_data[0];
				Vision.Radar_Send_Msg.Pack.master_control = can_rx_data[1];
				Vision.Radar_Send_Msg.Pack.master_control_x = can_rx_data[2]<<8 | can_rx_data[3];
				Vision.Radar_Send_Msg.Pack.master_control_y = can_rx_data[4]<<8 | can_rx_data[5];
				//�������ݵĽ��մ���
				ShootRecNum = can_rx_data[6];
				if(lastShootRecNum==250 && ShootRecNum==1)
				{
					mul_num++;
				}
				if(lastShootRecNum!=ShootRecNum)
				{
					Shoot.ShootNumTurnTiming = HAL_GetTick();
					Shoot.ShootNumTurnTime1 = Shoot.ShootNumTurnTiming-Shoot.ShootStartTiming[Shoot.Turn_Bar_ID];
					Shoot.ShootNumTurnTime2 = Shoot.ShootNumTurnTiming-Shoot.ShootOverTiming[Shoot.Turn_Bar_ID];;
				}
				TrueShootRecNum = mul_num*250+ShootRecNum;
				lastShootRecNum = ShootRecNum;
//				//��ȴ����
//				CoolBuff = can_rx_data[7];
//				if(CoolBuff>5 || CoolBuff==0)
//				{
//					CoolBuff=1;
//				}
				Enemy_Sentry_God_State=can_rx_data[7];
			break;
			
			case 0x343:
				Vision.Radar_Send_Msg.Pack.game_progress_remain = can_rx_data[0]<<8 | can_rx_data[1];
				Vision.Radar_Send_Msg.Pack.sentry_hp = can_rx_data[2]<<8 | can_rx_data[3];
				Vision.Radar_Send_Msg.Pack.bullet_remain = can_rx_data[4]<<8 | can_rx_data[5];
				Vision.Radar_Send_Msg.Pack.defense_hp = can_rx_data[6]<<8 | can_rx_data[4];
			break;

    default:
        break;
    }
}


/**
 * @brief  	    ��ȡ�����������Ϣ
 * @param[in]	None
 * @retval 	    robot data
 */
uint8_t Robot_classdef::Get_DetectColor()
{
    return DetectColor;
		//�Լ������Է��졪���Է��췢0
		//�Լ��죬�Է��������Է�����1
//	}
}
uint8_t Robot_classdef::Get_ID()
{
    // return Referee.GameRobotState.robot_id;
		return ID;
}
RobotState_e Robot_classdef::Get_State()
{
    return State;
}
DR16Status_Typedef Robot_classdef::Get_CtrlSource()
{
    return Ctrl_Source;
}
CHAS_CtrlMode_e Robot_classdef::Get_ChassisMode()
{
    return Chassis_Mode;
}
CHAS_CtrlMode_e Robot_classdef::Get_ChassisPreMode()
{
    return Chassis_PreMode;
}
Gimbal_CtrlMode_e Robot_classdef::Get_GimbalMode()
{
    return Gimbal_Mode;
}
Attack_CtrlMode_e Robot_classdef::Get_AttackMode()
{
    return Attack_Mode;
}
Fric_CtrlMode_e Robot_classdef::Get_FricMode()
{
    return Fric_Mode;
}
VisionMode_e Robot_classdef::Get_VisionMode()
{
    return Vision_Mode;
}
uint16_t Robot_classdef::Get_FPS()
{
    return FPS;
}

