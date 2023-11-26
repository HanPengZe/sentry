/* Includes ------------------------------------------------------------------*/
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "DJI_AIMU.h"

uint32_t Get_RefereeTime()
{
	return xTaskGetTickCount()*1000;
}

Robot_classdef::Robot_classdef()
{
    //--- 初始化
	// Referee.Init(&huart3, Get_RefereeTime);	
  
}

/**
 * @brief  	  机器人总控制
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Control()
{
    static uint8_t reset_cnt = 0;
		
    //纯底盘不用
    if(DevicesMonitor.Get_State(COMMU_0X340_MONITOR) == Off_line)
   {
       Chassis.Target_Vx = Chassis.Target_Vy = Chassis.Target_Vw = 0;
       Chassis_Mode = CHAS_DisableMode;
       Vision_Mode = Vision_Disable;

    //    SupCap.Main_Switch(SupCap_OFF, Charging_OFF, Power_NotSupply);

       for(uint8_t i = 0 ; i < 8 ; i++)
       {
           Write_Msg[i] = false;
       }
   }

//    SupCap.Control();   //--- 超电控制 哨兵不用
    
    Chassis.Control();  //--- 底盘控制

    // ShootFreq_Calc();   //--- 计算发射射频

//    Auto.Hurt_Position();
//		Auto.God_Detection();

    // ToDo...
    // if(Write_Msg[Robot_Reset] == true && DevicesMonitor.Get_State(COMMU_0X340_MONITOR) != Off_line)
    // {
    //     reset_cnt++;
    //     if(reset_cnt>=1000) //--- 收到重启信号持续2s重启
    //     {
    //         Reset();
    //     }
    // }
    // else
    // {
    //     reset_cnt = 0;
    // }

    //--- 裁判系统信息发送至云台主控
//    RefereeMsg_Send_Shoot(RefereeShootData);
//    RefereeMsg_Send_Pos(RefereePosData);
//    RefereeMsg_Send_HP(RefereeHpData);

    DevicesMonitor.Get_FPS(&Infantry.TIME, &Infantry.FPS);
}

/**
 * @brief  	  机器人使能
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Enable()
{
    if (State == Robot_Disability) //--- 发生状态跳变
    {
        State = Robot_Activating;
    }
    if (State != Robot_Initializing)
    {
        State = Robot_Activating;
    }
}

/**
 * @brief  	  机器人失能,状态复位
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Disable()
{
    if (Get_State() != Robot_Initializing || Get_State() != Robot_Disability)
    {
        State = Robot_Disability;
    }

    // CTRL_DR16.ExptData_Reset();
    Set_ChassisMode(CHAS_DisableMode);
    // Set_GimbalMode(Gimbal_DisableMode);
    // Set_AttackMode(Attack_Disable);
}

/**
 * @brief  	    机器人重启(软重启)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_classdef::Reset()
{   
    //--- 芯片复位
    __set_FAULTMASK(1);     //--- 关闭所有中断
    HAL_NVIC_SystemReset(); //--- 复位
}

/**
 * @brief  	  机器人控制源转换
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::CtrlSource_Switch(DR16Status_Typedef mode)
{
    //--- 发生模式跳变
    if (Get_CtrlSource() != mode)
    {
        // M6020 Yaw Reset
        // Gimbal.Motor[Yaw].Reset();
    }
    Ctrl_Source = mode;
}

/**
 * @brief  	  设置底盘运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_ChassisMode(CHAS_CtrlMode_e mode)
{
    if ((Get_ChassisMode() == CHAS_SpinMode || Get_ChassisMode() == CHAS_LockMode) && mode != CHAS_SpinMode)
    {
        // Reset M6020 Yaw
        // Gimbal.Motor[Yaw].Reset();
    }

    Chassis.Mode = Chassis_Mode = mode;
}

/**
 * @brief  	  设置云台运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
    Gimbal_Mode = mode;
}



/**
 * @brief      向云台主板发送裁判系统数据
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;
void Robot_classdef::RefereeMsg_Send_Shoot(uint8_t *data)
{  
    //--- 0x341
    uint8_t *temp;
    float bulletspeed = Referee.ShootData.bullet_speed;
    temp = (uint8_t*)(&bulletspeed);

    data[0] = temp[0]; //--- 枪管射速
    data[1] = temp[1];
    data[2] = temp[2];
    data[3] = temp[3];
    
    //当前热量下的可发射数量
		if(Referee.GameRobotState.shooter_id1_17mm_cooling_limit<Get_SentryGunHeat1()){data[4]=100;}
		else{data[4]=(uint8_t)(Referee.GameRobotState.shooter_id1_17mm_cooling_limit-Get_SentryGunHeat1())/10;}
		if(Referee.GameRobotState.shooter_id2_17mm_cooling_limit<Get_SentryGunHeat2()){data[5]=100;}
		else{data[5]=(uint8_t)(Referee.GameRobotState.shooter_id2_17mm_cooling_limit-Get_SentryGunHeat2())/10;}
			
//    data[4] = (uint8_t)(24-Get_SentryGunHeat1()/10);
//    data[5] = (uint8_t)(24-Get_SentryGunHeat2()/10);

   //被击打的装甲板
   Auto.Hurt_Position();
   data[6] = Auto.HurtArmor_NoZero;

    //bit7:对方颜色 bit6:发射机构源接口状态 bit5:枪管ID
    //bit4——bit0 敌方机器人是否处于无敌状态
    data[7] = 0;    
    data[7] |= Get_SentryEnemyColor()<<7;
    data[7] |= Referee.GameRobotState.mains_power_shooter_output == ON?1<<6:0;
		if(Referee.ShootData.shooter_id<1 || Referee.ShootData.shooter_id>2)
		{
			Referee.ShootData.shooter_id=1;
		}
    data[7] |= (Referee.ShootData.shooter_id-1)<<5;  //
    data[7] |= Auto.God_State[4]<<4;//
    data[7] |= Auto.God_State[3]<<3;//
    data[7] |= Auto.God_State[2]<<2;//
    data[7] |= Auto.God_State[1]<<1;//
    data[7] |= Auto.God_State[0]<<0;//

    CANx_SendData(&hcan1,0x341,data,8);
}

/**
 * @brief      向云台主板发送裁判系统数据
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;
void Robot_classdef::RefereeMsg_Send_Pos(uint8_t *data)
{   
    //--- 0x342
    uint16_t ParmData;
    data[0] = Referee.GameState.game_progress;//当前比赛阶段
    data[1] = Get_SentryKeyBoard();//云台手按键

    //目标点位置
    ParmData = Get_SentryMapX();//云台手指定位置X轴坐标
    data[2] = ParmData>>8; //--- 
    data[3] = ParmData;
    ParmData = Get_SentryMapY();//云台手指定位置Y轴坐标
    data[4] = ParmData>>8; //--- 
    data[5] = ParmData;
    
    //发射数据的接收次数
    if(Auto.ShootData_RecNum==251){Auto.ShootData_RecNum=1;}
    data[6] = Auto.ShootData_RecNum; //--- 

    //对方前哨战血量
    if(Get_ID()>100)//己方蓝,对方红
    {
			if(Referee.GameRobotHP.red_outpost_HP>0){data[7] = 1;}
			else{data[7] = 0;}
    }
    else//己方红,对方蓝
    {
			if(Referee.GameRobotHP.red_outpost_HP>0){data[7] = 1;}
			else{data[7] = 0;}
    }
		

    CANx_SendData(&hcan1,0x342,data,8);
}
/**
 * @brief      向云台主板发送裁判系统数据
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;
void Robot_classdef::RefereeMsg_Send_HP(uint8_t *data)
{   
    //--- 0x343
    uint16_t ParmData;
    //比赛剩余时间
	ParmData = Referee.GameState.stage_remain_time;
    data[0] = ParmData>>8; //--- 
    data[1] = ParmData;
    //自身剩余血量
    ParmData = Referee.GameRobotState.remain_HP;
    data[2] = ParmData>>8; //--- 
    data[3] = ParmData;
    //17mm 弹丸允许发弹量
    ParmData = Referee.BulletRemaining.bullet_remaining_num_17mm;
    data[4] = ParmData>>8; //--- 
    data[5] = ParmData;
    //己方前哨战血量
    if(Get_ID()>100)//己方蓝
    {
        ParmData = Referee.GameRobotHP.blue_outpost_HP;
    }
    else
    {
        ParmData = Referee.GameRobotHP.red_outpost_HP;
    }
    data[6] = ParmData>>8; //--- 
    data[7] = ParmData;

    CANx_SendData(&hcan1,0x343,data,8);
}

/**
 * @brief      写入从云台主板接收的数据
 * @param[in]  can_rx_data
 * @retval     None
 */
uint8_t test_vision[2];
void Robot_classdef::WriteMsgFromGimbal(uint8_t can_rx_data[])
{
    //--- 0x340

    //--- 底盘目标速度
    Chassis.Target_Vx = ((int16_t)((can_rx_data[0]<<8)|can_rx_data[1]));
    Chassis.Target_Vy = ((int16_t)((can_rx_data[2]<<8)|can_rx_data[3]));
    Chassis.Target_Vw = ((int16_t)((can_rx_data[4]<<8)|can_rx_data[5]));

    //--- 底盘模式(0-3bit)
    Chassis_Mode = Chassis.Mode = (CHAS_CtrlMode_e)(can_rx_data[6]&0x0F);

    //--- 视觉模式(4-6bit)
    Vision_Mode = (VisionMode_e)((can_rx_data[6]&0x70)>>4);

    if((can_rx_data[6]&0x80)>>7 == 0) //--- UI与设备标志位
    {
        for(uint8_t i = 0 ; i < 8 ; i++)
        {
            Write_Msg[i] = 1&(can_rx_data[7]>>i);
        }
    }
    else if((can_rx_data[6]&0x80)>>7 == 1) //--- 视觉深度
    {
        Vision_Depth = can_rx_data[7];
    }
}

/**
 * @brief  	    发射数据处理 用于UI显示
 * @param[in]	bullet_speed
 * @retval 	    None
 */
void Robot_classdef::ShootMsg_Process(float bullet_speed)
{
    
}

/**
 * @brief  	    根据枪管数据计算射频
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t Robot_classdef::ShootFreq_Calc()
{
//    if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat > 1/* CanShoot_Limit */) //--- 计算当前枪管剩余热量
//    {
//        if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat == 2) //--- 刚好满足阈值-能射两颗
//        {
//            Shoot_Freq = Get_CoolingRate()/Onebullet_heat;
//        }
//        else if(Get_CoolingLimit()-Get_GunHeat() <= Onebullet_heat*4) //--- 剩余在某个阈值之内 4颗子弹
//        {
//            Shoot_Freq = ((Get_CoolingLimit()-Get_GunHeat()) / (Onebullet_heat*2)) * (20 - Get_CoolingRate()/Onebullet_heat) + Get_CoolingRate()/Onebullet_heat;
//        }
//        else
//        {
//            Shoot_Freq = 20; //---未达到热量警告区间，满频输出
//        }
//    }
//    else //--- 不满足刚好发射阈值数子弹
//    {
//        Shoot_Freq = 0;
//    }

    return Shoot_Freq;
}


/**
 * @brief  	    获取机器人相关信息
 * @param[in]	mode
 * @retval 	    None
 */
uint8_t Robot_classdef::Get_ID()
{
    return Referee.GameRobotState.robot_id;
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
Gimbal_CtrlMode_e Robot_classdef::Get_GimbalMode()
{
    return Gimbal_Mode;
}
// Attack_CtrlMode_e Robot_classdef::Get_AttackMode()
// {
//     return Attack_Mode;
// }
// Fric_CtrlMode_e Robot_classdef::Get_FricMode()
// {
//     return Fric_Mode;
// }
// VisionMode_e Robot_classdef::Get_VisionMode()
// {
//     return Vision_Mode;
// }
// uint8_t Robot_classdef::Get_VisionDepth()
// {
//     return Vision_Depth;
// }
// uint16_t Robot_classdef::Get_CoolingLimit()
// {
//     return Referee.GameRobotState.shooter_id1_17mm_cooling_limit;
// }

// uint16_t Robot_classdef::Get_CoolingRate()
// {
//     return Referee.GameRobotState.shooter_id1_17mm_cooling_rate;
// }

// uint16_t Robot_classdef::Get_GunHeat()
// {
//     return Referee.PowerHeatData.shooter_id1_17mm_heat;
// }


uint16_t Robot_classdef::Get_SentryGunHeat1()
{
    return Referee.PowerHeatData.shooter_id1_17mm_heat;
}
uint16_t Robot_classdef::Get_SentryGunHeat2()
{
    return Referee.PowerHeatData.shooter_id2_17mm_heat;
}
uint16_t Robot_classdef::Get_SentryMapX()
{
    return (uint16_t)(Referee.mini_map_data.target_position_x*10);
}
uint16_t Robot_classdef::Get_SentryMapY()
{
    return (uint16_t)(Referee.mini_map_data.target_position_y*10);
}
uint8_t Robot_classdef::Get_SentryKeyBoard()
{
    return Referee.mini_map_data.commd_keyboard;
}
uint8_t Robot_classdef::Get_SentryEnemyColor()
{
    // return Referee.GameRobotState.robot_id;
	if(Referee.GameRobotState.robot_id>=100)
	{
		//自己蓝，对方红——对方红发0
		return 0;
	}
	else
	{
		//自己红，对方蓝——对方蓝发1
		return 1;
	}
}




