#include "Robot_Config.h"
#include "Control_Shoot.h"
#include "System_DataPool.h"
#include "tim.h"

// --- 单次卡弹时间上限
#define Limit_StuckDuration 120
// --- 拨盘单个齿轮的角度
#define GearTeeth_Angle     95784//(36859*52/20)测试拨盘电机 */ /* 32764 //(8191*36/9)9弹丸电机*/
// --- 拨弹装置1的转动方向，取反为 -1
#define Reload_Dir      1
// --- 摩擦轮安装方式 正置 1 倒置 -1
#define Fric_Install    1
//--- 反转齿轮数
#define RollBack_Num    2
//--- 射频
#define SHOOT_FREQ(x)  (uint16_t)(500/(x))

// --- 摩擦轮转速
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
	
		Barrel_PID[PID_Outer].SetPIDParam(0.4f, 0.0f, 0.0f, 10000, 10000, 0.002f);//枪管
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
 * @brief  	   发射控制
 * @param[in]  None
 * @retval 	   None
 */
int BarrelRecovery;
void Shoot_classdef::Control()
{
    static uint16_t Mag_off_cnt = 1;
		//弹仓拨弹计数_手动计算
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
		
		if(Infantry.Get_AttackMode() == Attack_Disable ||//失能发射机构
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
		
    Fric_Process();//控制摩擦轮
		//		ShootFreq_Calc();裁判系统限制发弹，传出射频:Shoot_Freq
    AttackMode_Ctrl();//判断射频，传出弹丸填装数:Reload.Num， Reload.CanShoot_Num = 10;
    Reload_Calc();//先由裁判系统判断热量(这里测试我先注释掉)，再使用Reload.Num累加拨盘的目标值
		Barrel_Calc();//枪口初始化，枪管运动

    //--- 防止操作手忘记关弹仓 哨兵不要
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
            Mag_Ctrl(OFF); //--- PC模式运动状态下1s关一次
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
 * @brief  	   摩擦轮处理
 * @param[in]  None
 * @retval 	   None
 */
void Shoot_classdef::Fric_Process()
{
    /*<! 此版本只有 M3508 摩擦轮电机处理
    自21赛季开始不再使用Snail电机 */

    // --- 判断发射系统是否正常工作
#if USE_RM_Referee
	if(Get_ShooterPower() == OFF)//裁判系统发送过来的数据
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

    FricSpeed_Adapt(); //--- 射速自调整

    if(Infantry.Get_CtrlSource() == CTRL_PC && Infantry.Get_AttackMode() != Attack_Disable)
    {
        // Laser_Ctrl(ON);

        if(Infantry.Get_VisionMode() == Vision_BIGWindmill)
        {
            Laser_Ctrl(OFF);  //--- 打符不开激光
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
            Laser_Ctrl(ON); //--- RC控制的时候关摩擦轮不关激光是为了不用开摩擦轮才有激光
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

    //--- 摩擦轮PID计算
    FricPID_Calc(Fric_Speed);

}

/**
 * @brief  	   摩擦轮转速调整 (Test ing)
 * @param[in]  枪管反馈速度 射速区间 递减值 递增值
 * @note       在转速调整的基础上加入温控处理，因为经过测试发现温度越高射速也越高
 * @note       优先测15 18 30，22几乎用不到
 * @retval 	   None
 */
float Pre_GunSpeed;
float temp_scope = 35;  //假设变化范围为35摄氏度
float temp_init = 35;    //初始温度设定为35摄氏度

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

    //--- 判断射速是否更新 因为裁判系统反馈的射速是float类型的 相邻两次的数据几乎不会相等
    if(Get_GunSpeed() != Pre_GunSpeed)
    {
        //----- 射速波动调整 -----//
        if(Get_GunSpeed() < min_speed[fricmode] && Get_GunSpeed() > 10.0f)
        {
            speederr_flag = 1;
        }
        else if(Get_GunSpeed() >= min_speed[fricmode] && Get_GunSpeed() <= max_speed[fricmode])
        {
            speederr_flag = 0;
        }

        if(speederr_flag == 1) //--- 射速偏低
        {
            speederr_flag = 0;
            Fric_SpeedOffset[fricmode] += add_val[fricmode];
        }
        if(Get_GunSpeed() > max_speed[fricmode]) //--- 射速偏高
        {
            Fric_SpeedOffset[fricmode] -= sub_val[fricmode];
        }

        //----- 电机温控补偿 -----//
        //--- 两个摩擦轮电机反馈回来的平均温度
        motor_temp = ((float)Fric_Motor[Fric_L].getTempature()+(float)Fric_Motor[Fric_R].getTempature()) / 2.0f;
        
        if(motor_temp >= temp_init) //--- 小幅度升温
        {
            Fric_TempOffset[fricmode] = (motor_temp - temp_init)/temp_scope * Fric_Temp_Coe[fricmode];
        }
        if(motor_temp < temp_init) //--- 没到达温度临界值
        {
            Fric_TempOffset[fricmode] = 0;
        }
        if(motor_temp > temp_init + temp_scope) //--- 升温太多
        {
            Fric_TempOffset[fricmode] = Fric_Temp_Coe[fricmode];
        }
    }

    Pre_GunSpeed = Get_GunSpeed(); 
}



/**
 * @brief  	    根据枪管数据计算射频
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t Shoot_classdef::ShootFreq_Calc()
{
	
    if(Infantry.CanShootNum[Turn_Bar_ID] > 1) //--- 计算当前枪管剩余热量
    {
        if(Infantry.CanShootNum[Turn_Bar_ID] == 2) //--- 刚好满足阈值-能射两颗
        {
            Shoot_Freq = 8;
        }
        else if(Infantry.CanShootNum[Turn_Bar_ID] <= 4) //--- 剩余在某个阈值之内 4颗子弹
        {
            Shoot_Freq = (Infantry.CanShootNum[Turn_Bar_ID] / 2) * 10 + 8;
        }
        else
        {
            Shoot_Freq = 40; //---未达到热量警告区间，满频输出
        }
    }
    else //--- 不满足刚好发射阈值数子弹
    {
        Shoot_Freq = 0;
    }

    return Shoot_Freq;
}


/**
 * @brief  	   拨盘拨弹计算函数
 * @param[in]  None
 * @retval 	   None
 */
uint8_t CanShoot_Limit = 4; //--- 可发射数阈值，避免因偷打超热量
uint8_t KaDanTime, KaDanTimeLimit=100;
uint8_t Turn_Bar_ID;
uint8_t turn_bar_flag;
uint8_t turning_bar_flag;
void Shoot_classdef::Reload_Calc()
{
    Reload_Motor.encoder_offset = 0;

    // --- 判断发射系统是否正常工作
#if USE_RM_Referee
    if(Get_ShooterPower() == OFF)
    {
        Reload.Num = 0;
        return;
    }
#endif

    if(Infantry.Get_AttackMode() == Attack_Zero || Infantry.Get_AttackMode() == Attack_Disable || DevicesMonitor.Get_State(RELOAD_MONITOR) == Off_line)
    {
        //--- 以当前位置为启动位置
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
		
		
		//存在问题——没写完
    //--- 先执行上次被激活的卡弹处理任务
    if(Reload.Stuck_Duration > Limit_StuckDuration)
    {
				//卡弹处理
				//卡弹处理结束
				//拨盘再前进一格
        //---- 已完成卡弹处理任务
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
	  //--- 有发射弹丸需求
    if(Reload.Num > 0)
    {
			/*-----------------------------------------------------------------------------------------改动

//				//根据裁判系统热量切枪管
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
//					Turn_Bar_ID=0;//暂时
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

				
        //--- 射频不满足发送需求
        if(Shoot_Freq <= 0)
        {
            Reload.Num = 0;
            return;
        }
				
				//视觉发射的特殊限制
				if(Vision.Sentry_Recv_Msg.Pack.auto_aim==0 && Vision.aim_flag == 1)
				{
					Reload.Num = 0;
				}
//				if(Reload.Target_Angle-Reload_Motor.get_totalencoder()>GearTeeth_Angle+16382)
//				{
//					Reload.Target_Angle -= GearTeeth_Angle * (int)((Reload.Target_Angle-Reload_Motor.get_totalencoder())/GearTeeth_Angle);
//				}

        // --- 新任务未开始执行
        if(Reload.Began == false && Reload.Num != 0)
        {
						ShootStartTiming[Turn_Bar_ID]=HAL_GetTick();
            // --- 重置准备进入状态
            Reload.Began = true;
            Reload_Reset();
            Reload.Init_Angle = Reload_Motor.get_totalencoder();
//            Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;
						Card_Target_Angle+=8191*12;        
				}

        // --- 拨弹任务完成
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
            // --- 更新剩余待拨动的弹丸
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
				

				
				//存在问题——没写完
				// --- 卡弹检测
//        if(Judge_ReloadStall() == true)
//        {
//            Reload.Stuck_Duration++;
//            if(Reload.Stuck_Duration > Limit_StuckDuration)
//            {
//								Reload.Began = false;
//                Reload.Stuck_Flag = true; // --- 激活卡弹任务，下一次开始反转
//                Reload.Motor_Direction = 1; // --- 开始执行首次卡弹处理任务
//								//根据测试看是否停在原地
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
    //--- 先执行上次被激活的卡弹任务
    if(Reload.Stuck_Duration > Limit_StuckDuration)
    {
        if(Judge_ReloadStall() == true) //--- 扭矩过大，避免执行反转任务时也卡弹
        {
            Reload.Stuck_Duration++;
            //--- 卡弹时间为 200~300，正转，需改变方向
            if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 0 && Reload.Motor_Direction == 1)
            {
                Reload.Re_TargetAngle += GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 0; //--- 进行正转反堵
            }
            else if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 1 && Reload.Motor_Direction == 0)
            {
                Reload.Re_TargetAngle -= GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 1; //--- 进行反转
            }
        }

        //---- 已完成卡弹反转任务
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
    
    //--- 有发射弹丸需求
    if(Reload.Num > 0)
    {
        // --- 不满足连发状态
        // if(Reload.CanShoot_Num < CanShoot_Limit && ContLaunch == true)
        // {
        //     Reload.Num = 0;
        //     return;
        // }
        // // --- 不满足单发状态
        // else if(Reload.CanShoot_Num < (CanShoot_Limit-2) && ContLaunch == false)
        // {
        //     Reload.Num = 0;
        //     return;
        // }

        //--- 射频不满足发送需求
        if(Shoot_Freq <= 0)
        {
            Reload.Num = 0;
            return;
        }

        // --- 新任务未开始执行
        if(Reload.Began == false)
        {
            // --- 重置准备进入状态
            Reload.Began = true;
            Reload_Reset();
            Reload.Init_Angle = Reload_Motor.get_totalencoder();
            Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;
        }

        // --- 拨弹任务完成
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
            // --- 更新剩余待拨动的弹丸
            Reload.Launched_Num = abs(Reload_Motor.get_totalencoder() - Reload.Init_Angle) / GearTeeth_Angle;
        }

        // --- 卡弹检测
        if(Judge_ReloadStall() == true)
        {
            Reload.Stuck_Duration++;
            if(Reload.Stuck_Duration > Limit_StuckDuration)
            {
                Reload.Stuck_Flag = true; // --- 激活卡弹任务，下一次开始反转
                Reload.Motor_Direction = 1; // --- 开始执行首次反转任务
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

    ReloadPID_Calc(Reload.Target_Angle+32764/4);//下面的是拨盘电机
		CardPID_Calc(Card_Target_Angle);//上面枪口前面的是卡弹电机
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
//			//--- 以当前位置为启动位置
//			tar_bar = Reload_Motor.get_totalencoder();
//			return;
//	} 
//	-1224 129920
	if(init_bar_flag)//初始化
	{
		//热量判断
		//根据裁判系统热量切枪管
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
			
			//自己算
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
				//右
				if(abs(Reload_Motor.get_totalencoder() - (Reload.Target_Angle+32764/4))<8192)
				{tar_bar = init_bar+132000+text_bar;}
			}
			else
			{
				//左 初始枪管
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
 * @brief  	   射击模式处理
 * @param[in]  None
 * @retval 	   None
 */
uint8_t biubiubiu; // 一次吐n颗
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
            Shoot_Freq = 20; //--- 换血模式解除热量限制
        }
        else if(Infantry.Get_VisionMode() == Vision_BIGWindmill && Shoot_Freq >= 10)
        {  
            //--- 打符连发射频给小点
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
    //--- 根据当前热量得出可击打弹丸数量
    //--- 换血模式无视热量
    Reload.CanShoot_Num = ((Rage_Mode == true || Infantry.Get_AttackMode() == Attack_Unload) ? 10 : (Get_CoolingLimit() - Get_GunHeat()) / Onebullet_heat);
#else
    Reload.CanShoot_Num = 10;
#endif


}


/**
 * @brief  	   拨盘重置
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
 * @brief  	   判断拨盘电机是否处于卡弹临界值
 * @param[in]  None
 * @retval 	   state
 */
bool Shoot_classdef::Judge_ReloadStall()
{
    return(abs(Reload_Motor.getSpeed()) < Reload.CriticalVal);
}

/**
 * @brief  	   发射机构 电机PID计算
 * @param[in]  None
 * @retval 	   None
 * @note       因为摩擦轮和拨盘电机是一样的标识符发送电流，不能再单独调用一次MotorMsgSend()来发送拨盘电流
 *             否则会造成摩擦轮的电流一直是0和目标电流来回跳动
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
    // --- 注意摩擦轮安装方式
    Fric_PID[Fric_L].Target = tar_speed*Fric_Install;
    Fric_PID[Fric_L].Current = Fric_Motor[Fric_L].getSpeed();
    Send_Motor[SendID_Fric_L].Out = Fric_Motor[Fric_L].Out = Fric_PID[Fric_L].Cal();
		

    Fric_PID[Fric_R].Target = -tar_speed*Fric_Install;
    Fric_PID[Fric_R].Current = Fric_Motor[Fric_R].getSpeed();
    Send_Motor[SendID_Fric_R].Out = Fric_Motor[Fric_R].Out = Fric_PID[Fric_R].Cal();
}

/**
 * @brief  	   激光红点控制
 * @param[in]  state
 * @retval 	   None
 */
void Shoot_classdef::Laser_Ctrl(bool state)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, (GPIO_PinState)state);
}
/**
 * @brief  	   弹仓开关控制
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
 * @brief  	   PWM初始化
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
 * @brief  	   设置弹丸装填数
 * @param[in]  num
 * @retval 	   None 
 */
void Shoot_classdef::Set_ReloadNum(uint16_t num)
{
    //--- 防止数据异常
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
 * @brief  	   获取裁判系统相关信息
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


