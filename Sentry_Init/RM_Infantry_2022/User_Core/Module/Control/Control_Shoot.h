#ifndef _CONTROL_SHOOT_H_
#define _CONTROL_SHOOT_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"



enum FricMotor_ID_e
{
    //面向枪管为准
    Fric_L,
    Fric_R
};

enum Attack_CtrlMode_e
{
    Attack_Disable,		//失能攻击
	Attack_15,	        //手动瞄准 - 低速
	Attack_18,	        //手动瞄准 - 中速
	Attack_22,	        //手动瞄准 - 中高速
	Attack_30,	        //手动瞄准 - 高速
	Attack_BigWindmill, //打符模式
	Attack_Auto,		//自动射击
	Attack_Zero,
	Attack_Unload		//退弹
	
};

//摩擦轮模式
enum Fric_CtrlMode_e
{
	Fric_Disable, // 关闭摩擦轮
	Fric_Zero, // 关闭摩擦
	Fric_Unload,
	Fric_15m_s,	  // 15m/s
	Fric_18m_s,	  // 18m/s
	Fric_22m_s,   // 22m/s
	Fric_30m_s    // 30m/s
};

/*拨盘参数*/
typedef struct
{
	uint8_t Began;		    // 开始供弹
	uint8_t Began2;		    // 开始供弹
	uint8_t Stuck_Flag;		// 卡弹标志位
	int8_t Motor_Direction;	// 电机转动方向  1 -> 正  | -1 -> 反
	int16_t Stuck_Duration;	// 卡弹持续的时长
    uint16_t Num;           // 弹丸装填数
    uint16_t CanShoot_Num;  // 可射击数/
	uint16_t CriticalVal;	// 拨盘的接线速度值->卡弹临界值
	int32_t Init_Angle;	// 初始位置
	int32_t Launched_Num;	// 已经发射的子弹数
	int32_t Target_Angle;   // 正常拨弹目标值
	int32_t Re_TargetAngle; // 解除卡弹反转目标值

} Reload_t;

class Shoot_classdef
{
private:
		
    Reload_t Reload;          /*<! 拨盘参数 */
	
	int16_t Fric_Speed;       /*<! 摩擦轮转速 */
	uint16_t Fric_Output;	  /*<! 摩擦轮输出 */

	int32_t Card_Target_Angle;

    void ReloadPID_Calc(float tar_angle); /*<! 拨盘电机PID计算 */
    void BarrelPID_Calc(float tar_angle); /*<! 拨盘电机PID计算 */
    void FricPID_Calc(float tar_speed);   /*<! 摩擦轮电机PID计算 */
		void CardPID_Calc(float tar_angle);   /*<! 卡弹电机PID计算 */

public:
		//枪管、Pit、左右摩擦轮
		Motor_M3508 Send_Motor[4] = {Motor_M3508(5), Motor_M3508(6),Motor_M3508(7),Motor_M3508(8)};  /*<! 第一个用于换枪管，第二个用于云台Pit，三四用于摩擦轮电机 */
    Motor_M3508 Fric_Motor[2] = {Motor_M3508(7), Motor_M3508(8)};   /*<! 摩擦轮电机 */
    Motor_M2006 Reload_Motor = Motor_M2006(6);                      /*<! 拨盘电机 */
    Motor_M2006 Barrel_Motor = Motor_M2006(5);                      /*<! 枪管电机 */
		Motor_M2006 Card_Motor = Motor_M2006(1);                        /*<! 卡弹电机 */

    PositionPID Reload_PID[2];   /*<! 拨盘电机 位置式PID */
		PositionPID Barrel_PID[2];	 /*<! 枪管电机 位置式PID */
		PositionPID Card_PID[2];		 /*<! 卡弹电机 位置式PID */
    IncrementPID Fric_PID[2];     /*<! 摩擦轮 增量式PID */

    bool Mag_Switch;          /*<! 弹仓开关标志位 */

    uint8_t Onebullet_heat = 10; /*<! 单个子弹的热量(小弹丸) */

    uint8_t Rage_Mode;        /*<! 狂暴模式,无视热量限制 */
    uint8_t ContLaunch;       /*<! 连发标志位 */
		
		uint8_t ShootID;
		uint8_t Turn_Bar_ID;

    uint8_t PowerState;
    uint16_t SpeedLimit;
    uint16_t CoolingRate;
    uint16_t CoolingLimit;
    uint16_t GunHeat;

    uint16_t Mshoot_Cnt,Ashoot_Cnt; /* <! 拨弹延时 */
    uint16_t Shoot_Freq=10;        /*<! 射频 */

    float BulletSpeed;      /*<! 裁判系统子弹速度 */
    float Fric_SpeedOffset[4] = {0}; /*<! 摩擦轮速度波动补偿值 */
    float Fric_TempOffset[4] = {0};  /*<! 摩擦轮温度波动补偿值 */
		
		uint16_t CanShoot_Num[2]={24,24};//哨兵240
		uint32_t ShootStartTiming[2],ShootOverTiming[2],ShootTime[2];
		uint32_t turnShoot_startTiming,turnShoot_overTiming,turnShoot_Time;
		uint32_t ShootNumTurnTiming,ShootNumTurnTime1,ShootNumTurnTime2;
		

    Shoot_classdef();

    void Control();          /*<! 发射控制 */
    void PWM_Init();         /*<! PWM 初始化 */
    void Devices_Init();     /*<! 模块初始化 */
    void Fric_Process();     /*<! 摩擦轮处理 */
		uint8_t ShootFreq_Calc();
    void Reload_Calc();      /*<! 拨盘计算 */
    void Barrel_Calc();      /*<! 枪管计算 */
    void Reload_Reset();     /*<! 拨盘重置 */
    void Set_ReloadNum(uint16_t num);   /*<! 设置发射弹丸数量 */
    void Mag_Ctrl(uint8_t state);       /*<! 弹仓控制 */
    void Laser_Ctrl(bool state);/*<! 激光控制 */
    void Set_FricOut();         /*<! 设置摩擦轮输出 */

    void FricSpeed_Adapt(); /*<! 摩擦轮转速调整 */

    void AttackMode_Ctrl();    /*<! 发射模式处理 */

    bool Judge_ReloadStall(); /*<! 判断是否堵转 */

    void ContShoot();
    
    bool Get_ShooterPower();     /*<! 获取发射模块电源状态 */
    uint16_t Get_CoolingLimit(); /*<! 获取枪管热量限制值 */
    uint16_t Get_CoolingRate();  /*<! 获取枪管冷却速率 */
    uint16_t Get_SpeedLimit();   /*<! 获取枪管射速限制 */
    uint16_t Get_GunHeat();      /*<! 获取枪管当前热量 */
    float Get_GunSpeed();        /*<! 获取当前射速 */


};

extern uint8_t biubiubiu;
extern uint8_t biubiubiu_flag;

#endif
