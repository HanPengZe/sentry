#ifndef _CONTROL_CHASSIS_H_
#define _CONTROL_CHASSIS_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include "Control_DR16.h"

#ifdef __cplusplus

#pragma diag_suppress 550

/* Exported types ------------------------------------------------------------*/


#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0


/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_201 = 0,
    LF_202 = 1,
    LB_203 = 2,
    RB_204 = 3
};
/* --- 底盘转向电机 ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_205 = 0,
    LF_206 = 1,
    LB_207 = 2,
    RB_208 = 3
};

/* --- 底盘控制模式 ------------------------------------------------------------*/
enum CHAS_CtrlMode_e
{
	CHAS_DisableMode,	    // 失能模式
	CHAS_FollowMode ,	    // 跟随模式
	CHAS_ReFollowMode,      // 反向跟随
	CHAS_SpinMode ,	        // 小陀螺模式
	CHAS_sinSpinMode,       // 变速小陀螺
	CHAS_AutoTrackMode,     // 自动追踪模式
	CHAS_LockMode,		    // 锁住底盘
	CHAS_NotFollowMode,	    // 无跟随模式
	CHAS_SentryFollowMode ,	    // 跟随模式
	CHAS_SentrySpinMode ,	        // 小陀螺模式
	CHAS_SentryNotFollowMode,
	CHAS_SentryPCMode,

	CHAS_SupplyMode,	    // 补给模式
	CHAS_SwingMode,		    // 扭腰模式
	CHAS_InitMode,          // 初始化阶段
};


/* --- 转向轮电机相关参数 -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // 初始化校准角度
    float Target_angle; // 目标角度
    float PreTar_angle; // 前一次目标角度
    float Total_angle;  // 当前总角度
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

/* 滤波后的目标值 */
typedef struct 
{
    float Vx;
    float Vy;
    float Vw;
}FilterTarget_t;

/* 加速曲线相关参数 */
typedef struct
{
    uint8_t accelerating;
    uint8_t decelerating;
    float linnerSpeed;
    float linnerSpeedLast;
    float accCnt;
    float accK = 0.25;
    float accKp;
    float deceleRecodeSpeed[2];
}AccelerateParam_t;



class Chassis_classdef
{
private:
    // float Target_Vx, Target_Vy, Target_Vw;
    int16_t Cal_Speed[4];
    uint8_t Lspeed_Flag;  /*<! 低速标志位(无裁判系统使用) */
    // float VxVy_Coe; //修改为 public
    // float VxVy_Limit;
    // float Vw_Limit;
    void Process(float Vx, float Vy, float Vw);             /*<! 底盘数据处理 */
    void Set_MaxSpeed();                                    /*<! 速度限制 */
    void Follow_Ctrl(float *Vx, float *Vy, float *Vw);      /*<! 跟随模式 */
		void Sentry_Follow_Ctrl(float *Vx, float *Vy, float *Vw);
    void ReFollow_Ctrl(float *Vx, float *Vy, float *Vw);    /*<! 反向跟随模式 */
    void Spin_Ctrl(float *Vx, float *Vy, float *Vw);        /*<! 自旋模式 */
		void Sentry_Spin_Ctrl(float *Vx, float *Vy, float *Vw);
    void Swing_Ctrl(float *Vx, float *Vy, float *Vw);       /*<! 扭腰模式 */
    void AutoTrack_Ctrl(float *Vx, float *Vy, float *Vw);   /*<! 追踪装甲板 */
    void NotFollwe_Ctrl(float *Vx, float *Vy, float *Vw);   /*<! 无跟随模式 */
		void Sentry_NotFollwe_Ctrl(float *Vx, float *Vy, float *Vw);
    void Speed_Decompose(const float angle, float *Vx, float *Vy);/*<! 速度分解 */

public:
    Chassis_classdef();
    CHAS_CtrlMode_e Mode;
    IncrementPID CHAS_DRV_PID[4];     /*<! 驱动轮 增量式PID */
    PositionPID CHAS_RUD_PID[4][2];   /*<! 转向轮 位置式PID */
    PositionPID Follow_PID[2];   /*<! 跟随模式 PID */
    PositionPID Swing_PID[2];    /*<! 扭腰模式 PID */
    PositionPID AutoTrack_PID[2];     /*<! 自动跟随模式 PID */
    Motor_M3508 DRV_Motor[4] = {Motor_M3508(1), Motor_M3508(2), Motor_M3508(3), Motor_M3508(4)};      /*<! 驱动轮 */
    Motor_GM6020 RUD_Motor[4] = {Motor_GM6020(1), Motor_GM6020(2), Motor_GM6020(3), Motor_GM6020(4)}; /*<! 转向轮 */

    RUD_Param_t RUD_Param[4]; /*<! 转向轮相关参数 */

    LowPassFilter Vx_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */
    LowPassFilter Vy_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */
    LowPassFilter Vw_LPF = LowPassFilter(0.01f); /*<! 低通滤波 */

    FilterTarget_t F_target;                  /*<! 滤波后的数据 */

    AccelerateParam_t Acc_Param;    /*<! 加速曲线参数 */

    float VxVy_Coe;
    float VxVy_Limit;
    float Vw_Limit;

    uint8_t ReF_Flag;     /*<! 反向跟随转换标志位 */
    int16_t ReF_Cnt;      /*<! 反向过度累计时间 */

    uint8_t Cap_switch;   /*<! 电容输出开关 */
    uint8_t Uphill_Mode;  /*<! 爬坡模式 */ 
    uint8_t NoMiddle_Mode;/*<! 底盘45度模式 */

    int16_t Target_Vx, Target_Vy, Target_Vw;

    int16_t Power_Limit;

    void Control();

    void Mecanum_Solve(int16_t Vx, int16_t Vy, int16_t Vz, int16_t *cal_speed);  /*<! 麦轮解算 */
    void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vz, int16_t *cal_speed);   /*<！舵轮解算 */
    void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vz);   /*<! 三轴速度解算舵角度 */
    void RUDTotalAngle_Calc(Motor_GM6020* rudder_motor , int8_t motor_num , int8_t reset , uint8_t opposite);  /*<! 转向轮总角度计算 */
    void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);                             /*<! 转向轮目标角度计算 */
    void DRV_PIDCalc(uint8_t motor, float target, float current);
    void RUD_PIDCalc(uint8_t motor, float target, float current);
    void Judge_DRV_Dir(uint8_t motor, float target, float current);   /*<! 判断驱动轮是否需要反向 */
    void Angle_Cnt();
    void Drv_Slow(float *rec , float target , float slow_Inc); /*<! 底盘速度斜坡 */

    void Cap_Ctrl(uint8_t state);
    void AcclerateCurve(float *Vx, float *Vy);  /*<! 加速曲线 */

    float Get_MinDeviation(int32_t target, int32_t current);
    float Turn_InferiorArc(uint8_t motor, float target, float current);  /*<! 劣弧旋转 */

    uint8_t IsMove();   /*<! 底盘是否有移动目标速度 */
    int16_t Get_TargetVx(); /*<! 获取底盘三个轴目标速度*/
    int16_t Get_TargetVy();
    int16_t Get_TargetVw();

    int16_t Get_PowerLimit();

};


// --- 用于上位机的观测
extern int16_t ssssspeed[3];
extern uint8_t Move_flag;
extern float offset_angle;

#endif

#endif
