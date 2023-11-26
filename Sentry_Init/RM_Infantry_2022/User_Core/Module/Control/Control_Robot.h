#ifndef _CONTROL_ROBOT_H_
#define _CONTROL_ROBOT_H_

#include "DR16.h"
#include "Control_Chassis.h"
#include "Control_Gimbal.h"
#include "Control_Shoot.h"
#include "Control_Vision.h"

//机器人工作状态
enum RobotState_e
{
	Robot_Initializing, //从失能进入到正常状态的过渡阶段，开机时候初始化的阶段，机器人设备离线检测
	Robot_Disability,	//失能状态，应对紧急情况和开机启动
	Robot_Activating	//正常运行状态
};



class Robot_classdef
{
public:
    RobotState_e State;
    DR16Status_Typedef Ctrl_Source;
    CHAS_CtrlMode_e Chassis_Mode;
    CHAS_CtrlMode_e Chassis_PreMode;
    Gimbal_CtrlMode_e Gimbal_Mode;
    Attack_CtrlMode_e Attack_Mode;
    Fric_CtrlMode_e Fric_Mode;
    VisionMode_e Vision_Mode;

    uint8_t ID;          /*<! 机器人ID */
    uint8_t PC_State;    /*<! PC状态 */
    uint8_t SendData[8]; /*<! 板间通信存储数据 */
    uint8_t Write_Msg[2][8];
    uint8_t Reflash_UI;  /*<! UI刷新 */
    uint8_t ResetFlag = false;   /*<! 重启标志位 */
    uint8_t Chas_Warning;   /*<! 底盘状态警告 */
    uint16_t FPS;
    WorldTime_t TIME;

		uint8_t DetectColor;
		uint8_t God_State[5];
		uint8_t CanShootNum[2];
		uint8_t HurtArmor_NoZero;
		uint8_t ShootRecNum,lastShootRecNum,mul_num;
		uint16_t TrueShootRecNum;
		uint8_t CoolBuff;
		uint8_t Enemy_Sentry_God_State;
		
    Robot_classdef();
    void Enable();
    void Disable();
    void Reset();
    void Control();
    void CtrlSource_Switch(DR16Status_Typedef mode);
    void Set_ChassisMode(CHAS_CtrlMode_e mode);
    void Set_GimbalMode(Gimbal_CtrlMode_e mode);
    void Set_AttackMode(Attack_CtrlMode_e mode);
    void Set_VisionMode(VisionMode_e mode);

    void Gimbal_Ctrl();

    void SendMsgtoCHAS();
    void RefereeMsg_Write(uint16_t id, uint8_t can_rx_data[]);

    uint8_t Get_ID();
		uint8_t Get_DetectColor();
    uint16_t Get_FPS();

    RobotState_e Get_State();
    DR16Status_Typedef Get_CtrlSource();
    CHAS_CtrlMode_e Get_ChassisMode();
    CHAS_CtrlMode_e Get_ChassisPreMode();
    Gimbal_CtrlMode_e Get_GimbalMode();
    Attack_CtrlMode_e Get_AttackMode();
    Fric_CtrlMode_e Get_FricMode();
    VisionMode_e Get_VisionMode();
    
};


#endif
