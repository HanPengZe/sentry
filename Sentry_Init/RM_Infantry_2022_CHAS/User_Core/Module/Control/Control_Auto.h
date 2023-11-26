#ifndef _CONTROL_AUTO_H_
#define _CONTROL_AUTO_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

class Auto_classdef
{
private:
    uint32_t God_Start_Timing[5];
    uint16_t Last_EnemyHp[5];
    uint8_t God_Mode[5];
public:
    Auto_classdef();
    bool OpenPointFlag, GoHomeFlag, InHomeFlag, ArmourPlateStrikeFlag, Spin_Flag;
    uint16_t Last_GameRobotState_remain_HP;
    uint8_t ShootData_RecNum;
    uint8_t GameRobotState_RecNum, Last_GameRobotState_RecNum;
    uint8_t HurtArmor_NoZero;
    uint8_t God_State[5];

    void Move_Point(void);//根据比赛状态和机器人血量设置移动点
    void Hurt_Position(void);//根据伤害来转头
    void Spin_Open(void);//根据比赛状态和机器人血量来开启小陀螺
    void God_Detection(void);//无敌检测
};
#endif

