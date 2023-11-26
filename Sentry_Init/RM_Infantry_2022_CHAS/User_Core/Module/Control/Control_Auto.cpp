#include "Control_Auto.h"
#include "System_DataPool.h"
#include <math.h>

Auto_classdef::Auto_classdef()
{
}

void Auto_classdef::Move_Point()
{
    if(Referee.GameState.game_progress==4 && OpenPointFlag==0)//比赛中
    {
        if(Referee.GameState.stage_remain_time<60*7)//比赛剩余时间
        {
            //根据战术进行行动
            OpenPointFlag=true;
        } 
    }

    if(Referee.GameRobotState.robot_id>100)
    {
        if(Referee.GameRobotHP.blue_outpost_HP<300 && GoHomeFlag==0 && InHomeFlag)
        {
            GoHomeFlag=true;//回家
        }
        // GameRobotHP.red_1_robot_HP
        // GameRobotHP.red_2_robot_HP
        // GameRobotHP.red_3_robot_HP
        // GameRobotHP.red_4_robot_HP
        // GameRobotHP.red_5_robot_HP
    }
    else if(Referee.GameRobotState.robot_id<100)
    {
        if(Referee.GameRobotHP.red_outpost_HP<300 && GoHomeFlag==0 && InHomeFlag)
        {
            GoHomeFlag=true;//回家
        }
        // GameRobotHP.blue_1_robot_HP
        // GameRobotHP.blue_2_robot_HP
        // GameRobotHP.blue_3_robot_HP
        // GameRobotHP.blue_4_robot_HP
        // GameRobotHP.blue_5_robot_HP
    }

    //基础是否在巡逻区
    if((Referee.RFID_Status.rfid_status>>14)&1)
    {
        //在巡逻区
        GoHomeFlag = false;
        InHomeFlag = true;
    }
    else
    {
        InHomeFlag = false;
    }
}

void Auto_classdef::Hurt_Position()
{   
    //血量变化是前哨站，其他时候可以根据GameRobotState的接收次数去判断是否被击打
    if(Referee.GameRobotState.remain_HP!=Last_GameRobotState_remain_HP ||\
        GameRobotState_RecNum != Last_GameRobotState_RecNum)
    {
        HurtArmor_NoZero=0;
        if(Referee.RobotHurt.hurt_type==0)//血量变化类型
        {
            switch(Referee.RobotHurt.armor_id)//装甲板ID
            {
                case 0: //前
                case 1: //左
                case 2: //后
                case 3: //右
                    HurtArmor_NoZero=Referee.RobotHurt.armor_id+1;
                break;

                default:HurtArmor_NoZero=0;break;
            }
            ArmourPlateStrikeFlag=true;
        }
        //英雄
        if(HurtArmor_NoZero!=0&&\
        Last_GameRobotState_remain_HP-Referee.GameRobotState.remain_HP>50)
        {
            HurtArmor_NoZero*=10;
        }
    }
    else
    {
        HurtArmor_NoZero=0;
    }
    Last_GameRobotState_remain_HP = Referee.GameRobotState.remain_HP;
    Last_GameRobotState_RecNum = GameRobotState_RecNum;
}

void Auto_classdef::Spin_Open()
{
    if(Referee.GameRobotState.robot_id>=100)
    {
        if(Referee.GameRobotHP.blue_outpost_HP<=100)
        {
            Spin_Flag=true;
        }
        else if(Referee.GameRobotHP.blue_outpost_HP<=200)
        {
            if((Referee.RFID_Status.rfid_status>>14)&1){Spin_Flag=false;}
            else{Spin_Flag=true;}
        }
    }
    else if(Referee.GameRobotState.robot_id<100)
    {
        if(Referee.GameRobotHP.red_outpost_HP<=100)
        {
            Spin_Flag=true;
        }
        else if(Referee.GameRobotHP.red_outpost_HP<=200)
        {
            if((Referee.RFID_Status.rfid_status>>14)&1){Spin_Flag=false;}
            else{Spin_Flag=true;}
        }
    }
}


void Auto_classdef::God_Detection()
{
    if(Referee.GameRobotState.robot_id>100)
    {
        if(Last_EnemyHp[0] == 0 && Referee.GameRobotHP.red_1_robot_HP > 0)
        {
            God_Start_Timing[0]=HAL_GetTick();
            if(abs(Referee.GameRobotHP.red_1_robot_HP-Last_EnemyHp[0])<50)
            {
                //等待复活无敌——拥有十秒无敌
                God_Mode[0]=10;
            }
            else
            {
                //购买复活无敌——拥有三秒无敌
                God_Mode[0]=3;

            }
        }
        if(Last_EnemyHp[1] == 0 && Referee.GameRobotHP.red_2_robot_HP > 0)
        {
            God_Start_Timing[1]=HAL_GetTick();
            if(abs(Referee.GameRobotHP.red_2_robot_HP-Last_EnemyHp[1])<50)
            {
                //等待复活无敌——拥有十秒无敌
                God_Mode[1]=10;
            }
            else
            {
                //购买复活无敌——拥有三秒无敌
                God_Mode[1]=3;
            }
        }
        if(Last_EnemyHp[2] == 0 && Referee.GameRobotHP.red_3_robot_HP > 0)
        {
            God_Start_Timing[2]=HAL_GetTick();
            if(abs(Referee.GameRobotHP.red_3_robot_HP-Last_EnemyHp[2])<50)
            {
                //等待复活无敌——拥有十秒无敌
                God_Mode[2]=10;
            }
            else
            {
                //购买复活无敌——拥有三秒无敌
                God_Mode[2]=3;
            }
        }
        if(Last_EnemyHp[3] == 0 && Referee.GameRobotHP.red_4_robot_HP > 0)
        {
            God_Start_Timing[3]=HAL_GetTick();
            if(abs(Referee.GameRobotHP.red_4_robot_HP-Last_EnemyHp[3])<50)
            {
                //等待复活无敌——拥有十秒无敌
                God_Mode[3]=10;
            }
            else
            {
                //购买复活无敌——拥有三秒无敌
                God_Mode[3]=3;
            }
        }
        if(Last_EnemyHp[4] == 0 && Referee.GameRobotHP.red_5_robot_HP > 0)
        {
            God_Start_Timing[4]=HAL_GetTick();
            if(abs(Referee.GameRobotHP.red_5_robot_HP-Last_EnemyHp[4])<50)
            {
                //等待复活无敌——拥有十秒无敌
                God_Mode[4]=10;
            }
            else
            {
                //购买复活无敌——拥有三秒无敌
                God_Mode[4]=3;
            }
        }

        for(int i=0; i<5;i++)
        {
            if(HAL_GetTick()-God_Start_Timing[i]>God_Mode[i]*1000)
            {
                God_State[i] = 0;
            }
            else
            {
                God_State[i] = 1;
            }
        }
        // 
        // Referee.GameRobotHP.red_2_robot_HP
        // Referee.GameRobotHP.red_3_robot_HP
        // Referee.GameRobotHP.red_4_robot_HP
        // Referee.GameRobotHP.red_5_robot_HP

        Last_EnemyHp[0] = Referee.GameRobotHP.red_1_robot_HP;
        Last_EnemyHp[1] = Referee.GameRobotHP.red_2_robot_HP;
        Last_EnemyHp[2] = Referee.GameRobotHP.red_3_robot_HP;
        Last_EnemyHp[3] = Referee.GameRobotHP.red_4_robot_HP;
        Last_EnemyHp[4] = Referee.GameRobotHP.red_5_robot_HP;
    }
}

