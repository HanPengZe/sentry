/* Includes ------------------------------------------------------------------*/
#include "Chassis_Power.h"
#include "System_DataPool.h"

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
CHAS_Power_classdef::CHAS_Power_classdef()
{

}

void CHAS_Power_classdef::Limit(int16_t *wheelCurrent, int8_t amount)
{
    float coe[4] = {0.0f};

    //--- 不限功率
    if(Referee.GameRobotState.max_chassis_power == 65535)
    {
        return;
    }

    SumCurrent_In = SumCurrent_Out = 0;

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        SumCurrent_In += abs(wheelCurrent[i]);
    }
    SumCurrent_Out = SumCurrent_In;  // 无处理时为原来的值

    // 计算每个电机的电流占比
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        coe[i] = ((float)(wheelCurrent[i])) / ((float)(SumCurrent_In));
    }

    Limit_Calc();

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        wheelCurrent[i] = ((SumCurrent_Out) * coe[i]);
    }
}

int16_t powerBuffErr;  // 用掉的缓冲能量
void CHAS_Power_classdef::Limit_Calc()
{
    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / 70.0f;
    DRV_CalcRatio *= DRV_CalcRatio;  // 平方的关系

    if(powerBuffErr > 0)  // 若用到缓冲功率则进行功率限制处理
    {
        SumCurrent_Out = SumCurrent_In * DRV_CalcRatio;
    }
}

void CHAS_Power_classdef::RUD_Limit(int16_t *DRVCurrent, int16_t *RUDCurrent, int8_t amount)
{
    static float DRV_coe[4] = {0.0f};
    static float RUD_coe[4] = {0.0f};
    static int16_t Total_Current = 0;

    //--- 裁判系统掉线，强制限制
    if(DevicesMonitor.Get_State(REFEREE_MONITOR) == Off_line)
    {
        
    }

    //--- 不限功率
    if(Referee.GameRobotState.max_chassis_power == 65535)
    {
        DRV_Current_Out = DRV_Current_In;
        RUD_Current_Out = RUD_Current_In;
        return;
    }

    //--- 清零
    RUD_Current_In = RUD_Current_Out = 0;
    DRV_Current_In = DRV_Current_Out = 0;

    //--- 计算转向轮和驱动轮的总电流值
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        DRV_Current_In += abs(DRVCurrent[i]);
        RUD_Current_In += abs(RUDCurrent[i]);
    }
    Total_Current = RUD_Current_In + DRV_Current_In; //--- 转向轮与驱动轮的总电流

    // 无处理时为原来的值
    DRV_Current_Out = DRV_Current_In;
    RUD_Current_Out = RUD_Current_In;

    // 计算每个电机的电流占比
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        DRV_coe[i] = ((float)(DRVCurrent[i])) / ((float)(DRV_Current_In));
        RUD_coe[i] = ((float)(RUDCurrent[i])) / ((float)(RUD_Current_In));
    }

    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    //--- 计算限制功率的系数
    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / 70.0f;
    DRV_CalcRatio *= DRV_CalcRatio;  // 平方的关系

    RUD_CalcRatio = 0;
    RUD_CalcRatio = (float)Power_Buffer / 90.0f;
    RUD_CalcRatio *= RUD_CalcRatio;  // 平方的关系

    if(powerBuffErr > 0)  // 若用到缓冲功率则进行功率限制处理
    {
        DRV_Current_Out = DRV_Current_In * DRV_CalcRatio; //--- 进行驱动轮的限制

        if(powerBuffErr < 30) //--- 若用了太多的缓冲功率
        {
            // DRV_Current_Out *= 0.55f;
            RUD_Current_Out = RUD_Current_In * RUD_CalcRatio; //--- 进行转向轮的限制
        }
    }

    //--- 分配电流
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        DRVCurrent[i] = ((DRV_Current_Out) * DRV_coe[i]);
        RUDCurrent[i] = ((RUD_Current_Out) * RUD_coe[i]);
    }
}

void CHAS_Power_classdef::RUD_Limit_Calc()
{
    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    RUD_CalcRatio = 0;
    RUD_CalcRatio = (float)Power_Buffer / 70.0f;
    RUD_CalcRatio *= RUD_CalcRatio;  // 平方的关系

    if(powerBuffErr > 0)  // 若用到缓冲功率则进行功率限制处理
    {
        SumCurrent_Out = SumCurrent_In * RUD_CalcRatio;
    }
}


