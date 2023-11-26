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

    //--- ���޹���
    if(Referee.GameRobotState.max_chassis_power == 65535)
    {
        return;
    }

    SumCurrent_In = SumCurrent_Out = 0;

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        SumCurrent_In += abs(wheelCurrent[i]);
    }
    SumCurrent_Out = SumCurrent_In;  // �޴���ʱΪԭ����ֵ

    // ����ÿ������ĵ���ռ��
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

int16_t powerBuffErr;  // �õ��Ļ�������
void CHAS_Power_classdef::Limit_Calc()
{
    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / 70.0f;
    DRV_CalcRatio *= DRV_CalcRatio;  // ƽ���Ĺ�ϵ

    if(powerBuffErr > 0)  // ���õ����幦������й������ƴ���
    {
        SumCurrent_Out = SumCurrent_In * DRV_CalcRatio;
    }
}

void CHAS_Power_classdef::RUD_Limit(int16_t *DRVCurrent, int16_t *RUDCurrent, int8_t amount)
{
    static float DRV_coe[4] = {0.0f};
    static float RUD_coe[4] = {0.0f};
    static int16_t Total_Current = 0;

    //--- ����ϵͳ���ߣ�ǿ������
    if(DevicesMonitor.Get_State(REFEREE_MONITOR) == Off_line)
    {
        
    }

    //--- ���޹���
    if(Referee.GameRobotState.max_chassis_power == 65535)
    {
        DRV_Current_Out = DRV_Current_In;
        RUD_Current_Out = RUD_Current_In;
        return;
    }

    //--- ����
    RUD_Current_In = RUD_Current_Out = 0;
    DRV_Current_In = DRV_Current_Out = 0;

    //--- ����ת���ֺ������ֵ��ܵ���ֵ
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        DRV_Current_In += abs(DRVCurrent[i]);
        RUD_Current_In += abs(RUDCurrent[i]);
    }
    Total_Current = RUD_Current_In + DRV_Current_In; //--- ת�����������ֵ��ܵ���

    // �޴���ʱΪԭ����ֵ
    DRV_Current_Out = DRV_Current_In;
    RUD_Current_Out = RUD_Current_In;

    // ����ÿ������ĵ���ռ��
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        DRV_coe[i] = ((float)(DRVCurrent[i])) / ((float)(DRV_Current_In));
        RUD_coe[i] = ((float)(RUDCurrent[i])) / ((float)(RUD_Current_In));
    }

    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    //--- �������ƹ��ʵ�ϵ��
    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / 70.0f;
    DRV_CalcRatio *= DRV_CalcRatio;  // ƽ���Ĺ�ϵ

    RUD_CalcRatio = 0;
    RUD_CalcRatio = (float)Power_Buffer / 90.0f;
    RUD_CalcRatio *= RUD_CalcRatio;  // ƽ���Ĺ�ϵ

    if(powerBuffErr > 0)  // ���õ����幦������й������ƴ���
    {
        DRV_Current_Out = DRV_Current_In * DRV_CalcRatio; //--- ���������ֵ�����

        if(powerBuffErr < 30) //--- ������̫��Ļ��幦��
        {
            // DRV_Current_Out *= 0.55f;
            RUD_Current_Out = RUD_Current_In * RUD_CalcRatio; //--- ����ת���ֵ�����
        }
    }

    //--- �������
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
    RUD_CalcRatio *= RUD_CalcRatio;  // ƽ���Ĺ�ϵ

    if(powerBuffErr > 0)  // ���õ����幦������й������ƴ���
    {
        SumCurrent_Out = SumCurrent_In * RUD_CalcRatio;
    }
}


