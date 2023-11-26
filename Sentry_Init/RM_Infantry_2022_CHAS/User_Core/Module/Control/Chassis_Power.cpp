/* Includes ------------------------------------------------------------------*/
#include "Chassis_Power.h"
#include "System_DataPool.h"
#include <math.h>
#include "DJI_AIMU.h"

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
CHAS_Power_classdef::CHAS_Power_classdef()
{
    // ���� K1 4 power offset 25
    K1 = 10.0f;  //16
    K2 = 0.012f; //0.02
    vel_coeff = 0.0048;
    power_offset = 20;  //0

    debug_coe = 50.0f;
}


uint8_t Uphill_flag;
void CHAS_Power_classdef::Limit(int16_t *wheelCurrent, int8_t amount)
{
    float coe[4] = {0.0f};

    //--- ����ϵͳ���� ǿ������
    if(DevicesMonitor.Get_State(REFEREE_MONITOR) == Off_line)
    {

    }

    //--- ���޹���
    if(Referee.GameRobotState.max_chassis_power == 65535)
    {
        SumCurrent_Out = SumCurrent_In;  //--- �޴���ʱΪԭ����ֵ
        return;
    }

    SumCurrent_In = SumCurrent_Out = 0;

    /*----------------------- ���¹����ٷ��� --------------------------*/
    float FWheel_coe[2][2] = {{0, 0}, {0.0}}; //--- ǰ�ֵ���ռ��
    int32_t Fwheel_IN[2] = {0}; //--- ǰ�ֵ���
    uint8_t i = 0;
    
    if (Infantry.Get_ChassisMode() == CHAS_FollowMode && Infantry.Write_Msg[Uphill_Mode]==true && abs(Chassis.Target_Vy)>1000 && imu.rol < 170.0f) //--- �����������ģʽ
    {
        //--- ����ǰ�ֵ���
        for (i = 0; i < amount / 2; i++)
        {
            Fwheel_IN[0] += abs(wheelCurrent[i]);
        }
        //--- ����ǰ�ְٷֱ�
        for (i = 0; i < amount / 2; i++)
        {
            FWheel_coe[0][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[0]));
        }
        //--- ��Ҫ������ֵĹ��� ��һ��ָ�����(����ֵ70%)
        int32_t temp_FWheel = (int32_t)Fwheel_IN[0] * 0.7f;
        Fwheel_IN[0] -= temp_FWheel;
        //--- �����������Ӹ��еĹ���
        for (i = 0; i < amount / 2; i++)
        {
            wheelCurrent[i] = (int16_t)(Fwheel_IN[0] * FWheel_coe[0][i]);
        }
    
        //--- �������ԭ�����������
        for (i = amount / 2; i < amount; i++)
        {
            Fwheel_IN[1] += abs(wheelCurrent[i]);
        }
        //--- �������ԭ��������ٷֱ�
        for (i = amount / 2; i < amount; i++)
        {
            FWheel_coe[1][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[1]));
        }
        //--- �����ּ��ϴ�ǰ���ڹ����ĵ���
        Fwheel_IN[1] += temp_FWheel;
        //--- �������󣬺������ո��еĵ���
        for (i = amount / 2; i < amount; i++)
        {
            wheelCurrent[i] = (int16_t)(Fwheel_IN[1] * FWheel_coe[1][i]);
        }
    }
    /*-----------------------------------------*/

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
float debug_powercoe = 80.0f;
void CHAS_Power_classdef::Limit_Calc()
{
    Power_Buffer = Referee.PowerHeatData.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / debug_powercoe;
    DRV_CalcRatio *= DRV_CalcRatio;  // ƽ���Ĺ�ϵ

    if(powerBuffErr > 0 /* && Infantry.Write_Msg[Cap_Ctrl] != true */)  // ���õ����幦������й������ƴ���
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

    if(powerBuffErr > 0 /* && SupCap.SendData.is_cap_output == OFF */)  // ���õ����幦������й������ƴ���
    {
        SumCurrent_Out = SumCurrent_In * RUD_CalcRatio;
    }
}


//  effort_coeff: 12.0
//  vel_coeff: 0.0048
//  power_offset: -3
/*
    //---��ȡ���������
    double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
    // Three coefficients of a quadratic equation in one variable
    double a = 0., b = 0., c = 0.;
    for (const auto& joint : joint_handles_)
    {
        double cmd_effort = joint.getCommand(); // ������ĵ�����
        double real_vel = joint.getVelocity();  // �����ǰת��
        if (joint.getName().find("wheel") != std::string::npos)  // The pivot joint of swerve drive doesn't need power limit
        {
            a += square(cmd_effort); // ԭ���������
            b += std::abs(cmd_effort * real_vel); // ת��*���
            c += square(real_vel); // ת�ٵ�ƽ��
        }
    }
    a *= effort_coeff_; // k1
    c = c * velocity_coeff_ - power_offset_ - power_limit; 
    // Root formula for quadratic equation in one variable
    double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
    for (auto joint : joint_handles_)
        if (joint.getName().find("wheel") != std::string::npos)
        {
            joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
        }
*/
/**
 *@note  k1 k2�ĵ��Է�����ʵʱ��ȡ����ϵͳ�����ĵ��̹���,���õ��̵����ת
 *       ����k1ʹ����ʵ�ʹ��ʴ��������ƹ������
 *       ���û�����ԭ��С����,����k2ʹ����ʵ�ʹ��ʴ��������ƹ������  
 */
float max_powerlimit;
double a = 0.0f, b = 0.0f, c = 0.0f;
uint8_t buffer_flag;
uint8_t buffer_reset_cnt;

float motor_rad_speed[4];
float rad_temp_min[2] = {-2.0f, 2.0f};
void CHAS_Power_classdef::Test_NewLimit(float *DRVCurrent, int16_t *RUDCurrent, int8_t amount)
{
    float rud_coe[4] = {0.0f};

    max_powerlimit = Chassis.Get_PowerLimit();

    // float a = 0.0f, b = 0.0f, c = 0.0f;
    a = 0.0f; b = 0.0f; c = 0.0f;

    /*----------------------- ���¹����ٷ��� --------------------------*/
    float FWheel_coe[2][2] = {{0, 0}, {0.0}}; //--- ǰ�ֵ���ռ��
    int32_t Fwheel_IN[2] = {0}; //--- ǰ�ֵ���
    uint8_t i = 0;
    
    //--- ����ģʽ����
    if (Infantry.Get_ChassisMode() == CHAS_FollowMode && Infantry.Write_Msg[Uphill_Mode]==true && abs(Chassis.Target_Vy)>1000 && imu.rol < 170.0f) //--- �����������ģʽ
    {
        //--- ����ǰ�ֵ���
        for (i = 0; i < amount / 2; i++)
        {
            Fwheel_IN[0] += abs(DRVCurrent[i]);
        }
        //--- ����ǰ�ְٷֱ�
        for (i = 0; i < amount / 2; i++)
        {
            FWheel_coe[0][i] = ((float)(DRVCurrent[i])) / ((float)(Fwheel_IN[0]));
        }
        //--- ��Ҫ������ֵĹ��� ��һ��ָ�����(����ֵ70%)
        int32_t temp_FWheel = (int32_t)Fwheel_IN[0] * 0.7f;
        Fwheel_IN[0] -= temp_FWheel;
        //--- �����������Ӹ��еĹ���
        for (i = 0; i < amount / 2; i++)
        {
            DRVCurrent[i] = (int16_t)(Fwheel_IN[0] * FWheel_coe[0][i]);
        }
    
        //--- �������ԭ�����������
        for (i = amount / 2; i < amount; i++)
        {
            Fwheel_IN[1] += abs(DRVCurrent[i]);
        }
        //--- �������ԭ��������ٷֱ�
        for (i = amount / 2; i < amount; i++)
        {
            FWheel_coe[1][i] = ((float)(DRVCurrent[i])) / ((float)(Fwheel_IN[1]));
        }
        //--- �����ּ��ϴ�ǰ���ڹ����ĵ���
        Fwheel_IN[1] += temp_FWheel;
        //--- �������󣬺������ո��еĵ���
        for (i = amount / 2; i < amount; i++)
        {
            DRVCurrent[i] = (int16_t)(Fwheel_IN[1] * FWheel_coe[1][i]);
        }
    }
    /*-----------------------------------------*/

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        // P(�ܹ���) = F(��ת�ص�����ʾ) * V(��ת�ٱ�ʾ)  //19.2032?
        // M3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
        // M3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���:0.0054533f
        motor_rad_speed[i] = (Chassis.DRV_Motor[i].getSpeed()*0.0054533f);

        if(motor_rad_speed[i] > 0 && motor_rad_speed[i] < rad_temp_min[1])
        {
            motor_rad_speed[i] = rad_temp_min[1];
        }
        else if(motor_rad_speed[i] < 0 && motor_rad_speed[i] > rad_temp_min[0])
        {
           motor_rad_speed[i] = rad_temp_min[0];
        }

        a += pow((DRVCurrent[i]*0.000366211f) ,2);
        b += abs((DRVCurrent[i]*0.000366211f) * (motor_rad_speed[i])); //--- ת��*���� /19.2032f*(6.28f/60.0f))
        c += pow((motor_rad_speed[i]), 2); //--- ת�ٵ�ƽ��
    }

    a *= K1; //--- K1
    c = c * K2 - power_offset - max_powerlimit;

    zoom_coeff = (pow(b,2) - 4 * a * c) > 0 ? ((-b + sqrt(pow(b,2) - 4 * a * c)) / (2 * a)) : 0.0f;

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        // DRVCurrent[i] = (zoom_coeff > 1 ? DRVCurrent[i] : DRVCurrent[i] * zoom_coeff);

        if(zoom_coeff < 1 && Chassis.Get_PowerBuffer() < 50) //--- �����õ�һ���ֻ��幦��
        {
            DRVCurrent[i] = DRVCurrent[i] * zoom_coeff;
            // DRVCurrent[i] /= 0.000366211f;
            buffer_flag = true;
            buffer_reset_cnt = 0;
        }
        else
        {
            if(buffer_flag == true)
            {
                DRVCurrent[i] = (zoom_coeff > 1 ? DRVCurrent[i] : DRVCurrent[i] * zoom_coeff);
            }
            else
            {
                DRVCurrent[i] = DRVCurrent[i];
            }
            // DRVCurrent[i] = DRVCurrent[i];
        }

        if(buffer_flag == true && Chassis.Get_PowerBuffer() >= 60)
        {
            if(buffer_reset_cnt < 125)
            {
                buffer_reset_cnt++;
            }
            else
            {
                buffer_flag = false;
            }
        }
    }

    //--- ����Ϊ6020��������� ----------------------------------------
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        SumCurrent_In += abs(RUDCurrent[i]);
    }
    SumCurrent_Out = SumCurrent_In;  // �޴���ʱΪԭ����ֵ

    // ����ÿ������ĵ���ռ��
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        rud_coe[i] = ((float)(RUDCurrent[i])) / ((float)(SumCurrent_In));
    }

    RUD_CalcRatio = 0;
    RUD_CalcRatio = (float)Chassis.Get_PowerBuffer() / debug_coe;
    RUD_CalcRatio *= RUD_CalcRatio;  // ƽ���Ĺ�ϵ
    // RUD_CalcRatio *= RUD_CalcRatio; // �����Ĺ�ϵ

    if(Chassis.Get_PowerBuffer() < 30)  //--- ���ﵽ���幦����ֵ�����6020�������ƴ���
    {
        if(SumCurrent_Out > 60000)
        {
            SumCurrent_Out = SumCurrent_In * RUD_CalcRatio;
        }
    }

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        RUDCurrent[i] = ((SumCurrent_Out) * rud_coe[i]);
    }
    //-----------------------------------------------------------------
}

