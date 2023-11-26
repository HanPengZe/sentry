#include "Dev_IMU.h"
#include "arm_math.h"

/**
 * @brief      Init
 * @param[in]  None
 * @retval     None
 */
IMU_classdef::IMU_classdef()
{

}

/**
 * @brief      IMU数据更新
 * @param[in]  angle,gyro
 * @retval     None
 */
void IMU_classdef::Update(float *angle, float *gyro)
{
    
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- 角度
        angle[i] = angle[i] * (180/PI) + 180; //--- 弧度转化为度(0 ~ 360°)
        //--- 角速度
        Gyro[i] = gyro[i] * (180/PI);
        //--- 过零处理,累计角度
        if(Angle[i] - Pre_Angle[i] < -180)
        {
            Turns_cnt[i]++;
        }
        else if(Angle[i] - Pre_Angle[i] > 180)
        {
            Turns_cnt[i]--;
        }

        TotalAngle[i] = Angle[i] + Turns_cnt[i] * 360.0f;
        Pre_Angle[i] = Angle[i];
    }
    
}


/**
 * @brief      获取IMU角度(角度制)
 * @param[in]  type
 * @retval     Angle
 */
float IMU_classdef::Get_Angle(Gimbal_type_e type)
{
    return Angle[type];
}

/**
 * @brief      获取IMU累计角度(角度制)
 * @param[in]  type
 * @retval     TotalAngle
 */
float IMU_classdef::Get_TotalAngle(Gimbal_type_e type)
{
    return TotalAngle[type];
}

/**
 * @brief      获取IMU角速度
 * @param[in]  type
 * @retval     Gyro
 */
float IMU_classdef::Get_Gyro(Gimbal_type_e type)
{
    return Gyro[type];
}

