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
 * @brief      IMU���ݸ���
 * @param[in]  angle,gyro
 * @retval     None
 */
void IMU_classdef::Update(float *angle, float *gyro)
{
    
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- �Ƕ�
        angle[i] = angle[i] * (180/PI) + 180; //--- ����ת��Ϊ��(0 ~ 360��)
        //--- ���ٶ�
        Gyro[i] = gyro[i] * (180/PI);
        //--- ���㴦��,�ۼƽǶ�
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
 * @brief      ��ȡIMU�Ƕ�(�Ƕ���)
 * @param[in]  type
 * @retval     Angle
 */
float IMU_classdef::Get_Angle(Gimbal_type_e type)
{
    return Angle[type];
}

/**
 * @brief      ��ȡIMU�ۼƽǶ�(�Ƕ���)
 * @param[in]  type
 * @retval     TotalAngle
 */
float IMU_classdef::Get_TotalAngle(Gimbal_type_e type)
{
    return TotalAngle[type];
}

/**
 * @brief      ��ȡIMU���ٶ�
 * @param[in]  type
 * @retval     Gyro
 */
float IMU_classdef::Get_Gyro(Gimbal_type_e type)
{
    return Gyro[type];
}

