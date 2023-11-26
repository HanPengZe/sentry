#ifndef _IMU_H_
#define _IMU_H_

#pragma once

// #ifdef  __cplusplus
// extern "C" {
// #endif

#include "Control_Gimbal.h"
#include<stdint.h>

class IMU_classdef
{
private:

public:
    float Angle[3];     /*<! IMU �Ƕ� */
    float Pre_Angle[3]; /*<! ��һ�νǶ� */
    float TotalAngle[3];/*<! IMU �ܽǶ�*/
    float Gyro[3];      /*<! IMU ���ٶ� */
    int32_t Turns_cnt[3]; /*<! �ۼ�Ȧ�� */

    IMU_classdef();

    void Update(float *angle, float *gyro); /*<! ���ݸ��� */
    float Get_Angle(Gimbal_type_e type);        /*<! ��ȡʵ�ʽǶ� */
    float Get_TotalAngle(Gimbal_type_e type);   /*<! ��ȡ�ۼƽǶ�*/
    float Get_Gyro(Gimbal_type_e type);         /*<! ��ȡ���ٶ� */

};

// extern "C" void Call_IMUUpdate_FUNC(IMU_classdef *p, float *angle, float *gyro)
// {
//     return p->Update();
// }


// #ifdef __cplusplus
// }
// #endif

#endif
