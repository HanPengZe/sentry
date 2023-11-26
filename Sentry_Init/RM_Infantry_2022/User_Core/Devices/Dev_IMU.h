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
    float Angle[3];     /*<! IMU 角度 */
    float Pre_Angle[3]; /*<! 上一次角度 */
    float TotalAngle[3];/*<! IMU 总角度*/
    float Gyro[3];      /*<! IMU 角速度 */
    int32_t Turns_cnt[3]; /*<! 累计圈数 */

    IMU_classdef();

    void Update(float *angle, float *gyro); /*<! 数据更新 */
    float Get_Angle(Gimbal_type_e type);        /*<! 获取实际角度 */
    float Get_TotalAngle(Gimbal_type_e type);   /*<! 获取累计角度*/
    float Get_Gyro(Gimbal_type_e type);         /*<! 获取角速度 */

};

// extern "C" void Call_IMUUpdate_FUNC(IMU_classdef *p, float *angle, float *gyro)
// {
//     return p->Update();
// }


// #ifdef __cplusplus
// }
// #endif

#endif
