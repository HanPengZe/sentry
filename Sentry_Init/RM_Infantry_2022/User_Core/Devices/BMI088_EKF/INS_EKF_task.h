/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#ifdef  __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "BMI088_EKF/BMI088driver.h"
#include "BMI088_EKF/QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    uint8_t mode;
    //
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //
    float max_iout; //

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //
    float error[3]; //

} pid_type_def;

typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float Gyro[3];  // ���ٶ�
    float Accel[3]; // ���ٶ�
    float MotionAccel_b[3]; // ����������ٶ�
    float MotionAccel_n[3]; // ����ϵ���ٶ�

    float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��

    // ���ٶ��ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // λ��
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;


/**
 * @brief ����������װ���Ĳ���,demo�п�����
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

void INS_EKF_Init(void);
void INS_EKF_Task(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#ifdef  __cplusplus
}
#endif


#endif
