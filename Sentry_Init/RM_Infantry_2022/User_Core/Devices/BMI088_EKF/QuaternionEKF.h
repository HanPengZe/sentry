/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include <stdint.h>
#include "kalman_filter_IMU.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

#define USE_IMU_EKF 1

typedef struct
{
    uint8_t Initialized;
    KkalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // ��Ԫ������ֵ
    float GyroBias[3]; // ��������ƫ����ֵ

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawAngleLast;
    float YawTotalAngle;

    float Q1; // ��Ԫ�����¹�������
    float Q2; // ��������ƫ��������
    float R;  // ���ٶȼ���������

    float dt;                     // ��̬��������
    float ChiSquare;              // ���������⺯��
    float ChiSquareTestThreshold; // ����������ֵ
    float lambda;                 // ��������

    float angle[3]; // �����ⲿ�ӿ�(yaw/pit/rol)
    int32_t round_cnt[3];


} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float LPF_Gyro_y;
extern float LPF_Gyro_z;
extern float yGyro_accLPFcoef;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
