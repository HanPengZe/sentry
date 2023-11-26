/**
 * @file IMU_Compensate.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 陀螺仪温漂补偿
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _IMU_COMPENSATE_H_
#define _IMU_COMPENSATE_H_

#ifdef  __cplusplus
extern "C" {
#endif

// #include <stdio.h>
#include <stdint.h>
// #include <string.h>
#include "tim.h"

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为?MPU6500_TEMP_PWM_MAX?-?1
#define GYRO_CONST_MAX_TEMP 40.0f
//陀螺仪开机校准的时间
#define GYRO_OFFSET_START_TIME 500
#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大

#define IMU_Calculating 1
#define Quaternion_Solution 0
#define ShangJiao_Solution 1



#define IMU_CompensateFUNInit        \
    {                                \
        &Preserve_temp,              \
            &IMU_GetData_Compensate, \
    }


typedef struct positionpid_t
{
	float Target;	  //设定目标值
	float Measured;	  //测量值
	float err;		  //本次偏差值
	float err_last;	  //上一次偏差
	float err_change; //误差变化率
	float Kp;
	float Ki;
	float Kd; //Kp, Ki, Kd控制系数
	float p_out;
	float i_out;
	float d_out;			   //各部分输出值
	float Output;				   //Output输出
	float MaxOutput;		   //输出限幅
	float Integral_Separation; //积分分离阈值
	float IntegralLimit;	   //积分限幅
	float (*IMUTemp_Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;


typedef struct
{
    void (*Preserve_temp)(float Real_temp);
    void (*IMU_GetData_Compensate)(void);
} IMU_CompensateFUN_t;

typedef struct
{
	float angle_degree[3];
	float Gyro[3];
	float last_angle_degree[3];
	float total[3];
	int16_t TurnsNum[3];
	float temp;
	HAL_StatusTypeDef InfoUpdateFlag;
	uint8_t OffLineFlag;

} mpu6500_Exportdata_t;



extern mpu6500_Exportdata_t mpu6500_Exportdata;
extern IMU_CompensateFUN_t IMU_CompensateFUN;

float IMUTemp_Position_PID(positionpid_t *pid_t, float target, float measured);

#ifdef __cplusplus
}
#endif

#endif /*_IMU_COMPENSATE_H_*/
