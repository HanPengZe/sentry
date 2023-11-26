/**
 * @file IMU_Compensate.c
 * @author Miraggio (w1159904119@gmail)
 * @brief ��������Ư����
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "IMU_Compensate.h"
#include "DJI_AIMU.h"
#include "mpu6500reg.h"
#include "AHRS.h"
#include "Filter.h"
// #include "User_Math.h"
#include "mytype.h"
// #include "BSP_ADC.h"



int8_t Calibrate_Temperate = 0;
positionpid_t imuTempPid = {                      \
        0,                 \
            0,             \
            0,             \
            0,             \
            0,             \
            1600.0f,       \
            0.2f,          \
            0.0f,          \
            0,             \
            0,             \
            0,             \
            0,             \
            4500.0f,       \
            0,             \
            4400.0f,       \
            &IMUTemp_Position_PID, \
    };
// #undef imuTempPidInit

void Preserve_temp(float Real_temp);
void IMU_GetData_Compensate(void);
IMU_CompensateFUN_t IMU_CompensateFUN = IMU_CompensateFUNInit;
#undef IMU_CompensateFUNInit

static mpu6500_real_data_t mpu6500_real_data; //ת���ɹ��ʵ�λ��MPU6500����
static ist8310_real_data_t ist8310_real_data; //ת���ɹ��ʵ�λ��IST8310����
mpu6500_Exportdata_t mpu6500_Exportdata;      //�������������ֵ
static float gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f}; //��������Ư
static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};

static float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //ŷ���� ��λ rad
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //��Ԫ��

static const float TimingTime = 0.002f; //�������е�ʱ�� ��λ s				//-------------------��Ҫ�ĳ��Լ�������ʱ��
uint8_t first_temperate = 0;

static float get_control_temperate(void)
{
    // return mpu6500_real_data.temp;
    return imu.temp;
}

/**
 * @brief �¶Ȳ���
 * 
 * @param temp 
 * @return  
 */
static void IMU_temp_Control(float temp)
{
    uint16_t tempPWM;
    // static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        imuTempPid.IMUTemp_Position_PID(&imuTempPid, temp, get_control_temperate());
        if (imuTempPid.Output < 0.0f)
        {
            imuTempPid.Output = 0.0f;
        }
        if(get_control_temperate() >= temp)
        {
            imu_Export.Temp_ReachFlag = 1;
        }
        tempPWM = (uint16_t)imuTempPid.Output;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        if (temp * 0.8f > get_control_temperate())
        {
            // temp_constant_time++;
            // if (temp_constant_time > 100)
            // {
            //     //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
            //     first_temperate = 1;
            //     imuTempPid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
            // }
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MPU6500_TEMP_PWM_MAX - 1);
        }
        else
        {
            imuTempPid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
            first_temperate = 1;
        }

        
    }
}

static void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    // if (imu_status & (1 << MPU_MOT_BIT))
    // {
    //     (*offset_time_count) = 0;
    //     return;
    // }

    // if (imu_status & (1 << MPU_DATA_READY_BIT))
    // {
    gyro_offset[0] = gyro_offset[0] - GYRO_OFFSET_KP * gyro[0];
    gyro_offset[1] = gyro_offset[1] - GYRO_OFFSET_KP * gyro[1];
    gyro_offset[2] = gyro_offset[2] - GYRO_OFFSET_KP * gyro[2];
    (*offset_time_count)++;
    // }
}

/**
 * @brief ADC�¶Ȼ�ȡ
 * 
 * @param Real_temp 
 * @return  
 */
void Preserve_temp(float Real_temp)
{
    Calibrate_Temperate = Real_temp;
    if (Calibrate_Temperate > (int8_t)GYRO_CONST_MAX_TEMP)
    {
        Calibrate_Temperate = (int8_t)GYRO_CONST_MAX_TEMP;
    }
}

/**
 * @brief ��������������
 * 
 * @return  
 */
float IMU_LpfAttFactor = 0.1;
static void Update_IMU_Data(void)
{
    DJI_IMUFUN.mpu_get_data();

    //����ȡ����ֵ���������Ľṹ��
    DJI_IMUFUN.IMU_data_read_over(&mpu6500_real_data, &ist8310_real_data);

    //��ȥ��Ư�Լ���ת����ϵ
    DJI_IMUFUN.IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);

    //���ٶȼƵ�ͨ�˲�
    static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    //�ж��Ƿ��һ�ν��룬�����һ�����ʼ����Ԫ����֮�������Ԫ������Ƕȵ�λrad
    static uint8_t updata_count = 0;

    // if (mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
    // {

    if (updata_count == 0)
    {
        //��ʼ����Ԫ��
        AHRS_init(INS_quat, INS_accel, INS_mag);
        get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
        updata_count++;
    }
    else
    {
        //���ٶȼƵ�ͨ�˲�
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        //������Ԫ��
        AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

        //�����ǿ���У׼
        {
            static uint16_t start_gyro_cali_time = 0;
            if (start_gyro_cali_time == 0)
            {
                Gyro_Offset[0] = gyro_cali_offset[0];
                Gyro_Offset[1] = gyro_cali_offset[1];
                Gyro_Offset[2] = gyro_cali_offset[2];
                start_gyro_cali_time++;
            }
            else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
            {
                // IMUWarnBuzzerOn();
                if (1)
                {
                    // ������gyro_offset������������˶�start_gyro_cali_time++��������˶� start_gyro_cali_time = 0
                    gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
                }
            }
            else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
            {
                start_gyro_cali_time++;
            }
        } //�����ǿ���У׼   code end

        //������ֵת��Ϊ360�Ƕ�ֵ
        //��̬�� ��rad ��� �ȣ����������̬�ǵĵ�λΪ�ȣ������ط�����̬�ǣ���λ��Ϊ����
        mpu6500_Exportdata.temp = mpu6500_real_data.temp;

        for (int i = 0; i < 3; i++)
        {
            mpu6500_Exportdata.angle_degree[i] = INS_Angle[i] * 57.3f + 180.0f;
            if (mpu6500_Exportdata.angle_degree[i] - mpu6500_Exportdata.last_angle_degree[i] < -300)
            { //����������ء�
                mpu6500_Exportdata.TurnsNum[i]++;
            }
            if (mpu6500_Exportdata.last_angle_degree[i] - mpu6500_Exportdata.angle_degree[i] < -300)
            {
                mpu6500_Exportdata.TurnsNum[i]--;
            }
            mpu6500_Exportdata.total[i] = mpu6500_Exportdata.angle_degree[i] + (360 * mpu6500_Exportdata.TurnsNum[i]);

            mpu6500_Exportdata.last_angle_degree[i] = mpu6500_Exportdata.angle_degree[i];

            // Filter_IIRLPF(&mpu6500_real_data.original_gyro[i], &mpu6500_Exportdata.Gyro[i], IMU_LpfAttFactor);
            // mpu6500_Exportdata.Gyro[i] = mpu6500_real_data.original_gyro[i];

            mpu6500_Exportdata.InfoUpdateFlag = mpu6500_real_data.InfoUpdateFlag;
        }

    } //update count if   code end
    // }     //mpu6500 status  if end
}

void HAL_Update_IMU_Data(void)
{
    DJI_IMUFUN.HAL_mpu_get_data();
    DJI_IMUFUN.HAL_imu_ahrs_update();
    DJI_IMUFUN.HAL_imu_attitude_update();
}

/**
 * @brief �²����ϻ�ȡ����
 * 
 * @return  
 */
void IMU_GetData_Compensate(void)
{
    IMU_temp_Control(Calibrate_Temperate);
#if IMU_Calculating == Quaternion_Solution
    Update_IMU_Data();
#elif IMU_Calculating == ShangJiao_Solution
    HAL_Update_IMU_Data();
#endif
}


/**
  * @brief  IMU�²�PID - ��Ϊ��ֲ�Ŀ�ܲ�һ���������޸�ֱ�ӵ�������һ������ 
  * @param  None
  * @retval None
  */
float IMUTemp_Position_PID(positionpid_t *pid_t, float target, float measured)
{

	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	pid_t->err_change = pid_t->err - pid_t->err_last;

	pid_t->p_out = pid_t->Kp * pid_t->err;
	pid_t->i_out += pid_t->Ki * pid_t->err;
	pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
	//�����޷�
	// Constrain(&pid_t->i_out, -pid_t->IntegralLimit, pid_t->IntegralLimit); //ȡ������������޷���
    if(pid_t->i_out < -pid_t->IntegralLimit)
        pid_t->i_out = -pid_t->IntegralLimit;
    else if(pid_t->i_out > pid_t->IntegralLimit)
        pid_t->i_out = pid_t->IntegralLimit;

	pid_t->Output = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

	//����޷�
	// Constrain(&pid_t->Output, -pid_t->MaxOutput, pid_t->MaxOutput);
    if(pid_t->Output < -pid_t->MaxOutput)
        pid_t->Output  = -pid_t->MaxOutput;
    else if(pid_t->Output > pid_t->MaxOutput)
        pid_t->Output  = pid_t->MaxOutput;

	pid_t->err_last = pid_t->err;
	return pid_t->Output;
}

