/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "../BMI088/INS_task.h"

#include "main.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "BSP_SPI.h"
#include "../BMI088/bmi088driver.h"
#include "../BMI088/ist8310driver.h"
#include "../BMI088/MahonyAHRS.h"
#include "../BMI088/calibrate_task.h"

#include "math.h"


#include "tim.h"

#include "arm_math.h"
// #include "System_DataPool.h"
// #include "Dev_IMU.h"


// void Call_IMUUpdate_FUNC(IMU_classdef *p, float *angle, float *gyro);

// #define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define SPEED_FILTER_LEN    6 //速度滤波长度

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

/**
  * @brief          计算静差的函数，如果有返回值则计算成功
  * @param[in]      none
  * @retval         none
  */
float Yaw_staticErr; //--- 静差
float Pit_StaticErr;
float Pit_staticTotalErr;
float Yaw_staticTotalErr;//静差计算
uint8_t Static_Error_Calculate(void);

/**
  * @brief          速度滤波
  * @param[in]      gx gy gz
  * @retval         none
  */
float Pit_total, Yaw_total;
float IMU_FilteSpeed[2];
void Speed_Filter(float gx, float gy, float gz);


void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float quat[4], float *yaw, float *pitch, float *roll);

void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
float PID_calc(pid_type_def *pid, float ref, float set);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];
float accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];
float mag_cali_offset[3];



static uint8_t first_temperate;
static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

//加速度计低通滤波
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

//static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
float INS_gyro[3] = {0.0f, 0.0f, 0.0f};

float INS_OffsetGyro[3]; //--- 解算之后的IMU速度

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
float temp_gz;
void INS_task(void const *pvParameters)
{
    static float roll_cal,pitch_cal,roll_cos,roll_sin,pitch_cos,pitch_sin;

    //--- 校准要用到的局部变量
	uint8_t key_level;	//key的电平信号	
	//初始化key电平
	key_level = HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin);

    //wait a time
    osDelay(INS_TASK_INIT_TIME);
	//等待BMI088初始完成
    while(BMI088_init())
    {
        osDelay(100);
    }
	//等待磁力计初始完成
    while(ist8310_init())
    {
        osDelay(100);
    }
		
	//读取BMI088的原始数据，通过读取相应寄存器的值
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //rotate and zero drift 
	//初始时就 通过 加入旋转 和 零飘 去计算陀螺仪的实际角度
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
	//控制温度的PID的初始化
    PID_init(&imu_temp_pid, 0, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	//姿态解算的初始化
    AHRS_init(INS_quat, INS_accel, INS_mag);
	//低通滤波的初始化
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    //get the handle of task
    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
	//等待SPI初始化成功
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

	//SPI DMA 初始化
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    
	//----------- Task Loop ------------------------------------------------//	
    for(;;)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
		//唤醒任务等待SPI传输数据
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }	
        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        //rotate and zero drift 
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


        //加速度计低通滤波
        //accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

        //--- 初始化获取IMU速度静差
        if(Static_Error_Calculate() == 1)
        {
            //--- 速度滤波
            Speed_Filter(INS_gyro[0], INS_gyro[1], INS_gyro[2]);
        }

        //---- 世界Yaw轴速度合成
        roll_cal = (INS_angle[2]);
        pitch_cal = (INS_angle[1]);

        roll_cos = arm_cos_f32(roll_cal);
        roll_sin = arm_sin_f32(roll_cal);

        pitch_cos = arm_cos_f32(pitch_cal);
        pitch_sin = arm_sin_f32(pitch_cal);

        temp_gz = (INS_gyro[2] * roll_cos + INS_gyro[1] * roll_sin + INS_gyro[2] * pitch_cos + INS_gyro[0] * pitch_sin) * (180/PI);
        INS_OffsetGyro[2] = (IMU_FilteSpeed[0]* roll_cos + INS_gyro[1] * roll_sin + IMU_FilteSpeed[0] * pitch_cos + IMU_FilteSpeed[1] * pitch_sin);
        // INS_OffsetGyro[2] = (IMU_FilteSpeed[0]* roll_cos + IMU_FilteSpeed[1] * roll_sin + IMU_FilteSpeed[0] * pitch_cos + INS_gyro[0] * pitch_sin);
        INS_OffsetGyro[1] = IMU_FilteSpeed[1];


        // -------------------
        
        //because no use ist8310 and save time, no use
        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            // ist8310_read_mag(ist8310_real_data.mag);
        }

        //---- IMU校准 ----//
        key_level = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
        IMU_Cali(key_level);
    }
}

/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}


/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
//传进来的参数 就是 校准的结构体变量 cail_sensor[i].flash_buf / 或者更准确说 是 校准变量 cail_sensor_buf[i] 
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
				
		//通过 INS_gyro 实时的gyro的数据 计算出 gyrp的偏移量(gyro_offset) 
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);
		//将计算出来的偏移量赋给 校准变量 cali_offset，即cail_sensor_buf[i] 
		//而校准变量 cail_sensor_buf[i] 就是 校准结构体变量 cail_sensor[i].flash_buf，所以就更新了flash中的值了
        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}


void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
		__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 40.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
		__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        //detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        //detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        //detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {

        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void  imu_cmd_spi_dma(void)
{

        //开启陀螺仪的DMA传输
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //开启加速度计的DMA传输
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        


        
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}


void DMA2_Stream0_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}


void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    // if (pid->mode == PID_POSITION)
    // {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    // }
    // else if (pid->mode == PID_DELTA)
    // {
    //     pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    //     pid->Iout = pid->Ki * pid->error[0];
    //     pid->Dbuf[2] = pid->Dbuf[1];
    //     pid->Dbuf[1] = pid->Dbuf[0];
    //     pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    //     pid->Dout = pid->Kd * pid->Dbuf[0];
    //     pid->out += pid->Pout + pid->Iout + pid->Dout;
    //     LimitMax(pid->out, pid->max_out);
    // }
    return pid->out;
}



/**
 * @brief   获取mpu速度静差
 * @param   None
 * @note    计算静差的函数，如果有返回值则计算成功
 * @note    如果上电的时候云台运动，会导致计算出来的静态偏差数值出错。如果每次上电的时候，静态偏差均
 *          差别不大的话，可以直接给定一个固定值。或者，另外对计算出来的偏差值做判断等。
 */
uint8_t Static_Error_Calculate(void)
{
    static uint16_t error_cnt = 0;
    static uint8_t calc_flag = 0;

    if(calc_flag == 0)
    {
        if(error_cnt < 1000)
        {
            Yaw_staticTotalErr += INS_gyro[2];
            Pit_staticTotalErr += INS_gyro[0];
        }
        else
        {
            Yaw_staticErr = Yaw_staticTotalErr/1000.f;
            Pit_StaticErr = Pit_staticTotalErr/1000.f;
            calc_flag = 1;
        }
        error_cnt++;

        return 0;
    }
    return 1;
}

/**
 * @brief   速度滤波
 * @param   gx gy gz
 * @note    由于安装的方式板子的Rol是机器人的Pit 
 */
void Speed_Filter(float gx, float gy, float gz)
{
    static float Pit_Filter[SPEED_FILTER_LEN];
    static float Yaw_Filter[SPEED_FILTER_LEN];
    static float Yaw_Gyro = 0;
    static float Pit_Gyro = 0;

    gz -= Yaw_staticErr; 
    gx -= Pit_StaticErr;

    //--- 数据填充更新
    for(int16_t i = (SPEED_FILTER_LEN-2) ; i > -1 ; i--)
    {
        Pit_Filter[i+1] = Pit_Filter[i];
        Yaw_Filter[i+1] = Yaw_Filter[i];
    }
    Pit_Filter[0] = gx;
    Yaw_Filter[0] = gz;

    Pit_total = 0;
    Yaw_total = 0;

    for(int16_t i = 0 ; i < SPEED_FILTER_LEN ; i++)
    {
        Pit_total += Pit_Filter[i];
        Yaw_total += Yaw_Filter[i];
    }

    Yaw_Gyro = Yaw_total/SPEED_FILTER_LEN;
    Pit_Gyro = Pit_total/SPEED_FILTER_LEN;
    
    //--- 死区处理 不好用
    // if(fabs(Yaw_Gyro) < 0.174532935f)
    // {
    //     Yaw_Gyro = 0;
    // }
    // if(fabs(Pit_Gyro) < 0.174532935f)
    // {
    //     Pit_Gyro = 0;
    // }
    
    IMU_FilteSpeed[0] = Yaw_Gyro;
    IMU_FilteSpeed[1] = Pit_Gyro;

}

