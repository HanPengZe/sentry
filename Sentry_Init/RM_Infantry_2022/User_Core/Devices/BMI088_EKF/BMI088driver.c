/**
 ******************************************************************************
 * @file    BMI088driver.c
 * @author
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "BMI088_EKF/BMI088driver.h"
#include "BMI088_EKF/BMI088reg.h"
#include "BMI088_EKF/BMI088Middleware.h"
#include "bsp_dwt.h"
#include <math.h>
#include "Task_Init.h"

#define key_down 0
#define key_up 1


float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;

BMI088_data_t BMI088;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void
BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)

#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void Calibrate_MPU_Offset(BMI088_data_t *bmi088);


/**
 * @brief      BMI088�ܳ�ʼ��
 * @param[in]  _SPI
 * @retval     None
 */
void BMI088_Init(SPI_HandleTypeDef *bmi088_SPI)
{
    while (bmi088_init(bmi088_SPI) != BMI088_NO_ERROR)
    {
        ;
    }
}

uint8_t bmi088_init(SPI_HandleTypeDef *bmi088_SPI)
{
    BMI088_SPI = bmi088_SPI;
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    //--- ��ȡflash����
    cail_data_read(&BMI088.flash_store.data);
    //--- У׼��־λ
    BMI088.cali_falg = BMI088.flash_store.Pack.cali_flag;

    //--- �Ƿ��Ѿ�У׼��
    if(BMI088.cali_falg != false)
    {
        //--- ����У׼��������
        if(BMI088.flash_store.Pack.offset[0] != 0 &&
            BMI088.flash_store.Pack.offset[1] != 0 &&
             BMI088.flash_store.Pack.offset[2] != 0 &&
              BMI088.flash_store.Pack.offset[3] != 0)
            {
                BMI088.GyroOffset[0] = BMI088.flash_store.Pack.offset[0];
                BMI088.GyroOffset[1] = BMI088.flash_store.Pack.offset[1];
                BMI088.GyroOffset[2] = BMI088.flash_store.Pack.offset[2];
                BMI088.gNorm = BMI088.flash_store.Pack.offset[3];
            }
            else
            {
                //--- ��������쳣��ʹ�ñ��س�ʼ��������
                BMI088.GyroOffset[0] = GxOFFSET;
                BMI088.GyroOffset[1] = GyOFFSET;
                BMI088.GyroOffset[2] = GzOFFSET;
                BMI088.gNorm = gNORM;
            }

        BMI088.AccelScale = 9.81f / BMI088.gNorm;
        BMI088.TempWhenCali = 40;
    }
    else
    {
        //--- ����У׼
        BMI088.start_cali = true;
        //--- �����־λ
        BMI088.cali_falg = false;

        Calibrate_MPU_Offset(&BMI088);
    }

    if(BMI088.start_cali == true && BMI088.flash_store.Pack.cali_flag != false && error == BMI088_NO_ERROR)
    {
        //--- flashд��У׼����
        BMI088.start_cali = false;
        cali_data_write(&BMI088.flash_store.data);
    }

    return error;
}


/**
 * @brief      BMI088����У׼
 * @param[in]  calibrate
 * @retval     None
 */
uint8_t cali_cmd = false;
uint8_t imu_cali_bell; // У׼��ʾ 
void Offset_reset(uint8_t calibrate)
{
    //--- ��ʼ��Ϊδ����
    static uint8_t key_last_level = key_up;

    //--- ��⵽״̬����
	if((calibrate == key_down && key_last_level == key_up))
	{
		cali_cmd = true;
	}
	//--- ������һʱ��key�ĵ�ƽ
	key_last_level = calibrate;

    if (cali_cmd == true)
    {
        Calibrate_MPU_Offset(&BMI088);
        cali_data_write(&BMI088.flash_store.data);
        cali_cmd = false;
        imu_cali_bell = true;
        
        DWT_Delay(1);

        // __set_FAULTMASK(1);    // �ر������ж�
        // HAL_NVIC_SystemReset();// ��λ
    }
}


/**
 * @brief      BMI088У׼��Ư
 * @param[in]  bmi088 data
 * @retval     None
 */
void Calibrate_MPU_Offset(BMI088_data_t *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 10000;  //6000 // ��Ҫ�㹻������ݲ��ܵõ���Ч��������ƫУ׼���
    // static uint16_t cali_cnt = 0;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;

    startTime = DWT_GetTimeline_s();

    do
    {
        if (DWT_GetTimeline_s() - startTime > 10)
        {
            // У׼��ʱ
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm = gNORM;
            bmi088->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.005);
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }

            // ��¼���ݼ���
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
                }
            }

            // ���ݲ��������Ϊ�յ������ţ�������У׼
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;
            DWT_Delay(0.0005);
        }

        // ȡƽ��ֵ�õ��궨���
        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++)
            bmi088->GyroOffset[i] /= (float)CaliTimes;

        // ��¼�궨ʱIMU�¶�
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;

    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    // ���ݱ궨���У׼���ٶȼƱ������
    bmi088->AccelScale = 9.81f / bmi088->gNorm;

    BMI088.flash_store.Pack.offset[0] = bmi088->GyroOffset[0];
    BMI088.flash_store.Pack.offset[1] = bmi088->GyroOffset[1];
    BMI088.flash_store.Pack.offset[2] = bmi088->GyroOffset[2];
    BMI088.flash_store.Pack.offset[3] = bmi088->gNorm;
    BMI088.flash_store.Pack.cali_flag = BMI088.cali_falg = true;


}


uint8_t bmi088_accel_init(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    // do
    // {
    //     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    // } while (res != BMI088_ACC_CHIP_ID_VALUE);

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            // write_reg_num--;
            // return write_BMI088_accel_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            write_reg_num--;
            // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_Read(BMI088_data_t *bmi088)
{
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//     BMI088_read_write_byte( reg );
//     while( len != 0 )
//     {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif




/**
 * @brief      У׼����д��flash
 * @param[in]  write_data
 * @retval     None
 */
static void cali_data_write(uint32_t *write_data)
{
    //copy the data of device calibration data
    //��������洢��cail_sensor[i].flash_buf�е�Ư��������д��flash�Ļ�����������
    // memcpy((void *)(BMI088.flash_store), (void *)write_data, sizeof(flash_write_buf));

    //erase the page
	//����flashҳ�� 1ҳ 1������
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
	//��Ҫд��flash�Ļ��������� ���� �������ĳ��� д��flash��
	//֮���� +3 ��Ϊ�˷�ֹ  FLASH_WRITE_BUF_LENGHT�Ǹ����� ����/4 �õ��˳��Ȳ�������+3 �Ϳ����ü������ȱ䳤��������ȫ��д��flaah�����ᶪʧ  
    cali_flash_write(FLASH_USER_ADDR, write_data, sizeof(write_data));
}


/**
 * @brief      У׼���ݴ�flash����
 * @param[in]  None
 * @retval     None
 */
static void cail_data_read(uint32_t *write_data)
{
    //read the data in flash, 
    //����������32λ����4���ֽڣ����ж�ȡ�ģ�����cali_sensor[i].flash_len�ĳ�ʼ������ͨ��sizeof()������ĳ��ȣ��ֽ�������Ҫ /4
    // cali_flash_read(FLASH_USER_ADDR, (uint32_t *)read_data, 4);
    //���ڴ�Ĵ洢���ֽ�Ϊ��λ���������Ƕ�ȡcali_sensor[i].flash_len���ȵ�32λ����4���ֽڣ������ݣ���ôʵ���϶�ȡ���ֽ�����cali_sensor[i].flash_len * 4 ���Ե�ַҲӦ��������ô��
    //read the name and cali flag,
    //CALI_SENSOR_HEAD_LEGHT ֻ��ȡ1��32λ����4���ֽڣ�������
    cali_flash_read(FLASH_USER_ADDR, write_data, sizeof(write_data)+1);

}



