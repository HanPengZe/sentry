/**
 ------------------------------------------------------------------------------
 * @file    Vision.cpp
 * @author  Shake
 * @brief   视觉数据通信
 * @version V1.0
 * @date    2021-11
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "Vision.h"

/* Private function declarations ---------------------------------------------*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);
const unsigned char MyCRC8Tab[] = MY_CRCTAB_INIT;

/* function prototypes -------------------------------------------------------*/
/**
 * @brief      Initialize Vision Class
 * @param[in]  None
 * @retval     None
 */
Vision_classdef::Vision_classdef()
{
    
}


/**
 * @brief      对接收的数据解包
 * @param[in]  data
 * @retval     None
 */
void Vision_classdef::Recv_Data(uint8_t *data)
{
    CRCBuf = Checksum_CRC8(data, 11);
  
	Recv_Pack.Start_Tag = data[0];
	Recv_Pack.mode = data[1];
	Recv_Pack.Aotu_Shoot = data[2];
	Recv_Pack.Yaw_sign = data[3];
	Recv_Pack.Yaw_DataL = data[4];
	Recv_Pack.Yaw_DataH = data[5];
	Recv_Pack.Pitch_sign = data[6];
	Recv_Pack.Pitch_DataL = data[7];
	Recv_Pack.Pitch_DataH = data[8];
	Recv_Pack.Depth_DataL = data[9];
	Recv_Pack.Depth_DataH = data[10];
	Recv_Pack.crc = data[11];
	Recv_Pack.End_Tag = data[12];

	Recv_Pack.x = (float)((Recv_Pack.Yaw_DataH << 8) | Recv_Pack.Yaw_DataL) / 100.0f;
	Recv_Pack.y = (float)((Recv_Pack.Pitch_DataH << 8) | Recv_Pack.Pitch_DataL) / 100.0f;
	Recv_Pack.depth = (Recv_Pack.Depth_DataH << 8) | Recv_Pack.Depth_DataL;

	(Recv_Pack.Yaw_sign == 0)?(Recv_Pack.x *= -1.0f):Recv_Pack.x;
	(Recv_Pack.Pitch_sign == 0)?(Recv_Pack.y *= -1.0f):Recv_Pack.y;
}


/**
 * @brief      设置给视觉发送的数据
 * @param[in]  angle_Yaw,angle_Pit,Gyro_w,Gyro_y
 * @retval     None
 */
void Vision_classdef::Set_TXData(float angle_Yaw, float angle_Pit, float Gyro_w, float Gyro_y)
{
    Tras_Pack.IMUangle_t.IMU_Yaw = angle_Yaw;
	Tras_Pack.IMUangle_t.IMU_Pit = angle_Pit;
	Tras_Pack.Gyro_z = Gyro_w * 100;
	Tras_Pack.GyroZ_H = Tras_Pack.Gyro_z << 8;
	Tras_Pack.GyroZ_L = Tras_Pack.Gyro_z;

	Tras_Pack.Gyro_y = Gyro_y * 100;
	Tras_Pack.GyroY_H = Tras_Pack.Gyro_y << 8;
	Tras_Pack.GyroY_L = Tras_Pack.Gyro_y;

	Update_TXBuf();
}

/**
 * @brief      更新发送的数据
 * @param[in]  None
 * @retval     None
 */
void Vision_classdef::Update_TXBuf()
{
    for (uint8_t n = 0; n < 5; n++)
	{
		TX_Buf[n][4] = Tras_Pack.IMUangle_t.IMUAngle_Data[0];
		TX_Buf[n][5] = Tras_Pack.IMUangle_t.IMUAngle_Data[1];
		TX_Buf[n][6] = Tras_Pack.IMUangle_t.IMUAngle_Data[2];
		TX_Buf[n][7] = Tras_Pack.IMUangle_t.IMUAngle_Data[3];

		TX_Buf[n][8] = Tras_Pack.IMUangle_t.IMUAngle_Data[4];
		TX_Buf[n][9] = Tras_Pack.IMUangle_t.IMUAngle_Data[5];
		TX_Buf[n][10] = Tras_Pack.IMUangle_t.IMUAngle_Data[6];
		TX_Buf[n][11] = Tras_Pack.IMUangle_t.IMUAngle_Data[7];

		TX_Buf[n][12] = Tras_Pack.GyroZ_H;
		TX_Buf[n][13] = Tras_Pack.GyroZ_L;

		TX_Buf[n][14] = Tras_Pack.GyroY_H;
		TX_Buf[n][15] = Tras_Pack.GyroY_L;

		TX_Buf[n][16] = 30/* ext_game_robot_state.data.shooter_id1_17mm_speed_limit */;
	}


	// switch (ROBOT_ID) // --- 步兵ID
	// {
	// 	// --- 己方红
	// case 3:
	// case 4:
	// case 5:
	// 	HAL_UART_Transmit(&huart7, TX_Buf[0], 18, 0xFF); //blue
	// 	/* for (int i = 0; i < 16; i++)
	// 	{
	// 		while ((USART6->SR & 0X40) == 0)
	// 			;
	// 		huart6.Instance->DR = Vision_SendBuf[0][i];
	// 	} */
	// 	break;
	// 	// --- 己方蓝
	// case 103:
	// case 104:
	// case 105:
	// 	HAL_UART_Transmit(&huart7, TX_Buf[1], 18, 0xFF); //red
	// 	/* for (int i = 0; i < 16; i++)
	// 	{
	// 		while ((USART6->SR & 0X40) == 0)
	// 			;
	// 		huart6.Instance->DR = Vision_SendBuf[0][i];
	// 	} */
	// 	break;
	// default:
	// 	HAL_UART_Transmit(&huart7, Vision_SendBuf[0], 18, 0xFF); //blue
	// 	break;
	// }
}



/**
  * @Data       2019-04-02 17:44
  * @brief      CRC8校验
  * @param[in]  buf
  * @param[in]  len
  * @retval     None
  */
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len)
{	
	uint8_t check = 0;

	while(len--)
	{
		check = MyCRC8Tab[check^(*buf++)];
	}

	return (check) & 0x00ff;
}


