/**
 ------------------------------------------------------------------------------
 * @file    Vision.cpp
 * @author  Shake
 * @brief   视觉数据通信
 * @version V1.6
 * @date    2022-03
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "Control_Vision.h"
#include "System_DataPool.h"
#include "Slope_Filter.h"
/* Private macros ------------------------------------------------------------*/
// --- 是否使用卡尔曼
#define USE_YAW_KALMAN_PREDICT  0
#define USE_PIT_KALMAN	0

#define CRC_CHECK_LEN  14
// --- 卡尔曼滤波结果
#define KF_ANGLE 0
#define KF_SPEED 1

#define YAW_MAX 30.0f
#define PIT_MAX 20.0f
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);
// const unsigned char MyCRC8Tab[] = {	
// 		0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
// 		157,195,33,127,252,162,64,30, 95,1,227,189,62,96,130,220,
// 		35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
// 		190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
// 		70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
// 		219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
// 		101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
// 		248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
// 		140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
// 		17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
// 		175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
// 		50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
// 		202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
// 		87,9,235,181,54,104,138,212,149,203, 41,119,244,170,72,22,
// 		233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
// 		116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
// 	};
//const unsigned char MyCRC8Tab[] = {	
//	0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31, 65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220, 35,  125, 159, 193, 66,
//    28,  254, 160, 225, 191, 93,  3,   128, 222, 60,  98,  190, 224, 2,   92, 223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218,
//    56,  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25, 71,  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122,
//    36,  248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,  231, 185, 140, 210, 48,  110, 237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243,
//    112, 46,  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19, 77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208, 83,  13,  239, 177, 240,
//    174, 76,  18,  145, 207, 45,  115, 202, 148, 118, 40,  171, 245, 23,  73, 8,   86,  180, 234, 105, 55,  213, 139, 87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170,
//    72,  22,  233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74, 20,  246, 168, 116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53
//};


 /**						CRC16							***/
/*CRC8校验*/
//unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);
/*CRC16添加校验位*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
/*CRC16校验*/
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
uint16_t CRC16_Check_Sum(uint8_t * pchMessage, uint16_t dwLength);
// Copyright (c) 2022 // Licensed under the Apache-2.0 License.
uint16_t CRC16_INIT = 0xFFFF;
const uint16_t W_CRC_TABLE[256] = {  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 
								0xbed3,  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c,
								0x75b7, 0x643e,  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102,
								0x308b, 0x0210, 0x1399,  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 
								0xeb6e, 0xfae7, 0xc87c, 0xd9f5,  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 
								0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 
								0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 
								0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,  0x14a1, 0x0528, 0x37b3, 0x263a, 
								0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,  0x6306, 0x728f, 0x4014, 
								0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,  0xa96a, 0xb8e3, 
								0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,  0xffcf, 
								0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,  
								0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 
								0x7cff,  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 
								0x3bd3, 0x2a5a,  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 
								0xf2a7, 0xc03c, 0xd1b5,  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
								0xb58b, 0xa402, 0x9699, 0x8710,  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 
								0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 
								0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 
								0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 
								0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,  0xa12a, 0xb0a3, 0x8238, 
								0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,  0xf78f, 0xe606, 
								0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,  0x3de3, 
								0x2c6a, 0x1ef1, 0x0f78};
/**  * @brief CRC16 Caculation function  
	* @param[in] pchMessage : Data to Verify,  
	* @param[in] dwLength : Stream length = Data + checksum  
	* @param[in] wCRC : CRC16 init value(default : 0xFFFF)  
	* @return : CRC16 checksum  */
uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{  
	uint8_t ch_data;
	if (pchMessage == NULL) 
		return 0xFFFF;  
	while (dwLength--) 
	{    
		ch_data = *pchMessage++;    
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];  
	}
  return wCRC;
}
/**  * @brief CRC16 Verify function  
	* @param[in] pchMessage : Data to Verify,  
	* @param[in] dwLength : Stream length = Data + checksum  
	* @return : True or False (CRC Verify Result)  
*/
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{ 
	uint16_t w_expected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) 
	{
		return 0;
	}
	w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);  
	return ((w_expected & 0xff) == pchMessage[dwLength - 2] && ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/**  * @brief Append CRC16 value to the end of the buffer  
	* @param[in] pchMessage : Data to Verify,  
	* @param[in] dwLength : Stream length = Data + checksum  
	* @return none  
*/
uint16_t w_crc = 0;
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{  
	
	if ((pchMessage == NULL) || (dwLength <= 2)) 
	{
		return;
	}
	w_crc = Get_CRC16_Check_Sum((uint8_t *)(pchMessage), dwLength - 2, CRC16_INIT);
	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0xff);  
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0xff);

}  // namespace crc16

uint16_t CRC16_Check_Sum(uint8_t * pchMessage, uint16_t dwLength)
{ 
	uint16_t w_expected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) 
	{
		return 0;
	}
	w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);  
	return w_expected & 0xffff;
}
/*-----------------------------------file of end------------------------------*/


// -------------- 视觉卡尔曼 -------------- //
// --- 视觉一阶卡尔曼
extKalman_t VisionDistance_Kalman;  // 深度 
// --- 视觉二阶卡尔曼
kalmanFilter_t yaw_kalman_filter;
kalmanFilter_t pit_kalman_filter;

speed_calc_data_t VisionKF_YawSpeed;
speed_calc_data_t VisionKF_PitSpeed;

// --- 卡尔曼Yaw,Pitch参数初始化
kalmanfilter_Init_t KalmanPara_Yaw /* =  KalmanParam_Yaw_Init */;
// #undef KalmanParam_Yaw_Init
kalmanfilter_Init_t KalmanPara_Pit /* = KalmanParam_Pit_Init */; 
// #undef KalmanParam_Pit_Init

// Vision_KFData_t Vision_KFData;

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
 * @brief      视觉通信串口注册 参数初始化
 * @param[in]  huart
 * @retval     None
 */
void Vision_classdef::Init(UART_HandleTypeDef *_huart)
{
	vision_uart = _huart;

	// --- 视觉一阶卡尔曼初始化
	KalmanCreate(&VisionDistance_Kalman, 1, 50); 

	// --- 视觉二阶卡尔曼初始化
	mat_init(&yaw_kalman_filter.Q,2,2, KalmanPara_Yaw.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, KalmanPara_Yaw.R_data);
	kalmanFilter_Init(&yaw_kalman_filter, &KalmanPara_Yaw);
	
	mat_init(&pit_kalman_filter.Q,2,2, KalmanPara_Pit.Q_data);
	mat_init(&pit_kalman_filter.R,2,2, KalmanPara_Pit.R_data);
	kalmanFilter_Init(&pit_kalman_filter, &KalmanPara_Pit);

	KF_Param.Forecast_Delay = 50.0f; //预测延时开启
	KF_Param.Yaw_Forecast = 25.0f; //移动预测系数,越大预测越多  90 
	KF_Param.YawForecast_Limit = 200.0f; // 预测限幅 220  280
	KF_Param.Yaw_Threshol.Speed_L = 0.5f; // 速度低于该值关闭预测
	KF_Param.Yaw_Threshol.Speed_H = 10.0f; // 速度高于该值关闭预测 10  
	KF_Param.Yaw_Threshol.Angle_H = 150.0f; // 角度大于该值关闭预测  120
	KF_Param.Temp.YawForecast_Para = 60.0f;

	KalmanCreate(&Predict.Angle_KF, 1, 50);
	KalmanCreate(&Predict.Omiga_KF, 1, 50);
	KalmanCreate(&Predict.Accel_KF, 1, 50);
	KalmanCreate(&Predict.Out_KF, 1, 35);

	Predict.Ko = 60.0f;
	Predict.Ka = 50.0f;

}



/**
 * @brief      对接收的数据解包
 * @param[in]  data
 * @retval     None
 */
#if WYT_AIM
uint16_t crc_jiajia;
uint16_t nan_jiajia;
void Vision_classdef::Sentry_RecvFromPC(uint8_t *data, uint16_t ReceiveLen)
{
	DevicesMonitor.Get_FPS(&TIME, &FPS);
	//--- 不校验帧头帧尾以及crc位
//	CRCBuf = Checksum_CRC8(Recv_Msg.data+1, sizeof(Recv_Msg.Pack)-3);
	//--- 硬件波动会出现Nan值!!!
	if(Sentry_Recv_Msg.Pack.header != 0xA5)
	{
		crc_jiajia++;
		Sentry_Recv_Msg.Pack.yaw = Sentry_Recv_Msg.Pack.pit /* = Recv_Msg.Pack.depth */ = 0;
		Sentry_Recv_Msg.Pack.enemy = Sentry_Recv_Msg.Pack.auto_aim = Update_Flag = false;
		return;
	}
	
	if(Sentry_Recv_Msg.Pack.yaw<30 && Sentry_Recv_Msg.Pack.yaw>-30)
	{
		
	}
	else
	{
		Sentry_Recv_Msg.Pack.enemy = Sentry_Recv_Msg.Pack.auto_aim = Update_Flag = false;
		Sentry_Recv_Msg.Pack.pit = 0;
		Sentry_Recv_Msg.Pack.yaw = 0;
		return;
	}
	if(Sentry_Recv_Msg.Pack.pit<30 && Sentry_Recv_Msg.Pack.pit>-30)
	{
		
	}
	else
	{
		Sentry_Recv_Msg.Pack.enemy = Sentry_Recv_Msg.Pack.auto_aim = Update_Flag = false;
		Sentry_Recv_Msg.Pack.pit = 0;
		Sentry_Recv_Msg.Pack.yaw = 0;
		return;
	}
	
	Sentry_Recv_Msg.Pack.yaw = Sentry_Recv_Msg.Pack.yaw;
	Sentry_Recv_Msg.Pack.pit = Sentry_Recv_Msg.Pack.pit;
//	if(abs(Recv_Msg.Pack.yaw)>1)
//	{
//		if(abs(Recv_Msg.Pack.yaw)>3)
//		{
//			Tar_Yaw_Vision = (Gimbal.Get_IMUTotalAngle(Yaw)+ Vision.Recv_Msg.Pack.yaw)*ENCODER_ANGLE_RATIO*1.2;
//		}
//		else
//		{
//			Tar_Yaw_Vision = (Gimbal.Get_IMUTotalAngle(Yaw)+ Vision.Recv_Msg.Pack.yaw)*ENCODER_ANGLE_RATIO*((abs(Recv_Msg.Pack.yaw)-1)/2*0.2+1);
//		}
//	}
//	else
//	{
//		Tar_Yaw_Vision = (Gimbal.Get_IMUTotalAngle(Yaw)+ Vision.Recv_Msg.Pack.yaw)*ENCODER_ANGLE_RATIO;
//	}

	Tar_Yaw_Vision = (Gimbal.Get_IMUTotalAngle(Yaw)+ Sentry_Recv_Msg.Pack.yaw)*ENCODER_ANGLE_RATIO;//在总角度的基础上加上视觉给的偏差值
	Tar_Pit_Vision = (Gimbal.Get_IMUTotalAngle(Rol)+ Sentry_Recv_Msg.Pack.pit)*ENCODER_ANGLE_RATIO;//在总角度的基础上加上视觉给的偏差值

	//敌人过滤处理
	switch(Sentry_Recv_Msg.Pack.enemy_id)
	{
		case 0://前哨站
			if(Radar_Gimbal_Mode!=2){Sentry_Recv_Msg.Pack.enemy=0;}
		break;
		case 1:if(Infantry.God_State[0]){Sentry_Recv_Msg.Pack.enemy=0;}break;
		case 2:if(Infantry.God_State[1]){Sentry_Recv_Msg.Pack.enemy=0;}break;
		case 3:if(Infantry.God_State[2]){Sentry_Recv_Msg.Pack.enemy=0;}break;
		case 4:if(Infantry.God_State[3]){Sentry_Recv_Msg.Pack.enemy=0;}break;
		case 5:if(Infantry.God_State[4]){Sentry_Recv_Msg.Pack.enemy=0;}break;
		case 6://哨兵
			//等哨兵无敌解除
			if(Infantry.Enemy_Sentry_God_State)
			{
				Sentry_Recv_Msg.Pack.enemy=0;
			};
		break;
		
		case 7://基地
			
		break;			
	}
	
	if(Sentry_Recv_Msg.Pack.enemy==1)
	{
		if(Sentry_Recv_Msg.Pack.auto_aim==1 && aim_flag)
		{
			Shoot.ContLaunch = true;
//			Shoot.Set_ReloadNum(1);
		}
		else
		{
			Shoot.ContLaunch = false;
//			Shoot.Set_ReloadNum(0);
		}
	}
	else
	{
		Sentry_Recv_Msg.Pack.auto_aim=0;
		Shoot.Set_ReloadNum(0);
	}
}
#else
uint16_t crc_jiajia;
uint16_t nan_jiajia;
void Vision_classdef::RecvFromPC(uint8_t *data, uint16_t ReceiveLen)
{
	DevicesMonitor.Get_FPS(&TIME, &FPS);
	//--- 不校验帧头帧尾以及crc位
	CRCBuf = Checksum_CRC8(Recv_Msg.data+1, sizeof(Recv_Msg.Pack)-3);
	//--- 硬件波动会出现Nan值!!!
	if( CRCBuf != Recv_Msg.Pack.crc || Recv_Msg.Pack.start_Tag != 'S' || Recv_Msg.Pack.end_Tag != 'E' /* || Recv_Msg.Pack.mode != true */)
	{
		crc_jiajia++;
		Recv_Msg.Pack.yaw = Recv_Msg.Pack.pit /* = Recv_Msg.Pack.depth */ = 0;
		Recv_Msg.Pack.mode = Recv_Msg.Pack.auto_Shoot = Update_Flag = false;
		return;
	}

	//--- Nan也能过CRC校验
	if (isnan(Recv_Msg.Pack.yaw)==true || isnan(Recv_Msg.Pack.pit)==true)
	{
		nan_jiajia++;
	}
	
	Recv_Msg.Pack.yaw = isnan(Recv_Msg.Pack.yaw)==true ? 0 : Recv_Msg.Pack.yaw;
	Recv_Msg.Pack.pit = isnan(Recv_Msg.Pack.pit)==true ? 0 : Recv_Msg.Pack.pit;

	// Constrain(&Recv_Msg.Pack.yaw, -YAW_MAX, YAW_MAX);
	// Constrain(&Recv_Msg.Pack.pit, -PIT_MAX, PIT_MAX);
	// Constrain(&Recv_Msg.Pack.auto_Shoot, (uint8_t)0, (uint8_t)20);

	Update_Flag = true;

	/*--------------------------------------- 分割线 -------------------------------------------------*/
	// static USART_COB Visionusart_RxCOB;
	// //Send To UART Receive Queue
	// if(Vision_QueueHandle != NULL)
	// {
	// 	Visionusart_RxCOB.port_num = 6; /*USART6*/
	// 	Visionusart_RxCOB.len      = ReceiveLen;
	// 	Visionusart_RxCOB.address  = data;
	//
	// 	//--- 通过队列发送出去与串口层解耦
	// 	xQueueSendFromISR(Vision_QueueHandle,&Visionusart_RxCOB,0);
	//
	// 	Update_Flag = true;
	// 	DevicesMonitor.Get_FPS(&TIME, &FPS);
	// }
	// return;
}
#endif


 float cunchu[5];
float XiangTongTime;
void Vision_classdef::Radar_RecvFromPC(uint8_t *data, uint16_t ReceiveLen)
{
	if(Radar_Recv_Msg.Pack.start != 'S' || Radar_Recv_Msg.Pack.end != 'E')
	{
		Radar_Recv_Msg.Pack.chassis_lx = 0;
		Radar_Recv_Msg.Pack.chassis_ly = 0;
		Radar_Recv_Msg.Pack.chassis_az = 0;
		Radar_Recv_Msg.Pack.chassis_forward = 0;
	}
	else
	{
		if(Radar_Recv_Msg.Pack.chassis_forward>30)
		{
			ForTurn_Yaw_Radar = 30;
		}
		else if(Radar_Recv_Msg.Pack.chassis_forward<-30)
		{
			ForTurn_Yaw_Radar = -30;
		}
		else
		{
			ForTurn_Yaw_Radar = Radar_Recv_Msg.Pack.chassis_forward;
		}
		ForTurn_Yaw_Radar = Radar_Recv_Msg.Pack.chassis_forward;
//		Radar_Recv_Msg.Pack.chassis_az = -Radar_Recv_Msg.Pack.chassis_az;
	 if(Radar_Recv_Msg.Pack.chassis_lx==cunchu[0] && Radar_Recv_Msg.Pack.chassis_ly==cunchu[0] &&\
		Radar_Recv_Msg.Pack.chassis_az==cunchu[0] && Radar_Recv_Msg.Pack.chassis_forward==cunchu[0])
	 {
			XiangTongTime++;
		 if(XiangTongTime>=10)
		 {
				Radar_Recv_Msg.Pack.chassis_lx = 0;
				Radar_Recv_Msg.Pack.chassis_ly = 0;
				Radar_Recv_Msg.Pack.chassis_az = 0;
				Radar_Recv_Msg.Pack.chassis_forward = 0;
		 }
	 }
	 else
	 {
			XiangTongTime=0;
			cunchu[0] = Radar_Recv_Msg.Pack.chassis_lx;
			cunchu[1] = Radar_Recv_Msg.Pack.chassis_ly;
			cunchu[2] = Radar_Recv_Msg.Pack.chassis_az;
			cunchu[3] = Radar_Recv_Msg.Pack.chassis_forward;
	 }
	 
		if(Radar_Recv_Msg.Pack.chassis_forward>180){Radar_Recv_Msg.Pack.chassis_forward-=360;}
		else if(Radar_Recv_Msg.Pack.chassis_forward<-180){Radar_Recv_Msg.Pack.chassis_forward+=360;}

//		Radar_Gimbal_Mode = Vision.Radar_Recv_Msg.Pack.mode%10;
//		Radar_Chassis_Mode = (Vision.Radar_Recv_Msg.Pack.mode-Radar_Gimbal_Mode)/10;

//		if(Radar_Chassis_Mode!=Last_Radar_Chassis_Mode){}
//		if(Radar_Gimbal_Mode!=Last_Radar_Gimbal_Mode){Gimbal.Enemy_VanishTime=0;}

//		Last_Radar_Chassis_Mode=Radar_Chassis_Mode;
//		Last_Radar_Gimbal_Mode=Radar_Gimbal_Mode;
	}
}


/**
 * @brief      打包数据发送至PC
 * @param[in]  Pack
 * @retval     None
 */
#if WYT_AIM
float yaw_cs,pi_cs,rol_cs;
void Vision_classdef::Sentry_SendToPC(Sentry_VisionSendMsg_u *pack2vision)
{
	//--- Test
	// static uint16_t crc_calc = 0;
	// crc_calc = Checksum_CRC8(pack2vision->data+1,CRC_CHECK_LEN);

	pack2vision->Pack.header = 0x5A;
	// pack2vision->Pack.crc = crc_calc;
	pack2vision->Pack.detect_color = Infantry.Get_DetectColor();
//	pack2vision->Pack.reset_tracker = 0;
//	pack2vision->Pack.reserved = 0;
	pack2vision->Pack.roll = Gimbal.Get_IMUAngle(Pit)-180;
	pack2vision->Pack.pit = Gimbal.Get_IMUAngle(Rol)-180;
	pack2vision->Pack.yaw = Gimbal.Get_IMUAngle(Yaw)-180;/* Gimbal.Get_IMUAngle(Rol) */; // 发yaw的imu速度给视觉测试用
//	pack2vision->Pack.checksum = Get_CRC16_Check_Sum(pack2vision->data, 14, CRC16_INIT);
	Append_CRC16_Check_Sum(pack2vision->data, VISION_Sentry_TX_SIZE);

//	pack2vision->Pack.checksum = 0;
//	for(int i=0;i<VISION_Sentry_TX_SIZE;i++)
//	{
//		HAL_UART_Transmit(&huart6, &Sentry_Send_Msg.data[i], 1, 0xff);	//串口传输
//	}
	HAL_UART_Transmit_DMA(&huart6 ,pack2vision->data, VISION_Sentry_TX_SIZE);
}
#else
/**
 * @brief      打包数据发送至PC
 * @param[in]  Pack
 * @retval     None
 */
void Vision_classdef::SendToPC(VisionSendMsg_u *pack2vision)
{
	//--- Test
	// static uint16_t crc_calc = 0;
	// crc_calc = Checksum_CRC8(pack2vision->data+1,CRC_CHECK_LEN);

	pack2vision->Pack.start_tag = 0x53; // 'S'
	// pack2vision->Pack.crc = crc_calc;
	pack2vision->Pack.robot_id = Infantry.Get_ID();
	pack2vision->Pack.mode = (uint8_t)Infantry.Get_VisionMode();
	pack2vision->Pack.yaw_angle = Gimbal.Get_IMUAngle(Yaw);
	pack2vision->Pack.pit_angle = Gimbal.Get_IMUGyro(Yaw)/* Gimbal.Get_IMUAngle(Rol) */; // 发yaw的imu速度给视觉测试用
	pack2vision->Pack.bullet_velocity = 30;
	pack2vision->Pack.end_tag = 0x45; // 'E'

	HAL_UART_Transmit_DMA(vision_uart ,pack2vision->data, sizeof(pack2vision->Pack));
}
#endif

uint8_t xoco_data[12];
void Vision_classdef::Radar_SendToPC(Sentry_RadarSendMsg_u *pack2vision)
{
	Radar_Send_Msg.Pack.start = 'S'; // 'B'
	Radar_Send_Msg.Pack.func = 'B'; // 'B'
	Radar_Send_Msg.Pack.mode = 0; // 'S'
	Radar_Send_Msg.Pack.end = 'E'; // 'E'
//	for(int i=0;i<VISION_Radar_TX_SIZE;i++)
//	{
////		Radar_Send_Msg.data[i]=0;
//		HAL_UART_Transmit(&huart6, &Radar_Send_Msg.data[i], 1, 0xff);	//串口传输
//	}
	HAL_UART_Transmit_DMA(&huart1 ,Radar_Send_Msg.data,  sizeof(Radar_Send_Msg.Pack));
}

/**
  * @Data       2019-04-02 17:44
  * @brief      CRC8校验
  * @param[in]  buf
  * @param[in]  len
  * @retval     None
  */
//unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len)
//{	
//	uint8_t check = 0;

//	while(len--)
//	{
//		check = MyCRC8Tab[check^(*buf++)];
//	}

//	return (check) & 0xff;
//}


/**
 * @brief      自瞄控制 - 卡尔曼滤波
 * @param[in]  state
 * @retval     None
 */
WorldTime_t KF_TIME;
// --- 预测系数
// float Yaw_FarParam = 80.0f;
// float Yaw_NearParam = 65.0f;

float Yaw_FarParam;
float Yaw_NearParam = 25.0f;
// --- 预测角度斜坡变化量
float ForecastAngle_Ramp = 20.0f;

uint8_t Missing_Flag = 1;


float Pre_PitTarget;
void Vision_classdef::KalmanFilter_Ctrl(bool state, float *yawresult, float *pitresult)
{
    if(state == ON)
	{
		if(Update_Flag == true)
		{
			KF_Param.Yaw_Record = (Gimbal.Get_IMUTotalAngle(Yaw) + Get_YawOffset()) * ENCODER_ANGLE_RATIO;
			// KF_Param.Yaw_Record = (Gimbal.Get_IMUTotalAngle(Yaw) + Predict_Ctrl()) * ENCODER_ANGLE_RATIO;

			KF_Param.Pit_Record = Gimbal.Get_CurrentAngle(Pit) + Get_PitOffset()*ENCODER_ANGLE_RATIO;

			KF_TIME.Now = xTaskGetTickCount();  // --- 记录新数据到来的时间
			
			Update_Flag = false; //  
		}

		// ----- 二阶卡尔曼计算 ----- //
		if(KF_TIME.Now != KF_TIME.Pre)
		{
			// --- 计算视觉数据延时
			KF_Param.Data_Delay = KF_TIME.Now - KF_TIME.Pre;

			// --- 更新卡尔曼滤波测量值
			KF_Param.YawAngle_Cur = KF_Param.Yaw_Record;
			KF_Param.PitAngle_Cur = KF_Param.Pit_Record;

			KF_TIME.Pre = KF_TIME.Now;
		}
		//--- 数据是否更新
		// ----- 目标速度解算 -----//
		if(Get_Enemy() == true)
		{
			// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			KF_Param.YawSpeed_Cur = TargetSpeed_Calc(&VisionKF_YawSpeed,\
															KF_TIME.Now,\
															KF_Param.YawAngle_Cur);
			KF_Param.PitSpeed_Cur = TargetSpeed_Calc(&VisionKF_PitSpeed,\
															KF_TIME.Now,\
															KF_Param.PitAngle_Cur);

			// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			KF_Param.YawKF_Result = kalmanFilter_Calc(&yaw_kalman_filter,\
															KF_Param.YawAngle_Cur,\
															KF_Param.YawSpeed_Cur);
			KF_Param.PitKF_Result = kalmanFilter_Calc(&pit_kalman_filter,\
															KF_Param.PitAngle_Cur,\
															KF_Param.PitSpeed_Cur);

		}
		else
		{
			// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			KF_Param.YawSpeed_Cur = TargetSpeed_Calc(&VisionKF_YawSpeed,\
															xTaskGetTickCount(),\
															Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO);
			KF_Param.PitSpeed_Cur = TargetSpeed_Calc(&VisionKF_PitSpeed,\
															xTaskGetTickCount(),\
															/*Gimbal.Motor[Pit].get_totalencoder()*/Gimbal.Get_CurrentAngle(Pit));

			// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			KF_Param.YawKF_Result = kalmanFilter_Calc(&yaw_kalman_filter,\
															Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO,\
															0);
			KF_Param.PitKF_Result = kalmanFilter_Calc(&pit_kalman_filter,\
															/*Gimbal.Motor[Pit].get_totalencoder()*/Gimbal.Get_CurrentAngle(Pit),\
															0);
			KF_Param.Temp.ForecastAngle_Ramp = 0.0f; // 预测角度斜坡暂存量清零

		}
		// ----- 二阶卡尔曼计算 - END ----- //

		if(Get_Enemy() == true)
		{
#if USE_YAW_KALMAN_PREDICT
			KF_Param.AutoAim_Delay++;  // --- 自瞄突然开启，滤波延时开启
			
			// Recv_Msg.Pack.depth = KalmanFilter(&VisionDistance_Kalman, Get_DepthOffset());
			
			//----- Yaw轴预测 ----- //
			if((fabs(Get_YawOffset()*ENCODER_ANGLE_RATIO) < KF_Param.Yaw_Threshol.Angle_H) &&  /* 预测角度阈值 */
				(KF_Param.AutoAim_Delay > KF_Param.Forecast_Delay) && 					        /* 预测开启延迟 */
				(fabs(KF_Param.YawKF_Result[KF_SPEED]) >= KF_Param.Yaw_Threshol.Speed_L) &&     /* 预测速度阈值 */
				(fabs(KF_Param.YawKF_Result[KF_SPEED]) < KF_Param.Yaw_Threshol.Speed_H))
			{
				// --- 条件达成，开启预测 --- //
				if(KF_Param.YawKF_Result[KF_SPEED] >= 0)
				{
					KF_Param.Temp.ForecastAngle_Ramp = KF_Param.Yaw_Forecast *\
															(KF_Param.YawKF_Result[KF_SPEED] + KF_Param.Yaw_Threshol.Speed_L);
				}
				else if(KF_Param.YawKF_Result[KF_SPEED] < 0)
				{
					KF_Param.Temp.ForecastAngle_Ramp = KF_Param.Yaw_Forecast *\
															(KF_Param.YawKF_Result[KF_SPEED] - KF_Param.Yaw_Threshol.Speed_L);
				}
			
				// if(Infantry.Vision_Mode == Vision_Default)
				// {
					// KF_Param.Temp.ForecastAngle_Ramp *= (Get_DepthOffset()/1000.0f) / (Shoot.Get_SpeedLimit() - 2);
				// }
			
				// 限幅
				Constrain(&KF_Param.Temp.ForecastAngle_Ramp,-KF_Param.YawForecast_Limit,KF_Param.YawForecast_Limit);
				// 预测缓慢变化
				KF_Param.Temp.YawAngle = RAMP_Output(KF_Param.Temp.ForecastAngle_Ramp,KF_Param.Temp.YawAngle,ForecastAngle_Ramp);
				// 限幅
				// VAL_LIMIT(Vision_KFData.Temp.YawAngle,-Vision_KFData.YawForecast_Limit,Vision_KFData.YawForecast_Limit);
			
				*yawresult = KF_Param.YawKF_Result[KF_ANGLE] + KF_Param.Temp.YawAngle;
			
				// ----- 预测到位判断 ----- //
			
				// ---------------------- //
			
			}
			else // 预测条件没达到，关闭预测
			{
				*yawresult = KF_Param.Yaw_Record;
				// *yawresult = KF_Param.YawKF_Result[KF_ANGLE];
			
				if(abs(Get_YawOffset()) < 1.5f)
				{
					
				}
			}
#else
			//---- 卡尔曼滤波
			*yawresult = KF_Param.YawKF_Result[KF_ANGLE];

			//--- 不加滤波
			// *yawresult = KF_Param.Yaw_Record;
#endif
			// ----- Pitch轴预测 ----- //

			//---- 卡尔曼滤波
			*pitresult = KF_Param.PitKF_Result[KF_ANGLE];

			//--- 不加滤波
			// *pitresult = KF_Param.Pit_Record;

			Missing_Flag = 0;
		}
		else  // --- 未识别到目标
		{
			KF_Param.AutoAim_Delay = 0; //---清零，为下次自瞄开启做准备

			// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			KF_Param.YawKF_Result = kalmanFilter_Calc(&yaw_kalman_filter,\
															Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO,\
															0);
			KF_Param.PitKF_Result = kalmanFilter_Calc(&pit_kalman_filter,\
															/* Gimbal.Motor[Pit].get_totalencoder() */Gimbal.Get_CurrentAngle(Pit),\
															0);

			if(Missing_Flag == 0)  // - 过渡
			{
				*yawresult = Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO;
				*pitresult = /* Gimbal.Motor[Pit].get_totalencoder() */Gimbal.Get_CurrentAngle(Pit);

				Missing_Flag = 1;
			}
			else
			{
				
			}
			
		}
	}
	else
	{
		// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		KF_Param.YawSpeed_Cur = TargetSpeed_Calc(&VisionKF_YawSpeed,\
														  xTaskGetTickCount(),\
														  Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO);
		KF_Param.PitSpeed_Cur = TargetSpeed_Calc(&VisionKF_PitSpeed,\
														  xTaskGetTickCount(),\
														  /* Gimbal.Motor[Pit].get_totalencoder() */Gimbal.Get_CurrentAngle(Pit));

		// --- 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		KF_Param.YawKF_Result = kalmanFilter_Calc(&yaw_kalman_filter,\
														 Gimbal.Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO,\
		                                                 0);
		KF_Param.PitKF_Result = kalmanFilter_Calc(&pit_kalman_filter,\
													     /* Gimbal.Motor[Pit].get_totalencoder() */Gimbal.Get_CurrentAngle(Pit),\
														 0);

		
		SlopeData_Reset();

		KF_Param.AutoAim_Delay = 0; //--- 清零，为下次自瞄开启做准备
		Predict.start_cnt = 0;
	}


}

/**
  * @brief  	计算预测速度量
  * @param[in]	S
  * @param[in]  time
  * @param[in]	position
  * @retval 	processed_speed
  */
float debug_speed;//左正右负,一般都在1左右,debug看
float Vision_classdef::TargetSpeed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}


/**
 * @brief      自瞄预测补偿
 * @param[in]  None
 * @retval     Predict param
 */

float Vision_classdef::Predict_Ctrl()
{
	Predict.start_cnt++;

	Predict.Angle = Get_YawOffset();

	Predict.Omiga = KalmanFilter(&Predict.Omiga_KF, SlopeFilter_Calc(Get_OmigaData(), Predict.Angle_queue, 5));

	Predict.Accel = KalmanFilter(&Predict.Accel_KF, SlopeFilter_Calc(Get_AccelData(), Predict.Omiga_queue, 5));

	// Predict.Omiga = SlopeFilter_Calc(Get_OmigaData(), Predict.Angle_queue, 5);

	// Predict.Accel = SlopeFilter_Calc(Get_AccelData(), Predict.Omiga_queue, 5);


	// Predict.Angle = Get_YawOffset();

	// Predict.Omiga = KalmanFilter(&Predict.Omiga_KF, Get_OmigaData());

	// Predict.Accel = KalmanFilter(&Predict.Accel_KF, Get_AccelData());

	// Predict.Omiga = Get_OmigaData();

	// Predict.Accel = Get_AccelData();

	//--- 获取三项输出
	Predict.Angle_Out = Predict.Angle;

	Predict.Omiga_Out = Predict.Omiga * Predict.Ko;

	Predict.Accel_Out = Predict.Accel/2.0f * Predict.Ka;

	if(Predict.start_cnt > 30)
	{ 
		Predict.Out = Predict.Angle_Out + Predict.Omiga_Out + Predict.Accel_Out;
	}
	else
	{
		Predict.Out = Predict.Angle_Out;
	}

	// Predict.Out = KalmanFilter(&Predict.Out_KF, Predict.Out);

	return Predict.Out;

}


/**
 * @brief      获取速度
 * @param[in]  None
 * @retval     OmigaData
 */
float temp_fuck = 0.5f;
float Pre_OmigaRes;
float Vision_classdef::Get_OmigaData()
{
	float time = (float)KF_Param.Data_Delay;
	float Res = 0.0f;

	Res = (Predict.Angle - Predict.pre_Angle) / time;

	if(fabs(Res) <= 0.5f)
	{
		Pre_OmigaRes = Res;
	}
	else
	{
		Res = Pre_OmigaRes;
	}

	Predict.pre_Angle = Predict.Angle;

	return Res;

}


/**
 * @brief      获取加速度
 * @param[in]  None
 * @retval     AccelData
 */
float Pre_AccelRes;
float Vision_classdef::Get_AccelData()
{
	float time = (float)KF_Param.Data_Delay;
	float Res = 0.0f;

	Res = (Predict.Omiga - Predict.pre_Omiga) / time;

	if(abs(Res) <= 0.5)
	{
		Pre_AccelRes = Res;
	}
	else
	{
		Res = Pre_AccelRes;
	}

	Predict.pre_Omiga = Predict.Omiga;

	return Res;
}

/**
 * @brief	清空滑动滤波队列
 */
void Vision_classdef::SlopeData_Reset()
{
	for(uint8_t i = 0 ; i < PredictQueue_LEN ; i++)
	{
		Predict.Angle_queue[i] = 0;
		Predict.Omiga_queue[i] = 0;
	}
}


/**
 * @brief      获取视觉相关信息
 * @param[in]  state
 * @retval     None
 */
#if WYT_AIM
uint8_t Vision_classdef::Get_Enemy()
{
	return Sentry_Recv_Msg.Pack.enemy;
}
uint8_t Vision_classdef::IsAutoAim()
{
	return Sentry_Recv_Msg.Pack.auto_aim;
}
#else
uint8_t Vision_classdef::Get_Mode()
{
	return Recv_Msg.Pack.mode;
}
uint8_t Vision_classdef::IsAutoShoot()
{
	return Recv_Msg.Pack.auto_Shoot;
}
#endif
float Vision_classdef::Get_YawOffset()
{
	return Sentry_Recv_Msg.Pack.yaw;
}
float Vision_classdef::Get_PitOffset()
{
	return Sentry_Recv_Msg.Pack.pit;
}
float Vision_classdef::Get_DepthOffset()
{
	// return Recv_Msg.Pack.depth;
	return 0;
}
uint16_t Vision_classdef::Get_TimeDiff()
{
	return (TIME.Now - TIME.Pre);
}
