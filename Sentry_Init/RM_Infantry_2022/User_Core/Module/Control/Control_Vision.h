#ifndef _CONTROL_VISION_H_
#define _CONTROL_VISION_H_

#pragma anon_unions

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "kalman_Filter.h"
#include "Devices_Monitor.h"
/* Private macros ------------------------------------------------------------*/
#define Comm_NewType   0 
#define VISION_TX_SIZE    15
#define VISION_RX_SIZE    13

#define PredictQueue_LEN	5

#define VISION_ON   1
#define VISION_OFF  0
#define KALMAN_ON	1
#define KALMAN_OFF	0
#define WYT_AIM 1


// --- 初始化Yaw的部分kalman参数 --- //
#define KalmanParam_Yaw_Init        \
	{                               \
		.P_data = {2, 0, 0, 2},     \
		.A_data = {1, 0.002, 0, 1}, \
		.H_data = {1, 0, 0, 1},     \
		.Q_data = {1, 0, 0, 1},     \
		.R_data = { 200,            \
					0,              \
					0,              \
					400 }           \
	}
// --- 初始化Pitch的部分Kalman参数 --- //
#define KalmanParam_Pit_Init        \
	{                               \
		.P_data = {2, 0, 0, 2},     \
		.A_data = {1, 0.002, 0, 1}, \
		.H_data = {1, 0, 0, 1},     \
		.Q_data = {1, 0, 0, 1},     \
		.R_data = { 200,            \
					0,              \
					0,              \
					400 }           \
	}



/* Exported types ------------------------------------------------------------*/

enum VisionMode_e
{
	Vision_Disable = 0,		// 视觉失能	
	Vision_Default = 1,		// 默认自瞄(视觉方面不加预测)
	Vision_BIGWindmill = 2, // 大风车
	// Vision_Sentry = 3,	// 哨兵
	// Vision_BASE = 4,		// 基地
	Vision_Top = 5,		// 陀螺
	// Vision_Record = 6	// 录像
	Vision_Forcast = 6		// 视觉预测

};


/**
 * @brief 设置结构体的对齐边界为1字节 让数据在内存储存中是连续的
 * @note  通过公用内存将数据同步 修改协议时修改Pack的数据与内存大小
 */
#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
    uint8_t  start_Tag; /*<! 起始位 */
    uint8_t  mode;      /*<! 是否有目标 */
    uint8_t  auto_Shoot;/*<! 自动射击 */
    float 	 yaw;	    /*<! Yaw 偏差角度 */
    float 	 pit;	    /*<! Pit 偏差角度 */
    // uint16_t depth;	    /*<! 深度 */  //--- 串口通信出现问题，深度用不上，去掉
    uint8_t  crc;       /*<! CRC校验码 */
    uint8_t  end_Tag;   /*<! 终止位 */

} VisionRecv_Pack_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint8_t start_tag;		/*<! 帧头 */
	// uint16_t crc;			/*<! crc检验位 */
	uint8_t robot_id;		/*<! 机器人ID */
	uint8_t mode;			/*<! 视觉模式 */
	float yaw_angle;		/*<! IMU Yaw */
	float pit_angle;		/*<! IMU Pit */
	uint8_t bullet_velocity;/*<! 射速 */
	uint8_t end_tag;		/*<! 帧尾'E' */

}VisionSend_Pack_t;
#pragma pack()
typedef union 
{
	uint8_t data[13];
	VisionSend_Pack_t Pack;

}VisionSendMsg_u;
typedef union
{
	uint8_t data[13];
	VisionRecv_Pack_t Pack;
}VisionRecvMsg_u;


#define VISION_Sentry_RX_SIZE    14
#define VISION_Sentry_TX_SIZE    16
#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
    uint8_t  header; /*<! 起始位 */ //0X A5
    uint8_t  enemy;      /*<! 是否有目标 */
    uint8_t  auto_aim;/*<! 自动射击 */
    float 	 pit;	    /*<! Pit 偏差角度 */
    float 	 yaw;	    /*<! Yaw 偏差角度 */
		uint8_t enemy_id; /*<! 敌人ID */ //0前哨站 1-5地面 6哨兵 7基地
//		uint8_t detect_id;
//		float    depth;
    uint16_t  checksum;       /*<! CRC校验码 */
//	uint8_t hit;		
} Sentry_VisionRecv_Pack_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
	uint8_t header;		/*<! 帧头 */
	uint8_t detect_color;		/*<! 对方机器人颜色 */
//	uint8_t reset_tracker;			/*<! 重置追踪 */
//	uint8_t reserved;
	float roll;
	float pit;		/*<! IMU Yaw */
	float yaw;		/*<! IMU Pit */
//	float x;
//	float y;		/*<! IMU Yaw */
//	float z;		/*<! IMU Pit */
	uint16_t  checksum;       /*<! CRC校验码 */
}Sentry_VisionSend_Pack_t;
#pragma pack()
typedef union 
{
	uint8_t data[VISION_Sentry_TX_SIZE];
	Sentry_VisionSend_Pack_t Pack;
}Sentry_VisionSendMsg_u;
typedef union
{
	uint8_t data[VISION_Sentry_RX_SIZE];
	Sentry_VisionRecv_Pack_t Pack;
}Sentry_VisionRecvMsg_u;


#define VISION_Radar_RX_SIZE    19
#define VISION_Radar_TX_SIZE    18
#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
    uint8_t start; /*<! 起始位 */
    float chassis_lx;//底盘X轴速度
    float chassis_ly;//底盘Y轴速度
    float chassis_az;//底盘自旋速度(顺时针负)
    float chassis_forward;//底盘前进方向(角度)
		uint8_t mode;//底盘模式 0 不动	1 低速	2 高速
//		uint8_t gimbal_mode;//云台模式 0 头不转	1 巡逻	2 推前哨
		uint8_t end;
} Sentry_RadarRecv_Pack_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
	uint8_t start; /*<! 起始位 */
	uint8_t func; /*<! 功能码 */
	uint8_t game_progress;//当前比赛阶段
	uint16_t game_progress_remain;//比赛剩余时间
	uint16_t sentry_hp;//自身剩余血量
	uint16_t bullet_remain;//17mm 弹丸允许发弹量
	uint16_t defense_hp;//己方前哨战血量
	uint8_t master_control;//云台手按键
	uint16_t master_control_x;//云台手指定位置X轴坐标
	uint16_t master_control_y;//云台手指定位置Y轴坐标
	uint8_t mode;//1激进 2保守
	uint8_t end;
}Sentry_RadarSend_Pack_t;
#pragma pack()
typedef union
{
	uint8_t data[VISION_Radar_RX_SIZE];
	Sentry_RadarRecv_Pack_t Pack;
}Sentry_RadarRecvMsg_u;
typedef union 
{
	uint8_t data[VISION_Radar_TX_SIZE];
	Sentry_RadarSend_Pack_t Pack;
}Sentry_RadarSendMsg_u;




// --- 视觉目标速度测量 ---//
typedef struct
{
	int delay_cnt; //计算相邻两帧目标不变持续时间,用来判断速度是否为0
	int freq;
	int last_time;		   //上次受到目标角度的时间
	float last_position;   //上个目标角度
	float speed;		   //速度
	float last_speed;	   //上次速度
	float processed_speed; //速度计算结果
} speed_calc_data_t;

// --- 相关阈值
typedef struct
{
	float Speed_L;
	float Speed_H;
	float Angle_L;
	float Angle_H;
} ThresholdVal_t;

typedef struct
{
	float YawAngle_Cur;	 //卡尔曼 Yaw 测量值
	float Yaw_Record;    // 记录 Yaw 测量值
	float PitAngle_Cur;	  //卡尔曼 Pitch 测量值
	float Pit_Record; // 记录 Pitch 测量值
	float YawSpeed_Cur; // 卡尔曼滤波 速度测量值
	float PitSpeed_Cur;	
	float *YawKF_Result; // 二阶卡尔曼滤波结果,0角度 1速度
	float *PitKF_Result;
	float Forecast_Delay; // 预测延迟开始
	float ForecastAngle_Ramp;	// 预测角度斜坡变化量
	float Yaw_Forecast; // 移动预测系数,越大预测越多
	float Pit_Forecast;
	float YawForecast_Limit; //预测量限幅
	float PitForecast_Limit;
	float YawSpeed_Lowest; // 速度过低
	int Data_Delay; // 视觉数据延迟

	uint16_t AutoAim_Delay; //自瞄突然开启的卡尔曼滤波延迟
	ThresholdVal_t Yaw_Threshol; // 速度和角度阈值
	ThresholdVal_t Pit_Threshol;

	struct
	{
		float ForecastAngle_Ramp; // 预测角度斜坡暂存量
		float YawAngle;			  // Yaw 预测暂存
		float YawForecast_Para;	  // Yaw 预测系数暂存
	} Temp;

} Vision_KFData_t;

typedef struct
{
	float Angle_queue[PredictQueue_LEN];
	float Omiga_queue[PredictQueue_LEN];

	float Ke,Ko,Ka;
	float Angle;                      //角度（坐标系的角度其实就是误差）
	float pre_Angle;
	float Omiga;                      //角速度
	float pre_Omiga;
	float Accel;                      //角加速度
	float pre_accel;

	float Angle_Out;//角度输出
	float Omiga_Out;//速度输出
	float Accel_Out;//角速度输出
	float Out;//总输出

	uint32_t start_cnt;

	extKalman_t Angle_KF;
  	extKalman_t Omiga_KF;
  	extKalman_t Accel_KF;
  	extKalman_t Out_KF;
}Predict_Data_t;

class Vision_classdef
{
private:
    uint8_t CRCBuf;
		UART_HandleTypeDef *vision_uart; // 外部句柄
public:
    WorldTime_t TIME;

	Vision_KFData_t KF_Param; /*<! 卡尔曼相关参数 */

		float Tar_Yaw_Vision;
		float Tar_Pit_Vision;
		float ForTurn_Yaw_Radar;
		float Tar_Yaw_Radar;

		// float Tar_Pit_Radar;
		uint8_t last_enemy,aim_flag;
#if WYT_AIM
	Sentry_VisionSendMsg_u Sentry_Send_Msg; /*<! 打包数据至PC */
	Sentry_VisionRecvMsg_u Sentry_Recv_Msg; /*<! 从PC解包数据 */	

	Sentry_RadarSendMsg_u Radar_Send_Msg; /*<! 打包数据至PC */
	Sentry_RadarRecvMsg_u Radar_Recv_Msg; /*<! 从PC解包数据 */	

	void Sentry_SendToPC(Sentry_VisionSendMsg_u *pack2vision);
	void Sentry_RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
	uint8_t Get_Enemy();
	uint8_t IsAutoAim();

	uint8_t Radar_Gimbal_Mode,Last_Radar_Gimbal_Mode;
	uint8_t Radar_Chassis_Mode,Last_Radar_Chassis_Mode;
	
#else
	VisionSendMsg_u Send_Msg; /*<! 打包数据至PC */
	VisionRecvMsg_u Recv_Msg; /*<! 从PC解包数据 */
	void SendToPC(VisionSendMsg_u *pack2vision);
	void RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
	uint8_t Get_Mode();
	uint8_t IsAutoShoot();
#endif

	Predict_Data_t Predict;

    uint8_t Update_Flag; /*<! 数据更新标志位 */
    uint16_t FPS;        /*<! 帧率 */

    Vision_classdef();

    void Init(UART_HandleTypeDef *_huart);	/* 视觉参数初始化 */
		void Radar_RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
		void Radar_SendToPC(Sentry_RadarSendMsg_u *pack2vision);
		
		uint16_t Get_TimeDiff();	/*<! 获取两帧数据之间的时间差 */
    float Get_YawOffset();
    float Get_PitOffset();
    float Get_DepthOffset();

	float Predict_Ctrl();	/*<! 预测补偿 */
	float Get_OmigaData();
	float Get_AccelData();
	void SlopeData_Reset();

    float TargetSpeed_Calc(speed_calc_data_t *S, uint32_t time, float position);
    void KalmanFilter_Ctrl(bool state, float *yawresult, float *pitresult);
};

extern extKalman_t VisionDistance_Kalman;


#endif
