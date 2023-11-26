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


// --- ��ʼ��Yaw�Ĳ���kalman���� --- //
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
// --- ��ʼ��Pitch�Ĳ���Kalman���� --- //
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
	Vision_Disable = 0,		// �Ӿ�ʧ��	
	Vision_Default = 1,		// Ĭ������(�Ӿ����治��Ԥ��)
	Vision_BIGWindmill = 2, // ��糵
	// Vision_Sentry = 3,	// �ڱ�
	// Vision_BASE = 4,		// ����
	Vision_Top = 5,		// ����
	// Vision_Record = 6	// ¼��
	Vision_Forcast = 6		// �Ӿ�Ԥ��

};


/**
 * @brief ���ýṹ��Ķ���߽�Ϊ1�ֽ� ���������ڴ洢������������
 * @note  ͨ�������ڴ潫����ͬ�� �޸�Э��ʱ�޸�Pack���������ڴ��С
 */
#pragma pack(1)
typedef struct  // �Ӿ���Э�� ����һ֡�����ݽṹ��
{
    uint8_t  start_Tag; /*<! ��ʼλ */
    uint8_t  mode;      /*<! �Ƿ���Ŀ�� */
    uint8_t  auto_Shoot;/*<! �Զ���� */
    float 	 yaw;	    /*<! Yaw ƫ��Ƕ� */
    float 	 pit;	    /*<! Pit ƫ��Ƕ� */
    // uint16_t depth;	    /*<! ��� */  //--- ����ͨ�ų������⣬����ò��ϣ�ȥ��
    uint8_t  crc;       /*<! CRCУ���� */
    uint8_t  end_Tag;   /*<! ��ֹλ */

} VisionRecv_Pack_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint8_t start_tag;		/*<! ֡ͷ */
	// uint16_t crc;			/*<! crc����λ */
	uint8_t robot_id;		/*<! ������ID */
	uint8_t mode;			/*<! �Ӿ�ģʽ */
	float yaw_angle;		/*<! IMU Yaw */
	float pit_angle;		/*<! IMU Pit */
	uint8_t bullet_velocity;/*<! ���� */
	uint8_t end_tag;		/*<! ֡β'E' */

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
typedef struct  // �Ӿ���Э�� ����һ֡�����ݽṹ��
{
    uint8_t  header; /*<! ��ʼλ */ //0X A5
    uint8_t  enemy;      /*<! �Ƿ���Ŀ�� */
    uint8_t  auto_aim;/*<! �Զ���� */
    float 	 pit;	    /*<! Pit ƫ��Ƕ� */
    float 	 yaw;	    /*<! Yaw ƫ��Ƕ� */
		uint8_t enemy_id; /*<! ����ID */ //0ǰ��վ 1-5���� 6�ڱ� 7����
//		uint8_t detect_id;
//		float    depth;
    uint16_t  checksum;       /*<! CRCУ���� */
//	uint8_t hit;		
} Sentry_VisionRecv_Pack_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
	uint8_t header;		/*<! ֡ͷ */
	uint8_t detect_color;		/*<! �Է���������ɫ */
//	uint8_t reset_tracker;			/*<! ����׷�� */
//	uint8_t reserved;
	float roll;
	float pit;		/*<! IMU Yaw */
	float yaw;		/*<! IMU Pit */
//	float x;
//	float y;		/*<! IMU Yaw */
//	float z;		/*<! IMU Pit */
	uint16_t  checksum;       /*<! CRCУ���� */
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
typedef struct  // �Ӿ���Э�� ����һ֡�����ݽṹ��
{
    uint8_t start; /*<! ��ʼλ */
    float chassis_lx;//����X���ٶ�
    float chassis_ly;//����Y���ٶ�
    float chassis_az;//���������ٶ�(˳ʱ�븺)
    float chassis_forward;//����ǰ������(�Ƕ�)
		uint8_t mode;//����ģʽ 0 ����	1 ����	2 ����
//		uint8_t gimbal_mode;//��̨ģʽ 0 ͷ��ת	1 Ѳ��	2 ��ǰ��
		uint8_t end;
} Sentry_RadarRecv_Pack_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
	uint8_t start; /*<! ��ʼλ */
	uint8_t func; /*<! ������ */
	uint8_t game_progress;//��ǰ�����׶�
	uint16_t game_progress_remain;//����ʣ��ʱ��
	uint16_t sentry_hp;//����ʣ��Ѫ��
	uint16_t bullet_remain;//17mm ������������
	uint16_t defense_hp;//����ǰ��սѪ��
	uint8_t master_control;//��̨�ְ���
	uint16_t master_control_x;//��̨��ָ��λ��X������
	uint16_t master_control_y;//��̨��ָ��λ��Y������
	uint8_t mode;//1���� 2����
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




// --- �Ӿ�Ŀ���ٶȲ��� ---//
typedef struct
{
	int delay_cnt; //����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
	int freq;
	int last_time;		   //�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
	float last_position;   //�ϸ�Ŀ��Ƕ�
	float speed;		   //�ٶ�
	float last_speed;	   //�ϴ��ٶ�
	float processed_speed; //�ٶȼ�����
} speed_calc_data_t;

// --- �����ֵ
typedef struct
{
	float Speed_L;
	float Speed_H;
	float Angle_L;
	float Angle_H;
} ThresholdVal_t;

typedef struct
{
	float YawAngle_Cur;	 //������ Yaw ����ֵ
	float Yaw_Record;    // ��¼ Yaw ����ֵ
	float PitAngle_Cur;	  //������ Pitch ����ֵ
	float Pit_Record; // ��¼ Pitch ����ֵ
	float YawSpeed_Cur; // �������˲� �ٶȲ���ֵ
	float PitSpeed_Cur;	
	float *YawKF_Result; // ���׿������˲����,0�Ƕ� 1�ٶ�
	float *PitKF_Result;
	float Forecast_Delay; // Ԥ���ӳٿ�ʼ
	float ForecastAngle_Ramp;	// Ԥ��Ƕ�б�±仯��
	float Yaw_Forecast; // �ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	float Pit_Forecast;
	float YawForecast_Limit; //Ԥ�����޷�
	float PitForecast_Limit;
	float YawSpeed_Lowest; // �ٶȹ���
	int Data_Delay; // �Ӿ������ӳ�

	uint16_t AutoAim_Delay; //����ͻȻ�����Ŀ������˲��ӳ�
	ThresholdVal_t Yaw_Threshol; // �ٶȺͽǶ���ֵ
	ThresholdVal_t Pit_Threshol;

	struct
	{
		float ForecastAngle_Ramp; // Ԥ��Ƕ�б���ݴ���
		float YawAngle;			  // Yaw Ԥ���ݴ�
		float YawForecast_Para;	  // Yaw Ԥ��ϵ���ݴ�
	} Temp;

} Vision_KFData_t;

typedef struct
{
	float Angle_queue[PredictQueue_LEN];
	float Omiga_queue[PredictQueue_LEN];

	float Ke,Ko,Ka;
	float Angle;                      //�Ƕȣ�����ϵ�ĽǶ���ʵ������
	float pre_Angle;
	float Omiga;                      //���ٶ�
	float pre_Omiga;
	float Accel;                      //�Ǽ��ٶ�
	float pre_accel;

	float Angle_Out;//�Ƕ����
	float Omiga_Out;//�ٶ����
	float Accel_Out;//���ٶ����
	float Out;//�����

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
		UART_HandleTypeDef *vision_uart; // �ⲿ���
public:
    WorldTime_t TIME;

	Vision_KFData_t KF_Param; /*<! ��������ز��� */

		float Tar_Yaw_Vision;
		float Tar_Pit_Vision;
		float ForTurn_Yaw_Radar;
		float Tar_Yaw_Radar;

		// float Tar_Pit_Radar;
		uint8_t last_enemy,aim_flag;
#if WYT_AIM
	Sentry_VisionSendMsg_u Sentry_Send_Msg; /*<! ���������PC */
	Sentry_VisionRecvMsg_u Sentry_Recv_Msg; /*<! ��PC������� */	

	Sentry_RadarSendMsg_u Radar_Send_Msg; /*<! ���������PC */
	Sentry_RadarRecvMsg_u Radar_Recv_Msg; /*<! ��PC������� */	

	void Sentry_SendToPC(Sentry_VisionSendMsg_u *pack2vision);
	void Sentry_RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
	uint8_t Get_Enemy();
	uint8_t IsAutoAim();

	uint8_t Radar_Gimbal_Mode,Last_Radar_Gimbal_Mode;
	uint8_t Radar_Chassis_Mode,Last_Radar_Chassis_Mode;
	
#else
	VisionSendMsg_u Send_Msg; /*<! ���������PC */
	VisionRecvMsg_u Recv_Msg; /*<! ��PC������� */
	void SendToPC(VisionSendMsg_u *pack2vision);
	void RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
	uint8_t Get_Mode();
	uint8_t IsAutoShoot();
#endif

	Predict_Data_t Predict;

    uint8_t Update_Flag; /*<! ���ݸ��±�־λ */
    uint16_t FPS;        /*<! ֡�� */

    Vision_classdef();

    void Init(UART_HandleTypeDef *_huart);	/* �Ӿ�������ʼ�� */
		void Radar_RecvFromPC(uint8_t *data, uint16_t ReceiveLen);
		void Radar_SendToPC(Sentry_RadarSendMsg_u *pack2vision);
		
		uint16_t Get_TimeDiff();	/*<! ��ȡ��֡����֮���ʱ��� */
    float Get_YawOffset();
    float Get_PitOffset();
    float Get_DepthOffset();

	float Predict_Ctrl();	/*<! Ԥ�ⲹ�� */
	float Get_OmigaData();
	float Get_AccelData();
	void SlopeData_Reset();

    float TargetSpeed_Calc(speed_calc_data_t *S, uint32_t time, float position);
    void KalmanFilter_Ctrl(bool state, float *yawresult, float *pitresult);
};

extern extKalman_t VisionDistance_Kalman;


#endif
