#ifndef _DEVICES_MONITOR_H_
#define _DEVICES_MONITOR_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macro Definitions ---------------------------------------------------------*/
#define DR16_MONITOR		    (1<<0)  // DR16
#define CAN1_MONITOR			(1<<1)	// CAN1
#define CAN2_MONITOR		    (1<<2)	// CAN2
#define GIMBAL_YAW_MONITOR		(1<<3)	// Gimbal Yaw 6020
#define GIMBAL_PIT_MONITOR		(1<<4)	// Gimbal Pit 6020
#define FRIC_L_MONITOR			(1<<5)	// Fric L
#define FRIC_R_MONITOR			(1<<6)	// Fric R
#define RELOAD_MONITOR			(1<<7)	// Reload
#define VISION_MONITOR		    (1<<8)	// Vision
#define COMMU_0X341_MONITOR		(1<<9) // Board Commu
#define COMMU_0X342_MONITOR		(1<<10) // Board Commu
#define COMMU_0X343_MONITOR		(1<<11) // Board Commu
#define VISIONRADAR_MONITOR		    (1<<12)	// VisionRadar

#define AllDevices_MONITOR		(0x3FFF)	// All Devices

#define clrbit(x,y)  x&=~(1<<y) // ĳһλ����


#define On_line   0
#define Off_line  1


#define CRITICAL_VAL_INIT \
{\
    1,1,1,1,1,1,1,1,1,1,1,1,1\
};


/* Private type --------------------------------------------------------------*/
enum FrameType_e
{
    Frame_DR16,
	Frame_CAN1,
	Frame_CAN2,
	Frame_GIMBAL_YAW,
	Frame_GIMBAL_PIT,
	Frame_FRIC_L,
	Frame_FRIC_R,
	Frame_RELOAD,
	Frame_VISION,
	Frame_COMMU_0X341,
	Frame_COMMU_0X342,
	Frame_COMMU_0X343,
	Frame_VISIONRADAR,
	FrameCount_NUM

};

typedef struct
{
	uint32_t Now;  //��ǰ����ʱ��
	uint32_t Pre;  //��һ������ʱ��
}WorldTime_t;

/* Exported types ------------------------------------------------------------*/
class DevicesMonitor_classdef
{
private:
    static uint16_t Critical_Val[FrameCount_NUM];        /*<! ��֡�ٽ�ֵ */
    static uint32_t Devices_Frame;                         /*<! ���ڼ���豸�Ƿ����� */

public:
    uint16_t FrameCounter[FrameCount_NUM] = {0};         /*<! ���豸֡�ʴ������ */
    void Devices_Detec(void);                              /*<! �豸֡�ʼ�� */
	void Update(FrameType_e device);
    uint8_t Frame_Detec(uint16_t counter, uint16_t len);   /*<! ֡��ֵ��� */
    uint8_t Get_State(uint32_t device);          /*<! ��ȡ�豸״̬ */
	void Get_FPS(WorldTime_t *time, uint16_t *FPS);		   /*<! ��ȡ֡�� */
	uint16_t FPS_Calc(uint16_t delta_time)				   /*<! ����֡�� */
	const {return (1.0f / (double)(delta_time)) * 1000.0f;} // ��������ת��Ϊ��������������о��ȶ�ʧ
};

#endif
