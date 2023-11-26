#ifndef _VISION_H_
#define _VISION_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private macros ------------------------------------------------------------*/
#define Comm_NewType   0 
#define ROBOTTYPE_NUM  5
#define TX_SIZE        18
#define RX_SIZE        15

#define MY_CRCTAB_INIT  \
{\
	0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,\
	157,195,33,127,252,162,64,30, 95,1,227,189,62,96,130,220,\
	35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,\
	190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,\
	70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,\
	219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,\
	101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,\
	248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,\
	140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,\
	17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,\
	175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,\
	50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,\
	202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,\
	87,9,235,181,54,104,138,212,149,203, 41,119,244,170,72,22,\
	233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,\
	116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53\
}



/* Exported types ------------------------------------------------------------*/
#if (Comm_NewType)
#pragma pack(1)
typedef struct  // 视觉的协议 接收一帧的数据结构体
{
    char Start_Tag;     /*<! 起始位 */
    int8_t mode;        /*<! 是否有目标 */
    int8_t Aotu_Shoot;  /*<! 自动射击 */
    float Yaw;		    /*<! Yaw 偏差角度 */
    float Pit;			/*<! Pit 偏差角度 */
    int16_t depth;		/*<! 深度 */
    int16_t crc;        /*<! CRC校验码 */
    char End_Tag;       /*<! 终止位 */

} RecvData_t;
#pragma pack()

#else

typedef struct
{
    char Start_Tag;
    int8_t mode;
    int8_t mode_select;
    uint8_t Aotu_Shoot; //是否可以射击(控制自动射击)

    float x;		   // yaw轴，为屏幕坐标点
    int8_t Yaw_sign;   // Yaw 轴正负号     0 -> 正 | 1-> 负
    uint8_t Yaw_DataH; // yaw高八位
    uint8_t Yaw_DataL; // yaw低八位

    float y;			 // pitch 轴
    int8_t Pitch_sign;	 // Pitch 轴正负号    0 -> 负 | 1-> 正
    uint8_t Pitch_DataH; // pitch高八位
    uint8_t Pitch_DataL; // pitch低八位

    int16_t depth;		 //深度
    uint8_t Depth_DataH; //深度高八位
    uint8_t Depth_DataL; //深度低八位

    int16_t crc;
    char End_Tag;

} RecvData_t; // 视觉的协议 接收一帧的数据结构体
#endif

typedef struct
{
	union
	{
		struct
		{
			float IMU_Yaw;	//陀螺仪YAW角度
			float IMU_Pit;  //陀螺仪Pit角度
		};
		uint8_t IMUAngle_Data[8];
	} IMUangle_t;

	int Gyro_z; //陀螺仪加速度小数点后两位
	int Gyro_y;

	uint8_t GyroZ_H; //陀螺仪加速度小数点后两位高八位
	uint8_t GyroZ_L; //陀螺仪加速度小数点后两位低八位

	uint8_t GyroY_H; //陀螺仪加速度小数点后两位高八位
	uint8_t GyroY_L; //陀螺仪加速度小数点后两位低八位


} TrasData_t;

class Vision_classdef
{
private:
    uint8_t CRCBuf;
    uint8_t TX_Buf[ROBOTTYPE_NUM][TX_SIZE];
    uint8_t RX_Buf[RX_SIZE];
    RecvData_t Recv_Pack;
    TrasData_t Tras_Pack;

public:
    Vision_classdef();
    void Recv_Data(uint8_t *data);
    void Update_TXBuf();
    void Set_TXData(float angle_Yaw, float angle_Pit, float Gyro_w, float Gyro_y);

};



#endif
