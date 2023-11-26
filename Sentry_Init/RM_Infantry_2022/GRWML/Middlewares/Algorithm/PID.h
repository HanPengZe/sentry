#pragma once

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <algorithm>

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum PIDCal_Type
{
  Outer,
  Inner
};

/** 
* @brief Class for traditional PID control.
*/
class PositionPID 
{
public:
    PositionPID() {}
    PositionPID(float _Kp, float _Ki, float _Kd,float _Dt) : Kp(_Kp), Ki(_Ki), Kd(_Kd), dt(_Dt){}
    void SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max,float _Dt)
    {
		Kp = _Kp;
		Ki = _Ki;
		Kd = _Kd;
		I_Term_Max = _I_Term_Max;
		Out_Max = _Out_Max;
		dt = _Dt;
    };
    float Cal();
	void Reset();
    float Target = 0, Current = 0, Error = 0;
    float Out = 0;

		float a_p,b_p,c_p;//��ṹKp
		float a_i,b_i,c_i;//��ṹKi
		
    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0;        /*<! I���޷� */
    float Out_Max = 0;           /*<! ����޷� */
		
	float dt = 0;

	float I_Term = 0;			/* ��������� */
    float P_Term = 0;			/* ��������� */
    float D_Term = 0;			/* ΢������� */

    float I_SeparThresh = 400;   /*!< ���ַ�����ֵ����Ϊ������fabs(error)���ڸ���ֵȡ���������á�*/


    float VarSpeed_I_A = ULONG_MAX; /*!< ���ٻ��� A����Ϊ������*/
    float VarSpeed_I_B = ULONG_MAX; /*!< ���ٻ��� B����Ϊ������ */

    float DeadZone = 0; 		    /*!< ��������Ϊ������fabs(error)С��DeadZoneʱ�����Ϊ0�� */



    bool D_of_Current = false; /*!< ����΢������ */

private:
    float pre_error = 0;
    float integral_e = 0;

    float pre_Current = 0;
};



class IncrementPID 
{
public:
    IncrementPID() {}
    IncrementPID(float _Kp, float _Ki, float _Kd,float _Dt) : Kp(_Kp), Ki(_Ki), Kd(_Kd), dt(_Dt){}
    void SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max,float _Dt)
    {
      	Kp = _Kp;
      	Ki = _Ki;
      	Kd = _Kd;
      	I_Term_Max = _I_Term_Max;
      	Out_Max = _Out_Max;
		dt = _Dt;
    };

    float Cal();
    // float Cal(float _target, float _current);
	void Reset();
    float Target = 0, Current = 0, Error = 0;
    float Out = 0;

    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0;        /*<! I���޷� */
    float Out_Max = 0;           /*<! ����޷� */
		
	float dt = 0;
	
	float I_Term = 0;			/* ��������� */
    float P_Term = 0;			/* ��������� */
    float D_Term = 0;			/* ΢������� */

    float I_SeparThresh = 400;   /*!< ���ַ�����ֵ����Ϊ������fabs(error)���ڸ���ֵȡ���������á�*/


    float VarSpeed_I_A = ULONG_MAX; /*!< ���ٻ��� A����Ϊ������*/
    float VarSpeed_I_B = ULONG_MAX; /*!< ���ٻ��� B����Ϊ������ \n
                                     �� error<=B �������ڣ�Ϊ��ͨ����Ч���� \n
                                     �� B<error<=A+B �������ڣ�Ϊ���ٻ���Ч���� \n
                                     �� A+B<error �������ڣ����������֡�*/

    float DeadZone = 0; 		    /*!< ��������Ϊ������fabs(error)С��DeadZoneʱ�����Ϊ0�� */



    bool D_of_Current = false; /*!< ����΢�����У�������Current������Process Variable(PV)�� */

private:
    float pre_error = 0;
	float prev_error = 0;
    float integral_e = 0;

    float pre_Current = 0;
};


/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
#endif


