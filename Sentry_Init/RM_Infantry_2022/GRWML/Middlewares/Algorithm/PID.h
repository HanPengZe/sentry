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

		float a_p,b_p,c_p;//变结构Kp
		float a_i,b_i,c_i;//变结构Ki
		
    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0;        /*<! I项限幅 */
    float Out_Max = 0;           /*<! 输出限幅 */
		
	float dt = 0;

	float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */

    float I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。fabs(error)大于该阈值取消积分作用。*/


    float VarSpeed_I_A = ULONG_MAX; /*!< 变速积分 A，需为正数。*/
    float VarSpeed_I_B = ULONG_MAX; /*!< 变速积分 B，需为正数， */

    float DeadZone = 0; 		    /*!< 死区，需为整数，fabs(error)小于DeadZone时，输出为0。 */



    bool D_of_Current = false; /*!< 启用微分先行 */

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
    float I_Term_Max = 0;        /*<! I项限幅 */
    float Out_Max = 0;           /*<! 输出限幅 */
		
	float dt = 0;
	
	float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */

    float I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。fabs(error)大于该阈值取消积分作用。*/


    float VarSpeed_I_A = ULONG_MAX; /*!< 变速积分 A，需为正数。*/
    float VarSpeed_I_B = ULONG_MAX; /*!< 变速积分 B，需为正数， \n
                                     在 error<=B 的区间内，为普通积分效果， \n
                                     在 B<error<=A+B 的区间内，为变速积分效果， \n
                                     在 A+B<error 的区间内，不继续积分。*/

    float DeadZone = 0; 		    /*!< 死区，需为整数，fabs(error)小于DeadZone时，输出为0。 */



    bool D_of_Current = false; /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */

private:
    float pre_error = 0;
	float prev_error = 0;
    float integral_e = 0;

    float pre_Current = 0;
};


/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
#endif


