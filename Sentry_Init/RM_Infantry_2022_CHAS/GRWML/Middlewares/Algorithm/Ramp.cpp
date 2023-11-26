/**
 ------------------------------------------------------------------------------
 * @file    Ramp.cpp
 * @author  Shake
 * @brief   数据斜坡处理函数
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (cpp) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "Ramp.h"
#include "User_Math.h"


/**
  * @brief  斜坡函数计算
  * @param	斜坡函数结构体
  * @retval None
  */
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp)
{

	SpeedRamp->count += SpeedRamp->rate;
	Constrain(&SpeedRamp->count, SpeedRamp->mincount, SpeedRamp->maxcount);

	return SpeedRamp->count;
}

/**
  * @brief  斜坡计数值归零
  * @param	斜坡函数结构体
  * @retval None
  */
void CountReset(SpeedRamp_t *SpeedRamp)
{

	if (abs(SpeedRamp->count) < abs(SpeedRamp->rate))
	{
		SpeedRamp->count = 0;
	}
	else
	{

		SpeedRamp->count -= SpeedRamp->count * 0.2;
	}
	//if (SpeedRamp->count > abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count -= abs(SpeedRamp->rate);
	//}
	//else if (SpeedRamp->count < -abs(SpeedRamp->rate))
	//{
	//	SpeedRamp->count += abs(SpeedRamp->rate);
	//}
	//else
	//{
	//
	//}
}

/**
  * @brief      斜坡函数,使目标输出值缓慢等于期望值
  * @param[in]  期望最终输出,当前输出,变化速度(越大越快)
  * @retval     当前输出
  * @attention  
  */
float RAMP_Output(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;

	if (buffer > 0)
	{
		if (buffer > ramp)
		{
			now += ramp;
		}
		else
		{
			now += buffer;
		}
	}
	else
	{
		if (buffer < -ramp)
		{
			now += -ramp;
		}
		else
		{
			now += buffer;
		}
	}

	return now;
}
