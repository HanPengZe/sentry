/**
  ==============================================================================
						    How to use this library 
  ==============================================================================
    @note
			- 低通滤波 
				-# LowPassFilter LF(trust);  
				-# 返回LF.f(num) 或者 LF << (in)  LF >> out 输出的时候才运算

			- 中值滤波 
				-# MedianFilter<Length> MDF;  
				-# 返回MDF.f(num) 或者 MDF << (in) MDF >> out 输出的时候才运算 

			- 均值滤波 
				-# MeanFilter<Length> MF;  
				-# 返回MF.f(num) 或者MF << (in)  MF >> out 输出的时候才运算 

  	@warning 
			- 低通滤波部分trust (0,1) ！！！注意超过不报错   中值滤波 均值滤波(长度[1,100])
			- Standard C++11 required! 
  
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

 /* Includes ------------------------------------------------------------------*/
#include "Filter.h"

/* Function prototypes -------------------------------------------------------*/
/* LowPassFilter */
void LowPassFilter::in(float num)							
{
	last_num = now_num;
	now_num = num;
}

float LowPassFilter::out()							
{
	return (now_num*Trust + last_num * (1 - Trust));
}


void LowPassFilter::operator <<(const float& num)			
{
	in(num);
}

void LowPassFilter::operator >>(float& num)
{
	num = out();
}

float LowPassFilter::f(float num)						
{
	in(num);
	return (out());
}
