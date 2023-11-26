/**
  ==============================================================================
						    How to use this library 
  ==============================================================================
    @note
			- ��ͨ�˲� 
				-# LowPassFilter LF(trust);  
				-# ����LF.f(num) ���� LF << (in)  LF >> out �����ʱ�������

			- ��ֵ�˲� 
				-# MedianFilter<Length> MDF;  
				-# ����MDF.f(num) ���� MDF << (in) MDF >> out �����ʱ������� 

			- ��ֵ�˲� 
				-# MeanFilter<Length> MF;  
				-# ����MF.f(num) ����MF << (in)  MF >> out �����ʱ������� 

  	@warning 
			- ��ͨ�˲�����trust (0,1) ������ע�ⳬ��������   ��ֵ�˲� ��ֵ�˲�(����[1,100])
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
