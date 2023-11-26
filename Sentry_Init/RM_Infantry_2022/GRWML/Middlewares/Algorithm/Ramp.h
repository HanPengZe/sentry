#ifndef _RAMP_H_
#define _RAMP_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#pragma diag_suppress 1035

#ifdef __cplusplus
extern "C"{
#endif
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float count;  //当前的总速度值
	float rate;   //每一次叠加的速度值
	float mincount;
	float maxcount;
}SpeedRamp_t;


void CountReset(SpeedRamp_t *SpeedRamp);
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp);
float RAMP_Output( float final, float now, float ramp );

#ifdef __cplusplus
}
#endif

#endif
