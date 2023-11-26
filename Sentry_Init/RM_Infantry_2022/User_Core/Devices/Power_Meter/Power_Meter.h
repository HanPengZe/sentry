#ifndef _POWER_METER_H_
#define _POWER_METER_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct 
{
    float Voltage; //--- 电压
    float Current; //--- 电流
    float Power;   //--- 功率
}PowerMeter_t;

void PowerMeter_Update(uint8_t recv_data[]);

extern PowerMeter_t Power_Meter;

#ifdef  __cplusplus
}
#endif

#endif
