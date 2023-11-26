#ifndef _POWER_METER_H_
#define _POWER_METER_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct 
{
    float Voltage; //--- ��ѹ
    float Current; //--- ����
    float Power;   //--- ����
}PowerMeter_t;

void PowerMeter_Update(uint8_t recv_data[]);

extern PowerMeter_t Power_Meter;

#ifdef  __cplusplus
}
#endif

#endif
