#ifndef _POWER_METER_H_
#define _POWER_METER_H_


// #ifdef  __cplusplus
// extern "C" {
// #endif

#pragma once

#include <stdint.h>

#define POWERMETER_RECV_ID 0x301


typedef struct 
{
    float Voltage; //--- 电压
    float Current; //--- 电流
    float Power;   //--- 功率
}PowerMeter_t;

//SEASKY-INA226——https://github.com/SEASKY-Master/Seasky_INA22
#define INA226_USART_SIZE 24
#define INA226_CAN_SIZE 8
#define INA226_Self 1

//串口——100hz(放弃治疗)

//CAN——1MHz，频率100Hz(放弃治疗)


typedef union 
{
	struct
    {
        int16_t Power_Val;//功率mW
        int16_t voltageVal;//mV
        int16_t Shunt_Current;//mA
        int16_t Shunt_voltage;//uV
    }Pack;
	uint8_t data[INA226_CAN_SIZE]; 
}ina226RecvMsg_u;

typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
	uint8_t data[INA226_CAN_SIZE]; 
}ina226RecvMsg_Self_u;

class PowerMeter_classdef
{
private:

public:
    PowerMeter_classdef();
    void PowerMeter_Update(uint8_t recv_data[]);
    PowerMeter_t Power_Meter;

    ina226RecvMsg_u RecvData;
    ina226RecvMsg_Self_u Self_RecvData;

    void Update(uint8_t can_rx_data[]);

    float Get_Power_Val();
    float Get_voltageVal();
    float Get_Shunt_Current();
    float Get_Shunt_voltage();
};

// extern PowerMeter_t Power_Meter;

// void PowerMeter_Update(uint8_t recv_data[]);

// #ifdef  __cplusplus
// }
// #endif

#endif
