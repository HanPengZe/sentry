#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#define LOW_BATTER_VOLTAGE   21.9f

extern void init_vrefint_reciprocal(void);
extern float get_temprate(void);
extern float get_battery_voltage(void);
extern uint8_t get_hardware_version(void);
#endif
