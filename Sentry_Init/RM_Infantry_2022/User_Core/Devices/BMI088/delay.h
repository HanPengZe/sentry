#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef  __cplusplus
extern "C" {
#endif

#include<stdint.h>

void delay_init(void);
void delay_ms(uint16_t nms);
extern void delay_us(uint16_t nus);

#ifdef __cplusplus
}
#endif

#endif

