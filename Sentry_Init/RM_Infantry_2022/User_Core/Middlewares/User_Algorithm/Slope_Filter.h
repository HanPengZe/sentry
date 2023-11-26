#ifndef _SLOPE_FILTER_H_
#define _SLOPE_FILTER_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

float SlopeFilter_Calc(float data ,float *queue ,uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
