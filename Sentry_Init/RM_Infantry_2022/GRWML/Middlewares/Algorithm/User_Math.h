#ifndef __USER_MATH_H
#define __USER_MATH_H
#define  __FPU_PRESENT  1U


#include <stdint.h>
#include "arm_math.h"

// #ifdef __cplusplus

#define HIGH 	0x1
#define LOW 	0x0
#define flaot0  1e-6  //当变量的绝对值小于此时，变量float为0 
//#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD) //angeTohudu
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

#define getBit(value, pos) ((value >> pos) & 1)
#define setBit(value, pos) value|(1 << pos)
#define clrBit(value, pos) value&(~(1 << pos))
#define toggleBit(value, pos) value^(1 << pos)

#define IndexOutofBounds(index,length) (index<0||index>length-1)


template<typename Type>
void Constrain(Type *val, Type min, Type max)
{
    if (*val <= min)
    {
        *val =  min;
    }
    else if(*val >= max)
    {
        *val =  max;
    }
}

template<typename Type>
Type abs(Type val) {return ((val > 0)? val : -val);}

// #endif

#endif /* __USERMATH_H */
