#ifndef _CHASSIS_POWER_H_
#define _CHASSIS_POWER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


class CHAS_Power_classdef
{
private:
    int16_t Power_Buffer;   /*<! 缓冲功率 */
    int32_t SumCurrent_In;  /*<! 电流输入总和 */
    int32_t SumCurrent_Out; /*<! 计算后的电流输出总和 */

    int32_t RUD_Current_In;  /*<! 电流输入总和 */
    int32_t RUD_Current_Out; /*<! 计算后的电流输出总和 */
    int32_t DRV_Current_In;
    int32_t DRV_Current_Out;

    float DRV_CalcRatio;       /*<! 用于计算限制功率的系数 */
    float RUD_CalcRatio;       /*<! 用于计算限制功率的系数 */

public:
    CHAS_Power_classdef();
    void Limit(int16_t *wheelCurrent, int8_t amount);
    void Limit_Calc();

    void RUD_Limit_Calc();
    void RUD_Limit(int16_t *DRVCurrent, int16_t *RUDCurrent, int8_t amount);

};


#endif
