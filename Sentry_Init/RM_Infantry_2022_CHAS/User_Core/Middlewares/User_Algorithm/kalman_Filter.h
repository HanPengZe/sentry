#ifndef _KALMAN_FILTERH_
#define _KALMAN_FILTERH_

#ifdef  __cplusplus
extern "C" {
#endif

#define __FPU_PRESENT 1U
#include "arm_math.h"

#define mat arm_matrix_instance_f32 //---浮点矩阵结构的简单实例结构.
//#define mat_64     arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32   //---浮点矩阵初始化.
#define mat_add arm_mat_add_f32     //---浮点矩阵加法.
#define mat_sub arm_mat_sub_f32     //---浮点矩阵减法.
#define mat_mult arm_mat_mult_f32   //---浮点矩阵乘法.
#define mat_trans arm_mat_trans_f32 //---浮点矩阵转置.
#define mat_inv arm_mat_inverse_f32 //---浮点矩阵的逆.
//#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
  float Previous_X;   //上一时刻的最优结果  X(k-|k-1)
  float Forecast_X;   //当前时刻的预测结果  X(k|k-1)
  float Current_X;    //当前时刻的最优结果  X(k|k)
  float Forecast_P;   //当前时刻预测结果的协方差  P(k|k-1)
  float Current_P;    //当前时刻最优结果的协方差  P(k|k)
  float Previous_P;   //上一时刻最优结果的协方差  P(k-1|k-1)
  float Tone_Up;      //kalman增益
  float SystemPara_A; //系统参数
  float SystemPara_B;
  float SystemPara_Q;
  float SystemPara_R;
  float SystemPara_H;
} extKalman_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalmanFilter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4] = {2, 0, 0, 2};
  float AT_data[4], HT_data[4];
  float A_data[4] = {1, 0.002, 0, 1};
  float H_data[4] = {1, 0, 0, 1};
  float Q_data[4] = {1, 0, 0, 1};
  float R_data[4] = { 200, 0, 0, 400 } ;
  
} kalmanfilter_Init_t;

void KalmanCreate(extKalman_t *Filter, float T_Q, float T_R);
float KalmanFilter(extKalman_t *Filter, float Data);
void kalmanFilter_Init(kalmanFilter_t *F, kalmanfilter_Init_t *I);
float *kalmanFilter_Calc(kalmanFilter_t *F, float signal_1, float signal_2);

#ifdef __cplusplus
}
#endif

#endif
