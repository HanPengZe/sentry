#ifndef _KALMAN_FILTERH_
#define _KALMAN_FILTERH_

#ifdef  __cplusplus
extern "C" {
#endif

#define __FPU_PRESENT 1U
#include "arm_math.h"

#define mat arm_matrix_instance_f32 //---�������ṹ�ļ�ʵ���ṹ.
//#define mat_64     arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32   //---��������ʼ��.
#define mat_add arm_mat_add_f32     //---�������ӷ�.
#define mat_sub arm_mat_sub_f32     //---����������.
#define mat_mult arm_mat_mult_f32   //---�������˷�.
#define mat_trans arm_mat_trans_f32 //---�������ת��.
#define mat_inv arm_mat_inverse_f32 //---����������.
//#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
  float Previous_X;   //��һʱ�̵����Ž��  X(k-|k-1)
  float Forecast_X;   //��ǰʱ�̵�Ԥ����  X(k|k-1)
  float Current_X;    //��ǰʱ�̵����Ž��  X(k|k)
  float Forecast_P;   //��ǰʱ��Ԥ������Э����  P(k|k-1)
  float Current_P;    //��ǰʱ�����Ž����Э����  P(k|k)
  float Previous_P;   //��һʱ�����Ž����Э����  P(k-1|k-1)
  float Tone_Up;      //kalman����
  float SystemPara_A; //ϵͳ����
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
