/**
  ******************************************************************************
  * @file    Kalman_Filter.c
  * @author  Athor
  * @version V1.0
  * @date
  * @brief   �������˲���
  ******************************************************************************
  */

#include "kalman_Filter.h"

/**    
  * @author  Liu heng
  * һ�׿������˲�������RoboMaster��̳  
  *   һά�������˲���                     
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲��� 
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
  *          ʹ��ʾ��                                             
  *          extKalman p;                  //����һ���������˲����ṹ��                                                 
  *          float SersorData;             //��Ҫ�����˲�������                                          
  *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����SystemPara_Q=20 SystemPara_R=200����                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //��ȡ����                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�                                                                            
  *          }                                                                            
  */

#include "kalman_Filter.h"

/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention SystemPara_R�̶���SystemPara_QԽ�󣬴���Խ���β���ֵ��SystemPara_Q�������ֻ�ò���ֵ
  *		       	��֮��SystemPara_QԽС����Խ����ģ��Ԥ��ֵ��SystemPara_QΪ������ֻ��ģ��Ԥ��
  *          
  *           ���ڹ�������Qֵ����ֵԽС�������Ƕ�ģ��Ԥ��ֵ����Խ�ߣ�ϵͳ������ҲԽ�죬��֮�෴��
  *           ���ڲ�������R����ֵԽ��������ǶԲ���ֵ����Խ�ͣ��������ʱϵͳ����Ϊ��Ӧ������С������ϵͳ�𵴡�
  *           ��������ʱ�̶�һ������һ������С�������Qֵʹϵͳ�����ٶ��������Ӵ�С����Rֵʹ�������ӽ���ʵ��
  *           ����X��P�ĳ�ʼֵ����ֵ�����˿�ʼʱ�������ٶȣ�һ������Ϊ������ֵ��ͬ���������߽�С������ 
  *           ����Ͽ�����������ŵ����Ľ��У�Pֵ������Ϊ��С�Ĺ���Э�������
  */
void KalmanCreate(extKalman_t *Filter, float T_Q, float T_R)
{
    Filter->Previous_X = (float)0;
    Filter->Previous_P = 0;
    Filter->SystemPara_Q = T_Q;
    Filter->SystemPara_R = T_R;
    Filter->SystemPara_A = 1;
    Filter->SystemPara_B = 0;
    Filter->SystemPara_H = 1;
    Filter->Forecast_X = Filter->Previous_X;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            SystemPara_A=1 SystemPara_B=0 SystemPara_H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��SystemPara_H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t *Filter, float Data)
{
    Filter->Forecast_X = Filter->SystemPara_A * Filter->Previous_X;                         //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = SystemPara_A*X(k-1|k-1)+SystemPara_B*U(k)+W(K)
    Filter->Forecast_P = Filter->SystemPara_A * Filter->Previous_P + Filter->SystemPara_Q;  //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = SystemPara_A*p(k-1|k-1)*SystemPara_A'+SystemPara_Q
    Filter->Tone_Up = Filter->Forecast_P / (Filter->Forecast_P + Filter->SystemPara_R);     //�ٶȶ�Ӧ��ʽ(4)    Tone_Up(k) = p(k|k-1)*SystemPara_H'/(SystemPara_H*p(k|k-1)*SystemPara_H'+SystemPara_R)
    Filter->Current_X = Filter->Forecast_X + Filter->Tone_Up * (Data - Filter->Forecast_X); //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+Tone_Up(k)*(Z(k)-SystemPara_H*X(k|k-1))
    Filter->Current_P = (1 - Filter->Tone_Up) * Filter->Forecast_P;                         //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-Tone_Up(k)*SystemPara_H)*P(k|k-1)
    Filter->Previous_P = Filter->Current_P;                                                 //״̬����
    Filter->Previous_X = Filter->Current_X;
    
    return Filter->Current_X; //���Ԥ����x(k|k)
}


/**
  *@param ��������ʼ��
  *@param 
  *@param 
  */
float matrix_value1;
float matrix_value2;
void kalmanFilter_Init(kalmanFilter_t *F, kalmanfilter_Init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);
  mat_init(&F->A,2,2,(float *)I->A_data);
  mat_init(&F->H,2,2,(float *)I->H_data);
  mat_init(&F->Q,2,2,(float *)I->Q_data);
  mat_init(&F->R,2,2,(float *)I->R_data);
  mat_init(&F->P,2,2,(float *)I->P_data);
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
  mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param �����������ṹ��
  *@param �Ƕ�
  *@param �ٶ�
  */
float *kalmanFilter_Calc(kalmanFilter_t *F, float signal_1, float signal_2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);//
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

  //--- ��ȡ����ֵ
  F->z.pData[0] = signal_1;//z(k)
  F->z.pData[1] = signal_2;//z(k)

  //1. xhat'(k)= A xhat(k-1)  xhat->ָ��^��x
  //����˻��ǵ�һ�������˵ڶ�������Ȼ�󽫽���ŵ���������������
	//��һ����������һʱ��Xhat���Ž�Ԥ�⵱ǰxhat
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
	//�ڶ������ɹ�ȥ��Э���������㵱ǰxhat��Э�������
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
	//�����������ݵ�ǰЭ���������㿨��������
	//��һ���������һ�����ⶼ����K���������������һ������
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	//���Ĳ�������QR���ۺ�Ԥ�����Ͳ���ֵ�����㵱ǰxhat���Ž�
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
	//���岽�����㵱ǰ���Ž��Э�������
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);


  //--- �õ����׿������ļ�����
  // matrix_value1 = F->xhat.pData[0];
  // matrix_value2 = F->xhat.pData[1];
  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  
  return F->filtered_value;
}

