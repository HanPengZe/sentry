/**
  ******************************************************************************
  * @file    Kalman_Filter.c
  * @author  Athor
  * @version V1.0
  * @date
  * @brief   卡尔曼滤波器
  ******************************************************************************
  */

#include "kalman_Filter.h"

/**    
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛  
  *   一维卡尔曼滤波器                     
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器 
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例                                             
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体                                                 
  *          float SersorData;             //需要进行滤波的数据                                          
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的SystemPara_Q=20 SystemPara_R=200参数                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //获取数据                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波                                                                            
  *          }                                                                            
  */

#include "kalman_Filter.h"

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention SystemPara_R固定，SystemPara_Q越大，代表越信任侧量值，SystemPara_Q无穷代表只用测量值
  *		       	反之，SystemPara_Q越小代表越信任模型预测值，SystemPara_Q为零则是只用模型预测
  *          
  *           对于过程噪声Q值，其值越小代表我们对模型预测值信任越高，系统收敛的也越快，反之相反；
  *           对于测量噪声R，其值越大代表我们对测量值信任越低，若过大此时系统表现为响应慢，过小则会出现系统震荡。
  *           调整参数时固定一个调整一个，从小到大调整Q值使系统收敛速度正常，从大到小调整R值使输出结果接近真实。
  *           对于X，P的初始值，其值决定了开始时的收敛速度，一般设置为与理想值相同数量级或者较小的数， 
  *           以求较快的收敛，随着迭代的进行，P值会收敛为最小的估计协方差矩阵。
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
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            SystemPara_A=1 SystemPara_B=0 SystemPara_H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶SystemPara_H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t *Filter, float Data)
{
    Filter->Forecast_X = Filter->SystemPara_A * Filter->Previous_X;                         //百度对应公式(1)    x(k|k-1) = SystemPara_A*X(k-1|k-1)+SystemPara_B*U(k)+W(K)
    Filter->Forecast_P = Filter->SystemPara_A * Filter->Previous_P + Filter->SystemPara_Q;  //百度对应公式(2)    p(k|k-1) = SystemPara_A*p(k-1|k-1)*SystemPara_A'+SystemPara_Q
    Filter->Tone_Up = Filter->Forecast_P / (Filter->Forecast_P + Filter->SystemPara_R);     //百度对应公式(4)    Tone_Up(k) = p(k|k-1)*SystemPara_H'/(SystemPara_H*p(k|k-1)*SystemPara_H'+SystemPara_R)
    Filter->Current_X = Filter->Forecast_X + Filter->Tone_Up * (Data - Filter->Forecast_X); //百度对应公式(3)    x(k|k) = X(k|k-1)+Tone_Up(k)*(Z(k)-SystemPara_H*X(k|k-1))
    Filter->Current_P = (1 - Filter->Tone_Up) * Filter->Forecast_P;                         //百度对应公式(5)    p(k|k) = (I-Tone_Up(k)*SystemPara_H)*P(k|k-1)
    Filter->Previous_P = Filter->Current_P;                                                 //状态更新
    Filter->Previous_X = Filter->Current_X;
    
    return Filter->Current_X; //输出预测结果x(k|k)
}


/**
  *@param 卡尔曼初始化
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
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
  */
float *kalmanFilter_Calc(kalmanFilter_t *F, float signal_1, float signal_2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);//
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

  //--- 获取测量值
  F->z.pData[0] = signal_1;//z(k)
  F->z.pData[1] = signal_2;//z(k)

  //1. xhat'(k)= A xhat(k-1)  xhat->指带^的x
  //矩阵乘积是第一个参数乘第二个参数然后将结果放到第三个参数里面
	//第一步，根据上一时刻Xhat最优解预测当前xhat
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
	//第二步，由过去的协方差矩阵计算当前xhat的协方差矩阵
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
	//第三步，根据当前协方差矩阵计算卡尔曼增益
	//这一步除了最后一句以外都是拿K当作储存计算结果的一个变量
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	//第四步，根据QR，综合预测量和测量值，计算当前xhat最优解
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
	//第五步，计算当前最优解的协方差矩阵
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);


  //--- 得到二阶卡尔曼的计算结果
  // matrix_value1 = F->xhat.pData[0];
  // matrix_value2 = F->xhat.pData[1];
  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  
  return F->filtered_value;
}

