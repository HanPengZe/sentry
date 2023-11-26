#ifndef _CONTROL_DR16_H_
#define _CONTROL_DR16_H_

/* Includes ------------------------------------------------------------------*/
#include "GRWML.h"
#include <stdint.h>
/* Private type --------------------------------------------------------------*/
typedef struct
{
    float Target_Vx;
    float Target_Vy;
    float Target_Vw;

    float Target_Yaw;
    float Target_Pit;
}DR16Export_t;

typedef struct
{
    float Vx_RC;
    float Vy_RC;
    float Vz_RC;

    float Yaw_RC;
    float Pit_RC;
    float Yaw_PC;
    float Pit_PC;

}ExpVal_Coefficient_e;

/* Exported types ------------------------------------------------------------*/
class CTRL_DR16_classdef
{
private:
    ExpVal_Coefficient_e Coe; /*<! ����ϵ�� */

protected:
    
public:
    DR16Export_t Expt;         /*<! ����������� */

    LowPassFilter Yaw_LPF = LowPassFilter(0.1f); /*<! ��ͨ�˲� */
    LowPassFilter Vw_LPF = LowPassFilter(0.01f); /*<! ��ͨ�˲� */


    CTRL_DR16_classdef();

    void LeverMode_Update();   //--- ����ģʽ����
    void CTRLSource_Update();  //--- ����Դ����

    void RCCtrl_Update();  //--- ң����ģʽ
    void PCCtrl_Update();  //--- PCģʽ
    void ExptData_Reset();  //--- �����������

    int8_t Data_Monitor();  //--- DR16���ݼ��

    /*<! ��ȡ����������� */
    float Get_ExptVx(); 
    float Get_ExptVy();
    float Get_ExptVw();
    float Get_ExptYaw();
    float Get_ExptPit();

};

#endif
