#include "Task_Debug.h"
#include "Task_Init.h"
#include "System_DataPool.h"
#include "SerialDebug.h"
#include "Control_Chassis.h"
#include "cmsis_os.h"
#include "ano.h"

/**
 * @brief      上位机任务
 * @param[in]  None
 * @retval     None
 */
void Task_Debug(void *argument)
{
   portTickType xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount();
   const TickType_t TimeIncrement = pdMS_TO_TICKS(5);  // --- 5MS 

    for(;;)
    {
        // SerialDebug.ANO_DataShow(debug_speed[0],debug_speed[1],debug_speed[2],0);
        // SerialDebug.ANO_DataShow(Chassis.RUD_Param[0].Target_angle,debug_speed[2],0,0);
        // SerialDebug.ANO_DataShow(Chassis.RUD_Param[0].Target_angle,Chassis.RUD_Param[0].Total_angle,Chassis.RUD_Param[3].Target_angle,Chassis.RUD_Param[3].Total_angle);

        // SerialDebug.ANO_DataShow(ssssspeed[0], ssssspeed[1], Chassis.Acc_Param.accKp,0);
        // SerialDebug.ANO_DataShow(Chassis.Target_Vy, Chassis.Target_Vx, Ramp_Vy, Ramp_Vx);
        // SerialDebug.ANO_DataShow(Chassis.Get_PowerLimit(),Chassis.Get_Power(),Chassis.Get_PowerBuffer(),CHAS_Power.debug_coe*1000);

        // SerialDebug.ANO_DataShow(Referee.ShootData.bullet_speed*10,0,0,0);

        // SerialDebug.ANO_DataShow(SupCap.RecvData.chassis_power*10, SupCap.RecvData.cap_cell, PowerMeter.Get_voltageVal()*10, PowerMeter.Get_Shunt_Current()*10);
        // SerialDebug.ANO_DataShow(SupCap.RecvData.cap_c ell, SupCap.SendData.charge_enable*10,0,0);

        // SerialDebug.ANO_DataShow(Chassis.Get_PowerLimit(), Chassis.Get_Power(), Chassis.Get_PowerBuffer(), 0);

        // SerialDebug.ANO_DataShow(Chassis.DRV_Motor[0].getSpeed(), Chassis.DRV_Motor[1].getSpeed(), Chassis.DRV_Motor[2].getSpeed(), Chassis.DRV_Motor[3].getSpeed());

//        SerialDebug.ANO_DataShow(Chassis.RUD_Param[0].Target_angle, Chassis.Target_Vx, Chassis.Target_Vy, Chassis.Target_Vw);
			Data_send(Chassis.Cal_Speed[RF_201],Chassis.DRV_Motor[0].getSpeed(), Chassis.Cal_Speed[LB_203],Chassis.DRV_Motor[1].getSpeed());

        // osDelay(1);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
