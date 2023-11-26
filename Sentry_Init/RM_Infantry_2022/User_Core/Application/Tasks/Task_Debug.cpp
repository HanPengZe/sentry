#include "Task_init.h"
#include "System_DataPool.h"
#include "SerialDebug.h"
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
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    for(;;)
    {
//				Data_send(Shoot.Fric_Motor[Fric_L].getSpeed(), -Shoot.Fric_Motor[Fric_R].getSpeed(),7400,0);
//Data_send(Gimbal.YawPID[C_IMU][PID_Outer].Target,Gimbal.YawPID[C_IMU][PID_Outer].Current,Gimbal.PitPID[C_IMU][PID_Outer].Target,Gimbal.PitPID[C_IMU][PID_Outer].Current);
Data_send(Gimbal.YawPID[C_IMU+3][PID_Outer].Target,Gimbal.YawPID[C_IMU+3][PID_Outer].Current,Gimbal.PitPID[C_IMU+3][PID_Outer].Target,Gimbal.PitPID[C_IMU+3][PID_Outer].Current);
        //SerialDebug.ANO_DataShow(Gimbal.YawPID[2][1].Out,Gimbal.Motor[0].getSpeed()*50,Chassis.Get_TargetVw(),Gimbal.Send_Motor[SendID_Yaw].Out);
        // SerialDebug.ANO_DataShow(Chassis.RUD_Param[0].Target_angle,debug_speed[2],0,0);
        // SerialDebug.ANO_DataShow(Chassis.RUD_Param[YawPID[C_IMU][PID_Outer].Current,0].Target_angle,Chassis.RUD_Param[0].Total_angle,Chassis.RUD_Param[3].Target_angle,Chassis.RUD_Param[3].Total_angle);

        // SerialDebug.ANO_DataShow(Gimbal.Motor[Yaw].getencoder(), Gimbal.Motor[Yaw].getAngle(), Gimbal.Motor[Yaw].get_totalencoder(),Chassis.Follow_PID[0].Out);

        // SerialDebug.ANO_DataShow(Shoot.Fric_Motor[Fric_L].getSpeed(), -Shoot.Fric_Motor[Fric_R].getSpeed(), Shoot.Get_GunSpeed(),0);

//        SerialDebug.ANO_DataShow(Vision.Get_YawOffset(),Vision.Get_PitOffset(),Vision.Get_Mode()*10,0);

        // SerialDebug.ANO_DataShow(Gimbal.YawPID[2][0].Target,Gimbal.YawPID[2][0].Current,Gimbal.YawPID[2][1].Target,Gimbal.YawPID[2][1].Current);
        // SerialDebug.ANO_DataShow(Gimbal.PitPID[0][0].Target,Gimbal.PitPID[0][0].Current,Gimbal.PitPID[0][1].Target,Gimbal.PitPID[0][1].Current);
        // SerialDebug.ANO_DataShow(Gimbal.PitPID[2][0].Target,Gimbal.PitPID[2][0].Current,Gimbal.PitPID[2][1].Target,Gimbal.PitPID[2][1].Current);
        // SerialDebug.ANO_DataShow(Gimbal.YawPID[5][0].Target,Gimbal.YawPID[5][0].Current,Gimbal.PitPID[3][0].Target,Gimbal.PitPID[3][0].Current);
        // SerialDebug.ANO_DataShow(Gimbal.YawPID[5][0].Target,Gimbal.YawPID[5][0].Current,Gimbal.YawPID[5][1].Target,Gimbal.YawPID[5][1].Current);

        // SerialDebug.ANO_DataShow(Gimbal.Motor[Yaw].get_totalencoder(), Gimbal.Motor[Yaw].getSpeed(), Gimbal.Motor[Pit].get_totalencoder(),  Gimbal.Motor[Pit].getSpeed());

        // SerialDebug.ANO_DataShow(Gimbal.Get_IMUGyro(Yaw), temp_gz, Gimbal.Get_IMUGyro(Pit), Gimbal.Get_IMUGyro(Rol));
        // SerialDebug.ANO_DataShow(Gimbal.Get_IMUGyro(Yaw), temp_gz, IMU_FilteSpeed[0]*(180/PI), INS_OffsetGyro[2]*(180/PI));
        // SerialDebug.ANO_DataShow(Gimbal.Get_IMUTotalAngle(Yaw), Gimbal.Get_IMUTotalAngle(Rol), IMU_FilteSpeed[0]*(180/PI), INS_OffsetGyro[2]*(180/PI));

        // SerialDebug.ANO_DataShow(Chassis.Acc_Param.linnerSpeed, Chassis.Acc_Param.linnerSpeedLast, Chassis.Target_Vy, Chassis.Target_Vx);
        // SerialDebug.ANO_DataShow(Shoot.Get_GunSpeed()*10,Shoot.Get_SpeedLimit()*10,0,0);

        // SerialDebug.ANO_DataShow(Gimbal.YawPID[5][0].Target,Gimbal.YawPID[5][0].Current, Vision.Predict.Angle_Out, Vision.Predict.Out);


        // SerialDebug.ANO_DataShow(Gimbal.YawPID[5][0].Target,Gimbal.YawPID[5][0].Current,Vision.Get_YawOffset(),Vision.Get_PitOffset());

        // SerialDebug.ANO_DataShow(Vision.IsAutoShoot(), biubiubiu, biubiubiu_flag, 0);

        // SerialDebug.ANO_DataShow(Vision.Get_YawOffset(), Vision.Predict.Out, Gimbal.YawPID[5][0].Target,Gimbal.YawPID[5][0].Current);

        // SerialDebug.ANO_DataShow(QEKF_INS.Yaw, QEKF_INS.IMU_QuaternionEKF.FilteredValue[0]*1000, BMI088.Gyro[0]*10, QEKF_INS.UpdateCount);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

