#include "Task_init.h"
#include "System_DataPool.h"
#include "Dev_Buzzer.h"
// #include "calibrate_task.h"
#include "Dev_OLED.h"
#include "BSP_ADC.h"
#include "../BMI088_EKF/BMI088driver.h"

/**
 * @brief      �豸�������
 * @param[in]  None
 * @retval     None
 */
float bat_voltage; //��Դ��ѹ
void Task_DevicesMonitor(void *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(50);  // --- 20MS 

    static uint16_t monitor_cnt = 0;
    static uint16_t oled_cnt = 0;
 
    static uint8_t gimbal_init = false;
    static uint8_t imu_cali_flag = false;
    static uint16_t imu_cali_cnt = 0;
    static uint8_t DR16_offline_flag = false;

    static int16_t low_bat_voltage_cnt = 600;

    for(;;)
    {
        // �豸��� ������ --------------------------------------------------------------
        if(((++monitor_cnt)%=4)==0) //--- 200ms
        {
            //--- �豸���
            DevicesMonitor.Devices_Detec();

            //--- ��Դ��ѹ���
            bat_voltage = get_battery_voltage();
            low_bat_voltage_cnt--;

            // ������ ---------------------------------------------------------------------
            if(DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
            {
                //--- ң������������һֱBBBB...
                Buzzer.Set_RingType(Ring_ContShort);
                Infantry.Disable();
                Infantry.CtrlSource_Switch(CTRL_OFF);
                DR16_offline_flag = true;
                
            }
            else if(DevicesMonitor.Get_State(DR16_MONITOR) == On_line && DR16_offline_flag == true)
            {
                Buzzer.Set_RingType(Ring_Stop);
                DR16_offline_flag = false;
            }
            else if(bat_voltage < LOW_BATTER_VOLTAGE && low_bat_voltage_cnt <= 600) //--- �͵�ѹ����
            {
                Buzzer.Set_RingType(Ring_5times); //--- 2����һ��BBBBB
                if(low_bat_voltage_cnt < 0)
                {
                    Buzzer.Set_RingType(Ring_Stop);
                    low_bat_voltage_cnt = 600;
                }
            }
            else
            {
                //--- ������̨��ʼ����� �����
                if(Gimbal.init_mode == false && gimbal_init == false)
                {
                    // Buzzer.Set_RingType(Ring_2times);
                    Buzzer.Music_Play(SuperMario, sizeof(SuperMario));
                    gimbal_init = true;
                }
                //--- IMUУ׼��ʾ�� һֱB~B~B~B~...
                else if(imu_cali_bell == true)
                {
                    if(++imu_cali_cnt > 20)
                    {
                        imu_cali_bell = false;
                        imu_cali_flag = true;
                    }
                    Buzzer.Set_RingType(Ring_ContLong);
                }
                else if(cali_cmd == false && imu_cali_flag == true)
                {
                    //--- У׼��� �����
                    Buzzer.Music_Play(SuperMario,sizeof(SuperMario));
                    Buzzer.Set_RingType(Ring_Stop);
                    imu_cali_flag = false; 
                }
            }


            Buzzer.Process(); //--- ����������
        }

        

        // OLED --------------------------------------------------------------
        oled_cnt++;
        if((DR16.Get_LX_Norm()||DR16.Get_LY_Norm())!=0)
        {
            if(DR16.Get_LX_Norm() <= -330)
            {
                OLED_ShowMessage(3); //--- �� �Ӿ�����
            }
            else if(DR16.Get_LX_Norm() >= 330)
            {
                OLED_ShowMessage(1); //--- �� ��̨����
            }
            else if(DR16.Get_LY_Norm() >= 330)
            {
                OLED_ShowMessage(2); //--- �� ��������
            }
            else if(DR16.Get_LY_Norm() <= -330)
            {
                // OLED_ShowMessage(4);  //--- ��
            }
        }
        else if(Infantry.Get_GimbalMode() == Gimbal_PCMode)
        {
            OLED_ShowMessage(3); //--- ����ģʽ����ʱ��ʾ�Ӿ�����
        }
        else
        {
            // uint32_t temp_monitor = AllDevices_MONITOR;
            if(DevicesMonitor.Get_State(0x3FF) == Off_line || 
               (Infantry.Write_Msg[0][0] || Infantry.Write_Msg[0][1] || Infantry.Write_Msg[0][2] || 
                Infantry.Write_Msg[0][3] || Infantry.Write_Msg[0][4] || Infantry.Write_Msg[0][5] ||
                Infantry.Write_Msg[0][6] || Infantry.Write_Msg[0][7] || Infantry.Write_Msg[1][0] || 
                Infantry.Write_Msg[1][1] || Infantry.Write_Msg[1][2] || Infantry.Write_Msg[1][3]) == Off_line) //--- �����can0x342
            {
                if(oled_cnt < 100)
                {
                    OLED_ShowMessage(0); //--- ��ʾ�쳣��Ϣ
                }
                // else if(oled_cnt < 200)
                // {
                //     OLED_ShowMessage(4); //--- ��ʾ�쳣��Ϣ
                // }
                else if(oled_cnt < 200)
                {
                    OLED_ShowMessage(5); //--- ��ʾLogo
                }
                else
                {
                    oled_cnt = 0;
                }
            }
            else
            {
                OLED_ShowMessage(5); //--- ȫ������ֻ��ʾlogo
            }
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

